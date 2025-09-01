from langgraph.graph import START, StateGraph
from langgraph.graph.state import CompiledStateGraph
from langgraph.prebuilt import ToolNode, tools_condition
from langgraph.checkpoint.memory import MemorySaver
from langchain_core.messages import HumanMessage, SystemMessage, AIMessage
from typing_extensions import TypedDict
from langgraph.graph.message import add_messages

from langchain_openai import ChatOpenAI
from typing import Annotated
import base64
import requests

def is_server_running(url):
    try:
        response = requests.get(url, timeout=2)
        return response.status_code == 200
    except Exception:
        return False
    
def displayGraph(graph : CompiledStateGraph):
    try:
        img_bytes = graph.get_graph().draw_mermaid_png()
        with open("graph.png", "wb") as f:
            f.write(img_bytes)
        import os
        os.startfile("graph.png")  # This will open the image with the default viewer on Windows
    except Exception as e:
        # This requires some extra dependencies and is optional
        print(e)

class State(TypedDict):
    # Messages have the type "list". The `add_messages` function
    # in the annotation defines how this state key should be updated
    # (in this case, it appends messages to the list, rather than overwriting them)
    messages: Annotated[list, add_messages]

class RobotPlannerGraph:
    def __init__(self, robot_tools, thread_id="1"):
        self.tools = robot_tools
        self.thread_id = thread_id
        self._has_vision = False

        prompt_name ="orange_prompt_short"
        init_prompt = ""
        with open(f'prompts/{prompt_name}.txt', 'r') as f:
            init_prompt = f.read()
        
        self.init_messages = [SystemMessage(content=init_prompt),]
    
        self.config = {"configurable": {"thread_id": thread_id}}

        # Define the tools list
        tools = [
            self.tools.get_pose_tool(),
            self.tools.get_move_through_locations_tool()
        ]

        # Check if vision LLM server is running
        text_tools_llm_url = "http://192.168.55.1:11434"
        local_text_tools_llm_avail = is_server_running(text_tools_llm_url)
        self.all_in_one_llm = None

        if not local_text_tools_llm_avail:
            # Initialize the chat model for planning
            self.all_in_one_llm = ChatOpenAI(
                model="gpt-4.1",
                max_tokens=200,
                max_retries=2,
            ).bind_tools(tools)
            self._has_vision = True      
            print("Local llm_text_tools offline, llm initialized from OpenAI")
        else:
            # Initialize both LLMs
            self.llm_text_tools = ChatOpenAI(
                base_url="http://192.168.55.1:11434/v1/",  # tools LM endpoint
                model="qwen2.5:7b-instruct",
                api_key="YOUR_API_KEY_HERE",
                temperature=0.0,
                max_tokens=200,
                max_retries=2,
            ).bind_tools(tools)
            print("llm_text_tools initialized from onboard Jetson")

            # Check if vision LLM server is running
            vision_llm_url = "http://192.168.1.41:11434"
            vision_llm_available = is_server_running(vision_llm_url)

            if vision_llm_available:
                self.llm_vision = ChatOpenAI(
                    base_url="http://192.168.1.41:11434/v1/",  # vision LM endpoint
                    model="qwen2.5vl:latest",
                    api_key="YOUR_API_KEY_HERE",
                    temperature=0.1,
                    max_tokens=200,
                    max_retries=2
                )
                self._has_vision = True
                print("VLM initialized from home 3090 server.")
            else:
                self.llm_vision = None
                print("Vision LLM server not available. Only using text/tools LLM.")

        # Define the graph
        self.builder = StateGraph(State)
        
        self.image_triggers = ["in the picture", "this image", "that photo", "what is shown"]

        # MultiLLMPlanner node
        def multi_llm_planner(state: State):
            last_msg = state["messages"][-1]
            if self.all_in_one_llm is not None:
                #print("last message:", last_msg)
                return {"messages": [self.all_in_one_llm.invoke(state["messages"])]}
            # If vision LLM is available and last Human message contains an image, use vision LLM
            if self.llm_vision:
                if isinstance(last_msg, HumanMessage) and isinstance(last_msg.content, list):
                    for part in last_msg.content:
                        # check if part.get("text") contains any of the image triggers
                        if isinstance(part, dict):
                            if part.get("type") == "text":
                                text = part.get("text", "").lower()
                                if any(trigger in text for trigger in self.image_triggers):
                                    use_vision = True
                                    break
                            if (part.get("type") == "image_url"):
                                use_vision = True
                                break
                    if use_vision:
                        print("Using vision LLM")
                        return {"messages": [self.llm_vision.invoke(state["messages"])]}
            # Otherwise, use text/tools LLM
            print("Using text/tools LLM")
            #print("last message:", state["messages"][-1])
            return {"messages": [self.llm_text_tools.invoke(state["messages"])]}

        self.builder.add_node("planner", multi_llm_planner)
        tool_node = ToolNode(tools=tools)
        self.builder.add_node("tools", tool_node)
        self.builder.add_conditional_edges("planner", tools_condition)
        
        # Edges:
        # Any time a tool is called, we return to the planner to decide which LLM to use next
        self.builder.add_edge("tools", "planner")
        self.builder.add_edge(START, "planner")

        self.memory = MemorySaver()
        # Compile the graph
        self.graph = self.builder.compile(checkpointer=self.memory)

        # Add the initial messages to the graph
        value = self.graph.invoke(input={"messages": self.init_messages}, config=self.config)
        self._greeting = value["messages"][-1].content

    @property
    def has_vision(self):
        return self._has_vision
        
    def reset_memory(self):
        """Reset the memory of the agent."""
        self.memory.delete_thread(self.thread_id)
        # Re-add the initial messages to the graph
        self.graph.invoke(input={"messages": self.init_messages}, config=self.config)

    def get_graph(self):        return self.graph

    def print_message_history(self):
        """Pretty print the messages stored in LangGraph InMemorySaver."""
        state = self.memory.get(self.config)
        messages = state.get("channel_values", {}).get("messages", [])

        print("=" * 40)
        print("Conversation History:")

        for msg in messages[2:]:  # Skip the first two messages (system messages)
            if hasattr(msg, "pretty_print") and callable(msg.pretty_print):
                msg.pretty_print()
            else:
                print(msg)  # fallback for non-message objects

        print("=" * 40)

    def add_to_memory(self, user_input: str="", robot_response: str=""):
        """Log a direct command and its handled response into LangGraph memory."""
        # Get current state
        state = self.memory.get(self.config)
        channel = state.get("channel_values", {}).get("messages", [])
        
        # Append human message
        if user_input !="":
            channel.append(HumanMessage(content=user_input))
        
        # Append AI message
        if robot_response != "":
            channel.append(AIMessage(content=robot_response, 
                                     additional_kwargs={"handled_directly": True}))
        
        # Update memory
        self.graph.update_state(self.config, {"messages": channel})

    def send_input(self, user_input: str, image=None):
        if image is None:
            message = HumanMessage(content=user_input)
        else:
            base64_image = base64.b64encode(image).decode('utf-8')
            message = HumanMessage(content=[
                    {"type": "text", "text": user_input},
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/jpeg;base64,{base64_image}",
                        },
                    },
                ])

        response = ""
        tool_call_log = ""
        for event in self.get_graph().stream({"messages": [message]}, config=self.config):
            for value in event.values():
                for msg in value["messages"]:
                    msg.pretty_print()

                last = value["messages"][-1]
                if isinstance(last, AIMessage):
                    response += last.content + "\n"
                    tool_call_log += last.pretty_repr() + "\n"
        return response, tool_call_log

    @property
    def greeting(self):
        """Return the greeting message."""
        return self._greeting

    @staticmethod
    def create_langgraph(move_through_locations=None, thread_id="1", sim=False):
        import langgraph_robot_tools as lrt
        robot_tools = lrt.RobotTools(move_through_locations_real=move_through_locations, sim=sim)
        return RobotPlannerGraph(robot_tools, thread_id=thread_id)

# Example run
if __name__ == "__main__":
    import tts.sapi
    import tts.flags

    def initialize_speech():
        global _voice
        _voice = tts.sapi.Sapi()
        _voice.set_voice("Mark") # David, Mark, Eva, or Zira. 
        _voice.voice.Volume = 100
        _voice.voice.SynchronousSpeakTimeout = 1 # timeout in milliseconds

    def speak(phrase, flag=tts.flags.SpeechVoiceSpeakFlags.Default.value, add_to_memory=True):
        global _last_phrase, _voice

        try:
            #print("speaking: ", phrase)
            _voice.say(phrase, flag)
            # add robot response to memory
        except Exception:
            print("Speak has timed out.")
            pass


    initialize_speech()
    import langgraph_robot_tools as lrt
    robotPlannerGraph = RobotPlannerGraph.create_langgraph(sim=True)
    print(robotPlannerGraph.greeting)
    image = None
    #robotPlannerGraph.add_to_memory("go to the kitchen", "Ok, I'm going to the kitchen.")
    #robotPlannerGraph.add_to_memory(robot_response="I've arrived.")
    
    while True:
        user_input = input("User: ")
        if user_input.lower() in ["quit", "exit", "q"]:
            print("Goodbye!")
            break
        elif ".png" in user_input or ".jpg" in user_input:
            image = user_input
            continue
        elif user_input.lower() == "reset memory":
            robotPlannerGraph.reset_memory()
            print("Memory reset.")
            continue
        elif user_input.lower() == "log":
            robotPlannerGraph.print_message_history()
            continue

        if image is not None:
            with open(image, "rb") as img_file:
                image = img_file.read()
                #resize the image's larger dimension to 512 pixels while maintaining aspect ratio
                from PIL import Image
                from io import BytesIO
                img = Image.open(BytesIO(image))
                max_size = 512
                if img.width > img.height:
                    new_width = max_size
                    new_height = int((max_size / img.width) * img.height)
                else:
                    new_height = max_size
                    new_width = int((max_size / img.height) * img.width)
                img = img.resize((new_width, new_height)) 
                img_byte_arr = BytesIO()
                img.save(img_byte_arr, format='JPEG')
                image = img_byte_arr.getvalue()
        else:
            image = None
        response, tool_log = robotPlannerGraph.send_input(user_input, image)
        #speak(response)
        #print(f"Orange: {response}")
        #if tool_log:
        #    print("Tool Call Log:")
        #    print(tool_log)
        image = None