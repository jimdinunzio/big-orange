from langgraph.graph import START, StateGraph
from langgraph.graph.state import CompiledStateGraph
from langgraph.checkpoint.memory import MemorySaver
from langchain_core.messages import HumanMessage, SystemMessage, AIMessage
from typing_extensions import TypedDict
from langgraph.graph.message import add_messages
from langchain_core.messages.tool import ToolMessage

from langchain_openai import ChatOpenAI
from typing import Annotated
from threading import Thread
import base64
import requests
import ast
import tts.flags

def is_server_running(url):
#    return False
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
    def __init__(self, robot_tools, thread_id="1", speak_function=None, 
                 wait_until_speech_done=None):
        self.tools = robot_tools
        self.thread_id = thread_id
        self._has_vision = False
        self.speak_function = speak_function
        self.wait_until_speech_done = wait_until_speech_done
        self._cancel = False
        self._suppress_speech = False
        self._is_processing = False

        prompt_name ="orange_prompt_short"
        init_prompt = ""
        with open(f'prompts/{prompt_name}.txt', 'r') as f:
            init_prompt = f.read()
        
        self.init_messages = [SystemMessage(content=init_prompt),]
    
        self.config = {"configurable": {"thread_id": thread_id}, "recursion_limit": 100}
        self._stream_thread = None

        # Get all available tools from RobotTools
        tools = self.tools.get_all_tools()

        # Check if vision LLM server is running
        text_tools_llm_url = "http://192.168.55.1:11434"
        local_text_tools_llm_avail = False # is_server_running(text_tools_llm_url)
        self.all_in_one_llm = None

        if not local_text_tools_llm_avail:
            # Initialize the chat model for planning with bind_tools for standard dispatch
            self.all_in_one_llm = ChatOpenAI(
                model="gpt-4.1",
                max_tokens=200,
                max_retries=2,
            ).bind_tools(tools)
            self._has_vision = True      
            print("Local llm_text_tools offline, llm initialized from OpenAI")
        else:
            # Initialize both LLMs with bind_tools for standard dispatch
            self.llm_text_tools = ChatOpenAI(
                base_url="http://192.168.55.1:11434/v1/",  # tools LM endpoint
                model="qwen2.5:7b-instruct",
                #model="mistral:7b-instruct",
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
        
        self.image_triggers = ["in that picture", "in that image", "in that photo", "what is shown"]

        # MultiLLMPlanner node
        def multi_llm_planner(state: State):
            print("*** Multi LLM Planner ***")
            last_msg = state["messages"][-1]
            
            def validate_and_fix_response(response):
                """Validate and fix tool call formatting if needed."""
                if isinstance(response, AIMessage):
                    content = response.content
                    if isinstance(content, str):
                        if "<tool_call>" in content:
                            # Extract tool name and arguments
                            import re
                            tool_match = re.search(r'<tool_call>(.*?)</tool_call>', content, re.DOTALL)
                            if tool_match:
                                # Convert to proper LangGraph tool call format
                                response.content = content.replace(tool_match.group(0), 
                                    f"I need to use {tool_match.group(1)}")
                    elif isinstance(content, list):
                        # Handle list content type (e.g., for multi-modal responses)
                        new_content = []
                        for item in content:
                            if isinstance(item, dict):
                                new_content.append(item)  # Keep dict items as is
                            elif isinstance(item, str) and "<tool_call>" in item:
                                import re
                                tool_match = re.search(r'<tool_call>(.*?)</tool_call>', item, re.DOTALL)
                                if tool_match:
                                    new_content.append(f"I need to use {tool_match.group(1)}")
                                else:
                                    new_content.append(item)
                            else:
                                new_content.append(item)
                        response.content = new_content
                return response
            
            if self.all_in_one_llm is not None:
                return {"messages": [self.all_in_one_llm.invoke(state["messages"])]}
            
            use_vision = False
            if self.llm_vision:
                # Check for image triggers in HumanMessage
                if isinstance(last_msg, HumanMessage):
                    if isinstance(last_msg.content, list):
                        for part in last_msg.content:
                            if isinstance(part, dict):
                                if part.get("type") == "text":
                                    text = part.get("text", "").lower()
                                    if any(trigger in text for trigger in self.image_triggers):
                                        use_vision = True
                                        break
                    elif isinstance(last_msg.content, str):
                        text = last_msg.content.lower()
                        if any(trigger in text for trigger in self.image_triggers):
                            use_vision = True
                # Check for image_url in AIMessage (tool output)
                elif isinstance(last_msg, ToolMessage):
                    content = last_msg.content
                    if isinstance(content, str):
                        try:
                            content_dict = ast.literal_eval(content)
                        except Exception:
                            content_dict = None
                    else:
                        content_dict = content
                    if isinstance(content_dict, dict) and content_dict.get("type") == "image_url":
                        #state["messages"].append(HumanMessage(content=[content_dict]))
                        state["messages"][-1]=HumanMessage(content=[content_dict])
                        use_vision = True
                    elif isinstance(content_dict, list):
                        for part in content_dict:
                            if isinstance(part, dict) and part.get("type") == "image_url":
                                #state["messages"].append(HumanMessage(content=[part]))
                                state["messages"][-1]=HumanMessage(content=[part])
                                use_vision = True
                                break        
                if use_vision:
                    print("Using vision LLM")
                    return {"messages": [self.llm_vision.invoke(state["messages"])]}
            
            # Default to text/tools LLM
            print("Using text/tools LLM")
            response = self.llm_text_tools.invoke(state["messages"])
            return {"messages": [validate_and_fix_response(response)]}

        # Speech node to announce the planner's intent before taking action
        def speech_node(state: State):
            print("*** Speech Node ***")
            last_msg = state["messages"][-1]
            if isinstance(last_msg, AIMessage) and last_msg.content:
                self._speak(
                    last_msg.content,
                    add_to_memory=False,
                )
            return {"messages": []}

        # Custom tool node that executes tools SEQUENTIALLY to avoid OpenAI API errors
        def custom_tool_node(state: State):
            print("*** Custom Tool Node ***")
            """Custom tool node that executes tools sequentially."""
            last_msg = state["messages"][-1]

            # TODO: Reintroduce mid-execution streaming writers once LangChain Runtime pattern is finalized.

            # Check if last message is AIMessage with tool calls
            if isinstance(last_msg, AIMessage) and hasattr(last_msg, 'tool_calls') and last_msg.tool_calls:
                
                # Process ALL tool calls in this message sequentially
                # We need to return responses for ALL tool_call_ids to satisfy OpenAI API requirements
                tool_results = []
                
                for tool_call in last_msg.tool_calls:
                    tool_name = tool_call['name']
                    tool_call_id = tool_call['id']
                    tool_args = tool_call.get('args', {})
                    
                    print(f"Executing tool: {tool_name} with args: {tool_args}, tool_call_id: {tool_call_id}")
                    
                    # Get the tool function from RobotTools
                    tool_func = getattr(self.tools, tool_name, None)
                    if not tool_func:
                        result = f"Unknown tool: {tool_name}"
                        tool_results.append(ToolMessage(content=result, tool_call_id=tool_call_id))
                    else:
                        try:
                            # Execute tool
                            result = tool_func(**tool_args)
                            tool_results.append(ToolMessage(content=str(result), tool_call_id=tool_call_id))
                            
                        except Exception as e:
                            tool_results.append(ToolMessage(content=f"Error executing {tool_name}: {str(e)}", tool_call_id=tool_call_id))
                
                # Return ALL tool results
                return {"messages": tool_results}
                
            return {"messages": []}

        # Reporter node to speak the LLM's final summary/response
        def reporter_node(state: State):
            print("*** Reporter Node ***")
            last_msg = state["messages"][-1]
            if isinstance(last_msg, AIMessage) and last_msg.content:
                self._speak(
                    last_msg.content,
                    add_to_memory=False,
                )
            return {"messages": []}

        def route_from_planner(state: State):
            print("*** Routing from planner ***")
            last_msg = state["messages"][-1]
            if isinstance(last_msg, AIMessage) and getattr(last_msg, "tool_calls", None):
                return "speech"
            return "reporter"
        
        def route_after_tools(state: State):
            print("*** Routing after tools ***")
            # Look back through messages for last AIMessage with tool_calls
            for msg in reversed(state["messages"]):
                if isinstance(msg, AIMessage) and getattr(msg, "tool_calls", None):
                    return "planner"  # Still have steps to execute
            return "reporter"  # No pending tool calls → finish


        self.builder.add_node("planner", multi_llm_planner)
        self.builder.add_node("speech", speech_node)
        self.builder.add_node("tools", custom_tool_node)
        self.builder.add_node("reporter", reporter_node)
        self.builder.add_node("route_from_planner", route_from_planner)

        self.builder.add_conditional_edges("planner", route_from_planner)
        self.builder.add_edge("speech", "tools")
        self.builder.add_conditional_edges("tools", route_after_tools)
        self.builder.add_edge(START, "planner")

        self.memory = MemorySaver()
        # Compile the graph
        self.graph = self.builder.compile(checkpointer=self.memory)

        #displayGraph(self.graph)
        
        # Add the initial messages to the graph
        
        value = self.graph.invoke(input={"messages": self.init_messages}, config=self.config)
        self._greeting = value["messages"][-1].content

    def _speak(self, content, add_to_memory=False):
        """Central speech entry point; skips if stream is cancelled or speech suppressed."""
        if self._cancel or self._suppress_speech:
            return
        if not self.speak_function or not content:
            return
        self.speak_function(content, flag=tts.flags.SpeechVoiceSpeakFlags.FlagsAsync.value,
                            add_to_memory=add_to_memory)
        # makes the speak call synchronous and interruptable because it is async under the hood
        if self.wait_until_speech_done:
            self.wait_until_speech_done()

    @property
    def has_vision(self):
        return self._has_vision

    @property
    def is_processing(self):
        """Return True if the agent is currently processing a request."""
        return self._is_processing

    def cancel_stream(self):
        # Request cancellation of the current run.
        # Actual stopping is coordinated inside _run_stream so that
        # we don't break the OpenAI tools protocol.
        self._cancel = True

    def shutdown(self):
        """Cancel any in-flight run, wait for the stream thread to finish, then
        release the robot tools' SDP client. Called by main during robot shutdown."""
        self.cancel_stream()
        t = self._stream_thread
        if t is not None and t.is_alive():
            t.join(timeout=5)
        self.tools.shutdown()

    def reset_memory(self):
        """Reset the memory of the agent."""
        self.memory.delete_thread(self.thread_id)
        
        # Suppress speech during re-initialization
        self._suppress_speech = True
        try:
            # Re-add the initial messages to the graph
            self.graph.invoke(input={"messages": self.init_messages}, config=self.config)
        finally:
            self._suppress_speech = False
        
        # Announce the reset (after suppression is cleared)
        if self.speak_function:
            self.speak_function("my memory was reset.", add_to_memory=False)

    def get_graph(self):
        return self.graph

    def print_message_history(self):
        """Pretty print the messages stored in LangGraph InMemorySaver, truncating image_url 'url' fields."""
        state = self.memory.get(self.config)
        messages = state.get("channel_values", {}).get("messages", [])

        print("=" * 40)
        print("Conversation History:")

        def truncate_image_url(obj):
            """Create a display copy of obj with truncated image URLs."""
            if isinstance(obj, dict):
                if obj.get("type") == "image_url":
                    # Copy and truncate the url
                    img_url = obj["image_url"].get("url", "")
                    short_url = img_url[:30] + "..." if len(img_url) > 33 else img_url
                    new_obj = obj.copy()
                    new_obj["image_url"] = obj["image_url"].copy()
                    new_obj["image_url"]["url"] = short_url
                    return new_obj
                # Deep copy for nested dicts
                return {k: truncate_image_url(v) for k, v in obj.items()}
            elif isinstance(obj, list):
                return [truncate_image_url(item) for item in obj]
            return obj

        def get_display_content(msg):
            """Create a display copy of the message content with truncated URLs."""
            if not hasattr(msg, "content"):
                return msg
            
            content = msg.content
            if isinstance(content, (dict, list)):
                return truncate_image_url(content)
            elif isinstance(content, str):
                try:
                    content_dict = ast.literal_eval(content)
                    if isinstance(content_dict, (dict, list)):
                        return truncate_image_url(content_dict)
                except:
                    pass
            return content

        for msg in messages[2:]:  # Skip the first two messages (system messages)
            if hasattr(msg, "pretty_print") and callable(msg.pretty_print):
                # Create a display copy of the message
                import copy
                display_msg = copy.copy(msg)
                if hasattr(display_msg, "content"):
                    display_msg.content = get_display_content(msg)
                display_msg.pretty_print()
            else:
                if isinstance(msg, dict):
                    print(truncate_image_url(msg))
                else:
                    # For non-dict messages, show the message with truncated content
                    display_content = get_display_content(msg)
                    print(f"{msg.__class__.__name__}: {display_content}")

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
        self._cancel = False
        self._is_processing = True
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

        def _run_stream(msg: HumanMessage):
            try:
                from langchain_core.messages import AIMessage  # local import to avoid circulars

                for event in self.graph.stream({"messages": [msg]}, config=self.config):
                    if not event:
                        continue

                    # Single node per event; get its value
                    node_name, value = next(iter(event.items()))

                    # If cancel is requested, purge history and break immediately
                    if self._cancel:
                        print("Stream cancelled - resetting conversation history.")
                        self._cancel = False
                        # Reset memory to clear any incomplete tool_calls
                        self.reset_memory()
                        break

                    # writer-based progress happens inside tools; speech happens in nodes
                    if not value or "messages" not in value:
                        continue

                    value_messages = value["messages"]
                    for m in value_messages:
                        m.pretty_print()
            except Exception as e:
                print(f"Error in background stream: {e}")
            finally:
                self._is_processing = False

        # Launch background streaming so callers (e.g., listen loop) never block
        self._stream_thread = Thread(target=_run_stream, args=(message,), daemon=True)
        self._stream_thread.start()

        # For now, return empty strings immediately; speech/progress happen asynchronourey
        return "", ""

    @property
    def greeting(self):
        """Return the greeting message."""
        return self._greeting

    @staticmethod
    def create_langgraph(langgraph_tool_funcs=None, thread_id="1", sim=False, speak_function=None,
                         wait_until_speech_done=None):
        import langgraph_robot_tools as lrt
        robot_tools = lrt.RobotTools(langgraph_tool_funcs, sim=sim)
        return RobotPlannerGraph(robot_tools, thread_id=thread_id, speak_function=speak_function,
                                 wait_until_speech_done=wait_until_speech_done)

# Example run
if __name__ == "__main__":
    import tts.sapi
    import tts.flags
    import sys

    def initialize_speech():
        global _voice
        _voice = tts.sapi.Sapi()
        _voice.set_voice("Mark") # David, Mark, Eva, or Zira. 
        _voice.voice.Volume = 100
        _voice.voice.SynchronousSpeakTimeout = 1 # timeout in milliseconds

    def speak(phrase, flag=tts.flags.SpeechVoiceSpeakFlags.Default.value, add_to_memory=True):
        global _last_phrase, _voice

        try:
            #print("SPEAKING: ", phrase)
            _voice.say(phrase, flag)
            # add robot response to memory
        except Exception:
            print("Speak has timed out.")
            pass


    initialize_speech()
    import langgraph_robot_tools as lrt
    
    # For simulation mode, langgraph_tool_funcs is not needed

    langgraph_tool_funcs = {
    # Location & Navigation
        "go_to_location": (None),  # Long-running, takes callback
        "move_in_dir_dist": (None),  # Long-running, takes callback
        "move_through_locations": (None)  # Long-running, takes callback
    }

    robotPlannerGraph = RobotPlannerGraph.create_langgraph(langgraph_tool_funcs, sim=True, speak_function=speak)
    print(robotPlannerGraph.greeting)
    image = None
    #robotPlannerGraph.add_to_memory("go to the kitchen", "Ok, I'm going to the kitchen.")
    #robotPlannerGraph.add_to_memory(robot_response="I've arrived.")
    
    while True:
        user_input = input("User: ")
        if not sys.stdin.isatty():
            print(user_input)
        if user_input.lower() in ["quit", "exit", "q"]:
            print("Goodbye!")
            break
        elif user_input.lower() == "cc":
            robotPlannerGraph.cancel_stream()
            print("Cancelling current operation...")
            continue
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
