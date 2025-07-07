from langgraph.graph import START, StateGraph
from langgraph.graph.state import CompiledStateGraph
from langgraph.prebuilt import ToolNode, tools_condition
from langchain_core.messages import HumanMessage, SystemMessage
from typing_extensions import TypedDict
from langchain.chat_models import init_chat_model
from langgraph.graph.message import add_messages
from typing import Annotated

LLM_PLANNER_SYSTEM_PROMPT = """
Yaw of 0: robot faces along the +X axis.
Yaw of 90: robot faces along the +Y axis.
Yaw of -90: robot faces along -Y axis.
A left turn means add 90 to yaw.
A right turn means subtract 90 from yaw. 
If yaw ever > 180, then yaw = yaw - 360.
Assume move commands are given from the POV of the robot."
"""

XLLM_PLANNER_SYSTEM_PROMPT = """
You are a planning assistant for a mobile robot that moves on a 2D plane. Your task is to break down the user's high-level commands into structured robot actions.

For movement commands, you must:
1. Obtain the robot's current pose using the "get_pose" tool.
2. Generate a list of (x, y) waypoints that describe the requested path relative to that pose.
3. Specify a final_yaw (in degrees) for the robot's desired orientation after completing the motion.

All numeric values should be in **meters** for x and y, and **degrees** for yaw.

Output your plan using the "move_through_locations" tool by providing:
- locations: list of {"x": number, "y": number}
- final_yaw: number (degrees)

If the user command cannot be converted into such a motion plan, respond with an appropriate clarification request.

Example commands and breakdowns:

User: "Drive in a square path with 1 meter sides starting in the center of the square."
→ get_pose()
→ move_through_locations(
    locations=[
        {"x": <computed>, "y": <computed>},
        {"x": <computed>, "y": <computed>},
        {"x": <computed>, "y": <computed>},
        {"x": <computed>, "y": <computed>}
    ],
    final_yaw=<computed>
)

User: "Go forward 3 meters, then turn left 90 degrees."
→ get_pose()
→ move_through_locations(
    locations=[
        {"x": <computed>, "y": <computed>}
    ],
    final_yaw=<computed>
)

"""

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
    def __init__(self, robot_tools):
        self.tools = robot_tools

        # Initialize the chat model for planning
        llm = init_chat_model("openai:gpt-4o")

        # Define the graph
        self.builder = StateGraph(State)

        tools = [
            self.tools.get_pose_tool(),
            self.tools.get_move_through_locations_tool()
        ]

        # Modification: tell the LLM which tools it can call
        llm_with_tools = llm.bind_tools(tools)

        def chatbot(state: State):
            return {"messages": [llm_with_tools.invoke(state["messages"])]}
        
        # The first argument is the unique node name
        # The second argument is the function or object that will be called whenever
        # the node is used.
        self.builder.add_node("chatbot", chatbot)

        # 2. Tool node for executing tools (get_pose, move_through_locations)
        tool_node = ToolNode(tools=tools)
        self.builder.add_node("tools", tool_node)

        self.builder.add_conditional_edges(
            "chatbot",
            tools_condition,
        )

        # Edges:
        # Any time a tool is called, we return to the chatbot to decide the next step
        self.builder.add_edge("tools", "chatbot")
        self.builder.add_edge(START, "chatbot")

        # Compile the graph
        self.graph = self.builder.compile()
    
    def get_graph(self):
        return self.graph
    
    def send_input(self, user_input: str):
        for event in self.get_graph().stream({"messages": [{"role": "user", "content": user_input}]}):
            for value in event.values():
                print("Assistant:", value["messages"][-1].content)

    @staticmethod
    def create_langgraph(move_through_locations=None, sim=False):
        import langgraph_robot_tools as lrt
        robot_tools = lrt.RobotTools(move_through_locations_real=move_through_locations, sim=sim)
        return RobotPlannerGraph(robot_tools)

# Example run
if __name__ == "__main__":
    import langgraph_robot_tools as lrt
    robotPlannerGraph = RobotPlannerGraph.create_langgraph(sim=True)
    
    def stream_graph_updates(user_input: str):
        for event in robotPlannerGraph.get_graph().stream({"messages": [{"role": "system", "content": LLM_PLANNER_SYSTEM_PROMPT},
                                                                        {"role": "user", "content": user_input}]}):
            for value in event.values():
                print("Assistant:", value["messages"][-1].content)

    while True:
        user_input = input("User: ")
        if user_input.lower() in ["quit", "exit", "q"]:
            print("Goodbye!")
            break

        if user_input == "":
            user_input = "Drive the robot as if drawing a closed square with 1 meter sides starting in the center of the square."
        stream_graph_updates(user_input)

    # result = robotPlannerGraph.graph.invoke({
    #     "messages": [
    #         SystemMessage(content=LLM_PLANNER_SYSTEM_PROMPT),
    #         HumanMessage(content=input_text)
    #     ]
    # })
    # print("Result:")
    # print(result)

