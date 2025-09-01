from typing import Dict, List, Callable
from typing_extensions import Self
from my_sdp_client import MyClient
import sdp_comm
from pydantic import BaseModel, Field, model_validator
from langchain.tools import StructuredTool

class Pose:
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw

class SDPSimClient:
    def __init__(self):
        self.pose = Pose(x=1.0, y=2.0, yaw=90.0)

    def get_pose(self):
        print("SIM POSE call")
        return self.pose
    
    def set_pose(self, x, y, yaw):
        # Simulated set_pose method
        self.pose = Pose(x=x, y=y, yaw=yaw)
        
    def disconnect(self):
        # Simulated disconnect method
        pass

    def shutdown_server32(self, kill_timeout=1):
        # Simulated shutdown method
        pass
    
class MoveLocationsInput(BaseModel):
    locations: List[Dict] = Field(
        ...,
        description="List of locations to visit, in order. Each location is a dictionary with 'x' and 'y' keys, e.g., {'x': 1.0, 'y': 2.0}."
    )
    final_yaw: float = Field(
        ...,
        description="Desired orientation (yaw) after reaching the final location, in degrees, [-180, 180]"
    )

    @model_validator(mode="before")
    def parse_locations(cls, values):
        locs = values.get('locations')
        if isinstance(locs, str):
            import ast
            values['locations'] = ast.literal_eval(locs)
        return values

def move_through_locations_sim(sdp, locations: List[Dict[str, float]], final_yaw: float):
    """Simulated function to move through locations."""
    for idx, loc in enumerate(locations):
        print(f"Simulated moving to location {idx + 1}: x={loc['x']}, y={loc['y']}")
        sdp.set_pose(loc['x'], loc['y'], final_yaw)
    print(f"Final desired yaw: {final_yaw}")

class RobotTools:
    def __init__(
        self, 
        move_through_locations_real: Callable[[object, List[Dict[str, float]], float], None] = move_through_locations_sim, 
        sim: bool = False
    ):
        if move_through_locations_real is None:
            move_through_locations_real = move_through_locations_sim
        self.move_through_locations_real = move_through_locations_real
        self.sim = sim
        if not self.sim:
            self.sdp : MyClient = MyClient()
            sdp_comm.connectToSdp(self.sdp)
        else:
            self.sdp = SDPSimClient()

    def __del__(self):
        self.sdp.disconnect()
        self.sdp.shutdown_server32(kill_timeout=1)
        self.sdp = None

    def get_pose(self) -> Dict[str, float]:
        """Returns the robot's current pose (x in meters, y in meters, yaw in degrees)."""
        print("get_pose() called")
        pose = self.sdp.get_pose()
        return {"x": pose.x, "y": pose.y, "yaw": pose.yaw}
        
    def move_through_locations(self,locations: List[dict], final_yaw: float) -> None:
        print("move_through_locations() called")
        """Moves the robot starting from current pose through each waypoint in list. Each waypoint is a dictionary with 'x' and 'y' keys and values in meters. Then rotate to the final yaw. (yaw in degrees)."""
        if not locations:
            raise ValueError("Locations list cannot be empty.")
        if not all('x' in loc and 'y' in loc for loc in locations):
            raise ValueError("Each location must have 'x' and 'y' keys.")
        if not isinstance(final_yaw, (int, float)):
            raise ValueError("Final yaw must be a number.")
        
        self.move_through_locations_real(self.sdp, locations, final_yaw)

    def get_move_through_locations_tool(self):
        return StructuredTool.from_function(
            func=self.move_through_locations,
            args_schema=MoveLocationsInput,
            name="move_through_locations",
            description="This func moves the robot from current pose to each absolute location in list. Each location is a dictionary with 'x' and 'y' keys and values in meters. Then it rotates robot to the final yaw. (yaw in degrees)."
        )
    
    def get_pose_tool(self):
        return StructuredTool.from_function(
            func=self.get_pose,
            name="get_pose",
            description="This func returns the robot's current pose as dict with x in meters, y in meters, yaw in degrees [-180, 180]."
        )