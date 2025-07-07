from typing import Dict, List, Callable
from my_sdp_client import MyClient
import sdp_comm
from pydantic import BaseModel, Field
from langchain.tools import StructuredTool

class SDPSimClient:
    def pose(self):
        # Simulated pose method
        class Pose:
            def __init__(self, x, y, yaw):
                self.x = x
                self.y = y
                self.yaw = yaw
        print("SIM POSE call")
        # Return a simulated pose object
        return Pose(x=0.0, y=0.0, yaw=90)
    
    def disconnect(self):
        # Simulated disconnect method
        pass

    def shutdown_server32(self, kill_timeout=1):
        # Simulated shutdown method
        pass
    
class MoveLocationsInput(BaseModel):
    locations: List = Field(
        ...,
        description="List of locations to visit, in order. Each location is a dictionary with 'x' and 'y' keys, e.g., {'x': 1.0, 'y': 2.0}."
    )
    final_yaw: float = Field(
        ...,
        description="Desired orientation (yaw) after reaching the final location, in degrees, [-180, 180]"
    )

def move_through_locations_sim(sdp, locations: List[Dict[str, float]], final_yaw: float):
    """Simulated function to move through locations."""
    for idx, loc in enumerate(locations):
        print(f"Simulated moving to location {idx + 1}: x={loc['x']}, y={loc['y']}")
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
        """Get the robot's current pose (x in meters, y in meters, yaw in degrees)."""
        pose = self.sdp.pose()
        print("returning pose")
        return {"x": pose.x, "y": pose.y, "yaw": pose.yaw}
        
    def move_through_locations(self,locations: List[dict], final_yaw: float) -> None:
        print("trying to move through locations")
        """Move the robot along a list of (x in meters, y in meters) locations ending at the specified orientation (yaw in degrees, [-180, 180])."""
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
            description="Move the robot along a list of locations each a dictionary with 'x' and 'y' keys and values in meters, ending at the specified orientation (yaw in degrees)."
        )
    
    def get_pose_tool(self):
        return StructuredTool.from_function(
            func=self.get_pose,
            name="get_pose",
            description="Get the robot's current pose (x in meters, y in meters, yaw in degrees [-180, 180])."
        )