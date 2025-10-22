from typing import Dict, List, Callable, Tuple
from typing_extensions import Self
from my_sdp_client import MyClient
from my_sdp_server import ActionStatus
import sdp_comm
from pydantic import BaseModel, Field, model_validator
from langchain.tools import StructuredTool
import os
import base64
from PIL import Image
from io import BytesIO
from move_through_locations_alert import post_alert

class Pose:
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw

class SDPSimClient:
    def __init__(self):
        self._pose = Pose(x=0.0, y=0.0, yaw=0.0)  # Initial pose
        self._progress = 0

    def pose(self):
        # Return a copy of the pose to avoid external modification
        return Pose(self._pose.x, self._pose.y, self._pose.yaw)
    
    def set_pose(self, x, y, yaw):
        # Simulated set_pose method
        self._pose.x=x
        self._pose.y=y
        self._pose.yaw=yaw
        
    def disconnect(self):
        # Simulated disconnect method
        pass

    def shutdown_server32(self, kill_timeout=1):
        # Simulated shutdown method
        pass

    def getMoveActionStatus(self):
        # Simulated status
        print("SIM getMoveActionStatus call")
        return ActionStatus.Finished
    
    def getMoveActionError(self):
        return "No error in simulation."
    
class MoveLocationsInput(BaseModel):
    locations: List[Dict[str, float]] = Field(
        ...,
        description="[REQUIRED] List of movement deltas. Example: [{'dx': 1.0, 'dy': 0.0}]. Must be non-empty list of {dx,dy} dictionaries."
    )
    final_yaw: float = Field(
        ...,
        description="[REQUIRED] Final orientation in degrees (yaw), must be between -180 and 180.",
        ge=-180,
        le=180
    )

    @model_validator(mode="before")
    def parse_locations(cls, values):
        locs = values.get('locations')
        if not locs:
            raise ValueError("locations list cannot be empty")
        if isinstance(locs, str):
            import ast
            values['locations'] = ast.literal_eval(locs)
        # Ensure locations are dictionaries
        if isinstance(locs, (list, tuple)):
            values['locations'] = [dict(loc) if not isinstance(loc, dict) else loc for loc in locs]
        return values

def move_through_locations_sim(sdp, locations: List[Dict[str, float]], final_yaw: float) -> str:
    """Simulated function to move through locations."""
    scale = 100  # Scale factor to convert meters to pixels for plotting
    pose = sdp.pose()
    abs_points = [(pose.x * scale, pose.y * scale)]
    points = []

    for idx, loc in enumerate(locations):
        pose.x = pose.x + loc['dx']
        pose.y = pose.y + loc['dy']
        abs_points.append((pose.x * scale, pose.y * scale))
        points.append((pose.x, pose.y))
    approved = post_alert(abs_points)
    if not approved:
        return "Movement cancelled by user because it does not match the intended shape."
    
    for idx, loc in enumerate(points):
        print(f"Simulated moving to location {idx + 1}: x={loc[0]}, y={loc[1]}")
        sdp.set_pose(loc[0], loc[1], final_yaw)
    print(f"Final desired yaw: {final_yaw}")
    
    return "Movement started."

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
            self.sdp : SDPSimClient = SDPSimClient()

    def __del__(self):
        self.sdp.disconnect()
        self.sdp.shutdown_server32(kill_timeout=1)
        self.sdp = None

    def getMoveActionStatus(self):
        status = self.sdp.getMoveActionStatus()
        if status == ActionStatus.Finished:
            return "move action finished."
        elif status == ActionStatus.Error:
            return "error:" + self.sdp.getMoveActionError()
        elif status == ActionStatus.Stopped:
            return "action has been cancelled due to incorrect shape."
        return "move action running."

    def get_pose(self) -> Dict[str, float]:
        """Returns the robot's current pose (position) as a dict (x in meters, y in meters, yaw in degrees)."""
        pose = self.sdp.pose()
        print("get_pose() returns (%.2f, %.2f, %.2f)" % (pose.x, pose.y, pose.yaw))
        return {"x": pose.x, "y": pose.y, "yaw": pose.yaw}
        
    def move_through_locations(self,locations: List[dict], final_yaw: float) -> True:
        print("move_through_locations() called")
        """Moves the robot a list of deltas (dx,dy) asynchronously. Each delta is a dictionary with 'dx' and 'dy' keys and values in meters. Then rotate to the final yaw. (yaw in degrees)."""
        if not locations:
            raise ValueError("Locations list cannot be empty.")
        if not all('dx' in loc and 'dy' in loc for loc in locations):
            raise ValueError("Each location must have 'dx' and 'dy' keys.")
        if not isinstance(final_yaw, (int, float)):
            raise ValueError("Final yaw must be a number.")
        
        return self.move_through_locations_real(self.sdp, locations, final_yaw)

    def check_move_progress(self) -> str:
        """Returns a string indicating the current status of the move action."""
        # This is a placeholder implementation. Replace with actual status checking logic.
        return self.getMoveActionStatus()
        
    def get_num_pictures_in_album(self, album_path: str) -> int:
        """Returns the number of pictures in the specified album directory."""
        import os
        album_path = "albums/" + album_path
        if not os.path.exists(album_path) or not os.path.isdir(album_path):
            return 0
        files = [f for f in os.listdir(album_path) if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
        return len(files)
    
    def get_nth_picture_from_album(self, album_path: str, n: int):
        """Returns the nth (index, 0 based) image/picture/photo from the album as a JSON dict image_url or None if n is out of range."""
        import os
        import base64
        album_path = "albums/" + album_path
        files = [f for f in os.listdir(album_path) if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
        if not files:
            return None
        if n >= len(files):
            return None
        file_path = os.path.join(album_path, files[n])
        with open(file_path, "rb") as img_file:
            image = img_file.read()
            #resize the image's larger dimension to 512 pixels while maintaining aspect ratio
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
            base64_image = base64.b64encode(image).decode("utf-8")

        image_json = {
            "type": "image_url",
            "image_url": {
                "url": f"data:image/jpeg;base64,{base64_image}",
            }
        }
        return image_json
    
    def get_move_through_locations_tool(self):
        return StructuredTool.from_function(
            func=self.move_through_locations,
            args_schema=MoveLocationsInput,
            name="move_through_locations",
            
            description="This func moves the robot by a list of delta offsets (dx,dy) asynchronously. Each delta is a dictionary with 'dx' and 'dy' keys and values in meters. Then it rotates robot to the final yaw. (yaw in degrees)."
        )
    
    def get_pose_tool(self):
        return StructuredTool.from_function(
            func=self.get_pose,
            name="get_pose",
            description="This func returns the robot's current pose as dict with x in meters, y in meters, yaw in degrees [-180, 180]."
        )
    
    def get_check_move_progress(self):
        return StructuredTool.from_function(
            func=self.check_move_progress,
            name="check_move_progress",
            description="This func returns a string indicating the current status of the move action."
        )

    def get_num_pictures_in_album_tool(self):
        return StructuredTool.from_function(
            func=self.get_num_pictures_in_album,
            name="get_num_pictures_in_album",
            description="This func returns the number of pictures in the specified album directory."
        )

    class GetNthPictureInput(BaseModel):
        album_path: str = Field(..., description="Relative path to the album directory containing jpg/png images.")
        n: int = Field(..., description="n (index, 0 based) of the image/picture/photo to return.")

    def get_nth_picture_from_album_tool(self):
        return StructuredTool.from_function(
            func=self.get_nth_picture_from_album,
            args_schema=self.GetNthPictureInput,
            name="get_nth_picture_from_album",
            description="This func returns the nth (0 based) image/picture/photo from the album as a JSON dict image_url or None if n is out of range. The album is a relative directory containing jpg/png images."
        )
