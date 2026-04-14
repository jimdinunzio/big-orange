from typing import Dict, List, Callable, Tuple, Union, Optional
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
from move_by_deltas_alert import post_alert
import math
import time

class Pose:
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw

class SDPSimClient:
    def __init__(self):
        self._pose = Pose(x=0.0, y=0.0, yaw=0.0)  # Initial pose
        self._progress = 0
        self._battery = 85
        self._is_charging = False
        self._board_temp = 45.0
        self._speed = 2

    def pose(self):
        # Return a copy of the pose to avoid external modification
        return Pose(self._pose.x, self._pose.y, self._pose.yaw)
    
    def set_pose(self, x, y, yaw):
        # Simulated set_pose method
        self._pose.x = x
        self._pose.y = y
        self._pose.yaw = yaw
        
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
    
    # Battery and system simulation methods
    def battery(self):
        return self._battery
    
    def getBatteryIsCharging(self):
        return self._is_charging
    
    def getBoardTemperature(self):
        return self._board_temp
    
    # Movement control simulation methods
    def cancelMoveAction(self):
        print("SIM cancelMoveAction call")
        return 0
    
    def home(self):
        print("SIM home call")
        self.set_pose(0.0, 0.0, 0.0)
        return 0
    
    def setSpeed(self, speed_level):
        print(f"SIM setSpeed call: {speed_level}")
        self._speed = speed_level
        return speed_level
    
    def wakeup(self):
        print("SIM wakeup call")
        return 0
    
    # Map management simulation methods
    def clearSlamtecMap(self):
        print("SIM clearSlamtecMap call")
        return 0
    
    def setMapUpdate(self, enabled):
        print(f"SIM setMapUpdate call: {enabled}")
        return 0
    
    def saveSlamtecMap(self, filename):
        print(f"SIM saveSlamtecMap call: {filename}")
        return 0
    
    def loadSlamtecMap(self, filename):
        print(f"SIM loadSlamtecMap call: {filename}")
        return 0
    
class MoveDeltasInput(BaseModel):
    deltas: List[Dict[str, float]] = Field(
        ...,
        description="[REQUIRED] List of deltas (dx,dy) to move. Example: [{'dx': 1.0, 'dy': 0.0}]. Must be non-empty list of {dx,dy} dictionaries."
    )
    final_yaw: float = Field(
        ...,
        description="[REQUIRED] Final orientation in degrees (yaw), must be between -180 and 180.",
        ge=-180,
        le=180
    )

    @model_validator(mode="before")
    def parse_deltas(cls, values):
        deltas = values.get('deltas')
        if not deltas:
            raise ValueError("deltas list cannot be empty")
        if isinstance(deltas, str):
            import ast
            values['deltas'] = ast.literal_eval(deltas)
        # Ensure deltas are dictionaries
        if isinstance(deltas, (list, tuple)):
            values['deltas'] = [dict(delta) if not isinstance(delta, dict) else delta for delta in deltas]
        return values

class RobotTools:
    def __init__(
        self,
        langgraph_tool_funcs: Optional[Dict[str, Callable]] = None,
        sim: bool = False
    ):
        self.sim = sim
        # langgraph_tool_funcs maps tool names directly to helper callables in main.py
        if not self.sim and langgraph_tool_funcs is None:
            raise ValueError("langgraph_tool_funcs must be provided when sim=False")
        self.langgraph_tool_funcs = langgraph_tool_funcs or {}
        
        self._sdp = None
        if self.sim:
            self._sdp: Union[MyClient, SDPSimClient] = SDPSimClient()
        else:
            self._sdp: Union[MyClient, SDPSimClient] = MyClient()
            sdp_comm.connectToSdp(self._sdp)

    @property
    def sdp(self):
        return self._sdp
    
    @sdp.setter
    def sdp(self, value):
        self._sdp = value

    def move_by_deltas_sim(self, sdp, deltas: List[Dict[str, float]], final_yaw: float) -> str:
        """Simulated function to move by deltas."""
        scale = 100  # Scale factor to convert meters to pixels for plotting
        pose = sdp.pose()
        abs_points = [(pose.x * scale, pose.y * scale)]
        points = []

        for idx, delta in enumerate(deltas):
            pose.x = pose.x + delta['dx']
            pose.y = pose.y + delta['dy']
            abs_points.append((pose.x * scale, pose.y * scale))
            points.append((pose.x, pose.y))
        approved = post_alert(abs_points)
        if not approved:
            return "Movement cancelled by user because it does not match the intended shape."
        
        for idx, delta in enumerate(points):
            print(f"Simulated moving by delta {idx + 1}: x={delta[0]}, y={delta[1]}")
            sdp.set_pose(delta[0], delta[1], final_yaw)
        print(f"Final desired yaw: {final_yaw}")
        return "Movement completed."

    def get_pose(self) -> Dict[str, float]:
        """Returns the robot's current pose (position) as a dict (x in meters, y in meters, yaw in degrees)."""
        pose = self._sdp.pose()
        print("get_pose() returns (%.2f, %.2f, %.2f)" % (pose.x, pose.y, pose.yaw))
        return {"x": pose.x, "y": pose.y, "yaw": pose.yaw}
        
    def move_by_deltas(self, deltas: List[dict], final_yaw: float) -> str:
        print("move_by_deltas() called")
        """Moves the robot by a list of deltas (dx,dy). Each delta is a dictionary with 'dx' and 'dy' keys and values in meters. Then rotate to the final yaw. (yaw in degrees)."""
        if not deltas:
            raise ValueError("Delta list cannot be empty.")
        if not all('dx' in delta and 'dy' in delta for delta in deltas):
            raise ValueError("Each delta must have 'dx' and 'dy' keys.")
        if not isinstance(final_yaw, (int, float)):
            raise ValueError("Final yaw must be a number.")
        
        if self.sim:
            return "Simulated: " + self.move_by_deltas_sim(self._sdp, deltas, final_yaw)
        else:
            return self.call_tool_helper("move_by_deltas", self._sdp, deltas, final_yaw)
        
    # Battery & System Info Tools
    def get_battery_info(self) -> Dict[str, Union[int, bool, float]]:
        """Get comprehensive battery and system information."""
        if self.sim:
            return {"percentage": 85, "is_charging": False, "board_temperature": 45.0}
        return {
            "percentage": self._sdp.battery(),
            "is_charging": self._sdp.getBatteryIsCharging(),
            "board_temperature": self._sdp.getBoardTemperature()
        }

    def cancel_action(self) -> str:
        """Cancel current action (movement, etc.)."""
        if self.sim:
            return "Current action cancelled (simulated)."
        return self.call_tool_helper("cancel_action", self._sdp)

    def go_recharge(self) -> str:
        """Return robot to recharge dock. Not same as home."""
        if self.sim:
            self._sdp.set_pose(0.0, 0.0, 0.0)
            return "Going to recharge dock (simulated)."
        return self.call_tool_helper("go_recharge", self._sdp)

    def set_robot_speed(self, speed_level: int) -> str:
        """Set robot movement speed: 1=low, 2=medium, 3=high."""
        if self.sim:
            speed_names = {1: "low", 2: "medium", 3: "high"}
            return f"Speed set to {speed_names.get(speed_level, 'unknown')} (simulated)."
        result = self._sdp.setSpeed(speed_level)
        speed_names = {1: "low", 2: "medium", 3: "high"}
        return f"Speed set to {speed_names.get(result, 'unknown')}."

    def wake_up_robot(self) -> str:
        """Wake up the robot from sleep mode."""
        if self.sim:
            return "Robot awakened (simulated)."
        self._sdp.wakeup()
        return "Robot awakened."

    # Map Management Tools  
    def clear_map(self) -> str:
        """Clear the current SLAM map."""
        if self.sim:
            return "Map cleared and mapping re-enabled (simulated)."
        result = self._sdp.clearSlamtecMap()
        if result == 0:
            self._sdp.setMapUpdate(True)
            return "Map cleared and mapping re-enabled."
        return "Failed to clear map."

    def save_map(self, filename: str) -> str:
        """Save current map to file."""
        if self.sim:
            return f"Map saved as {filename} (simulated)."
        result = self._sdp.saveSlamtecMap(filename.encode() + b'.stcm')
        return f"Map saved as {filename}." if result == 0 else "Failed to save map."

    def load_map(self, filename: str) -> str:
        """Load map from file."""
        if self.sim:
            return f"Map {filename} loaded (simulated)."
        result = self._sdp.loadSlamtecMap(filename.encode() + b'.stcm')
        return f"Map {filename} loaded." if result == 0 else "Failed to load map."

    def set_mapping_enabled(self, enabled: bool) -> str:
        """Enable or disable active mapping."""
        if self.sim:
            status = "enabled" if enabled else "disabled"
            return f"Mapping {status} (simulated)."
        self._sdp.setMapUpdate(enabled)
        status = "enabled" if enabled else "disabled"
        return f"Mapping {status}."

    # Basic Movement Tools (using main.py helpers)
    def move_forward(self, seconds: float = 0.5) -> str:
        """Move robot forward for specified seconds."""
        if self.sim:
            # Simulate forward movement
            pose = self._sdp.pose()
            distance = seconds * 0.5  # Assume 0.5 m/s speed
            new_x = pose.x + distance * math.cos(math.radians(pose.yaw))
            new_y = pose.y + distance * math.sin(math.radians(pose.yaw))
            time.sleep(seconds)
            self._sdp.set_pose(new_x, new_y, pose.yaw)
            return f"Moved forward for {seconds} seconds (simulated)."
        if "forward" in self.langgraph_tool_funcs:
            n = int(seconds * 10)  # Convert to 0.1 second increments
            self.langgraph_tool_funcs["forward"](self._sdp, n)
            return f"Moved forward for {seconds} seconds."
        return "Forward movement not available."

    def move_backward(self, seconds: float = 0.5) -> str:
        """Move robot backward for specified seconds."""
        if self.sim:
            # Simulate backward movement
            pose = self._sdp.pose()
            distance = seconds * 0.5  # Assume 0.5 m/s speed
            new_x = pose.x - distance * math.cos(math.radians(pose.yaw))
            new_y = pose.y - distance * math.sin(math.radians(pose.yaw))
            time.sleep(seconds)
            self._sdp.set_pose(new_x, new_y, pose.yaw)
            return f"Moved backward for {seconds} seconds (simulated)."
        if "backup" in self.langgraph_tool_funcs:
            n = int(seconds * 10)  # Convert to 0.1 second increments
            self.langgraph_tool_funcs["backup"](self._sdp, n)
            return f"Moved backward for {seconds} seconds."
        return "Backward movement not available."

    def turn_robot(self, degrees: float) -> str:
        """Turn robot by specified degrees (positive=left, negative=right)."""
        if self.sim:
            # Simulate turning
            pose = self._sdp.pose()
            new_yaw = pose.yaw + degrees
            # Normalize to [-180, 180]
            while new_yaw > 180:
                new_yaw -= 360
            while new_yaw < -180:
                new_yaw += 360
            self._sdp.set_pose(pose.x, pose.y, new_yaw)
            return f"Turned {degrees} degrees (simulated)."
        if "turn" in self.langgraph_tool_funcs:
            self.langgraph_tool_funcs["turn"](degrees, self._sdp)
            return f"Turned {degrees} degrees."
        return "Turn movement not available."

    # Tool helper wrapper functions for complex behaviors from main.py
    def call_tool_helper(self, tool_name: str, *args, **kwargs) -> str:
        """Call a tool helper function from main.py."""
        if tool_name in self.langgraph_tool_funcs:
            try:
                func = self.langgraph_tool_funcs[tool_name]
                return func(*args, **kwargs)
            except Exception as e:
                return f"Error calling {tool_name}: {str(e)}"
        return f"Tool {tool_name} not available."

    def list_locations(self) -> str:
        """List all known locations."""
        if self.sim:
            return "Known locations (simulated): home, kitchen, living_room, bedroom"
        return self.call_tool_helper("list_locations")

    def go_to_location_by_coords(self, x: float, y: float, yaw: float) -> str:
        """Go to a location with given coordinates."""
        if self.sim:
            # Simulate going to location by updating pose
            self._sdp.set_pose(x, y, yaw)
            time.sleep(5)  # Simulate time taken to go to location
            return f"arrived at ({x}, {y}, {yaw}) (simulated)."
        else:
            return self.call_tool_helper("go_to_location_by_coords", self._sdp, x, y, yaw)
        
    def go_to_location(self, location_name: str) -> str:
        """Go to a specific named location."""
        
        if self.sim:
            # Simulate going to location by updating pose
            locations = {
                "home": (0.0, 0.0, 0.0),
                "kitchen": (2.0, 1.0, 90.0),
                "living_room": (-1.0, 2.0, 45.0),
                "bedroom": (1.0, -1.0, -90.0)
            }
            if location_name in locations:
                x, y, yaw = locations[location_name]
                self._sdp.set_pose(x, y, yaw)
                
                time.sleep(5)
                    
                return f"arrived at {location_name} (simulated)."
            else:
                return f"Location '{location_name}' not found (simulated)."
        else:
            # For real hardware, call helper directly; progress is handled inside helpers
            return self.call_tool_helper("go_to_location", self._sdp, location_name)

    def go_to_location_with_narration(self, location_name: str, narration_interval_seconds: int = 5) -> str:
        """Go to a specific named location while periodically describing the scene.
        
        This tool navigates to the specified location and every narration_interval_seconds,
        captures and speaks a description of what is seen. The descriptions are spoken
        aloud (which also records them to conversation history).
        
        Args:
            location_name: The name of the destination location
            narration_interval_seconds: How often to describe the scene (default 5 seconds)
            
        Returns:
            A summary including arrival status and all observations made during the journey
        """
        if self.sim:
            # Simulate going to location with narration
            locations = {
                "home": (0.0, 0.0, 0.0),
                "kitchen": (2.0, 1.0, 90.0),
                "living_room": (-1.0, 2.0, 45.0),
                "bedroom": (1.0, -1.0, -90.0)
            }
            if location_name not in locations:
                return f"Location '{location_name}' not found (simulated)."
            
            observations = []
            sim_scenes = [
                "I see the hallway with some pictures on the wall.",
                "There's a chair on my left and a window ahead.",
                "I'm passing by the living room area."
            ]
            
            # Simulate 3 observations during journey
            for i, scene in enumerate(sim_scenes):
                time.sleep(narration_interval_seconds)
                observations.append(f"[{(i+1)*narration_interval_seconds}s] {scene}")
                print(f"[SIM] Narration: {scene}")
            
            x, y, yaw = locations[location_name]
            self._sdp.set_pose(x, y, yaw)
            
            return f"Arrived at {location_name}. Observations during journey: {'; '.join(observations)} (simulated)"
        else:
            return self.call_tool_helper("go_to_location_with_narration", self._sdp, location_name, narration_interval_seconds)

    def track_object(self, object_name: str, height: str = "eye level", duration: int = 30) -> str:
        """Track an object via NanoOWL as fast as possible with no robot movement."""
        if self.sim:
            return f"Tracked '{object_name}' for {duration}s (simulated)."
        return self.call_tool_helper("track_object", object_name, height, duration)

    def search_for_object(self, object_name: str, height: str, rot_clockwise: bool = True) -> str:
        """Search for a specific object by name by rotating and scanning."""
        if self.sim:
            # Simulate rotating 360 degrees to search
            pose = self._sdp.pose()
            self._sdp.set_pose(pose.x, pose.y, pose.yaw + 360)
            # Simulate finding the object if name is in known list
            known_objects = ["apple", "book", "bottle"]
            if object_name in known_objects:
                return f"Completed 360° search for {object_name} (simulated), found {object_name}."
            else:
                return f"Completed 360° search for {object_name} (simulated), did not find {object_name}."
        return self.call_tool_helper("search_for_object", self._sdp, object_name, height, rot_clockwise)

    def where_am_i(self) -> str:
        """Get current location information."""
        if self.sim:
            pose = self._sdp.pose()
            # Simple location detection based on coordinates
            if abs(pose.x) < 0.5 and abs(pose.y) < 0.5:
                return "You are at home (simulated)."
            elif pose.x > 1.5 and pose.y > 0.5:
                return "You are in the kitchen (simulated)."
            elif pose.x < -0.5 and pose.y > 1.5:
                return "You are in the living room (simulated)."
            elif pose.x > 0.5 and pose.y < -0.5:
                return "You are in the bedroom (simulated)."
            else:
                return f"You are at position ({pose.x:.1f}, {pose.y:.1f}) facing {pose.yaw:.1f}° (simulated)."
        return self.call_tool_helper("where_am_i", self._sdp)

    def recover_localization(self, rect: Optional[dict] = None) -> str:
        """Recover localization, optionally limiting to a given rectangle of the map."""
        if self.sim:
            # Simulate relocalization by resetting pose
            self._sdp.set_pose(0.0, 0.0, 0.0)
            return "Relocalization completed (simulated)."
        return self.call_tool_helper("recover_localization", self._sdp, rect)

    def move_in_dir_dist(self, direction: str, distance: float, unit: str = "meters") -> str:
        """Move in specified direction (forward, backward, right, left) for specified distance."""
        if self.sim:
            # Convert to meters
            if unit == "cm":
                distance = distance / 100
            elif unit == "inches":
                distance = distance * 0.0254
            elif unit == "feet":
                distance = distance * 0.3048
            
            pose = self._sdp.pose()
            if direction == "forward":
                new_x = pose.x + distance * math.cos(math.radians(pose.yaw))
                new_y = pose.y + distance * math.sin(math.radians(pose.yaw))
            elif direction == "backward":
                new_x = pose.x - distance * math.cos(math.radians(pose.yaw))
                new_y = pose.y - distance * math.sin(math.radians(pose.yaw))
            elif direction == "left":
                # Move perpendicular to current direction
                new_x = pose.x + distance * math.cos(math.radians(pose.yaw + 90))
                new_y = pose.y + distance * math.sin(math.radians(pose.yaw + 90))
            elif direction == "right":
                # Move perpendicular to current direction
                new_x = pose.x + distance * math.cos(math.radians(pose.yaw - 90))
                new_y = pose.y + distance * math.sin(math.radians(pose.yaw - 90))
            else:
                return f"Unknown direction '{direction}' (simulated)."
            
            self._sdp.set_pose(new_x, new_y, pose.yaw)
            return f"Moved {direction} {distance} {unit} (simulated)."
        return self.call_tool_helper("move_in_dir_dist", self._sdp, direction, distance, unit)

    def search_for_person(self) -> str:
        """Search for closest person by rotating and scanning."""
        if self.sim:
            # Simulate rotating 360 degrees to search
            pose = self._sdp.pose()
            self._sdp.set_pose(pose.x, pose.y, pose.yaw + 360)
            return "Completed 360° search for person (simulated), found one person."
        return self.call_tool_helper("search_for_person", self._sdp)

    def search_for_face(self, name: str, rot_clockwise: bool=True) -> str:
        """Search for a specific person by name by rotating and scanning and stopping if found."""
        if self.sim:
            # Simulate rotating 360 degrees to search
            pose = self._sdp.pose()
            self._sdp.set_pose(pose.x, pose.y, pose.yaw + 360)
            # Simulate finding the person if name is in known list
            known_faces = ["Alice", "Bob", "Charlie"]
            if name in known_faces:
                return f"Completed 360° search for {name} (simulated), found {name}."
            else:
                return f"Completed 360° search for {name} (simulated), did not find {name}."
        return self.call_tool_helper("search_for_face", self._sdp, name, rot_clockwise)
    
    def get_loc_of_person_from_voice(self) -> Tuple[float, float, float]:
        """From direction of their voice get the x,y, and yaw to use to go up to a person"""
        if self.sim:
            # Simulate turning towards voice direction
            pose = self._sdp.pose()
            new_yaw = pose.yaw + 45  # Assume voice came from 45 degrees to the right
            self._sdp.set_pose(pose.x, pose.y, new_yaw)
            # computed position of person
            loc = (pose.x + 1.0, pose.y + 1.0, math.radians(new_yaw))
            return loc
        return self.call_tool_helper("get_loc_of_person_from_voice", self._sdp)

    def get_known_faces(self) -> str:
        """Get list of all known faces."""
        if self.sim:
            return "Known faces (simulated): Alice, Bob, Charlie"
        return self.call_tool_helper("get_known_faces")

    def identify_any_visible_face(self) -> str:
        """Identify any face in view."""
        if self.sim:
            known_faces = ["Alice", "Bob", "Charlie"]
            import random
            identified_face = random.choice(known_faces + [None])
            if identified_face:
                return f"Identified {identified_face} (simulated)."
            return "No known face identified (simulated)."
        return self.call_tool_helper("identify_visible_face", self._sdp)

    def memorize_a_face(self, name) -> str:
        """Memorize a new face with a name."""
        if self.sim:
            return f"New face {name} memorized (simulated)."
        return self.call_tool_helper("memorize_a_face", name)
    
    def while_go_to_location_find_face(self, name: str, location_name: str) -> str:
        """Go to location while searching for a specific person by name."""
        if self.sim:
            found = False
            # Simulate going to location by updating pose
            locations = {
                "home": (0.0, 0.0, 0.0),
                "kitchen": (2.0, 1.0, 90.0),
                "living_room": (-1.0, 2.0, 45.0),
                "bedroom": (1.0, -1.0, -90.0)
            }
            if location_name in locations:
                x, y, yaw = locations[location_name]
                self._sdp.set_pose(x, y, yaw)                        

                time.sleep(5)  # Simulate time taken to go to location
                # Simulate face detection
                if location_name == "kitchen" and name == "Alice":
                    found = True
                if found:
                    return f"arrived at {location_name} and found {name} (simulated)."
                else:
                    return f"Could not find {name} while going to {location_name} (simulated)."
            else:
                return f"Location '{location_name}' not found (simulated)."
        return self.call_tool_helper("while_go_to_location_find_face", self._sdp, name, location_name)
    
    def while_go_to_loc_find_object(self, object_name: str, location_name: str, height: str = "eye level") -> str:
        """Go to location while searching for a specific object by name."""
        if self.sim:
            found = False
            # Simulate going to location by updating pose
            locations = {
                "home": (0.0, 0.0, 0.0),
                "kitchen": (2.0, 1.0, 90.0),
                "living_room": (-1.0, 2.0, 45.0),
                "bedroom": (1.0, -1.0, -90.0),
                "": (0.0, 0.0, 0.0)
            }
            if location_name in locations:
                if location_name != '':
                    x, y, yaw = locations[location_name]
                    self._sdp.set_pose(x, y, yaw)

                time.sleep(5)  # Simulate time taken to go to location
                # Simulate object detection
                if location_name == "kitchen" and object_name == "apple":
                    return f"on way to {location_name} I found {object_name} (simulated)."
                elif location_name == '' and object_name == "sofa":
                    return f"I found {object_name} crossing the room.(simulated)."
                else:
                    return f"Could not find {object_name} while going to {location_name} (simulated)."
            else:
                return f"Location '{location_name}' not found (simulated)."
        self._sdp.wakeup()
        return self.call_tool_helper("while_go_to_loc_find_object", self._sdp, object_name, location_name, height)


    def aim_camera(self, yaw: Optional[int] = None, pitch: Optional[int] = None) -> str:
        """Aim camera to specific yaw and/or pitch angles."""
        if self.sim:
            yaw_str = f"yaw={yaw}" if yaw is not None else ""
            pitch_str = f"pitch={pitch}" if pitch is not None else ""
            parts = [s for s in [yaw_str, pitch_str] if s]
            angles = ", ".join(parts)
            return f"Camera aimed to {angles} (simulated)."
        return self.call_tool_helper("aim_camera", yaw, pitch)

    def home_camera(self) -> str:
        """Return camera to home position."""
        if self.sim:
            return "Camera returned to home position (simulated)."
        return self.call_tool_helper("home_camera")

    def take_picture(self) -> str:
        """Take a picture using the camera."""
        if self.sim:
            from playsound import playsound
            _sounds_dir = "sounds"
            filepath = _sounds_dir + "//camera-shutter.wav"
            playsound(filepath, block=True)
            return "Picture taken and saved (simulated)."
        return self.call_tool_helper("take_picture")

    def follow_me(self) -> str:
        """Start following a person."""
        if self.sim:
            return "Started following person (simulated)."
        return self.call_tool_helper("follow_me")

    def stop_following(self) -> str:
        """Stop following a person."""
        if self.sim:
            return "Stopped following person (simulated)."
        return self.call_tool_helper("stop_following")

    def track_me(self) -> str:
        """Start tracking a person with camera."""
        if self.sim:
            return "Started tracking person with camera (simulated)."
        return self.call_tool_helper("track_me")

    def stop_tracking(self) -> str:
        """Stop tracking a person."""
        if self.sim:
            return "Stopped tracking person with camera (simulated)."
        return self.call_tool_helper("stop_tracking")
        
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

    def get_yolo_object_detections(self) -> List[Dict[str, Union[str, Tuple[float, float, float]]]]:
        """Get a list of objects and persons (with coordinates) visible in the scene using YOLO model."""
        if self.sim:
            # Simulated detections
            return [
                {"label": "person", "coords": (1.2, 0.5, 90.0)},
                {"label": "chair", "coords": (0.8, 1.5, 45.0)}
            ]
        result = self.call_tool_helper("get_yolo_detections", self._sdp)
        if isinstance(result, list):
            return result
        return []

    def describe_scene(self) -> str:
        """Describe the scene concisely."""
        if self.sim:
            return "I see a simulated room with furniture and a person."
        return self.call_tool_helper("describe_scene")
    
    def ask_question_about_scene(self, question: str) -> str:
        """Ask a question about the current scene."""
        if self.sim:
            return f"Simulated answer to the question: {question}"
        return self.call_tool_helper("ask_question_about_scene", question)
    
    # Tool definitions for LangGraph

    def get_move_by_deltas_tool(self):
        return StructuredTool.from_function(
            func=self.move_by_deltas,
            args_schema=MoveDeltasInput,
            name="move_by_deltas",
            description="This func moves the robot by a non-empty list of deltas (dx,dy). Each delta is a dictionary with 'dx' and 'dy' keys and values in meters. Then it rotates robot to the final yaw. (yaw in degrees)."
        )
    
    def get_pose_tool(self):
        return StructuredTool.from_function(
            func=self.get_pose,
            name="get_pose",
            description="This func returns the robot's current pose as dict with x in meters, y in meters, yaw in degrees [-180, 180]."
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

    # Additional tool definitions for the new methods
    class SpeedInput(BaseModel):
        speed_level: int = Field(..., description="Speed level: 1=low, 2=medium, 3=high", ge=1, le=3)

    class MovementInput(BaseModel):
        seconds: float = Field(default=0.5, description="Duration in seconds", ge=0.1, le=10.0)

    class TurnInput(BaseModel):
        degrees: float = Field(..., description="Degrees to turn (positive=left, negative=right)", ge=-360, le=360)

    class FilenameInput(BaseModel):
        filename: str = Field(..., description="Name of the file (without extension)")

    class MappingInput(BaseModel):
        enabled: bool = Field(..., description="True to enable mapping, False to disable")

    class RecoverLocalizationInput(BaseModel):
        rect: Optional[dict] = Field(None, description="Optional rectangle to limit relocalization")

    class LocationInput(BaseModel):
        location_name: str = Field(..., description="Name of the location to go to")

    class LocationWithNarrationInput(BaseModel):
        location_name: str = Field(..., description="Name of the location to go to")
        narration_interval_seconds: Optional[int] = Field(5, description="How often to describe the scene in seconds")
        
    class GoToLocationByCoordsInput(BaseModel):
        x: float = Field(..., description="X coordinate in meters")
        y: float = Field(..., description="Y coordinate in meters")
        yaw: float = Field(..., description="Yaw orientation in degrees (-180 to 180)", ge=-180, le=180)
        
    class FaceInput(BaseModel):
        name: str = Field(..., description="Name of the person/face")   

    class SearchForFaceInput(FaceInput):
        rot_clockwise: Optional[bool] = Field(None, description="rotate clockwise (True) and Default, or counter-clockwise (False) while searching")

    class FaceAndLocationInput(BaseModel):
        name: str = Field(..., description="Name of the person/face to find")
        location_name: str = Field(..., description="Name of the location to go to or 'across the room'")
        
    class ObjAndLocationInput(BaseModel):
        object_name: str = Field(..., description="Name of the object to find")
        location_name: str = Field(..., description="Name of the location to go to or 'across the room'")
        height: str = Field("eye level", description="Height to search at: floor, eye level (default), up high")

    class ObjTrackInput(BaseModel):
        object_name: str = Field(..., description="Name of the object to track")
        height: str = Field("eye level", description="Height to search at: floor, eye level (default), up high")
        duration: int = Field(30, description="How many seconds to track (default 30)")

    class ObjSearchInput(BaseModel):
        object_name: str = Field(..., description="Name of the object to search for")
        height: str = Field(..., description="Height to search at: floor, eye level (default), up high")

    class DirectionDistanceInput(BaseModel):
        direction: str = Field(..., description="Direction: forward, backward, left, right")
        distance: float = Field(..., description="Distance to move", gt=0)
        unit: str = Field(default="meters", description="Unit: meters, cm, in, ft")

    class AimCameraInput(BaseModel):
        yaw: Optional[int] = Field(None, description="Yaw angle for camera. 90 is straight ahead, 100 aims left side of tray, 80 right side of tray.")
        pitch: Optional[int] = Field(None, description="Pitch angle for camera. 115 is straight ahead, 75 aims up for seeing faces, 135 down for objects on floor.")

    def get_all_tools(self):
        """Get all available LangGraph tools."""
        return [
            # Core movement and navigation
            self.get_pose_tool(),
            
            # Battery and system tools
            StructuredTool.from_function(
                func=self.get_battery_info,
                name="get_battery_info",
                description="Get battery and system info including percentage, charging status, and temperature."
            ),
            
            StructuredTool.from_function(
                func=self.cancel_action,
                name="cancel_action",
                description="Cancel current action (movement, etc.)."
            ),
            
            StructuredTool.from_function(
                func=self.go_recharge,
                name="go_recharge",
                description="Return robot to charging station. Not same as home."
            ),
            
            StructuredTool.from_function(
                func=self.set_robot_speed,
                args_schema=self.SpeedInput,
                name="set_robot_speed",
                description="Set robot movement speed: 1=low, 2=medium, 3=high."
            ),
            
            StructuredTool.from_function(
                func=self.wake_up_robot,
                name="wake_up_robot",
                description="Wake up the robot from sleep mode."
            ),
            
            # Map management tools
            StructuredTool.from_function(
                func=self.clear_map,
                name="clear_map",
                description="Clear the current SLAM map."
            ),
            
            StructuredTool.from_function(
                func=self.save_map,
                args_schema=self.FilenameInput,
                name="save_map",
                description="Save current map to file."
            ),
            
            StructuredTool.from_function(
                func=self.load_map,
                args_schema=self.FilenameInput,
                name="load_map",
                description="Load map from file."
            ),
            
            StructuredTool.from_function(
                func=self.set_mapping_enabled,
                args_schema=self.MappingInput,
                name="set_mapping_enabled",
                description="Enable or disable active mapping."
            ),
            
            # Basic movement tools
            StructuredTool.from_function(
                func=self.move_forward,
                args_schema=self.MovementInput,
                name="move_forward",
                description="Move robot forward for specified seconds."
            ),
            
            StructuredTool.from_function(
                func=self.move_backward,
                args_schema=self.MovementInput,
                name="move_backward",
                description="Move robot backward for specified seconds."
            ),
            
            StructuredTool.from_function(
                func=self.turn_robot,
                args_schema=self.TurnInput,
                name="turn_robot",
                description="Turn robot by specified degrees (positive=left, negative=right)."
            ),
            
            # Complex behavior tools (using main.py functions)
            StructuredTool.from_function(
                func=self.list_locations,
                name="list_locations",
                description="List all known locations."
            ),
            
            StructuredTool.from_function(
                func=self.go_to_location_by_coords,
                args_schema=self.GoToLocationByCoordsInput,
                name="go_to_location_by_coords",
                description="Go to a location with given coordinates."
            ),

            StructuredTool.from_function(
                func=self.go_to_location,
                args_schema=self.LocationInput,
                name="go_to_location",
                description="Go to a specific named location."
            ),
            
            StructuredTool.from_function(
                func=self.where_am_i,
                name="where_am_i",
                description="Get current location information."
            ),

            StructuredTool.from_function(
                func=self.recover_localization,
                args_schema=self.RecoverLocalizationInput,
                name="recover_localization",
                description="For when lost recover localization, optionally limiting to a given rectangle of map."
            ),
            
            StructuredTool.from_function(
                func=self.move_in_dir_dist,
                args_schema=self.DirectionDistanceInput,
                name="move_in_dir_dist",
                description="Move in specified direction (forward, backward, right, left) for specified distance."
            ),
            
            StructuredTool.from_function(
                func=self.search_for_person,
                name="search_for_person",
                description="Search for closest person by rotating and scanning."
            ),

            StructuredTool.from_function(
                func=self.search_for_face,
                args_schema=self.SearchForFaceInput,
                name="search_for_face",
                description="Search for a specific person by name by rotating and scanning and stopping if found."
            ),
            
            StructuredTool.from_function(
                func=self.identify_any_visible_face,
                name="identify_any_visible_face",
                description="Identify any face in view."
            ),
            
            StructuredTool.from_function(
                func=self.memorize_a_face,
                name="memorize_a_face",
                args_schema=self.FaceInput,
                description="Memorize a new face with a name."
            ),
            
            StructuredTool.from_function(
                func=self.get_known_faces,
                name="get_known_faces",
                description="Get list of all known faces."
            ),

            StructuredTool.from_function(
                func=self.while_go_to_location_find_face,
                args_schema=self.FaceAndLocationInput,
                name="while_go_to_location_find_face",
                description="While going to a loc, look for face by name and stop as soon as it is seen and report its coords."
            ),
            
            StructuredTool.from_function(
                func=self.while_go_to_loc_find_object,
                args_schema=self.ObjAndLocationInput,
                name="while_go_to_loc_find_object",
                description="While going to loc, look for object and stop as soon as it is seen and report its coords."
            ),

            StructuredTool.from_function(
                func=self.aim_camera,
                args_schema=self.AimCameraInput,
                name="aim_camera",
                description="Aim camera to specific yaw and/or pitch angles."
            ),
            
            StructuredTool.from_function(
                func=self.home_camera,
                name="home_camera",
                description="Return camera to home position."
            ),
            
            StructuredTool.from_function(
                func=self.take_picture,
                name="take_picture",
                description="Take a picture using the camera."
            ),
            
            StructuredTool.from_function(
                func=self.follow_me,
                name="follow_me",
                description="Start following a person."
            ),
            
            StructuredTool.from_function(
                func=self.stop_following,
                name="stop_following",
                description="Stop following a person."
            ),
            
            StructuredTool.from_function(
                func=self.track_me,
                name="track_me",
                description="Start tracking a person with camera."
            ),
            
            StructuredTool.from_function(
                func=self.stop_tracking,
                name="stop_tracking",
                description="Stop tracking a person."
            ),
            
            StructuredTool.from_function(
                func=self.get_loc_of_person_from_voice,
                name="get_loc_of_person_from_voice",
                description="Get location of person from direction of voice."
            ),
            
            # Picture management tools
            self.get_num_pictures_in_album_tool(),
            self.get_nth_picture_from_album_tool(),
            
            # Scene description & VLM tools
            StructuredTool.from_function(
                func=self.describe_scene,
                name="describe_scene",
                description="[VISION] Get current camera view description. MUST call for 'what do you see' questions."
            ),

            StructuredTool.from_function(
                func=self.ask_question_about_scene,
                name="ask_question_about_scene",
                description="[VISION] Ask yes/no or specific question about current camera view. MUST call for 'is there a X' questions."
            ),

            StructuredTool.from_function(
                func=self.go_to_location_with_narration,
                args_schema=self.LocationWithNarrationInput,
                name="go_to_location_with_narration",
                description="Go to a specific named location while periodically describing the scene."
            ),

            StructuredTool.from_function(
                func=self.track_object,
                args_schema=self.ObjTrackInput,
                name="track_object",
                description="Track an object via NanoOWL with no robot movement. Shows a green detection box around the object. Returns hit rate."
            ),

            StructuredTool.from_function(
                func=self.search_for_object,
                args_schema=self.ObjSearchInput,
                name="search_for_object",
                description="Search for a specific object by name by rotating and scanning."
            ),

            StructuredTool.from_function(
                func=self.get_yolo_object_detections,
                name="get_yolo_object_detections",
                description="Get a list of objects and persons (with coordinates) visible in the scene using YOLO model."
            ),

            self.get_move_by_deltas_tool(),
        ]
