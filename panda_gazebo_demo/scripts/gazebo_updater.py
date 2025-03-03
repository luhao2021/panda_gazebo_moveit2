#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from geometry_msgs.msg import Pose, Point
import subprocess
import time
import re
import os
import trimesh
import argparse
import yaml

class GazeboPoseUpdater(Node):
    def __init__(self, config):
        super().__init__('gazebo_pose_updater')
        self.config = config
        # Load commands from config.
        self.availability_check_cmd = self.config["availability_check_service"]["command"].split() + \
                                        self.config["availability_check_service"]["parameters"].split()
        self.initial_scene_layout_cmd = self.config["initial_scene_layout_service"]["command"].split() + \
                                        ["-s", self.config["initial_scene_layout_service"]["service_name"]] + \
                                        self.config["initial_scene_layout_service"]["parameters"].split()
        self.pose_update_cmd = self.config["pose_update_topic"]["command"].split() + \
                               ["-t", self.config["pose_update_topic"]["topic_name"]] + \
                               self.config["pose_update_topic"]["parameters"].split()
        self.objects_of_interest = set(self.config["objects_of_interest"])
        
        print("availability check command:", self.availability_check_cmd)
        print("initial scene info service command:", self.initial_scene_layout_cmd)
        print("pose update command:", self.pose_update_cmd)
        print("objects of interests:", self.objects_of_interest)

        self.get_logger().info("Initializing GazeboPoseUpdater from config...")
        # Publisher for updating the planning scene
        self.planning_scene_pub = self.create_publisher(PlanningScene, 'planning_scene', 10)
        # Call the initial service to get the scene layout.
        self.initial_scene_text = self.call_ign_service()
        self.objects = self.parse_scene(self.initial_scene_text)
        # Here you might also build and store your complete CollisionObjects for each object.
        # For this example, we simply store the parsed object pose information.
        self.get_logger().info(f"Initialized with {len(self.objects)} objects from scene.")

        # Create a periodic timer to update poses (using the pose update topic command).
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz update

        if not self.objects:
            self.get_logger().warn("No models found in the scene info")
            return

        for obj in self.objects:
            print("Object:", obj["name"])
            print("  Pose:", obj["pose"])
            if "geometry" in obj:
                print("  Geometry:", obj["geometry"])
            print()

        # Create a planning scene update message.
        planning_scene_msg = PlanningScene()
        planning_scene_msg.is_diff = True

        # For each model of interest, create a CollisionObject.
        for model in self.objects:
            if self.match_interests(model.get('name')):
                collision_object = self.create_collision_object(model)
                planning_scene_msg.world.collision_objects.append(collision_object)
                self.get_logger().info(f"Updated collision object: {model.get('name')}")

        # Publish the planning scene update.
        self.planning_scene_pub.publish(planning_scene_msg)

        # Create a periodic timer to update poses from the Gazebo topic.
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz update

    def wait_for_ign_service(self, service_name, timeout_sec=300, poll_interval=1.0):
        """
        Waits until the given Ignition service is available.
        Checks both the service info and the list of services.
        """
        self.get_logger().info(f"Waiting for service {service_name} to become available...")
        start_time = time.time()
        error_message = f"No service providers on service [{service_name}]"
        while time.time() - start_time < timeout_sec:
            try:
                cmd_list = self.config["availability_check_service"]["command"].split() + \
                           self.config["availability_check_service"]["parameters"].split()
                output_list = subprocess.check_output(cmd_list, universal_newlines=True)
                if service_name in output_list:
                    self.get_logger().info(f"Service {service_name} is available (via service list).")
                    return True
            except Exception as e:
                self.get_logger().debug(f"Service list check failed: {e}")

            time.sleep(poll_interval)
        self.get_logger().error(f"Timeout waiting for service {service_name}.")
        return False

    def call_ign_service(self):
        """
        Waits for the initial scene service to become available, then calls it.
        """
        service_name = self.config["initial_scene_layout_service"]["service_name"]
        if not self.wait_for_ign_service(service_name):
            return ""
        try:
            output = subprocess.check_output(self.initial_scene_layout_cmd, universal_newlines=True)
            self.get_logger().info("Initial scene info retrieved.")
            return output
        except Exception as e:
            self.get_logger().error(f"Error calling ign service: {e}")
            return ""

    def timer_callback(self):
        """
        Periodic callback that reads pose updates from Gazebo and updates object poses.
        """
        try:
            output = subprocess.check_output(self.pose_update_cmd, universal_newlines=True)
        except Exception as e:
            self.get_logger().error(f"Error reading pose topic: {e}")
            return

        pose_info = self.parse_pose_info(output)

        # Process each object using your rules.
        for obj in self.objects:
            if not self.match_interests(obj["name"]):
                continue
            #print(obj["name"] + ":")
            tokens = obj["name"].split("_")
            if tokens[-1].lower() == "visual":
                # Case 1: Object name ends with '_visual'
                search_name = "_".join(tokens[:-1])
                if search_name in pose_info:
                    obj["pose"] = pose_info[search_name]
                    self.get_logger().info(f"Updated {obj['name']} using pose for '{search_name}'")
                else:
                    self.get_logger().warn(f"No pose found for '{search_name}'")
            else:
                # Case 2: Object name is composite (e.g., 'Table_back_left_leg')
                # Find the first token that is one of the objects_of_interest.
                first = None
                for token in tokens:
                    if token in self.objects_of_interest:
                        first = token
                        break
                if first is not None:
                    remaining_tokens = tokens[tokens.index(first)+1:]
                    second = "_".join(remaining_tokens) if remaining_tokens else ""
                    pose_first = pose_info.get(first, {})
                    pose_second = pose_info.get(second, {})
                    updated_pose = self.merge_pose(pose_first, pose_second)
                    obj["pose"] = updated_pose
                    self.get_logger().info(f"Updated {obj['name']} using merged pose from '{first}' and '{second}'")
                else:
                    if obj["name"] in pose_info:
                        obj["pose"] = pose_info[obj["name"]]
                        self.get_logger().info(f"Updated {obj['name']} using its own pose")
                    else:
                        self.get_logger().warn(f"No matching pose found for {obj['name']}")
        
        # After updating all object poses, update the planning scene.
        self.update_planning_scene()

    def update_planning_scene(self):
        """
        Updates the planning scene by modifying the stored collision objects.
        For each object, the stored collision object's pose is updated and its operation
        is set to MOVE before publishing the diff.
        """
        ps = PlanningScene()
        ps.is_diff = True

        for obj in self.objects:
            if not self.match_interests(obj["name"]):
                continue
            # Retrieve the pre-built collision object
            co = obj.get("collision_object", None)
            if co is None:
                continue  # Skip if no collision object stored
            
            # Update the pose from the current object pose (updated from the ign topic)
            pose = Pose()
            pos = obj["pose"].get("position", {})
            ori = obj["pose"].get("orientation", {})

            pose.position.x = pos.get("x", 0.0)
            pose.position.y = pos.get("y", 0.0)
            pose.position.z = pos.get("z", 0.0)
            pose.orientation.x = ori.get("x", 0.0)
            pose.orientation.y = ori.get("y", 0.0)
            pose.orientation.z = ori.get("z", 0.0)
            pose.orientation.w = ori.get("w", 1.0)

            #print(obj["name"], pose)

            # Update the stored collision object's pose.
            # We assume the collision object has either a primitive or a mesh.
            if co.primitives:
                # For primitive geometries.
                if len(co.primitive_poses) > 0:
                    #print("Update Primitive " + obj["name"] + ":", co.primitives[0].type, pose)
                    co.primitive_poses[0] = pose
                else:
                    co.primitive_poses.append(pose)
            elif co.meshes:
                # For mesh geometries.
                if len(co.mesh_poses) > 0:
                    #print("Update Mesh " + obj["name"] + ":", pose)
                    co.mesh_poses[0] = pose
                else:
                    co.mesh_poses.append(pose)
            else:
                # If neither exists, you could store the pose in a generic field if needed.
                pass

            # Set the operation to MOVE (or MODIFY, depending on your interface)
            co.operation = CollisionObject.ADD  # Alternatively, use CollisionObject.MODIFY if appropriate.
            
            # Add the updated collision object to the planning scene diff.
            ps.world.collision_objects.append(co)

        self.planning_scene_pub.publish(ps)
        self.get_logger().info("Published updated collision object poses to the planning scene.")


    def match_interests(self, string):
        tokens = string.split('_')

        if tokens[-1] == 'visual':  
            substring = '_'.join(tokens[:-1])  
        else:  
            substring = tokens[0]  

        if substring in self.objects_of_interest:
            return True
        else:
            return False

    def extract_block(self, text, start_keyword):
        """
        Extract a block of text starting at start_keyword and ending at the matching closing brace.
        Returns the substring including start_keyword.
        """
        index = text.find(start_keyword)
        if index == -1:
            return None
        # Find the first '{' after the keyword.
        open_brace_index = text.find("{", index)
        if open_brace_index == -1:
            return None

        count = 0
        i = open_brace_index
        while i < len(text):
            if text[i] == '{':
                count += 1
            elif text[i] == '}':
                count -= 1
                if count == 0:
                    return text[index:i+1]
            i += 1
        return None

    def extract_all_blocks(self, text, start_keyword):
        """
        Extract all blocks in text that start with start_keyword.
        Returns a list of block strings.
        """
        blocks = []
        start = 0
        while True:
            index = text.find(start_keyword, start)
            if index == -1:
                break
            block = self.extract_block(text[index:], start_keyword)
            if block:
                blocks.append(block)
                start = index + len(block)
            else:
                break
        return blocks

    def extract_pose_fields(self, block, tag="pose {"):
        """
        Extracts position and orientation values from a pose block.
        Returns a dictionary that may contain:
           { "position": {"x":..., "y":..., "z":...},
             "orientation": {"x":..., "y":..., "z":..., "w":...} }
        Only fields that are explicitly specified are returned.
        """
        pose_block = self.extract_block(block, tag)
        result = {}
        if not pose_block:
            return result

        pos_block = self.extract_block(pose_block, "position {")
        pos = {}
        if pos_block:
            for axis in ['x', 'y', 'z']:
                m = re.search(rf'{axis}:\s*([-\d\.e]+)', pos_block)
                if m:
                    pos[axis] = float(m.group(1))
        if pos:
            result['position'] = pos

        ori_block = self.extract_block(pose_block, "orientation {")
        ori = {}
        if ori_block:
            for comp in ['x', 'y', 'z', 'w']:
                m = re.search(rf'{comp}:\s*([-\d\.e]+)', ori_block)
                if m:
                    ori[comp] = float(m.group(1))
        if ori:
            result['orientation'] = ori

        return result

    def parse_pose_info(self, text):
        """
        Parses the output from 'ign topic ...' into a dictionary mapping pose name to pose data.
        """
        pose_blocks = self.extract_all_blocks(text, "pose {")
        pose_dict = {}
        for block in pose_blocks:
            m = re.search(r'name:\s*"([^"]+)"', block)
            if not m:
                continue
            name = m.group(1)
            pose_data = self.extract_pose_fields(block, "pose {")
            pose_dict[name] = pose_data
        return pose_dict

    def merge_pose(self, parent_pose, visual_pose):
        """
        Merge two pose dictionaries.
        For each field in visual_pose, override the parent_pose.
        """
        merged = {}
        merged['position'] = parent_pose.get('position', {}).copy()
        merged['orientation'] = parent_pose.get('orientation', {}).copy()
        if 'position' in visual_pose:
            for axis, val in visual_pose['position'].items():
                if axis in merged['position'].keys():
                    merged['position'][axis] += val
                else:
                    merged['position'][axis] = val
        if 'orientation' in visual_pose:
            for comp, val in visual_pose['orientation'].items():
                if comp in merged['orientation'].keys():
                    merged['orientation'][comp] += val
                else:
                    merged['orientation'][comp] = val
        return merged

    def parse_scene(self, text):
        """
        Parse an Ignition scene description text and return a list of objects.
        Each object corresponds to a visual tag, with a merged pose and geometry.
        This function supports multiple model tags.
        """
        objects = []
        # Extract all model blocks
        model_blocks = self.extract_all_blocks(text, "model {")
        for model_block in model_blocks:
            # Get the model name.
            name_match = re.search(r'name:\s*"([^"]+)"', model_block)
            if not name_match:
                continue
            model_name = name_match.group(1)
            # Extract parent pose from the model block.
            parent_pose = self.extract_pose_fields(model_block, "pose {")

            # Extract all visual blocks from the model block.
            visual_blocks = self.extract_all_blocks(model_block, "visual {")
            if not visual_blocks:
                # If no visual blocks, use the model as one object.
                obj = {"name": model_name, "pose": parent_pose}
                objects.append(obj)
            else:
                for visual_block in visual_blocks:
                    # Get visual name if available.
                    visual_name_match = re.search(r'name:\s*"([^"]+)"', visual_block)
                    visual_name = visual_name_match.group(1) if visual_name_match else ""
                    # Extract visual's own pose (if provided).
                    visual_pose = self.extract_pose_fields(visual_block, "pose {")
                    # Merge parent's pose with visual's pose (visual overrides parent's values)
                    merged_pose = self.merge_pose(parent_pose, visual_pose)
                    
                    # Extract geometry from the visual block.
                    geometry = None
                    geometry_block = self.extract_block(visual_block, "geometry {")
                    if geometry_block:
                        # Check for box geometry: support "type: box" or "box {"
                        if re.search(r'(type:\s*box)|(box\s*{)', geometry_block, re.IGNORECASE):
                            if re.search(r'box\s*{', geometry_block, re.IGNORECASE):
                                box_block = self.extract_block(geometry_block, "box {")
                                size_block = self.extract_block(box_block, "size {")
                            else:
                                size_block = self.extract_block(geometry_block, "size {")
                            dimensions = {}
                            if size_block:
                                for axis in ['x','y','z']:
                                    m = re.search(rf'{axis}:\s*([-\d\.e]+)', size_block)
                                    dimensions[axis] = float(m.group(1)) if m else 1.0
                            geometry = {'type': 'box', 'size': dimensions}
                        # Check for cylinder geometry
                        elif re.search(r'(type:\s*cylinder)|(cylinder\s*{)', geometry_block, re.IGNORECASE):
                            if re.search(r'cylinder\s*{', geometry_block, re.IGNORECASE):
                                cyl_block = self.extract_block(geometry_block, "cylinder {")
                                radius_match = re.search(r'radius:\s*([-\d\.e]+)', cyl_block)
                                length_match = re.search(r'length:\s*([-\d\.e]+)', cyl_block)
                            else:
                                radius_match = re.search(r'radius:\s*([-\d\.e]+)', geometry_block)
                                length_match = re.search(r'length:\s*([-\d\.e]+)', geometry_block)
                            if radius_match and length_match:
                                geometry = {
                                    'type': 'cylinder',
                                    'radius': float(radius_match.group(1)),
                                    'length': float(length_match.group(1))
                                }
                        # Check for mesh geometry
                        elif re.search(r'(type:\s*mesh)|(mesh\s*{)', geometry_block, re.IGNORECASE):
                            if re.search(r'mesh\s*{', geometry_block, re.IGNORECASE):
                                mesh_block = self.extract_block(geometry_block, "mesh {")
                                filename_match = re.search(r'filename:\s*"([^"]+)"', mesh_block)
                            else:
                                filename_match = re.search(r'filename:\s*"([^"]+)"', geometry_block)
                            if filename_match:
                                geometry = {'type': 'mesh', 'filename': filename_match.group(1)}
                    
                    # Compose the object name by combining model and visual names.
                    obj_name = f"{model_name}_{visual_name}" if visual_name else model_name
                    obj = {
                        "name": obj_name,
                        "pose": merged_pose
                    }
                    if geometry:
                        obj["geometry"] = geometry
                    objects.append(obj)
        return objects

    def create_collision_object(self, model):
        """
        Converts a model dictionary into a MoveIt2 CollisionObject.
        """
        collision_object = CollisionObject()
        collision_object.id = model.get('name')
        collision_object.header.frame_id = "world"
        collision_object.operation = CollisionObject.ADD

        # Create the pose for the object.
        pose = Pose()
        pos = model.get('pose', {}).get('position', {})
        ori = model.get('pose', {}).get('orientation', {})
        pose.position.x = pos.get('x', 0.0)
        pose.position.y = pos.get('y', 0.0)
        pose.position.z = pos.get('z', 0.0)
        pose.orientation.x = ori.get('x', 0.0)
        pose.orientation.y = ori.get('y', 0.0)
        pose.orientation.z = ori.get('z', 0.0)
        pose.orientation.w = ori.get('w', 1.0)

        # Depending on the geometry type, create either a primitive or a mesh.
        if 'geometry' in model:
            geom = model['geometry']
            if geom['type'] == 'box':
                primitive = SolidPrimitive()
                primitive.type = SolidPrimitive.BOX
                dims = geom.get('size', {})
                primitive.dimensions = [
                    dims.get('x', 1.0),
                    dims.get('y', 1.0),
                    dims.get('z', 1.0)
                ]
                collision_object.primitives.append(primitive)
                collision_object.primitive_poses.append(pose)
            elif geom['type'] == 'cylinder':
                primitive = SolidPrimitive()
                primitive.type = SolidPrimitive.CYLINDER
                # The dimensions for a cylinder are [height, radius].
                collision_object.primitives.append(primitive)
                primitive.dimensions = [
                    geom.get('length', 1.0),
                    geom.get('radius', 0.1)
                ]
                collision_object.primitive_poses.append(pose)
            elif geom['type'] == 'mesh':
                # For meshes, you would normally load the mesh file into a shape_msgs/Mesh.
                # This example uses a placeholder function.
                mesh = self.load_mesh(geom['filename'])
                if mesh is not None:
                    collision_object.meshes.append(mesh)
                    collision_object.mesh_poses.append(pose)
                else:
                    self.get_logger().warn(f"Failed to load mesh for {model.get('name')}")

        model["collision_object"] = collision_object

        return collision_object

    def load_mesh(self, filename):
        """
        Load a mesh from a given resource filename and convert it into a shape_msgs/Mesh.

        This function mimics the C++ approach using shapes::createMeshFromResource and
        shapes::constructMsgFromShape. It uses trimesh to load the mesh and then constructs
        a Mesh message that can be used in a CollisionObject.
    
        :param filename: A string representing the resource path, e.g.,
                     "package://sim_move/meshes/ring.STL"
        :return: A shape_msgs/Mesh message, or None if loading fails.
        """
        try:
            # Resolve package:// URI if necessary
            if filename.startswith("package://"):
                parts = filename[len("package://"):].split('/', 1)
                package_name = parts[0]
                relative_path = parts[1]
                package_share = get_package_share_directory(package_name)
                filepath = os.path.join(package_share, relative_path)
            else:
                filepath = filename

            # Load the mesh using trimesh
            mesh = trimesh.load(filepath, force='mesh')
            if mesh is None:
                self.get_logger().error(f"Failed to load mesh from {filepath}")
                return None

            # Create a Mesh message and populate its vertices and triangles
            mesh_msg = Mesh()

            # Fill vertices (each vertex is a geometry_msgs/Point)
            for vertex in mesh.vertices:
                pt = Point(x=float(vertex[0]), y=float(vertex[1]), z=float(vertex[2]))
                mesh_msg.vertices.append(pt)

            # Fill triangles (each face is a MeshTriangle with vertex indices)
            for face in mesh.faces:
                triangle = MeshTriangle(vertex_indices=[int(face[0]), int(face[1]), int(face[2])])
                mesh_msg.triangles.append(triangle)

            self.get_logger().info(f"Mesh loaded from: {filepath}")
            return mesh_msg

        except Exception as e:
            self.get_logger().error("Exception in load_mesh: " + str(e))
            return None

def main(args=None):
    parser = argparse.ArgumentParser(description="Gazebo Pose Updater.")
    parser.add_argument("--config", type=str, required=True,
                        help="Path to the YAML configuration file.")
    args, unknown = parser.parse_known_args()

    with open(args.config, 'r') as f:
        config = yaml.safe_load(f)

    rclpy.init(args=None)
    node = GazeboPoseUpdater(config)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

