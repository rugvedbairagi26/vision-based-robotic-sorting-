
#!/usr/bin/env python3

import time
import threading
from pathlib import Path

import rclpy
import yaml

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration

from moveit_msgs.msg import (
    PlanningScene,
    CollisionObject,
    AttachedCollisionObject,
    ObjectColor,
)

from pymoveit2 import MoveIt2


# =========================================================
# CONFIGURATION
# =========================================================

WORKSPACE = Path.home() / "master_ros2_ws"
CONFIG = WORKSPACE / "src/robot_arm_practice/config"

ROBOT_POSES_FILE = CONFIG / "recorded_tf_poses_green.yaml"
CUBE_POSES_FILE = CONFIG / "green_cube_poses.yaml"

BASE_LINK = "base_link"
GRIPPER_LINK = "gripper_mount_link"

CUBE_ID = "green_cube"
GRIPPER_ACTION = "/gripper_controller/follow_joint_trajectory"


# =========================================================
# HELPER FUNCTIONS
# =========================================================

def make_box(size):
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = list(size)
    return box


def cube_pose_from_yaml(data):
    position = data.get("position", data.get("translation"))
    orientation = data.get(
        "orientation_xyzw",
        data.get("rotation"),
    )

    if position is None or orientation is None:
        raise ValueError(
            "Cube pose is missing position or orientation."
        )

    pose = Pose()

    pose.position.x = float(position["x"])
    pose.position.y = float(position["y"])
    pose.position.z = float(position["z"])

    pose.orientation.x = float(orientation["x"])
    pose.orientation.y = float(orientation["y"])
    pose.orientation.z = float(orientation["z"])
    pose.orientation.w = float(orientation["w"])

    return pose


def robot_pose_from_yaml(data):
    position = data.get("position", data.get("translation"))
    orientation = data.get(
        "orientation_xyzw",
        data.get("rotation"),
    )

    if position is None or orientation is None:
        raise ValueError(
            "Robot pose is missing position or orientation."
        )

    xyz = [
        float(position["x"]),
        float(position["y"]),
        float(position["z"]),
    ]

    quaternion = [
        float(orientation["x"]),
        float(orientation["y"]),
        float(orientation["z"]),
        float(orientation["w"]),
    ]

    frame_id = data.get("frame_id", BASE_LINK)

    return xyz, quaternion, frame_id


# =========================================================
# GREEN CUBE SORTING NODE
# =========================================================

class GreenCubeSorting(Node):

    def __init__(self):
        super().__init__("pick_place_green")

        # Check that both YAML files exist.
        if not ROBOT_POSES_FILE.is_file():
            raise FileNotFoundError(
                f"Robot pose file not found: {ROBOT_POSES_FILE}"
            )

        if not CUBE_POSES_FILE.is_file():
            raise FileNotFoundError(
                f"Cube pose file not found: {CUBE_POSES_FILE}"
            )

        # Load the recorded poses.
        with open(ROBOT_POSES_FILE, "r") as file:
            self.robot_poses = yaml.safe_load(file)

        with open(CUBE_POSES_FILE, "r") as file:
            self.cube_poses = yaml.safe_load(file)

        # Read the cube dimensions and recorded poses.
        initial = self.cube_poses["initial"]

        self.cube_size = [
            float(initial["dimensions"]["x"]),
            float(initial["dimensions"]["y"]),
            float(initial["dimensions"]["z"]),
        ]

        self.initial_cube_pose = cube_pose_from_yaml(initial)

        self.relative_cube_pose = cube_pose_from_yaml(
            self.cube_poses["gripper_relative"]
        )

        self.final_cube_pose = cube_pose_from_yaml(
            self.cube_poses["green_bin_final"]
        )

        # Planning scene publisher.
        self.scene_pub = self.create_publisher(
            PlanningScene,
            "/planning_scene",
            10,
        )

        # Gripper action client.
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            GRIPPER_ACTION,
        )

        # MoveIt arm configuration.
        self.arm = MoveIt2(
            node=self,
            joint_names=[
                "joint1",
                "joint2",
                "joint3",
                "joint4",
                "joint5",
            ],
            base_link_name=BASE_LINK,
            end_effector_name=GRIPPER_LINK,
            group_name="arm",
        )

        self.get_logger().info("Green cube configuration loaded.")
        self.get_logger().info(
            f"Robot poses: {ROBOT_POSES_FILE}"
        )
        self.get_logger().info(
            f"Cube poses: {CUBE_POSES_FILE}"
        )

    # -----------------------------------------------------
    # PLANNING SCENE
    # -----------------------------------------------------

    def publish_scene(self, scene):
        scene.is_diff = True
        self.scene_pub.publish(scene)

        # Allow time for the planning-scene update to propagate.
        time.sleep(0.8)

    def clear_previous_cube(self):
        scene = PlanningScene()
        scene.robot_state.is_diff = True

        # Remove the cube from the world.
        world_cube = CollisionObject()
        world_cube.header.frame_id = BASE_LINK
        world_cube.id = CUBE_ID
        world_cube.operation = CollisionObject.REMOVE

        scene.world.collision_objects.append(world_cube)

        # Remove any previously attached copy.
        attached_cube = AttachedCollisionObject()
        attached_cube.link_name = GRIPPER_LINK
        attached_cube.object.id = CUBE_ID
        attached_cube.object.operation = CollisionObject.REMOVE

        scene.robot_state.attached_collision_objects.append(
            attached_cube
        )

        self.publish_scene(scene)

        self.get_logger().info("Cleared previous green cube state.")

    def add_cube_to_scene(self):
        cube = CollisionObject()
        cube.header.frame_id = BASE_LINK
        cube.id = CUBE_ID
        cube.operation = CollisionObject.ADD
        cube.pose = self.initial_cube_pose

        cube.primitives.append(make_box(self.cube_size))

        primitive_pose = Pose()
        primitive_pose.orientation.w = 1.0
        cube.primitive_poses.append(primitive_pose)

        scene = PlanningScene()
        scene.world.collision_objects.append(cube)

        color = ObjectColor()
        color.id = CUBE_ID
        color.color.r = 0.0
        color.color.g = 1.0
        color.color.b = 0.0
        color.color.a = 1.0

        scene.object_colors.append(color)

        self.publish_scene(scene)

        self.get_logger().info(
            "Added green_cube to MoveIt's planning scene."
        )

    # -----------------------------------------------------
    # ARM MOVEMENT USING RECORDED YAML POSES
    # -----------------------------------------------------

    def move_to(self, name):
        if name not in self.robot_poses:
            raise KeyError(
                f"Missing pose '{name}' in "
                f"{ROBOT_POSES_FILE.name}."
            )

        # Use the exact position and quaternion from the YAML.
        position, quaternion, frame_id = robot_pose_from_yaml(
            self.robot_poses[name]
        )

        target = {
            "position": position,
            "quat_xyzw": quaternion,
            "frame_id": frame_id,
            "target_link": GRIPPER_LINK,
            "tolerance_position": 0.02,
            "tolerance_orientation": 3.14,
            "timeout_sec": 15.0,
        }

        self.get_logger().info(
            f"Planning movement to '{name}'..."
        )

        for attempt in range(1, 4):
            planned = self.arm.move_to_pose(**target)

            if planned is not False:
                break

            self.get_logger().warning(
                f"Planning failed for '{name}', "
                f"attempt {attempt}/3."
            )

            time.sleep(0.5)

        else:
            raise RuntimeError(
                f"MoveIt could not plan to '{name}'."
            )

        completed = self.arm.wait_until_executed(
            timeout_sec=20.0
        )

        if not completed:
            raise RuntimeError(
                f"Motion timed out at '{name}'."
            )

        self.get_logger().info(
            f"Completed pose '{name}'."
        )

    # -----------------------------------------------------
    # RETURN HOME DIRECTLY AFTER RELEASE
    # -----------------------------------------------------

    def retreat_and_return_home(self):
        """
        Skip the failing green_retreat and green_lift targets.
        Attempt to move directly to the recorded home pose.
        """

        self.get_logger().info(
            "Cube released. Moving directly to recorded home pose."
        )

        self.move_to("home")

    # -----------------------------------------------------
    # GRIPPER CONTROL
    # -----------------------------------------------------

    def move_gripper(self, position):
        if not self.gripper_client.wait_for_server(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                f"Gripper action server unavailable: "
                f"{GRIPPER_ACTION}"
            )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["joint6"]

        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start = Duration(
            sec=2,
            nanosec=0,
        )

        goal.trajectory.points = [point]

        self.get_logger().info(
            f"Commanding gripper to {position:.3f}."
        )

        send_future = self.gripper_client.send_goal_async(goal)
        deadline = time.monotonic() + 5.0

        while (
            rclpy.ok()
            and not send_future.done()
            and time.monotonic() < deadline
        ):
            time.sleep(0.05)

        if not send_future.done():
            raise RuntimeError(
                "Timed out sending gripper goal."
            )

        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Gripper goal was rejected.")

        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + 10.0

        while (
            rclpy.ok()
            and not result_future.done()
            and time.monotonic() < deadline
        ):
            time.sleep(0.05)

        if not result_future.done():
            goal_handle.cancel_goal_async()

            raise RuntimeError(
                "Gripper execution timed out."
            )

        result = result_future.result().result

        if (
            result.error_code
            != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            raise RuntimeError(
                f"Gripper failed with error code "
                f"{result.error_code}."
            )

        self.get_logger().info("Gripper action succeeded.")

    # -----------------------------------------------------
    # ATTACH CUBE TO GRIPPER
    # -----------------------------------------------------

    def attach_cube(self):
        attached = AttachedCollisionObject()
        attached.link_name = GRIPPER_LINK

        attached.object.header.frame_id = GRIPPER_LINK
        attached.object.id = CUBE_ID
        attached.object.operation = CollisionObject.ADD

        attached.object.primitives.append(
            make_box(self.cube_size)
        )

        attached.object.primitive_poses.append(
            self.relative_cube_pose
        )

        attached.touch_links = [GRIPPER_LINK]

        # Remove the cube from the world.
        remove = CollisionObject()
        remove.header.frame_id = BASE_LINK
        remove.id = CUBE_ID
        remove.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.robot_state.is_diff = True

        scene.world.collision_objects.append(remove)

        scene.robot_state.attached_collision_objects.append(
            attached
        )

        self.publish_scene(scene)

        self.get_logger().info(
            "Requested attachment of green_cube."
        )

    # -----------------------------------------------------
    # RELEASE CUBE AT GREEN BIN
    # -----------------------------------------------------

    def release_cube_at_bin(self):
        # Detach the cube from the gripper.
        detach = AttachedCollisionObject()
        detach.link_name = GRIPPER_LINK
        detach.object.id = CUBE_ID
        detach.object.operation = CollisionObject.REMOVE

        # Add the cube at the recorded final bin pose.
        cube = CollisionObject()
        cube.header.frame_id = BASE_LINK
        cube.id = CUBE_ID
        cube.operation = CollisionObject.ADD
        cube.pose = self.final_cube_pose

        cube.primitives.append(make_box(self.cube_size))

        primitive_pose = Pose()
        primitive_pose.orientation.w = 1.0
        cube.primitive_poses.append(primitive_pose)

        scene = PlanningScene()
        scene.robot_state.is_diff = True

        scene.robot_state.attached_collision_objects.append(
            detach
        )

        scene.world.collision_objects.append(cube)

        color = ObjectColor()
        color.id = CUBE_ID
        color.color.r = 0.0
        color.color.g = 1.0
        color.color.b = 0.0
        color.color.a = 1.0

        scene.object_colors.append(color)

        self.publish_scene(scene)

        self.get_logger().info(
            "Requested release at green_bin_final."
        )

    # -----------------------------------------------------
    # COMPLETE PICK-AND-PLACE SEQUENCE
    # -----------------------------------------------------

    def run_sort(self):
        # Reset scene and add the green cube.
        self.clear_previous_cube()
        self.add_cube_to_scene()

        # Open gripper.
        self.move_gripper(0.0)

        # Approach and grasp.
        self.move_to("green_pre_grasp")
        self.move_to("green_grasp")

        # Close gripper and attach cube.
        self.move_gripper(0.035)
        self.attach_cube()

        # Lift and move to the bin.
        self.move_to("green_lift")
        self.move_to("green_container_pre_release")
        self.move_to("green_release")

        # Open gripper and release cube.
        self.move_gripper(0.0)
        self.release_cube_at_bin()

        # Skip green_retreat and return directly home.
        self.retreat_and_return_home()

        self.get_logger().info(
            "Green cube pick-and-place sequence finished."
        )


# =========================================================
# MAIN FUNCTION
# =========================================================

def main():
    rclpy.init()

    node = None
    executor = MultiThreadedExecutor()
    spin_thread = None

    try:
        node = GreenCubeSorting()

        executor.add_node(node)

        spin_thread = threading.Thread(
            target=executor.spin,
            daemon=True,
        )
        spin_thread.start()

        input(
            "Keep demo.launch.py running with RViz open. "
            "Press ENTER to start, or Ctrl+C to cancel: "
        )

        node.run_sort()

    except (
        KeyboardInterrupt,
        RuntimeError,
        KeyError,
        OSError,
        ValueError,
    ) as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f"Green cube sequence stopped: {error}")

    finally:
        executor.shutdown()

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        if spin_thread is not None:
            spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
