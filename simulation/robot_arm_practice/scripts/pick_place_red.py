
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
from visualization_msgs.msg import Marker
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


WORKSPACE = Path.home() / "master_ros2_ws"
CONFIG = WORKSPACE / "src/robot_arm_practice/config"

ROBOT_POSES_FILE = CONFIG / "recorded_tf_poses.yaml"
CUBE_POSES_FILE = CONFIG / "red_cube_pose.yaml"

BASE_LINK = "base_link"
GRIPPER_LINK = "gripper_mount_link"
CUBE_ID = "red_cube"
MARKER_TOPIC = "/sorting_cube_markers"
GRIPPER_ACTION = "/gripper_controller/follow_joint_trajectory"


def pose_from_yaml(data):
    pose = Pose()

    pose.position.x = float(data["position"]["x"])
    pose.position.y = float(data["position"]["y"])
    pose.position.z = float(data["position"]["z"])

    q = data["orientation_xyzw"]
    pose.orientation.x = float(q["x"])
    pose.orientation.y = float(q["y"])
    pose.orientation.z = float(q["z"])
    pose.orientation.w = float(q["w"])

    return pose


def make_box(size):
    shape = SolidPrimitive()
    shape.type = SolidPrimitive.BOX
    shape.dimensions = list(size)
    return shape


class CubeSorting(Node):
    def __init__(self):
        super().__init__("sort_with_cube")

        with open(ROBOT_POSES_FILE) as f:
            self.robot_poses = yaml.safe_load(f)

        with open(CUBE_POSES_FILE) as f:
            self.cube_poses = yaml.safe_load(f)

        self.cube_size = None
        self.marker_received = False

        self.marker_sub = self.create_subscription(
            Marker,
            MARKER_TOPIC,
            self.marker_callback,
            10,
        )

        self.scene_pub = self.create_publisher(
            PlanningScene, "/planning_scene", 10
        )

        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            GRIPPER_ACTION,
        )

        self.arm = MoveIt2(
            node=self,
            joint_names=[
                "joint1", "joint2", "joint3", "joint4", "joint5"
            ],
            base_link_name=BASE_LINK,
            end_effector_name=GRIPPER_LINK,
            group_name="arm",
        )

    def marker_callback(self, marker):
        self.cube_marker = marker
        self.cube_size = [
            float(marker.scale.x),
            float(marker.scale.y),
            float(marker.scale.z),
        ]
        self.marker_received = True

    def wait_for_marker(self):
        self.get_logger().info(
            f"Waiting for the existing cube marker on {MARKER_TOPIC}..."
        )

        deadline = time.monotonic() + 10.0

        while rclpy.ok() and not self.marker_received:
            rclpy.spin_once(self, timeout_sec=0.2)

            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "No marker with valid dimensions received. "
                    "Check that the existing cube marker publisher is running."
                )

        self.get_logger().info(
            f"Using marker dimensions: {self.cube_size} m"
        )

    def publish_scene(self, scene):
        scene.is_diff = True
        self.scene_pub.publish(scene)

        # Give the MoveIt planning-scene monitor time to process the update.
        time.sleep(0.8)

    def clear_previous_cube_from_moveit(self):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        world_cube = CollisionObject()
        world_cube.header.frame_id = BASE_LINK
        world_cube.id = CUBE_ID
        world_cube.operation = CollisionObject.REMOVE
        scene.world.collision_objects.append(world_cube)

        attached_cube = AttachedCollisionObject()
        attached_cube.link_name = GRIPPER_LINK
        attached_cube.object.id = CUBE_ID
        attached_cube.object.operation = CollisionObject.REMOVE
        scene.robot_state.attached_collision_objects.append(attached_cube)

        self.publish_scene(scene)
        self.get_logger().info(
            "Cleared any previous red_cube collision object."
        )

    def add_cube_to_moveit(self):
        marker = getattr(self, "cube_marker", None)
        if marker is None:
            raise RuntimeError("Existing cube marker pose not received.")

        obj = CollisionObject()
        obj.header.frame_id = marker.header.frame_id or BASE_LINK
        obj.id = CUBE_ID
        obj.operation = CollisionObject.ADD
        obj.pose = marker.pose
        obj.primitives.append(make_box(self.cube_size))

        primitive_pose = Pose()
        primitive_pose.orientation.w = 1.0
        obj.primitive_poses.append(primitive_pose)

        scene = PlanningScene()
        scene.world.collision_objects.append(obj)

        color = ObjectColor()
        color.id = CUBE_ID
        color.color.r = 1.0
        color.color.g = 0.0
        color.color.b = 0.0
        color.color.a = 1.0
        scene.object_colors.append(color)

        self.publish_scene(scene)
        self.get_logger().info(
            f"Registered collision cube at marker pose in "
            f"frame '{obj.header.frame_id}'."
        )

    def move_to(self, name):
        data = self.robot_poses[name]
        p = data["position"]
        q = data["orientation_xyzw"]

        target = {
            "position": [p["x"], p["y"], p["z"]],
            "quat_xyzw": [q["x"], q["y"], q["z"], q["w"]],
            "frame_id": data.get("frame_id", BASE_LINK),
            "target_link": GRIPPER_LINK,
            "tolerance_position": 0.02,
            "tolerance_orientation": 3.14,
            "timeout_sec": 15.0,
        }

        self.get_logger().info(f"Moving arm to saved pose: {name}")

        planned = False
        for attempt in range(1, 4):
            planned = self.arm.move_to_pose(**target)
            if planned is not False:
                break
            if attempt < 3:
                self.get_logger().warning(
                    f"Planning to '{name}' failed, attempt {attempt}/3; retrying."
                )
                time.sleep(0.5)

        if planned is False:
            raise RuntimeError(
                f"MoveIt could not plan to '{name}' after 3 attempts."
            )

        completed = self.arm.wait_until_executed(timeout_sec=20.0)
        if not completed:
            raise RuntimeError(f"Motion timed out at pose '{name}'.")

        self.get_logger().info(f"Completed pose: {name}")

    def move_gripper(self, position):
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(
                f"Gripper action server unavailable: {GRIPPER_ACTION}"
            )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["joint6"]

        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start = Duration(sec=2, nanosec=0)
        goal.trajectory.points = [point]

        self.get_logger().info(
            f"Commanding gripper joint6 to {position:.3f} rad..."
        )

        send_future = self.gripper_client.send_goal_async(goal)

        deadline = time.monotonic() + 5.0
        while rclpy.ok() and not send_future.done() and time.monotonic() < deadline:
            time.sleep(0.05)

        if not send_future.done():
            raise RuntimeError("Timed out sending gripper goal.")

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Gripper action goal was rejected.")

        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + 8.0
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            time.sleep(0.05)

        if not result_future.done():
            goal_handle.cancel_goal_async()
            raise RuntimeError("Gripper action timed out.")

        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"Gripper action failed with error code {result.error_code}."
            )

        self.get_logger().info("Gripper action succeeded.")

    def attach_cube(self):
        attached = AttachedCollisionObject()
        attached.link_name = GRIPPER_LINK
        attached.object.header.frame_id = GRIPPER_LINK
        attached.object.id = CUBE_ID
        attached.object.operation = CollisionObject.ADD

        # Preserve the recorded cube-to-gripper transform.
        relative_pose = pose_from_yaml(
            self.cube_poses["gripper_relative"]
        )

        attached.object.primitives.append(make_box(self.cube_size))
        attached.object.primitive_poses.append(relative_pose)

        # Avoid guessing unverified finger-link names.
        attached.touch_links = [GRIPPER_LINK]

        remove = CollisionObject()
        remove.header.frame_id = BASE_LINK
        remove.id = CUBE_ID
        remove.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.world.collision_objects.append(remove)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)

        self.publish_scene(scene)
        self.get_logger().info("Requested cube attachment to the gripper.")

    def release_cube_at_bin(self):
        detach = AttachedCollisionObject()
        detach.link_name = GRIPPER_LINK
        detach.object.id = CUBE_ID
        detach.object.operation = CollisionObject.REMOVE

        target_pose = pose_from_yaml(
            self.cube_poses["pose6_released"]
        )

        obj = CollisionObject()
        obj.header.frame_id = BASE_LINK
        obj.id = CUBE_ID
        obj.operation = CollisionObject.ADD
        obj.pose = target_pose
        obj.primitives.append(make_box(self.cube_size))

        primitive_pose = Pose()
        primitive_pose.orientation.w = 1.0
        obj.primitive_poses.append(primitive_pose)

        scene = PlanningScene()
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(detach)
        scene.world.collision_objects.append(obj)

        color = ObjectColor()
        color.id = CUBE_ID
        color.color.r = 1.0
        color.color.a = 1.0
        scene.object_colors.append(color)

        self.publish_scene(scene)
        self.get_logger().info(
            "Requested cube release at saved pose6_released."
        )

    def run_sort(self):
        # Wait for the existing cube marker and read its dimensions.
        self.wait_for_marker()

        # Remove stale collision state, then immediately restore the cube.
        self.clear_previous_cube_from_moveit()
        self.add_cube_to_moveit()

        # Begin sorting directly; do not require an initial home motion.
        self.move_gripper(0.0)
        self.move_to("pose2_pre_grasp")
        self.move_to("pose3_grasp")

        # Close the gripper and attach the cube in MoveIt.
        self.move_gripper(0.035)
        self.attach_cube()

        # Lift and transport the cube to its red-bin release pose.
        self.move_to("pose4_lift")
        self.move_to("pose5_red_bin_pre_release")
        self.move_to("pose6_red_bin_release")

        # Open the gripper and register the released cube.
        self.move_gripper(0.0)
        self.release_cube_at_bin()

        # Retreat and return home after releasing the cube.
        self.move_to("pose7_retreat")
        self.move_to("pose8_home_return")

        self.get_logger().info("Sorting sequence finished.")

def main():
    rclpy.init()
    node = CubeSorting()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        input(
            "Start demo.launch.py and keep RViz open. "
            "Press ENTER to start, or Ctrl+C to cancel: "
        )
        node.run_sort()
    except (KeyboardInterrupt, RuntimeError, KeyError, OSError) as exc:
        node.get_logger().error(str(exc))
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
