#!/usr/bin/env python3
import rclpy
import time
import threading
from rclpy.callback_groups import ReentrantCallbackGroup
from pymoveit2 import MoveIt2
from geometry_msgs.msg import Point
from tf_transformations import quaternion_from_euler
import math

# Exactly what /joint_states publishes — order and names must match
JOINT_NAMES  = ["joint2", "joint3", "joint1", "joint4", "joint5", "joint6"]
BASE_LINK    = "link_base"
END_EFFECTOR = "link_eef"
GROUP_NAME   = "xarm6"

def main():
    rclpy.init()
    node = rclpy.create_node("xarm_controller")
    callback_group = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=JOINT_NAMES,
        base_link_name=BASE_LINK,
        end_effector_name=END_EFFECTOR,
        group_name=GROUP_NAME,
        callback_group=callback_group,
    )
    moveit2.planning_time = 10.0
    moveit2.max_velocity = 0.3
    moveit2.max_acceleration = 0.3

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Wait for joint states to be available before doing anything
    node.get_logger().info("Waiting for joint states...")
    time.sleep(2.0)

    # ── Test 1: move to home (all zeros) ──────────────────────────────────
    node.get_logger().info("Moving to HOME...")
    #                       j2    j3    j1    j4    j5    j6
    moveit2.move_to_configuration([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    moveit2.wait_until_executed()
    time.sleep(1.5)

    # ── Test 2: move to hold-up ───────────────────────────────────────────
    node.get_logger().info("Moving to HOLD-UP...")
    #                       j2    j3    j1    j4      j5      j6
    moveit2.move_to_configuration([0.0, 0.0, 0.0, 0.0, -1.5708, 0.0])
    moveit2.wait_until_executed()
    time.sleep(1.5)

    # ── Test 3: Cartesian pose ────────────────────────────────────────────
    node.get_logger().info("Moving to Cartesian pose...")
    q = quaternion_from_euler(math.pi, 0.0, 0.0)
    moveit2.move_to_pose(
        position=Point(x=0.3, y=0.1, z=0.4),
        quat_xyzw=[q[0], q[1], q[2], q[3]],
    )
    moveit2.wait_until_executed()

    node.get_logger().info("All motions complete!")
    rclpy.shutdown()
    executor_thread.join()

if __name__ == "__main__":
    main()