import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import xarm6  # pre-built config for xArm6!
import asyncio

def main():
    rclpy.init()                          # Start ROS2
    node = Node("move_xarm")             # Create a node (a named program)

    # Create the MoveIt2 interface for xArm6
    moveit2 = MoveIt2(
        node=node,
        joint_names=xarm6.joint_names(),          # The 6 joint names
        base_link_name=xarm6.base_link_name(),    # The fixed base
        end_effector_name=xarm6.end_effector(),   # The tip of the arm
        group_name=xarm6.MOVE_GROUP_ARM,          # The planning group
    )

    # Move to a position: x, y, z in meters, from the base of the arm
    moveit2.move_to_pose(
        position=[0.3, 0.0, 0.3],        # 30cm forward, centered, 30cm up
        quat_xyzw=[0.0, 0.0, 0.0, 1.0], # Orientation (straight down — no rotation)
        cartesian=False                   # Plan freely, not in a straight line
    )
    moveit2.wait_until_executed()         # Wait for the move to finish

    rclpy.shutdown()                      # Clean up ROS2

if __name__ == "__main__":
    main()