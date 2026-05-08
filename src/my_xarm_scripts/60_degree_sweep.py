#!/usr/bin/env python3
import rclpy
import time
import threading
from rclpy.callback_groups import ReentrantCallbackGroup
from pymoveit2 import MoveIt2
from geometry_msgs.msg import Point
from tf_transformations import quaternion_from_euler
import math

# My arm simulated in gazebo is in this order, 2-3-1-4-5-6, so I have to tell MoveIt to use this order for the joints
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

    # Wait for joint states to load
    node.get_logger().info("Waiting for joint states...")
    time.sleep(2.0)

    # Move to origin
    node.get_logger().info("Moving to origin")
    #                             j2    j3    j1    j4    j5    j6
    moveit2.move_to_configuration([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    moveit2.wait_until_executed()
    time.sleep(1.5)
    
    #angle sweep
    #node.get_logger().info("angle sweep")
    #for angle in range(-180, 180, 10):
       #rad = math.radians(angle)
        #xpath = 0.2 + 1 - math.cos(rad)
        #ypath = 0.2 + math.sin(rad)
        #theta = math.radians(angle)
        #q = quaternion_from_euler(theta, 0, 0)
        #oveit2.move_to_pose(
            #position=Point(x=0.3, y=0.3, z=0.3),
            #quat_xyzw=[q[0], q[1], q[2], q[3]],
        #)
        #moveit2.wait_until_executed()
    
    #draw a circular path
    node.get_logger().info("Arc Path")

    #Sweep
    for angle in range(-30, 30, 5):
        rad = math.radians(angle)
        xpath = 0.3 + 0.5*(1-math.cos(rad))
        ypath = 0.5*math.sin(rad)
        theta = -math.radians(angle)
        q = quaternion_from_euler(math.pi/2, 0, theta+math.pi/2)
        moveit2.move_to_pose(
            position=Point(x=xpath, y=ypath, z=0.2),
            quat_xyzw=[q[0], q[1], q[2], q[3]],
        )
        moveit2.wait_until_executed()
        time.sleep(0.5)
        node.get_logger().info(f"Pose at angle {angle} degrees: x={xpath:.3f}, y={ypath:.3f}, theta={theta:.3f} radians")   


    node.get_logger().info("All motions complete!")
    rclpy.shutdown()
    executor_thread.join()

if __name__ == "__main__":
    main()