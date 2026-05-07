#!/usr/bin/env python3
"""
UFactory xArm6 controller via MoveIt 2 Python API (ROS 2)
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit.planning import MoveItPy, PlanningComponent
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import Pose, Point, Quaternion
from tf_transformations import quaternion_from_euler
import math


class XArmController(Node):
    def __init__(self):
        super().__init__('xarm_controller')

        # Initialize MoveItPy — matches your MoveIt config group name
        self.moveit = MoveItPy(node_name="xarm_moveit_py")
        self.arm = self.moveit.get_planning_component("xarm6")  # your planning group
        self.robot_model = self.moveit.get_robot_model()

        self.get_logger().info("XArm Controller Ready")

    # ── Joint-space movement ──────────────────────────────────────────────────
    def move_to_joint_positions(self, joint_angles_deg: list[float]) -> bool:
        """
        Move arm to specific joint angles.
        joint_angles_deg: list of 6 angles in DEGREES [j1, j2, j3, j4, j5, j6]
        """
        joint_angles_rad = [math.radians(a) for a in joint_angles_deg]

        # Build a RobotState with the target joints
        robot_state = RobotState(self.robot_model)
        robot_state.set_joint_group_positions("xarm6", joint_angles_rad)

        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(robot_state=robot_state)

        return self._plan_and_execute()

    # ── Cartesian / pose-space movement ──────────────────────────────────────
    def move_to_pose(self, x: float, y: float, z: float,
                     roll=0.0, pitch=0.0, yaw=0.0) -> bool:
        """
        Move the end-effector to (x, y, z) in meters with RPY orientation in radians.
        """
        q = quaternion_from_euler(roll, pitch, yaw)

        target_pose = Pose(
            position=Point(x=x, y=y, z=z),
            orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        )

        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(
            configuration_name=None,
            motion_plan_constraints=None,
            pose_stamped_msg=target_pose,
            pose_link="link_eef"          # your end-effector link name
        )

        return self._plan_and_execute()

    # ── Named configuration (e.g. "home", "zero") ────────────────────────────
    def move_to_named_config(self, config_name: str) -> bool:
        """
        Move to a pre-defined named configuration from your SRDF.
        Common names: 'home', 'zero_point'
        """
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name=config_name)
        return self._plan_and_execute()

    # ── Cartesian path (straight-line waypoints) ─────────────────────────────
    def move_cartesian_path(self, waypoints: list[Pose],
                             step_size=0.01, avoid_collisions=True) -> bool:
        """
        Move through a list of Pose waypoints in a straight Cartesian path.
        step_size: interpolation resolution in meters
        Returns True if >95% of path was executed.
        """
        self.arm.set_start_state_to_current_state()

        path, fraction = self.arm.compute_cartesian_path(
            waypoints=waypoints,
            max_step=step_size,
            avoid_collisions=avoid_collisions
        )

        self.get_logger().info(f"Cartesian path coverage: {fraction*100:.1f}%")

        if fraction < 0.95:
            self.get_logger().warn("Path coverage below 95% — aborting")
            return False

        return self.moveit.execute(path, controllers=[])

    # ── Internal: plan + execute ──────────────────────────────────────────────
    def _plan_and_execute(self) -> bool:
        plan_result = self.arm.plan()

        if not plan_result:
            self.get_logger().error("Planning FAILED")
            return False

        self.get_logger().info("Plan succeeded — executing...")
        execute_result = self.moveit.execute(plan_result.trajectory, controllers=[])

        if not execute_result:
            self.get_logger().error("Execution FAILED")
            return False

        self.get_logger().info("Motion complete ✓")
        return True
    
def main(args=None):
    rclpy.init(args=args)
    controller = XArmController()

    try:
        # 1. Go to named home position
        controller.move_to_named_config("home")

        # 2. Move joints directly (degrees)
        controller.move_to_joint_positions([0, -45, 0, 45, 0, 0])

        # 3. Move end-effector to Cartesian pose (meters + radians)
        controller.move_to_pose(
            x=0.4, y=0.0, z=0.3,
            roll=0.0, pitch=math.pi/2, yaw=0.0
        )

        # 4. Execute a Cartesian waypoint path (straight lines)
        waypoints = []
        for i in range(5):
            p = Pose()
            p.position.x = 0.4
            p.position.y = -0.1 + i * 0.05   # sweep in Y
            p.position.z = 0.3
            q = quaternion_from_euler(0, math.pi/2, 0)
            p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            waypoints.append(p)

        controller.move_cartesian_path(waypoints)

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()