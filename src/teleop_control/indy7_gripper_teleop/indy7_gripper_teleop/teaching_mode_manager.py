#!/usr/bin/env python3
"""Teaching Mode Manager for Indy7 Direct Teaching"""

import rclpy
from rclpy.node import Node
from indy_interfaces.srv import IndyService


# Message codes from indy_define.py
MSG_RECOVER = 1
MSG_MOVE_HOME = 2
MSG_MOVE_ZERO = 3
MSG_TELE_TASK_ABS = 4
MSG_TELE_TASK_RLT = 5
MSG_TELE_JOINT_ABS = 6
MSG_TELE_JOINT_RLT = 7
MSG_TELE_STOP = 8
MSG_DIRECT_TEACHING_ON = 9
MSG_DIRECT_TEACHING_OFF = 10

# Operational states from indy_define.py
OP_SYSTEM_OFF = 0
OP_SYSTEM_ON = 1
OP_VIOLATE = 2
OP_RECOVER_HARD = 3
OP_RECOVER_SOFT = 4
OP_IDLE = 5
OP_MOVING = 6
OP_TEACHING = 7
OP_COLLISION = 8
OP_STOP_AND_OFF = 9
OP_COMPLIANCE = 10


class TeachingModeManager:
    """
    Manages robot teaching mode activation/deactivation.

    This class provides an interface to enable gravity compensation mode
    on the Indy7 robot, allowing the user to physically move the arm.
    """

    def __init__(self, node: Node):
        """
        Initialize the teaching mode manager.

        Args:
            node: ROS2 node for creating service clients
        """
        self.node = node
        self.logger = node.get_logger()

        # Create service client for indy_srv
        self.indy_service_client = node.create_client(IndyService, 'indy_srv')

        # Wait for service to be available
        self.logger.info("Waiting for indy_srv service...")
        while not self.indy_service_client.wait_for_service(timeout_sec=1.0):
            self.logger.info("indy_srv service not available, waiting...")

        self.logger.info("indy_srv service connected!")
        self.is_teaching_enabled = False

    def enable_teaching_mode(self) -> bool:
        """
        Enable direct teaching mode on the robot.

        This will enable gravity compensation/compliance mode, allowing
        the user to physically move the robot arm.

        Uses the Neuromeka SDK's set_direct_teaching(True) method via
        the MSG_DIRECT_TEACHING_ON service command.

        Returns:
            bool: True if teaching mode was successfully enabled
        """
        self.logger.info("Enabling direct teaching mode...")

        # Call the direct teaching service
        if not self._call_indy_service(MSG_DIRECT_TEACHING_ON):
            self.logger.error("Failed to enable direct teaching mode")
            return False

        self.is_teaching_enabled = True
        self.logger.info("Direct teaching mode enabled! Robot is now manually movable.")

        return True

    def disable_teaching_mode(self) -> bool:
        """
        Disable teaching mode and return robot to normal operation.

        Returns:
            bool: True if teaching mode was successfully disabled
        """
        if not self.is_teaching_enabled:
            self.logger.warn("Teaching mode is not enabled")
            return True

        self.logger.info("Disabling direct teaching mode...")

        # Call the direct teaching off service
        if not self._call_indy_service(MSG_DIRECT_TEACHING_OFF):
            self.logger.error("Failed to disable direct teaching mode")
            return False

        self.is_teaching_enabled = False
        self.logger.info("Direct teaching mode disabled. Robot returned to normal operation.")

        return True

    def recover_robot(self) -> bool:
        """
        Recover robot from error state.

        Returns:
            bool: True if recovery was successful
        """
        self.logger.info("Recovering robot from error state...")

        success = self._call_indy_service(MSG_RECOVER)

        if success:
            self.logger.info("Robot recovered successfully")
        else:
            self.logger.error("Failed to recover robot")

        return success

    def move_to_home(self) -> bool:
        """
        Move robot to home position.

        Returns:
            bool: True if move was successful
        """
        self.logger.info("Moving robot to home position...")

        success = self._call_indy_service(MSG_MOVE_HOME)

        if success:
            self.logger.info("Robot moved to home position")
        else:
            self.logger.error("Failed to move to home position")

        return success

    def _call_indy_service(self, command: int, timeout_sec: float = 5.0) -> bool:
        """
        Call the indy_srv service with a command.

        Args:
            command: Message code from indy_define.py
            timeout_sec: Service call timeout in seconds

        Returns:
            bool: True if service call succeeded
        """
        request = IndyService.Request()
        request.data = command

        try:
            future = self.indy_service_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)

            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.logger.debug(f"Service call succeeded: {response.message}")
                    return True
                else:
                    self.logger.error(f"Service call failed: {response.message}")
                    return False
            else:
                self.logger.error("Service call timed out or failed")
                return False

        except Exception as e:
            self.logger.error(f"Exception during service call: {str(e)}")
            return False

    def is_enabled(self) -> bool:
        """
        Check if teaching mode is currently enabled.

        Returns:
            bool: True if teaching mode is enabled
        """
        return self.is_teaching_enabled
