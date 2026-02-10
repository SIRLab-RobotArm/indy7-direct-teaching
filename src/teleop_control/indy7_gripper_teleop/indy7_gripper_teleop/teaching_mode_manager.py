#!/usr/bin/env python3
"""Teaching Mode Manager for Indy7 Direct Teaching"""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
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
MSG_MOVE_SAFE = 11  # Move joints near limits to safe positions

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

# State names for display
OP_STATE_NAMES = {
    OP_SYSTEM_OFF: "SYSTEM_OFF",
    OP_SYSTEM_ON: "SYSTEM_ON",
    OP_VIOLATE: "VIOLATE (Error)",
    OP_RECOVER_HARD: "RECOVER_HARD",
    OP_RECOVER_SOFT: "RECOVER_SOFT",
    OP_IDLE: "IDLE (Ready)",
    OP_MOVING: "MOVING",
    OP_TEACHING: "TEACHING (Direct Teaching Active)",
    OP_COLLISION: "COLLISION (Error)",
    OP_STOP_AND_OFF: "STOP_AND_OFF (Error)",
    OP_COMPLIANCE: "COMPLIANCE",
}

# Error states that require recovery
ERROR_STATES = [OP_VIOLATE, OP_COLLISION, OP_STOP_AND_OFF]


class TeachingModeManager:
    """
    Manages robot teaching mode activation/deactivation.

    This class provides an interface to enable gravity compensation mode
    on the Indy7 robot, allowing the user to physically move the arm.
    """

    # Joint limits for Indy7 (degrees)
    JOINT_LIMITS_DEG = [
        (-175.0, 175.0),  # Joint 0
        (-175.0, 175.0),  # Joint 1
        (-175.0, 175.0),  # Joint 2
        (-175.0, 175.0),  # Joint 3
        (-175.0, 175.0),  # Joint 4
        (-175.0, 175.0),  # Joint 5
    ]

    # Warning threshold (degrees from limit)
    JOINT_LIMIT_WARNING_THRESHOLD = 15.0

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

        # Subscribe to joint states for monitoring
        self.last_joint_state = None
        self.last_joint_state_time = None
        self.joint_state_sub = node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )

        # Error state tracking
        self.is_in_error_state = False
        self.last_error_message = ""

    def _joint_state_callback(self, msg: JointState):
        """Store the latest joint state for monitoring."""
        self.last_joint_state = msg
        self.last_joint_state_time = time.time()

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

    def move_to_safe(self) -> bool:
        """
        Move joints near limits to safe positions.

        This is more efficient than move_to_home() because it only moves
        the joints that are actually near their limits, keeping other
        joints at their current positions.

        Returns:
            bool: True if move was successful
        """
        self.logger.info("Moving joints near limits to safe positions...")

        # Use longer timeout for movement command
        success = self._call_indy_service(MSG_MOVE_SAFE, timeout_sec=15.0)

        if success:
            self.logger.info("Joints moved to safe positions")
        else:
            self.logger.error("Failed to move joints to safe positions")

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

    def get_robot_status(self) -> dict:
        """
        Get comprehensive robot status information.

        Returns:
            dict: Robot status including joint positions, limits, and warnings
        """
        import math

        status = {
            'connected': False,
            'teaching_enabled': self.is_teaching_enabled,
            'in_error_state': self.is_in_error_state,
            'last_error': self.last_error_message,
            'joint_positions_deg': [],
            'joint_velocities': [],
            'joints_near_limit': [],
            'data_age_sec': None,
        }

        if self.last_joint_state is None:
            return status

        status['connected'] = True

        if self.last_joint_state_time:
            status['data_age_sec'] = time.time() - self.last_joint_state_time

        # Convert positions from radians to degrees
        positions_deg = [math.degrees(p) for p in self.last_joint_state.position]
        status['joint_positions_deg'] = positions_deg
        status['joint_velocities'] = list(self.last_joint_state.velocity)

        # Check which joints are near limits
        for i, pos_deg in enumerate(positions_deg):
            if i < len(self.JOINT_LIMITS_DEG):
                min_limit, max_limit = self.JOINT_LIMITS_DEG[i]
                distance_to_min = pos_deg - min_limit
                distance_to_max = max_limit - pos_deg

                if distance_to_min < self.JOINT_LIMIT_WARNING_THRESHOLD:
                    status['joints_near_limit'].append({
                        'joint': i,
                        'position': pos_deg,
                        'limit': min_limit,
                        'distance': distance_to_min,
                        'side': 'min'
                    })
                elif distance_to_max < self.JOINT_LIMIT_WARNING_THRESHOLD:
                    status['joints_near_limit'].append({
                        'joint': i,
                        'position': pos_deg,
                        'limit': max_limit,
                        'distance': distance_to_max,
                        'side': 'max'
                    })

        return status

    def print_status(self):
        """Print formatted robot status to console."""
        status = self.get_robot_status()

        print("\n" + "="*60)
        print("  ROBOT STATUS")
        print("="*60)

        # Connection status
        if status['connected']:
            print(f"  Connection: ✓ Connected")
            if status['data_age_sec'] is not None:
                print(f"  Data Age: {status['data_age_sec']:.2f} seconds")
        else:
            print(f"  Connection: ✗ No data received")
            print("="*60 + "\n")
            return

        # Teaching mode
        if status['teaching_enabled']:
            print(f"  Teaching Mode: ✓ ENABLED")
        else:
            print(f"  Teaching Mode: ✗ DISABLED")

        # Error state
        if status['in_error_state']:
            print(f"  Error State: ⚠ ERROR - {status['last_error']}")
        else:
            print(f"  Error State: ✓ Normal")

        # Joint positions
        print("\n  Joint Positions (degrees):")
        for i, pos in enumerate(status['joint_positions_deg']):
            min_lim, max_lim = self.JOINT_LIMITS_DEG[i] if i < len(self.JOINT_LIMITS_DEG) else (-175, 175)
            # Check if near limit
            near_limit = ""
            for warning in status['joints_near_limit']:
                if warning['joint'] == i:
                    near_limit = f" ⚠ NEAR {warning['side'].upper()} LIMIT!"
                    break
            print(f"    Joint {i}: {pos:7.2f}° (limit: {min_lim}° ~ {max_lim}°){near_limit}")

        # Warnings
        if status['joints_near_limit']:
            print("\n  ⚠ WARNING: Joints near limits!")
            print("  Press [E] to auto-recover (moves only problem joints to safe range)")
            print("  Press [H] to move all joints to home position")

        print("="*60 + "\n")

    def recover_and_resume(self) -> bool:
        """
        Attempt to recover from error and resume teaching mode.

        This method:
        1. Calls recover to clear error state
        2. Moves only problematic joints to safe positions (not all joints to home)
        3. Re-enables teaching mode

        Returns:
            bool: True if recovery and resume was successful
        """
        self.logger.info("Attempting error recovery and resume...")

        # Step 1: Recover from error
        print("  Step 1/3: Recovering from error state...")
        if not self.recover_robot():
            self.logger.error("Failed to recover robot")
            self.is_in_error_state = True
            self.last_error_message = "Recovery failed"
            return False

        # Wait a moment for robot to stabilize
        time.sleep(0.5)

        # Step 2: Move joints near limits to safe positions
        print("  Step 2/3: Moving joints to safe positions...")
        status = self.get_robot_status()

        if status['joints_near_limit']:
            joints_info = ", ".join([f"J{w['joint']}" for w in status['joints_near_limit']])
            print(f"  ⚠ Joints near limits: {joints_info}")
            print("  Moving only these joints to safe range (keeping other joints in place)...")

            if not self.move_to_safe():
                self.logger.error("Failed to move joints to safe positions")
                # Fallback to home position
                print("  Falling back to home position...")
                if not self.move_to_home():
                    self.logger.error("Failed to move to home")
                    return False
                time.sleep(2.0)
            else:
                # Wait for movement to complete
                time.sleep(1.0)
        else:
            print("  ✓ All joints in safe range")

        # Step 3: Re-enable teaching mode
        print("  Step 3/3: Re-enabling teaching mode...")
        if self.enable_teaching_mode():
            self.is_in_error_state = False
            self.last_error_message = ""
            print("\n  ✓ Recovery complete! Teaching mode re-enabled.\n")
            return True
        else:
            self.is_in_error_state = True
            self.last_error_message = "Failed to re-enable teaching mode"
            return False

    def set_error_state(self, error_message: str):
        """Set the error state with a message."""
        self.is_in_error_state = True
        self.last_error_message = error_message
        self.is_teaching_enabled = False
