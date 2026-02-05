#!/usr/bin/env python3
"""Direct Teaching Node for Indy7 Robot"""

import sys
import termios
import tty
import select
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from indy7_gripper_teleop.teaching_mode_manager import TeachingModeManager
from indy7_gripper_teleop.data_logger import DataLogger


# Keyboard codes
KEYCODE_SPACE = 0x20  # Start/stop recording
KEYCODE_Q = 0x71      # Quit
KEYCODE_R = 0x72      # Reset episode counter
KEYCODE_H = 0x68      # Move to home
KEYCODE_C = 0x63      # Clear screen


class KeyboardReader:
    """Non-blocking keyboard input reader."""

    def __init__(self):
        """Initialize keyboard reader with terminal settings."""
        # Check if stdin is a TTY
        if not sys.stdin.isatty():
            raise RuntimeError(
                "Cannot read keyboard input: stdin is not a TTY.\n"
                "This usually happens when running through ros2 launch.\n"
                "Please run the node directly:\n"
                "  ros2 run indy7_gripper_teleop teaching_control.py\n"
                "Or use 'prefix' in launch file to run in a separate terminal."
            )
        self.settings = termios.tcgetattr(sys.stdin)

    def read_one(self) -> str:
        """
        Read a single key press without blocking.

        Returns:
            str: The key that was pressed
        """
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def restore_settings(self):
        """Restore terminal settings to original state."""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


class DirectTeachingNode(Node):
    """
    Main ROS2 node for direct teaching mode and data collection.

    This node coordinates the teaching mode manager and data logger,
    providing a keyboard interface for starting/stopping recordings.
    """

    def __init__(self):
        """Initialize the direct teaching node."""
        super().__init__('direct_teaching_node')

        # Parameters
        self.declare_parameter('data_dir', '~/teaching_data')
        self.declare_parameter('auto_enable_teaching', True)

        data_dir = self.get_parameter('data_dir').get_parameter_value().string_value
        auto_enable = self.get_parameter('auto_enable_teaching').get_parameter_value().bool_value

        # Initialize components
        self.teaching_manager = TeachingModeManager(self)
        self.data_logger = DataLogger(self, data_dir=data_dir)

        # Status publisher
        self.status_pub = self.create_publisher(String, '/teaching_status', 10)

        # Create timer for status updates
        self.status_timer = self.create_timer(0.5, self.publish_status)

        # Keyboard reader
        self.keyboard_reader = KeyboardReader()

        # State
        self.is_running = True
        self.last_status_update = time.time()

        # Enable teaching mode on startup if requested
        if auto_enable:
            self.get_logger().info("Auto-enabling teaching mode...")
            if self.teaching_manager.enable_teaching_mode():
                self.get_logger().info("Teaching mode enabled successfully!")
            else:
                self.get_logger().error("Failed to enable teaching mode!")

        # Print instructions
        self.print_instructions()

    def print_instructions(self):
        """Print keyboard control instructions to the terminal."""
        print("\n" + "="*60)
        print("  INDY7 DIRECT TEACHING MODE - Data Collection")
        print("="*60)
        print("\nControls:")
        print("  [SPACE]  - Start/Stop recording episode")
        print("  [Q]      - Quit and disable teaching mode")
        print("  [R]      - Reset episode counter")
        print("  [H]      - Move robot to home position")
        print("  [C]      - Clear screen and show instructions")
        print("\nStatus:")
        print(f"  Teaching Mode: {'ENABLED' if self.teaching_manager.is_enabled() else 'DISABLED'}")
        print(f"  Total Episodes: {self.data_logger.get_total_episodes()}")
        print(f"  Data Directory: {self.data_logger.data_dir}")
        print("="*60 + "\n")

    def publish_status(self):
        """Publish current teaching status."""
        status_msg = String()

        if self.data_logger.is_recording_active():
            duration = self.data_logger.get_current_duration()
            sample_count = self.data_logger.get_current_sample_count()
            status_msg.data = (
                f"RECORDING - Episode {self.data_logger.get_current_episode_id()}: "
                f"{duration:.1f}s, {sample_count} samples"
            )
        else:
            status_msg.data = (
                f"IDLE - Total Episodes: {self.data_logger.get_total_episodes()}"
            )

        self.status_pub.publish(status_msg)

    def handle_spacebar(self):
        """Handle spacebar press - toggle recording."""
        if self.data_logger.is_recording_active():
            # Stop recording
            self.get_logger().info("Stopping recording...")
            metadata = self.data_logger.stop_episode(save=True)

            print("\n" + "-"*60)
            print(f"✓ Episode {metadata['episode_id']} saved!")
            print(f"  Duration: {metadata['duration']:.2f} seconds")
            print(f"  Samples: {metadata['sample_count']}")
            print(f"  Sample Rate: {metadata['sample_rate']:.1f} Hz")
            print("-"*60 + "\n")

        else:
            # Start recording
            if not self.teaching_manager.is_enabled():
                self.get_logger().warn("Teaching mode not enabled! Enabling now...")
                if not self.teaching_manager.enable_teaching_mode():
                    self.get_logger().error("Failed to enable teaching mode!")
                    return

            episode_id = self.data_logger.start_episode()
            print(f"\n● Recording Episode {episode_id} - Press [SPACE] to stop\n")

    def handle_quit(self):
        """Handle quit command."""
        print("\nShutting down...")

        # Stop recording if active
        if self.data_logger.is_recording_active():
            self.get_logger().info("Saving current episode before quit...")
            self.data_logger.stop_episode(save=True)

        # Disable teaching mode
        if self.teaching_manager.is_enabled():
            self.get_logger().info("Disabling teaching mode...")
            self.teaching_manager.disable_teaching_mode()

        self.is_running = False
        print("Goodbye!\n")

    def handle_reset(self):
        """Handle reset episode counter."""
        if self.data_logger.is_recording_active():
            self.get_logger().warn("Cannot reset while recording!")
            return

        self.data_logger.reset_episode_counter()
        print("\n✓ Episode counter reset to 0\n")

    def handle_home(self):
        """Handle move to home position."""
        if self.data_logger.is_recording_active():
            self.get_logger().warn("Cannot move to home while recording!")
            return

        print("\nMoving to home position...")
        if self.teaching_manager.move_to_home():
            print("✓ Robot moved to home\n")
        else:
            print("✗ Failed to move to home\n")

    def handle_clear_screen(self):
        """Clear screen and show instructions."""
        print("\033[2J\033[H")  # ANSI escape code to clear screen
        self.print_instructions()

    def run(self):
        """
        Main control loop - processes keyboard input.

        This method runs in the main thread and blocks on keyboard input.
        """
        try:
            while self.is_running and rclpy.ok():
                # Read keyboard input (non-blocking)
                key = self.keyboard_reader.read_one()

                if ord(key) == KEYCODE_SPACE:
                    self.handle_spacebar()

                elif ord(key) == KEYCODE_Q:
                    self.handle_quit()
                    break

                elif ord(key) == KEYCODE_R:
                    self.handle_reset()

                elif ord(key) == KEYCODE_H:
                    self.handle_home()

                elif ord(key) == KEYCODE_C:
                    self.handle_clear_screen()

                # Small sleep to prevent CPU spinning
                time.sleep(0.01)

        except KeyboardInterrupt:
            self.get_logger().info("Keyboard interrupt received")
            self.handle_quit()

        finally:
            # Restore terminal settings
            self.keyboard_reader.restore_settings()

    def destroy_node(self):
        """Clean up node resources."""
        # Stop recording if active
        if self.data_logger.is_recording_active():
            self.data_logger.stop_episode(save=True)

        # Disable teaching mode
        if self.teaching_manager.is_enabled():
            self.teaching_manager.disable_teaching_mode()

        super().destroy_node()


def main(args=None):
    """Main entry point for the direct teaching node."""
    rclpy.init(args=args)

    node = DirectTeachingNode()

    # Create a separate thread for ROS2 spinning
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Run the main keyboard control loop in main thread
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
