#!/usr/bin/env python3
"""Data Logger for Indy7 Direct Teaching Demonstrations"""

import os
import json
import pickle
from datetime import datetime
from typing import List, Dict, Any, Optional
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import JointState


class DataLogger:
    """
    Collects and stores joint state data during direct teaching demonstrations.

    Data is collected at 20 Hz (same as joint_states publication rate) and
    can be saved in multiple formats for downstream processing.
    """

    def __init__(self, node: Node, data_dir: str = "~/teaching_data"):
        """
        Initialize the data logger.

        Args:
            node: ROS2 node for creating subscriptions
            data_dir: Directory to save demonstration data
        """
        self.node = node
        self.logger = node.get_logger()

        # Expand and create data directory
        self.data_dir = os.path.expanduser(data_dir)
        os.makedirs(self.data_dir, exist_ok=True)
        self.logger.info(f"Data will be saved to: {self.data_dir}")

        # Subscribe to joint states
        self.joint_state_sub = node.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Recording state
        self.is_recording = False
        self.current_episode_id = 0

        # Data buffers for current episode
        self.timestamps: List[float] = []
        self.joint_positions: List[List[float]] = []
        self.joint_velocities: List[List[float]] = []
        self.joint_efforts: List[List[float]] = []
        self.joint_names: List[str] = []

        # Statistics
        self.total_episodes = 0
        self.recording_start_time: Optional[float] = None

    def start_episode(self, episode_id: Optional[int] = None) -> int:
        """
        Start recording a new demonstration episode.

        Args:
            episode_id: Optional episode ID. If None, uses auto-incremented ID.

        Returns:
            int: The episode ID for this recording
        """
        if self.is_recording:
            self.logger.warn("Already recording! Stop current episode first.")
            return self.current_episode_id

        # Set episode ID
        if episode_id is not None:
            self.current_episode_id = episode_id
        else:
            self.current_episode_id = self.total_episodes + 1

        # Clear buffers
        self.timestamps = []
        self.joint_positions = []
        self.joint_velocities = []
        self.joint_efforts = []

        # Start recording
        self.is_recording = True
        self.recording_start_time = self.node.get_clock().now().nanoseconds / 1e9

        self.logger.info(f"Started recording episode {self.current_episode_id}")

        return self.current_episode_id

    def stop_episode(self, save: bool = True) -> Dict[str, Any]:
        """
        Stop recording the current episode.

        Args:
            save: If True, save the episode data to disk

        Returns:
            dict: Episode metadata including duration and sample count
        """
        if not self.is_recording:
            self.logger.warn("Not currently recording!")
            return {}

        self.is_recording = False

        # Calculate statistics
        duration = 0.0
        if len(self.timestamps) > 0:
            duration = self.timestamps[-1] - self.timestamps[0]

        sample_count = len(self.timestamps)
        sample_rate = sample_count / duration if duration > 0 else 0.0

        metadata = {
            'episode_id': self.current_episode_id,
            'duration': duration,
            'sample_count': sample_count,
            'sample_rate': sample_rate,
            'joint_names': self.joint_names,
            'timestamp': datetime.now().isoformat()
        }

        self.logger.info(
            f"Stopped recording episode {self.current_episode_id}: "
            f"{duration:.2f}s, {sample_count} samples, {sample_rate:.1f} Hz"
        )

        # Save data if requested
        if save and sample_count > 0:
            self._save_episode(metadata)
            self.total_episodes += 1

        return metadata

    def joint_state_callback(self, msg: JointState):
        """
        Callback for joint state messages.

        Args:
            msg: JointState message containing positions, velocities, and efforts
        """
        if not self.is_recording:
            return

        # Store joint names (only once)
        if len(self.joint_names) == 0:
            self.joint_names = list(msg.name)

        # Get timestamp (relative to recording start)
        current_time = self.node.get_clock().now().nanoseconds / 1e9
        relative_time = current_time - self.recording_start_time

        # Store data
        self.timestamps.append(relative_time)
        self.joint_positions.append(list(msg.position))
        self.joint_velocities.append(list(msg.velocity))
        self.joint_efforts.append(list(msg.effort))

    def _save_episode(self, metadata: Dict[str, Any]):
        """
        Save episode data to disk in multiple formats.

        Args:
            metadata: Episode metadata dictionary
        """
        episode_id = metadata['episode_id']
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Create episode filename
        base_filename = f"episode_{episode_id:03d}_{timestamp_str}"

        # Prepare data dictionary
        episode_data = {
            'metadata': metadata,
            'timestamps': np.array(self.timestamps),
            'joint_positions': np.array(self.joint_positions),
            'joint_velocities': np.array(self.joint_velocities),
            'joint_efforts': np.array(self.joint_efforts),
            'joint_names': self.joint_names
        }

        # Save as pickle (Python native format)
        pickle_path = os.path.join(self.data_dir, f"{base_filename}.pkl")
        try:
            with open(pickle_path, 'wb') as f:
                pickle.dump(episode_data, f)
            self.logger.info(f"Saved episode data to: {pickle_path}")
        except Exception as e:
            self.logger.error(f"Failed to save pickle file: {str(e)}")

        # Save as numpy arrays (for easy loading in Python)
        npz_path = os.path.join(self.data_dir, f"{base_filename}.npz")
        try:
            np.savez(
                npz_path,
                timestamps=episode_data['timestamps'],
                joint_positions=episode_data['joint_positions'],
                joint_velocities=episode_data['joint_velocities'],
                joint_efforts=episode_data['joint_efforts'],
                joint_names=episode_data['joint_names']
            )
            self.logger.info(f"Saved numpy arrays to: {npz_path}")
        except Exception as e:
            self.logger.error(f"Failed to save npz file: {str(e)}")

        # Save metadata as JSON
        json_path = os.path.join(self.data_dir, f"{base_filename}_metadata.json")
        try:
            with open(json_path, 'w') as f:
                # Convert numpy types to native Python types for JSON serialization
                json_metadata = {
                    k: (v.tolist() if isinstance(v, np.ndarray) else v)
                    for k, v in metadata.items()
                }
                json.dump(json_metadata, f, indent=2)
            self.logger.info(f"Saved metadata to: {json_path}")
        except Exception as e:
            self.logger.error(f"Failed to save JSON metadata: {str(e)}")

    def reset_episode_counter(self):
        """Reset the episode counter to 0."""
        self.total_episodes = 0
        self.current_episode_id = 0
        self.logger.info("Episode counter reset to 0")

    def get_current_episode_id(self) -> int:
        """Get the current episode ID."""
        return self.current_episode_id

    def get_total_episodes(self) -> int:
        """Get the total number of recorded episodes."""
        return self.total_episodes

    def is_recording_active(self) -> bool:
        """Check if currently recording."""
        return self.is_recording

    def get_current_sample_count(self) -> int:
        """Get the number of samples in the current recording."""
        return len(self.timestamps)

    def get_current_duration(self) -> float:
        """Get the duration of the current recording in seconds."""
        if not self.is_recording or len(self.timestamps) == 0:
            return 0.0
        return self.timestamps[-1] - self.timestamps[0]
