#!/usr/bin/env python3
"""Camera Data Logger for Indy7 Direct Teaching with RealSense D435"""

import os
import json
import pickle
from datetime import datetime
from typing import List, Dict, Any, Optional
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from cv_bridge import CvBridge
import message_filters


class CameraDataLogger:
    """
    Collects and stores synchronized joint state and camera data during
    direct teaching demonstrations.

    Uses ApproximateTimeSynchronizer to align joint states with RGB and
    depth images from RealSense D435 camera.

    Data is collected at approximately 10-15 Hz (limited by camera frame rate
    and synchronization) and saved in multiple formats.
    """

    # Default image size (can be changed via parameters)
    DEFAULT_IMAGE_WIDTH = 640
    DEFAULT_IMAGE_HEIGHT = 480

    # Resize target for imitation learning (smaller for faster training)
    RESIZE_WIDTH = 224
    RESIZE_HEIGHT = 224

    def __init__(
        self,
        node: Node,
        data_dir: str = "~/teaching_data",
        rgb_topic: str = "/camera/camera/color/image_raw",
        depth_topic: str = "/camera/camera/aligned_depth_to_color/image_raw",
        joint_topic: str = "/joint_states",
        enable_depth: bool = True,
        resize_images: bool = True,
        sync_slop: float = 0.1,  # Time tolerance for synchronization (seconds)
        queue_size: int = 10
    ):
        """
        Initialize the camera data logger with synchronized subscriptions.

        Args:
            node: ROS2 node for creating subscriptions
            data_dir: Directory to save demonstration data
            rgb_topic: Topic name for RGB image
            depth_topic: Topic name for depth image (aligned to color)
            joint_topic: Topic name for joint states
            enable_depth: Whether to record depth images
            resize_images: Whether to resize images to RESIZE_WIDTH x RESIZE_HEIGHT
            sync_slop: Maximum time difference for message synchronization
            queue_size: Queue size for message filters
        """
        self.node = node
        self.logger = node.get_logger()
        self.enable_depth = enable_depth
        self.resize_images = resize_images

        # Expand and create data directory
        self.data_dir = os.path.expanduser(data_dir)
        os.makedirs(self.data_dir, exist_ok=True)
        self.logger.info(f"Camera data will be saved to: {self.data_dir}")

        # CV Bridge for image conversion
        self.cv_bridge = CvBridge()

        # Recording state
        self.is_recording = False
        self.current_episode_id = 0

        # Data buffers for current episode
        self.timestamps: List[float] = []
        self.joint_positions: List[List[float]] = []
        self.joint_velocities: List[List[float]] = []
        self.joint_efforts: List[List[float]] = []
        self.joint_names: List[str] = []
        self.rgb_images: List[np.ndarray] = []
        self.depth_images: List[np.ndarray] = []

        # Statistics
        self.total_episodes = 0
        self.recording_start_time: Optional[float] = None
        self.sync_callback_count = 0

        # Setup synchronized subscribers using message_filters
        self._setup_synchronized_subscribers(
            rgb_topic, depth_topic, joint_topic, sync_slop, queue_size
        )

        self.logger.info("CameraDataLogger initialized with synchronized subscriptions")

    def _setup_synchronized_subscribers(
        self,
        rgb_topic: str,
        depth_topic: str,
        joint_topic: str,
        sync_slop: float,
        queue_size: int
    ):
        """
        Setup synchronized subscribers for joint states and camera images.

        Uses ApproximateTimeSynchronizer to align messages from multiple topics
        based on their timestamps.
        """
        # Create message filter subscribers
        self.joint_sub = message_filters.Subscriber(
            self.node, JointState, joint_topic
        )
        self.rgb_sub = message_filters.Subscriber(
            self.node, Image, rgb_topic
        )

        if self.enable_depth:
            self.depth_sub = message_filters.Subscriber(
                self.node, Image, depth_topic
            )
            # Synchronize all three topics
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [self.joint_sub, self.rgb_sub, self.depth_sub],
                queue_size=queue_size,
                slop=sync_slop
            )
            self.sync.registerCallback(self._synchronized_callback_with_depth)
            self.logger.info(
                f"Subscribed to: {joint_topic}, {rgb_topic}, {depth_topic}"
            )
        else:
            # Synchronize only joint states and RGB
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [self.joint_sub, self.rgb_sub],
                queue_size=queue_size,
                slop=sync_slop
            )
            self.sync.registerCallback(self._synchronized_callback_rgb_only)
            self.logger.info(f"Subscribed to: {joint_topic}, {rgb_topic}")

    def _synchronized_callback_with_depth(
        self,
        joint_msg: JointState,
        rgb_msg: Image,
        depth_msg: Image
    ):
        """
        Callback for synchronized joint, RGB, and depth messages.

        Args:
            joint_msg: JointState message
            rgb_msg: RGB Image message
            depth_msg: Depth Image message
        """
        if not self.is_recording:
            return

        self._process_synchronized_data(joint_msg, rgb_msg, depth_msg)

    def _synchronized_callback_rgb_only(
        self,
        joint_msg: JointState,
        rgb_msg: Image
    ):
        """
        Callback for synchronized joint and RGB messages (no depth).

        Args:
            joint_msg: JointState message
            rgb_msg: RGB Image message
        """
        if not self.is_recording:
            return

        self._process_synchronized_data(joint_msg, rgb_msg, None)

    def _process_synchronized_data(
        self,
        joint_msg: JointState,
        rgb_msg: Image,
        depth_msg: Optional[Image]
    ):
        """
        Process and store synchronized sensor data.

        Args:
            joint_msg: JointState message
            rgb_msg: RGB Image message
            depth_msg: Depth Image message (or None if depth disabled)
        """
        self.sync_callback_count += 1

        # Store joint names (only once)
        if len(self.joint_names) == 0:
            self.joint_names = list(joint_msg.name)

        # Get timestamp (relative to recording start)
        current_time = self.node.get_clock().now().nanoseconds / 1e9
        relative_time = current_time - self.recording_start_time

        # Store joint data
        self.timestamps.append(relative_time)
        self.joint_positions.append(list(joint_msg.position))
        self.joint_velocities.append(list(joint_msg.velocity))
        self.joint_efforts.append(list(joint_msg.effort))

        # Convert and store RGB image
        try:
            rgb_cv = self.cv_bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8')
            if self.resize_images:
                rgb_cv = self._resize_image(rgb_cv)
            self.rgb_images.append(rgb_cv)
        except Exception as e:
            self.logger.error(f"Failed to convert RGB image: {e}")
            return

        # Convert and store depth image
        if depth_msg is not None:
            try:
                # Depth is typically 16UC1 (16-bit unsigned, in millimeters)
                depth_cv = self.cv_bridge.imgmsg_to_cv2(
                    depth_msg, desired_encoding='passthrough'
                )
                if self.resize_images:
                    depth_cv = self._resize_image(depth_cv, is_depth=True)
                self.depth_images.append(depth_cv)
            except Exception as e:
                self.logger.error(f"Failed to convert depth image: {e}")

    def _resize_image(self, image: np.ndarray, is_depth: bool = False) -> np.ndarray:
        """
        Resize image to target size for imitation learning.

        Args:
            image: Input image (RGB or depth)
            is_depth: If True, use nearest-neighbor interpolation for depth

        Returns:
            Resized image
        """
        try:
            import cv2
            if is_depth:
                # Use nearest-neighbor for depth to preserve values
                return cv2.resize(
                    image,
                    (self.RESIZE_WIDTH, self.RESIZE_HEIGHT),
                    interpolation=cv2.INTER_NEAREST
                )
            else:
                # Use bilinear for RGB
                return cv2.resize(
                    image,
                    (self.RESIZE_WIDTH, self.RESIZE_HEIGHT),
                    interpolation=cv2.INTER_LINEAR
                )
        except ImportError:
            self.logger.warn("cv2 not available, returning original size image")
            return image

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
        self.rgb_images = []
        self.depth_images = []
        self.sync_callback_count = 0

        # Start recording
        self.is_recording = True
        self.recording_start_time = self.node.get_clock().now().nanoseconds / 1e9

        self.logger.info(f"Started recording episode {self.current_episode_id} (with camera)")

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
            'timestamp': datetime.now().isoformat(),
            'has_rgb': len(self.rgb_images) > 0,
            'has_depth': len(self.depth_images) > 0,
            'rgb_count': len(self.rgb_images),
            'depth_count': len(self.depth_images),
            'image_size': [self.RESIZE_WIDTH, self.RESIZE_HEIGHT] if self.resize_images else 'original',
            'camera_enabled': True
        }

        self.logger.info(
            f"Stopped recording episode {self.current_episode_id}: "
            f"{duration:.2f}s, {sample_count} sync frames, {sample_rate:.1f} Hz"
        )

        # Save data if requested
        if save and sample_count > 0:
            self._save_episode(metadata)
            self.total_episodes += 1

        return metadata

    def _save_episode(self, metadata: Dict[str, Any]):
        """
        Save episode data to disk in multiple formats.

        Args:
            metadata: Episode metadata dictionary
        """
        episode_id = metadata['episode_id']
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Create episode filename with camera suffix
        base_filename = f"episode_{episode_id:03d}_{timestamp_str}_camera"

        # Prepare data dictionary
        episode_data = {
            'metadata': metadata,
            'timestamps': np.array(self.timestamps),
            'joint_positions': np.array(self.joint_positions),
            'joint_velocities': np.array(self.joint_velocities),
            'joint_efforts': np.array(self.joint_efforts),
            'joint_names': self.joint_names,
            'rgb_images': np.array(self.rgb_images, dtype=np.uint8),
        }

        if len(self.depth_images) > 0:
            episode_data['depth_images'] = np.array(self.depth_images, dtype=np.uint16)

        # Save as pickle (Python native format)
        pickle_path = os.path.join(self.data_dir, f"{base_filename}.pkl")
        try:
            with open(pickle_path, 'wb') as f:
                pickle.dump(episode_data, f)
            self.logger.info(f"Saved episode data to: {pickle_path}")
        except Exception as e:
            self.logger.error(f"Failed to save pickle file: {str(e)}")

        # Save as compressed numpy arrays (more efficient for images)
        npz_path = os.path.join(self.data_dir, f"{base_filename}.npz")
        try:
            save_dict = {
                'timestamps': episode_data['timestamps'],
                'joint_positions': episode_data['joint_positions'],
                'joint_velocities': episode_data['joint_velocities'],
                'joint_efforts': episode_data['joint_efforts'],
                'rgb_images': episode_data['rgb_images'],
            }
            if 'depth_images' in episode_data:
                save_dict['depth_images'] = episode_data['depth_images']

            np.savez_compressed(npz_path, **save_dict)
            self.logger.info(f"Saved compressed numpy arrays to: {npz_path}")
        except Exception as e:
            self.logger.error(f"Failed to save npz file: {str(e)}")

        # Save metadata as JSON
        json_path = os.path.join(self.data_dir, f"{base_filename}_metadata.json")
        try:
            with open(json_path, 'w') as f:
                # Convert numpy types to native Python types for JSON serialization
                json_metadata = {}
                for k, v in metadata.items():
                    if isinstance(v, np.ndarray):
                        json_metadata[k] = v.tolist()
                    elif isinstance(v, (np.integer, np.floating)):
                        json_metadata[k] = v.item()
                    else:
                        json_metadata[k] = v
                json.dump(json_metadata, f, indent=2)
            self.logger.info(f"Saved metadata to: {json_path}")
        except Exception as e:
            self.logger.error(f"Failed to save JSON metadata: {str(e)}")

        # Log file sizes
        self._log_file_sizes(pickle_path, npz_path)

    def _log_file_sizes(self, pickle_path: str, npz_path: str):
        """Log the sizes of saved files for information."""
        try:
            pkl_size = os.path.getsize(pickle_path) / (1024 * 1024)  # MB
            npz_size = os.path.getsize(npz_path) / (1024 * 1024)  # MB
            self.logger.info(f"File sizes - Pickle: {pkl_size:.2f} MB, NPZ: {npz_size:.2f} MB")
        except Exception:
            pass

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
        """Get the number of synchronized samples in the current recording."""
        return len(self.timestamps)

    def get_current_duration(self) -> float:
        """Get the duration of the current recording in seconds."""
        if not self.is_recording or len(self.timestamps) == 0:
            return 0.0
        return self.timestamps[-1] - self.timestamps[0]

    def get_sync_status(self) -> Dict[str, Any]:
        """
        Get synchronization status for debugging.

        Returns:
            dict: Status information about synchronized data collection
        """
        return {
            'is_recording': self.is_recording,
            'sync_callback_count': self.sync_callback_count,
            'current_samples': len(self.timestamps),
            'rgb_frames': len(self.rgb_images),
            'depth_frames': len(self.depth_images),
            'depth_enabled': self.enable_depth
        }
