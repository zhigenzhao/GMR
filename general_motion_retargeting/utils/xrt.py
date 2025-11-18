import numpy as np
from scipy.spatial.transform import Rotation as R

try:
    import xrobotoolkit_sdk as xrt_sdk
    XROBOT_AVAILABLE = True
except ImportError:
    XROBOT_AVAILABLE = False
    print("Warning: XRoboToolkit SDK not available. Real-time streaming will not work.")


# XRoboToolkit joint names (24 joints)
XRT_JOINT_NAMES = [
    "Pelvis", "Left_Hip", "Right_Hip", "Spine1",
    "Left_Knee", "Right_Knee", "Spine2", "Left_Ankle",
    "Right_Ankle", "Spine3", "Left_Foot", "Right_Foot",
    "Neck", "Left_Collar", "Right_Collar", "Head",
    "Left_Shoulder", "Right_Shoulder", "Left_Elbow", "Right_Elbow",
    "Left_Wrist", "Right_Wrist", "Left_Hand", "Right_Hand"
]

def quat_mul_np(x, y, scalar_first=True):
    """
    Performs quaternion multiplication on arrays of quaternions
    :param x: tensor of quaternions of shape (..., 4)
    :param y: tensor of quaternions of shape (..., 4)
    :param scalar_first: True if quaternions are in [w, x, y, z] format
    :return: quaternion multiplication result in same format
    """
    if scalar_first:
        pass
    else: # convert to scalar-first
        x = x[..., [3, 0, 1, 2]]
        y = y[..., [3, 0, 1, 2]]

    x0, x1, x2, x3 = x[..., 0:1], x[..., 1:2], x[..., 2:3], x[..., 3:4]
    y0, y1, y2, y3 = y[..., 0:1], y[..., 1:2], y[..., 2:3], y[..., 3:4]

    res = np.concatenate([
        x0 * y0 - x1 * y1 - x2 * y2 - x3 * y3,
        x0 * y1 + x1 * y0 + x2 * y3 - x3 * y2,
        x0 * y2 - x1 * y3 + x2 * y0 + x3 * y1,
        x0 * y3 + x1 * y2 - x2 * y1 + x3 * y0
    ], axis=-1)

    if scalar_first:
        pass
    else:
        res = res[..., [1, 2, 3, 0]]  # back to [w, x, y, z]

    return res


def transform_xr_pose_to_world(position, quaternion):
    """
    Transform XR pose from headset frame to world frame.

    Args:
        position: Position in headset frame [x, y, z]
        quaternion: Quaternion in scalar-first format [qw, qx, qy, qz]

    Returns:
        position_world: Position in world frame [x, y, z]
        quaternion_world: Quaternion in world frame [qw, qx, qy, qz]
    """

    R_GMR = np.array([
        [1, 0, 0],
        [0, 0, -1],
        [0, 1, 0]
    ])
    # Transform position from headset to world
    # position_world = R_HEADSET_TO_WORLD @ position
    position_world = position @ R_GMR.T

    # as_quat() returns (x,y,z,w) by default in scipy 1.10.1
    # Convert to (w,x,y,z) for scalar_first=True compatibility
    R_quat_xyzw = R.from_matrix(R_GMR).as_quat()
    R_quat = np.array([R_quat_xyzw[3], R_quat_xyzw[0], R_quat_xyzw[1], R_quat_xyzw[2]])
    quaternion_world = quat_mul_np(R_quat, quaternion, scalar_first=True)

    # Normalize quaternion
    quaternion_world = quaternion_world / np.linalg.norm(quaternion_world)

    return position_world, quaternion_world


class XRobotClient:
    """Client for real-time XRoboToolkit body tracking."""

    def __init__(self):
        if not XROBOT_AVAILABLE:
            raise ImportError("XRoboToolkit SDK not available. Please install xrobotoolkit_sdk.")

        self.initialized = False

    def connect(self):
        """Initialize XRoboToolkit SDK."""
        try:
            xrt_sdk.init()
            self.initialized = True
            print("XRoboToolkit SDK initialized successfully")
            return True
        except Exception as e:
            print(f"Failed to initialize XRoboToolkit SDK: {e}")
            self.initialized = False
            return False

    def get_body_tracking_data(self):
        """
        Get current body tracking data from XRoboToolkit.

        Returns:
            frame_dict: Dictionary with structure:
                {"Pelvis": (position, quaternion), "Left_Hip": ...}
            or None if no data available
        """
        if not self.initialized:
            return None

        try:
            # Check if body data is available
            if not xrt_sdk.is_body_data_available():
                return None

            # Get body tracking data from XRoboToolkit
            body_poses = xrt_sdk.get_body_joints_pose()

            if body_poses is None or len(body_poses) != 24:
                return None

            # Convert to GMR format with coordinate transformation
            frame_dict = {}

            for i, joint_name in enumerate(XRT_JOINT_NAMES):
                # XRoboToolkit provides: [x, y, z, qx, qy, qz, qw]
                joint_data = body_poses[i]

                if len(joint_data) < 7:
                    continue

                position_headset = np.array([joint_data[0], joint_data[1], joint_data[2]])

                # Convert quaternion from [qx, qy, qz, qw] to [qw, qx, qy, qz] (scalar-first)
                quaternion_headset = np.array([joint_data[6], joint_data[3], joint_data[4], joint_data[5]])

                # Transform from headset frame to world frame
                position_world, quaternion_world = transform_xr_pose_to_world(position_headset, quaternion_headset)

                frame_dict[joint_name] = (position_world, quaternion_world)

            return frame_dict

        except Exception as e:
            print(f"Error getting body tracking data: {e}")
            return None

    def get_headset_pose(self):
        """
        Get current headset pose from XRoboToolkit.

        Returns:
            tuple: (position, quaternion) in world frame
                - position: np.array([x, y, z])
                - quaternion: np.array([qw, qx, qy, qz]) scalar-first
            or None if no data available
        """
        if not self.initialized:
            return None

        try:
            # Get headset pose from XRoboToolkit
            # Format: [x, y, z, qx, qy, qz, qw] in headset frame
            headset_pose_xr = xrt_sdk.get_headset_pose()

            if headset_pose_xr is None or len(headset_pose_xr) < 7:
                return None

            position_headset = np.array([headset_pose_xr[0], headset_pose_xr[1], headset_pose_xr[2]])

            # Convert quaternion from [qx, qy, qz, qw] to [qw, qx, qy, qz] (scalar-first)
            quaternion_headset = np.array([headset_pose_xr[6], headset_pose_xr[3], headset_pose_xr[4], headset_pose_xr[5]])

            # Transform from headset frame to world frame
            position_world, quaternion_world = transform_xr_pose_to_world(position_headset, quaternion_headset)

            return position_world, quaternion_world

        except Exception as e:
            print(f"Error getting headset pose: {e}")
            return None

    def disconnect(self):
        """Cleanup (XRoboToolkit SDK doesn't require explicit disconnect)."""
        self.initialized = False
        print("XRoboToolkit SDK client closed")


def load_xrt_file(file_path):
    """
    Load saved XRoboToolkit body tracking data from file.

    Args:
        file_path: Path to saved XRoboToolkit data file (pickle format)

    Returns:
        frames: List of frame dictionaries with structure:
            [{"Pelvis": (position, quaternion), ...}, ...]
        human_height: Estimated human height in meters
    """
    import pickle

    # Load the pickle file
    with open(file_path, 'rb') as f:
        data = pickle.load(f)

    # Handle different possible data structures
    if isinstance(data, list):
        # List of frame dictionaries (expected format from example_body_tracking.py)
        frames = data
    elif isinstance(data, dict):
        # Single frame or dict with 'frames' key
        if 'frames' in data:
            frames = data['frames']
        else:
            # Single frame
            frames = [data]
    else:
        raise ValueError(f"Unsupported data format in {file_path}")

    # Ensure frames have correct structure
    processed_frames = []
    for frame in frames:
        processed_frame = {}
        for joint_name in XRT_JOINT_NAMES:
            if joint_name in frame:
                joint_data = frame[joint_name]
                # Ensure numpy arrays
                if isinstance(joint_data, (list, tuple)) and len(joint_data) == 2:
                    position = np.array(joint_data[0])
                    quaternion = np.array(joint_data[1])
                    processed_frame[joint_name] = (position, quaternion)

        if processed_frame:
            processed_frames.append(processed_frame)

    if not processed_frames:
        raise ValueError(f"No valid frames found in {file_path}")

    # Estimate human height from first frame
    human_height = estimate_human_height(processed_frames[0])

    return processed_frames, human_height


def save_xrt_file(frames, file_path):
    """
    Save XRoboToolkit body tracking data to file.

    Args:
        frames: List of frame dictionaries with structure:
            [{"Pelvis": (position, quaternion), ...}, ...]
        file_path: Path to save the data (pickle format)
    """
    import pickle
    import os

    # Ensure directory exists
    save_dir = os.path.dirname(file_path)
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)

    # Convert numpy arrays to lists for serialization
    serializable_frames = []
    for frame in frames:
        serializable_frame = {}
        for joint_name, (position, quaternion) in frame.items():
            serializable_frame[joint_name] = [
                position.tolist() if isinstance(position, np.ndarray) else position,
                quaternion.tolist() if isinstance(quaternion, np.ndarray) else quaternion
            ]
        serializable_frames.append(serializable_frame)

    # Save to pickle file
    with open(file_path, 'wb') as f:
        pickle.dump(serializable_frames, f)

    print(f"Saved {len(serializable_frames)} frames to {file_path}")


def estimate_human_height(frame_data):
    """
    Estimate human height from body tracking data.

    Args:
        frame_data: Dictionary with joint positions and rotations

    Returns:
        height: Estimated human height in meters
    """
    if "Head" not in frame_data or "Pelvis" not in frame_data:
        # Use default if required joints not available
        return 1.75

    # Get relevant joint positions
    head_pos = frame_data["Head"][0]

    # Get foot positions
    left_foot_pos = frame_data.get("Left_Foot", (np.zeros(3),))[0]
    right_foot_pos = frame_data.get("Right_Foot", (np.zeros(3),))[0]

    # Find lowest point (ground level)
    min_foot_z = min(left_foot_pos[2], right_foot_pos[2])

    # Calculate height from lowest foot to head
    height = head_pos[2] - min_foot_z

    # Add small offset for full height (head center to top of head)
    height += 0.15

    # Sanity check
    if height < 1.4 or height > 2.2:
        print(f"Warning: Estimated height {height:.2f}m seems unusual. Using default 1.75m")
        return 1.75

    return height


def connect_xrt_realtime():
    """
    Initialize XRoboToolkit SDK for real-time body tracking.

    Returns:
        client: XRobotClient instance if successful, None otherwise
    """
    client = XRobotClient()
    if client.connect():
        return client
    return None


def get_xrt_frame_realtime(client):
    """
    Get current body tracking frame from XRoboToolkit client.

    Args:
        client: XRobotClient instance

    Returns:
        frame_dict: Dictionary with structure:
            {"Pelvis": (position, quaternion), ...}
        or None if no data available
    """
    if client is None:
        return None

    return client.get_body_tracking_data()
