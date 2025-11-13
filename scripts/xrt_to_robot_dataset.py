import argparse
import pathlib
import os
import multiprocessing as mp

import numpy as np
from natsort import natsorted
from rich import print
import torch
import pickle
import gc
import time
import psutil

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting.utils.xrt import load_xrt_file
from general_motion_retargeting.kinematics_model import KinematicsModel


def check_memory(threshold_gb=30):
    """Check if available memory is below threshold."""
    mem = psutil.virtual_memory()
    used_memory_gb = (mem.total - mem.available) / (1024 ** 3)
    available_memory_gb = mem.available / (1024 ** 3)
    if available_memory_gb < threshold_gb:
        print(f"[WARNING] Memory usage:{used_memory_gb:.2f} GB, available:{available_memory_gb:.2f} GB, exceeding the threshold of {threshold_gb} GB.")
        return True
    return False


HERE = pathlib.Path(__file__).parent


def process_file(xrt_file_path, tgt_file_path, tgt_robot, tgt_folder, total_files, verbose=False):
    """
    Process a single XRT body tracking file and save retargeted robot motion.

    Args:
        xrt_file_path: Path to input XRT pickle file
        tgt_file_path: Path to output robot motion pickle file
        tgt_robot: Target robot model name
        tgt_folder: Root folder for target files (for progress tracking)
        total_files: Total number of files to process (for progress tracking)
        verbose: Enable verbose memory logging
    """
    def log_memory(message):
        if verbose:
            process = psutil.Process(os.getpid())
            memory_usage = process.memory_info().rss / (1024 ** 3)  # Convert to GB
            print(f"[MEMORY] {message}: {memory_usage:.2f} GB")

    # Initial memory check
    log_memory("Initial memory usage")

    num_pause = 0
    while check_memory():
        print(f"[PAUSE] Paused processing {xrt_file_path} to prevent memory overflow. num_pause: {num_pause}")
        time.sleep(60*2)
        num_pause += 1
        if num_pause > 10:
            print(f"[ERROR] Memory usage is still high after 10 pauses. Exiting.")
            return

    try:
        # Load XRT body tracking data
        xrt_frames, actual_human_height = load_xrt_file(xrt_file_path)
        log_memory("After loading XRT data")
    except Exception as e:
        print(f"[ERROR] Loading {xrt_file_path}: {e}")
        return

    # Use 30 FPS as default for XRT data
    tgt_fps = 30

    # Initialize retargeting system
    try:
        retargeter = GMR(
            src_human="xrt",
            tgt_robot=tgt_robot,
            actual_human_height=actual_human_height,
        )
    except Exception as e:
        print(f"[ERROR] Initializing retargeter for {xrt_file_path}: {e}")
        return

    # Retarget all frames
    qpos_list = []
    for frame_data in xrt_frames:
        try:
            qpos = retargeter.retarget(frame_data, offset_to_ground=True)
            qpos_list.append(qpos.copy())
        except Exception as e:
            print(f"[ERROR] Retargeting frame in {xrt_file_path}: {e}")
            return

    qpos_list = np.array(qpos_list)
    log_memory("After retargeting")

    # Compute forward kinematics for body positions
    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    kinematics_model = KinematicsModel(retargeter.xml_file, device=device)

    try:
        root_pos = qpos_list[:, :3]
        root_rot = qpos_list[:, 3:7]
        # Convert quaternion from wxyz to xyzw format
        root_rot[:, [0, 1, 2, 3]] = root_rot[:, [1, 2, 3, 0]]
        dof_pos = qpos_list[:, 7:]
        num_frames = root_pos.shape[0]
    except Exception as e:
        print(f"[ERROR] Processing qpos for {xrt_file_path}: {e}")
        return

    # Compute local body positions using forward kinematics
    fk_root_pos = torch.zeros((num_frames, 3), device=device)
    fk_root_rot = torch.zeros((num_frames, 4), device=device)
    fk_root_rot[:, -1] = 1.0  # Identity quaternion (xyzw format)

    local_body_pos, _ = kinematics_model.forward_kinematics(
        fk_root_pos, fk_root_rot, torch.from_numpy(dof_pos).to(device=device, dtype=torch.float)
    )

    log_memory("After forward kinematics")

    body_names = kinematics_model.body_names

    # Height adjustment to ensure the lowest part is on the ground
    HEIGHT_ADJUST = True
    if HEIGHT_ADJUST:
        body_pos, _ = kinematics_model.forward_kinematics(
            torch.from_numpy(root_pos).to(device=device, dtype=torch.float),
            torch.from_numpy(root_rot).to(device=device, dtype=torch.float),
            torch.from_numpy(dof_pos).to(device=device, dtype=torch.float)
        )
        ground_offset = 0.0
        lowest_height = torch.min(body_pos[..., 2]).item()
        root_pos[:, 2] = root_pos[:, 2] - lowest_height + ground_offset

    # Root position offset to start at origin (XY plane)
    ROOT_ORIGIN_OFFSET = True
    if ROOT_ORIGIN_OFFSET:
        root_pos[:, :2] -= root_pos[0, :2]

    # Prepare motion data dictionary
    motion_data = {
        "fps": tgt_fps,
        "root_pos": root_pos,
        "root_rot": root_rot,
        "dof_pos": dof_pos,
        "local_body_pos": local_body_pos.detach().cpu().numpy(),
        "link_body_list": body_names,
    }

    # Save to file
    os.makedirs(os.path.dirname(tgt_file_path), exist_ok=True)
    with open(tgt_file_path, "wb") as f:
        pickle.dump(motion_data, f)

    # Progress tracking
    done = 0
    for _, _, files in os.walk(tgt_folder):
        done += len([f for f in files if f.endswith('.pkl')])
    print(f"[PROGRESS] Processed {done}/{total_files}: {tgt_file_path}")

    # Clean up memory
    if torch.cuda.is_available():
        torch.cuda.empty_cache()
    gc.collect()


def main():
    # Set multiprocessing start method to 'spawn' for CUDA compatibility
    mp.set_start_method('spawn', force=True)

    parser = argparse.ArgumentParser(
        description="Batch process XRT body tracking files to robot motion datasets"
    )
    parser.add_argument(
        "--robot",
        default="unitree_g1",
        choices=[
            "unitree_g1", "unitree_g1_with_hands", "unitree_h1", "unitree_h1_2",
            "booster_t1", "booster_t1_29dof", "stanford_toddy", "fourier_n1",
            "engineai_pm01", "kuavo_s45", "hightorque_hi", "galaxea_r1pro",
            "berkeley_humanoid_lite", "booster_k1", "pnd_adam_lite",
            "openloong", "tienkung"
        ],
        help="Target robot model"
    )
    parser.add_argument(
        "--src_folder",
        type=str,
        required=True,
        help="Source folder containing XRT pickle files"
    )
    parser.add_argument(
        "--tgt_folder",
        type=str,
        required=True,
        help="Target folder for retargeted robot motion files"
    )
    parser.add_argument(
        "--override",
        default=False,
        action="store_true",
        help="Override existing files in target folder"
    )
    parser.add_argument(
        "--num_cpus",
        default=4,
        type=int,
        help="Number of CPUs to use for parallel processing"
    )
    parser.add_argument(
        "--verbose",
        default=False,
        action="store_true",
        help="Enable verbose memory logging"
    )
    args = parser.parse_args()

    # Print system info
    print(f"[cyan]Total CPUs: {mp.cpu_count()}[/cyan]")
    print(f"[cyan]Using {args.num_cpus} CPUs[/cyan]")
    print(f"[cyan]CUDA available: {torch.cuda.is_available()}[/cyan]")

    src_folder = args.src_folder
    tgt_folder = args.tgt_folder

    # Build list of files to process
    file_args_list = []
    for dirpath, _, filenames in os.walk(src_folder):
        for filename in natsorted(filenames):
            if filename.endswith(".pkl"):
                xrt_file_path = os.path.join(dirpath, filename)
                # Maintain directory structure in target folder
                tgt_file_path = xrt_file_path.replace(src_folder, tgt_folder)

                # Add to processing list if file doesn't exist or override is enabled
                if not os.path.exists(tgt_file_path) or args.override:
                    file_args_list.append((xrt_file_path, tgt_file_path, args.robot, tgt_folder))

    total_files = len(file_args_list)
    print(f"[green]Total number of files to process: {total_files}[/green]")

    if total_files == 0:
        print("[yellow]No files to process. Exiting.[/yellow]")
        return

    # Process files in parallel
    with mp.Pool(args.num_cpus) as pool:
        pool.starmap(process_file, [file_args + (total_files, args.verbose) for file_args in file_args_list])

    print(f"[green bold]Done! Saved to {tgt_folder}[/green bold]")


if __name__ == "__main__":
    main()
