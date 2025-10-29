import argparse
import pathlib
import time
import signal
import sys

import numpy as np

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting import RobotMotionViewer
from general_motion_retargeting.utils.xrt import connect_xrt_realtime, get_xrt_frame_realtime, estimate_human_height, save_xrt_file

from rich import print

# Global variables for graceful shutdown
running = True
xrt_client = None
robot_motion_viewer = None


def signal_handler(_sig, _frame):
    """Handle Ctrl+C gracefully."""
    global running
    print("\n[yellow]Shutting down...[/yellow]")
    running = False


if __name__ == "__main__":

    HERE = pathlib.Path(__file__).parent

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--robot",
        choices=["unitree_g1", "unitree_g1_with_hands", "unitree_h1", "unitree_h1_2",
                 "booster_t1", "booster_t1_29dof","stanford_toddy", "fourier_n1",
                "engineai_pm01", "kuavo_s45", "hightorque_hi", "galaxea_r1pro", "berkeley_humanoid_lite", "booster_k1",
                "pnd_adam_lite", "openloong", "tienkung"],
        default="unitree_g1",
        help="Target robot model.",
    )

    parser.add_argument(
        "--save_path",
        default=None,
        help="Path to save the captured XRT body tracking data (optional).",
    )

    parser.add_argument(
        "--visualize",
        default=True,
        action="store_true",
        help="Show MuJoCo visualization.",
    )

    parser.add_argument(
        "--no_visualize",
        dest="visualize",
        action="store_false",
        help="Disable MuJoCo visualization.",
    )

    parser.add_argument(
        "--record_video",
        default=False,
        action="store_true",
        help="Record the video.",
    )

    parser.add_argument(
        "--human_height",
        type=float,
        default=None,
        help="Human height in meters (if not specified, will estimate from first frame).",
    )

    parser.add_argument(
        "--rate_limit",
        default=False,
        action="store_true",
        help="Limit the rate to 30 FPS (otherwise runs as fast as possible).",
    )

    args = parser.parse_args()

    # Set up signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)

    # Connect to XRoboToolkit
    print("[cyan]Connecting to XRoboToolkit SDK...[/cyan]")
    xrt_client = connect_xrt_realtime()

    if xrt_client is None:
        print("[red]Failed to connect to XRoboToolkit SDK. Make sure the service is running.[/red]")
        sys.exit(1)

    # Wait for body tracking data to be available
    print("[cyan]Waiting for body tracking data...[/cyan]")
    frame_data = None
    while frame_data is None and running:
        frame_data = get_xrt_frame_realtime(xrt_client)
        if frame_data is None:
            time.sleep(0.01)

    if not running:
        sys.exit(0)

    print("[green]Body tracking data is available![/green]")

    # Estimate human height if not provided
    actual_human_height = args.human_height
    if actual_human_height is None:
        actual_human_height = estimate_human_height(frame_data)
        print(f"Estimated human height: {actual_human_height:.2f}m")
    else:
        print(f"Using specified human height: {actual_human_height:.2f}m")

    # Initialize the retargeting system
    print(f"[cyan]Initializing retargeting for {args.robot}...[/cyan]")
    retarget = GMR(
        actual_human_height=actual_human_height,
        src_human="xrt",
        tgt_robot=args.robot,
    )

    # Initialize viewer if visualization is enabled
    if args.visualize:
        robot_motion_viewer = RobotMotionViewer(
            robot_type=args.robot,
            motion_fps=30,
            transparent_robot=0,
            record_video=args.record_video,
            video_path=f"videos/{args.robot}_xrt_realtime.mp4",
        )

    # Storage for saving data
    captured_frames = []

    # FPS measurement variables
    fps_counter = 0
    fps_start_time = time.time()
    fps_display_interval = 2.0  # Display FPS every 2 seconds

    print("[green]Starting real-time retargeting! Press Ctrl+C to stop.[/green]")

    # Main loop
    while running:
        try:
            # Get current body tracking data
            frame_data = get_xrt_frame_realtime(xrt_client)

            if frame_data is None:
                # No data available, wait a bit
                time.sleep(0.001)
                continue

            # Retarget to robot (with ground alignment to handle headset-centered coordinates)
            qpos = retarget.retarget(frame_data, offset_to_ground=True)

            # Visualize if enabled
            if args.visualize and robot_motion_viewer is not None:
                robot_motion_viewer.step(
                    root_pos=qpos[:3],
                    root_rot=qpos[3:7],
                    dof_pos=qpos[7:],
                    human_motion_data=retarget.scaled_human_data,
                    human_pos_offset=np.array([0.0, 0.0, 0.0]),
                    show_human_body_name=False,
                    rate_limit=args.rate_limit,
                )

            # Save frame if requested
            if args.save_path is not None:
                captured_frames.append(frame_data)

            # FPS measurement
            fps_counter += 1
            current_time = time.time()
            if current_time - fps_start_time >= fps_display_interval:
                actual_fps = fps_counter / (current_time - fps_start_time)
                print(f"[yellow]Actual retargeting FPS: {actual_fps:.2f}[/yellow]")
                fps_counter = 0
                fps_start_time = current_time

            # Rate limiting if enabled
            if args.rate_limit and not args.visualize:
                time.sleep(1/30)

        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"[red]Error during retargeting: {e}[/red]")
            import traceback
            traceback.print_exc()
            break

    # Save captured data if requested
    print(f"[cyan]Captured {len(captured_frames)} frames total[/cyan]")
    if args.save_path is not None and len(captured_frames) > 0:
        print(f"[cyan]Saving {len(captured_frames)} frames to {args.save_path}...[/cyan]")
        save_xrt_file(captured_frames, args.save_path)
        print(f"[green]Saved to {args.save_path}[/green]")
    elif args.save_path is not None and len(captured_frames) == 0:
        print(f"[yellow]No frames captured, nothing to save[/yellow]")

    # Cleanup
    if xrt_client is not None:
        xrt_client.disconnect()

    if robot_motion_viewer is not None:
        robot_motion_viewer.close()

    print("[green]Shutdown complete.[/green]")
