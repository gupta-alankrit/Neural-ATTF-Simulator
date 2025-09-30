import numpy as np
import matplotlib.pyplot as plt
from matplotlib.cm import get_cmap

from rps.robotarium import Robotarium
from rps.utilities.controllers import create_si_position_controller
from rps.utilities.barrier_certificates import create_single_integrator_barrier_certificate_with_boundary
from rps.utilities.transformations import create_si_to_uni_dynamics_with_backwards_motion, create_si_to_uni_mapping

def transform_A_to_R(w_A, dims=[66, 35], limits=[1.6, 1.0], waypoint_e=[0.0, 0.0]):
    '''
    This function transforms the waypoints in the warehouse frame
    from the Json file to the Robotarium frame.

    Args:
        w_A: Waypoints in the warehouse frame in meters.
        dims: Dimensions of the warehouse floor (the warehouse frame) in meters.
        limits: Half lenghts of the Robotarium floor (the Robotarium frame) in meters.
        waypoint_e: The offset between the center of warehouse floor to the center of 
                    the Robotarium floor in meters.

    Returns:
        w_R: Waypoints in the Robotarium frame in meters.
    '''

    # Converting args into NumPy arrays
    w_A = np.asarray(w_A)
    dims = np.asarray(dims, dtype=float)
    limits = np.asarray(limits, dtype=float)
    waypoint_e = np.asarray(waypoint_e, dtype=float)

    # Calculating a magnification matrix "M" between the two frames
    alpha = (2*limits[0])/dims[0]
    beta = (2*limits[1])/dims[1]
    M = np.array([[alpha, 0], [0, beta]], dtype=float)

    # Calculating transformed waypoints in the Robotarium frame.
    # Note that there is no rotation between the two frames, only translation.
    offset = np.array(limits - waypoint_e, dtype=float)
    w_R = np.zeros(w_A.shape, dtype=float)
    for i in range(w_R.shape[0]):
        temp = np.matmul(M, w_A[i])
        for j in range(w_R.shape[2]):
            w_R[i, :, j] = temp[:, j] - offset

    return w_R
######################################################################################


def linear_interpolator(waypoints, freq=30):
    '''
    This function takes the waypoints at 1 Hz (time step 1 sec) and creates
    waypoints at 30 Hz (time step 0.033 sec) by linearly interpolating points
    between any two given points.

    Args:
        waypoints: Waypoints at 1 Hz
        freq: Frequency at which the output waypoints are generated

    Returns:
        trajectory: Linearly interpolated waypoints at the given frequency
    '''

    waypoints = np.asarray(waypoints)
    l, h, w = waypoints.shape
    trajectory = np.zeros((1+(l-1)*freq, h, w), dtype=float)
    for i in range(l-1):
        ds = (waypoints[i+1, :, :] - waypoints[i, :, :])/freq
        for j in range(freq):
            trajectory[freq*i + j, :, :] = waypoints[i, :, :] + j*ds
    trajectory[-1, :, :] = waypoints[-1, :, :]

    return trajectory
######################################################################################


def perform_experiment(N, control_params, traj, img, exp_time=133, limits=[1.6, 1.0], img_e=[0.0, 0.0], playback=1.0):
    '''
    This function sets a background image and completes the experiment in the
    Robotarium simulator.

    Args:
        N: Number of robots
        control_params: [si_x_vel_gain, si_y_vel_gain, uni_lin_vel_gain, uni_ang_vel_limit, safe_rad]
        traj: Waypoints of each robot at each time step in the Robotarium frame
        img: Background image
        exp_time: Total experiment time in seconds.
        limits: Working half-width and half-height of the Robotarium arena.
                The actual limits of the robotarium arena are set as default.
        img_e: Offset between the centers of Robotarium arena and
                the background image in the Robotarium frame.
    '''

    limits = np.asarray(limits, dtype=float)
    img_e  = np.asarray(img_e, dtype=float)
    si_vx_gain, si_vy_gain, uni_lin_vel_gain, uni_ang_vel_limit, safe_r = map(float, control_params)

    # Initiating the Robotarium object
    r = Robotarium(number_of_robots=N, show_figure=True)
    ax = plt.gca()

    # Background image with arena-aligned extent (no vertical flip needed if origin='lower')
    x_min = -limits[0] - img_e[0]
    x_max =  limits[0] - img_e[0]
    y_min = -limits[1] + img_e[1]
    y_max =  limits[1] + img_e[1]
    ax.imshow(np.flipud(img), extent=[x_min, x_max, y_min, y_max], origin='lower', interpolation='nearest')
    # ax.set_xlim(-limits[0], limits[0])
    # ax.set_ylim(-limits[1], limits[1])
    ax.set_xlim(-1.6, 1.6)
    ax.set_ylim(-1, 1)

    # Initiating a trail manager for the drawing the past positions of the robot
    seg_len = 100
    trail = create_trail_manager(N, seg_len, ax=ax)

    # Creating a position controller in the single-integrator dynamics
    si_pos_K = create_si_position_controller(
        x_velocity_gain=si_vx_gain,
        y_velocity_gain=si_vy_gain
    )

    # Creating barrier certificate instance to ensure safety.
    # (This avoid robot-robot and robot-boundary collisions)
    si_barrier_certificate  = create_single_integrator_barrier_certificate_with_boundary(
        safety_radius=safe_r,
        barrier_gain=1e6,          # "UnsafeBarrierGain" analog
    )

    # Creating instances for converting unicycle dynamics to single-integrator dynamics and vice-versa
    _, uni_to_si_states = create_si_to_uni_mapping()
    si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion(
        linear_velocity_gain=uni_lin_vel_gain,
        angular_velocity_limit=uni_ang_vel_limit
    )

    # Main loop
    iterations = round((int(exp_time * 30)/playback)) + 10
    robot_ids = np.arange(N)
    L = traj.shape[0]

    for t in range(iterations):
        # Current poses
        x = r.get_poses()           # 3 x N: [x, y, theta]
        x_si = uni_to_si_states(x)  # 2 x N: [x, y]

        # Picking the goal positoins for the current time step
        idx = min(t, L-1)
        x_goal = traj[idx, :, :] # shape (2, N)

        # Single integrator dynamics control to desired positions
        dx_si = si_pos_K(x_si, x_goal)

        # Enforce barrier certificates and mapping the dyanmics to unicycle model
        dx_si_safe = si_barrier_certificate(dx_si, x_si)
        dx_uni = si_to_uni_dyn(dx_si_safe, x)

        # Clipping the robots' velocities wihtin the maximum limits
        v_max = getattr(r, "max_linear_velocity", 0.2)
        w_max = getattr(r, "max_angular_velocity", np.pi)
        dx_uni[0, :] = np.clip(dx_uni[0, :], -v_max, v_max)
        dx_uni[1, :] = np.clip(dx_uni[1, :], -w_max, w_max)

        # Apply and step
        r.set_velocities(robot_ids, dx_uni)
        r.step()

        # Update trail with current (x,y)
        update_trails(trail, x[:2, :])

    # End-of-script hook (saves data/figures, releases sim, etc., per RPS)
    r.call_at_scripts_end()

    return
######################################################################################


def create_trail_manager(N, seg_len, ax=None):
    '''
    This function creates a trajectory trail of the given "seg_len"
    number of past iterations for each of the robot.

    Args:
        N: Number of robots
        seg_len : Number of past iterations to keep
        ax : Axes to draw on. Defaults to current axes.

    Returns:
        trail : dict
            {
            'segLen': int,
            'histX' : np.ndarray of shape (seg_len+1, N),
            'histY' : np.ndarray of shape (seg_len+1, N),
            'hSeg'  : list[matplotlib.lines.Line2D] length N
            }
    '''

    if ax is None:
        ax = plt.gca()

    trail = {
        'segLen': seg_len,
        'histX': np.full((seg_len + 1, N), np.nan, dtype=float),
        'histY': np.full((seg_len + 1, N), np.nan, dtype=float),
        'hSeg': [None] * N,
    }

    # Use a qualitative colormap for discrete N colors
    cmap = get_cmap('tab20', N)

    for k in range(N):
        color = cmap(k)
        line, = ax.plot([], [], linewidth=2, color=color, clip_on=True)
        trail['hSeg'][k] = line

    return trail
######################################################################################


def update_trails(trail, x_y):
    '''
    This function updates the trajector trail of each of the robot at each time instant.

    Args:
        trail: As returned by create_trail_manager(...).
                A dict. Keys: 'segLen', 'histX', 'histY', 'hSeg'
        x_y: Current positions, shape (2, N): first row = x, second row = y.

    Returns:
        trail: Updated trail.
    '''

    x_y = np.asarray(x_y, dtype=float)
    if x_y.shape[0] != 2 or x_y.shape[1] != trail['histX'].shape[1]:
        raise ValueError("x_y must have shape (2, N) matching trail['histX'] columns.")

    # Shift history up and append newest positions at the end
    trail['histX'][:-1, :] = trail['histX'][1:, :]
    trail['histY'][:-1, :] = trail['histY'][1:, :]
    trail['histX'][-1, :] = x_y[0, :]
    trail['histY'][-1, :] = x_y[1, :]

    # Update each line with available (non-NaN) history
    N = len(trail['hSeg'])
    for k in range(N):
        hx = trail['histX'][:, k]
        hy = trail['histY'][:, k]

        # Find first non-NaN in hx
        valid_start_mask = ~np.isnan(hx)
        if not valid_start_mask.any():
            continue
        first_valid = int(np.argmax(valid_start_mask))

        xpath = hx[first_valid:]
        ypath = hy[first_valid:]
        valid = ~np.isnan(xpath) & ~np.isnan(ypath)

        line = trail['hSeg'][k]
        if line is not None:
            line.set_data(xpath[valid], ypath[valid])

    return trail
######################################################################################

# Other miscellaneous functions and classes
from pathlib import Path
import json

class objectview(object):
    '''Allows to use dict.key instead of dict["key"] in a dictionary.'''
    def __init__(self, d):
        self.__dict__ = d
        self.__json__ = None
    

# Prefers to use "imageio" library if available (handles PNG/JPG consistently).
try:
    import imageio.v2 as imageio
    _IMG_LOADER = "imageio"
except Exception:
    from matplotlib import image as mpl_image
    _IMG_LOADER = "mpl"


def load_background(directory: Path, img_file: str):
    '''Load the background image (if present); otherwise return None.'''
    path = directory / img_file
    if path.exists():
        if _IMG_LOADER == "imageio":
            return imageio.imread(path)
        else:
            return mpl_image.imread(path.as_posix())
    else:
        print("Warning: Image file not found. Skipping background.")
        return None