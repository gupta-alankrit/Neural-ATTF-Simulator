import json
import yaml
from pathlib import Path
import numpy as np
import utility

def load_json_data(directory: Path, json_file: str):
    '''Load and parse a JSON file from the given directory, returns a plain dict.'''
    path = directory / json_file
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)
    
def load_yaml_data(dir, yaml_file):
    '''Load a YAML file from the given directory, returns a plain dict.'''
    path = dir / yaml_file
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)
    

def main():
    # Resolve this script's directory (so paths work no matter where the file is run from)
    script_dir = Path(__file__).resolve().parent

    # Load the environment data
    env_dict = load_json_data(script_dir, "env.json")
    env = utility.objectview(env_dict)
    map_data = utility.objectview(env.map)
    dims = map_data.dimensions
    agents = env.agents
    n_agents = len(agents)

    # Load the schedule data
    schedule_dict = load_json_data(script_dir, "schedule.json")
    agent_schedule = schedule_dict["schedule"]
    agent_names = list(agent_schedule.keys())

    # Load the background image
    img = utility.load_background(script_dir, "warehouse_image.png")

    # Load the experiment parameters
    config_dict = load_yaml_data(script_dir, "config.yaml")
    config = utility.objectview(config_dict)

    N = int(config.N)                      # Number of robots utilized for the experiment
    exp_time = float(config.exp_time)      # Total experiment time in seconds
    limits = np.array([config.limits["x_lim"], config.limits["y_lim"]], dtype=float)     # Robotarium arena half-lengths in meters
    img_e = np.array(config.img_e, dtype=float)       # [x_offset, y_offset]
    waypoint_e = np.array(config.waypoint_e, dtype=float)

    control_params = utility.objectview(config.control_params)
    vx_K = control_params.si_x_vel_gain       # XVelocityGain(SI)
    vy_K = control_params.si_y_vel_gain       # YVelocityGain(SI)
    v_K = control_params.uni_lin_vel_gain     # LinearVelocityGain(Unicycle)
    w_lim = control_params.uni_ang_vel_limit  # AngularVelocityLimit(Unicycle)
    safe_r = control_params.safe_rad          # Safe radius
    control_params = np.array([vx_K, vy_K, v_K, w_lim, safe_r], dtype=float)

    # Build waypoints from Neural ATTF schedule data
    if not agent_names:
        raise ValueError("No agents found in schedule.json")

    first_agent_waypoints = agent_schedule[agent_names[0]]
    L = len(first_agent_waypoints)
    waypoints_A = np.zeros((L, 2, n_agents), dtype=float)
    for i in range(L):
        for j, name in enumerate(agent_names):
            wp_dict = agent_schedule[name][i]
            waypoints_A[i, 0, j] = float(wp_dict["x"])
            waypoints_A[i, 1, j] = float(wp_dict["y"])

    # Transform waypoints from the warehouse frame to the Robotarium frame
    waypoints_R = utility.transform_A_to_R(waypoints_A, dims, limits, waypoint_e)

    # Interpolate trajectory points at 30 Hz (each Robotarium time step)
    traj = utility.linear_interpolator(waypoints_R, freq=30)

    # Set the background and perform the experiment
    utility.perform_experiment(N, control_params, traj, img, exp_time, limits, img_e)


if __name__ == "__main__":
    main()