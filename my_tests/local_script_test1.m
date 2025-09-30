% Robotarium Experiment

%% Resolve this script's directory (so paths work no matter where you run from)
script_dir = fileparts(mfilename('fullpath'));

%% Load the environment (env.json)
env = load_json_data(script_dir, 'env.json');
agents = env.agents;
n_agents = numel(agents);
mapData = env.map;
dims = mapData.dimensions;

% Other information can also be retrieved from "env" as below.
% delays = env.delays;
% scale = mapData.scale;
% goal_locations = mapData.goal_locations;
% non_task_endpoints = mapData.non_task_endpoints;
% obstacles = mapData.obstacles;
% start_locations = mapData.start_locations;
% n_delays_per_agent = env.n_delays_per_agent;
% n_tasks = env.n_tasks;
% task_freq = env.task_freq;
% tasks = env.tasks;


%% Load the schedule data (schedule.json)
schedule = load_json_data(script_dir, 'schedule.json');
agent_schedule = schedule.schedule;
agent_names = fieldnames(agent_schedule);

% Other information can also be retrieved from "schedule" as below.
% completed_tasks_times = schedule.completed_tasks_times;
% cost = schedule.cost;
% dyn_ob_path = schedule.dyn_ob_path;
% n_replans = schedule.n_replans;
% schedule_agent1 = agent_schedule.agent1;
% schedule_agent2 = agent_schedule.agent2;
% schedule_agent3 = agent_schedule.agent3;
% schedule_agent4 = agent_schedule.agent4;
% schedule_agent5 = agent_schedule.agent5;
% schedule_agent6 = agent_schedule.agent6;


%% Load the warehouse image
img = load_background(script_dir, 'warehouse_image.png');


%% Load the experiment parameters
config = load_json_data(script_dir, 'config.json');
N = config.N; % Number of robots utilized for the experiment
exp_time = config.exp_time; % Total experiment time in seconds
limits = config.limits; % Robotarium boundary limit in meters
img_e = config.img_e; % Background shift from the center in the Robotarium frame
waypoint_e = config.waypoint_e; % Waypoints shift from the center in Robotarium

% control_params(1): XVelocityGain in SI dynamics
% control_params(2): YVelocityGain in SI dynamics
% control_params(3): LinearVelocityGain in Unicycle dynamics
% control_params(4): AngularVelocityLimit in Unicycle dynamics
% control_params(5): Safe radius
control_params = config.control_params;


%% Trajectory calculation for all the robots in the Robotarium frame
% Loading waypoints from the loaded Neural ATTF data
waypoints_A = zeros(2, n_agents, size(agent_schedule.agent1,1));
for i = 1:size(waypoints_A, 3)
    for j = 1:size(waypoints_A, 2)
        waypoints_A(1, j, i) = agent_schedule.(agent_names{j})(i).x;
        waypoints_A(2, j, i) = agent_schedule.(agent_names{j})(i).y;
    end
end

% Waypoints tranfromation from Neural ATTF frame to Robotarium frame
waypoints_R = utility.transform_A_to_R(waypoints_A, dims, limits, waypoint_e);

% Interpolating trajectory points at 30Hz (each Robotarium time step)
traj = utility.linear_interpolator(waypoints_R);


%% Set the background and perform the experiment
utility.perform_experiment(N, control_params, exp_time, traj, img, limits, img_e);


%% Function Definitions
% -------------------------------------------------------------------------
% Function to load data from .json files
function data = load_json_data(dir, json_file)
    path = fullfile(dir, json_file);
    text = fileread(path);
    data = jsondecode(text);
end

% -------------------------------------------------------------------------
% Function to load background image
function img = load_background(dir, img_file)
    path = fullfile(dir, img_file);
    if exist(path, 'file')
        img = imread(path);
    else
        warning('Image file not found. Skipping background.');
        img = [];
    end
end