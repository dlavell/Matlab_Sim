

% t*10^3 gives about t hours of sim time for 5m/s vehicles 
% example... 10000 = 10*10^3 gives about 10 hours of sim time
T = 5000; 
ref.time = 0:.01:T*3;
t = ref.time;


%% Initial condition:
    % sets initial condition for integrator
    % sets warehouse origin in destination generator
IC = Simulink.Parameter;
IC.DataType = 'double';

%% max Radius:
    % sets maximum radius for deliveries around current warehouse
maxRadius = Simulink.Parameter;
maxRadius.DataType = 'double';

%% request generator:
%
pop_density = 1;
sim_start = 0;
sim_step = 1;
sim_stop = T;
MAX_REQUEST_PER_SIM = 20;
%
req_que = Simulink.Parameter;
req_que.DataType = 'double';
req_que.Value = req_queue_generator(pop_density,sim_start,sim_step,sim_stop,MAX_REQUEST_PER_SIM);


%% current waypoint:
curr_wp = Simulink.Signal;
curr_wp.DataType = 'double';
curr_wp.Complexity = 'real';
curr_wp.SamplingMode = 'Sample based';

%% current trajectory point:
curr_tp = Simulink.Signal;
curr_tp.DataType = 'double';
curr_tp.Complexity = 'real';
curr_tp.SamplingMode = 'Sample based';

%% list of destinations:
destinations = Simulink.Signal;
destinations.DataType = 'double';
destinations.Complexity = 'real';
destinations.SamplingMode = 'Sample based';

%% list of trajectories:
trajectories = Simulink.Signal;
trajectories.DataType = 'double';
trajectories.Complexity = 'real';
trajectories.SamplingMode = 'Sample based';

%% Queue of requests:
current_que = Simulink.Signal;
current_que.DataType = 'double';
current_que.Complexity = 'real';
current_que.SamplingMode = 'Sample based';

%% Queue of requests:
n_quads = Simulink.Signal;
n_quads.DataType = 'double';
n_quads.Complexity = 'real';
n_quads.SamplingMode = 'Sample based';

%% Position of quadcopters:
pos = Simulink.Signal;
pos.DataType = 'double';
pos.Complexity = 'real';
pos.SamplingMode = 'Sample based';

%% Time stamp:
time_stamp = Simulink.Signal;
time_stamp.DataType = 'double';
time_stamp.Complexity = 'real';
time_stamp.SamplingMode = 'Sample based';

%% Queue_idx :
Queue_idx = Simulink.Signal;
Queue_idx.DataType = 'double';
Queue_idx.Complexity = 'real';
Queue_idx.SamplingMode = 'Sample based';

%% Elevation
elevation = Simulink.Parameter;
elevation.DataType = 'double';
data = csvread('SJ-lat-lng-3decimal.csv');
elevation.Value = data;


