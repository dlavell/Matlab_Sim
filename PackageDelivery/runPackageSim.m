
% Initialize workspace
clear all
warning ('off','all');
display('Initializing Sim...')

warehouses = [37.3154997	-121.8728929;
              37.2912864	-121.9896046;
              37.313593	    -121.7731222;
              37.3217542	-121.9732432;
              37.2554222	-121.8801746;
              37.3310001	-121.860433 ];


%% Macro constants
SetSimVariables();

numRuns = length(warehouses(:,1));
%% Iterate over all warehouses
for simRun = 1:numRuns
    %% set global vaiables
    % set initial condition - different for each run
    IC.Value = repmat([warehouses(simRun,1) ; warehouses(simRun,2) ; 0], 100, 1);
    
    % clear/reset global variables
    curr_wp      .InitialValue = 'zeros(100,1)';
    curr_tp      .InitialValue = 'ones(100,1)*2';
    destinations .InitialValue = 'zeros(100,3)';
    trajectories .InitialValue = 'zeros(7,3,100)';
    current_que  .InitialValue = 'req_que';
    n_quads      .InitialValue = '10';
    pos          .InitialValue = 'zeros(100,3)';
    time_stamp   .InitialValue = '1';
    Queue_idx    .InitialValue = '1';

    % determine max radius of delivery relative to next closest warehouse
    maxDist = inf;
    for w = 1:numRuns
       % skip current warehouse
       if(warehouses(simRun,:) == warehouses(w,:)) continue; end 
       dist = deg2nm(warehouses(simRun,1),warehouses(simRun,2),...
           warehouses(w,1),warehouses(w,2));
       % find closet warehouse
       if(dist < maxDist) maxDist = dist; end
    end
    maxRadius.Value = 8;
    
    %% Run Simulation
    fprintf('Running Sim for warehouse %d of %d\n',simRun,numRuns)
    sim('PackageDeliverySim')
    
    
    %% Save quadcopter states into one variable quads for video.... NEEDS WORK
    if(simRun == 1)
        quads = state;
        break;
    else % voodoo magic
        if(length(quads.signals.values(:,1)) > length(state.signals.values(:,1)))
            quads.signals.values(1:length(state.signals.values(:,1)),((simRun*30)-29):(simRun*30)) = state.signals.values(:,1:30);
        else
            quads.signals.values(:,((simRun*30)-29):(simRun*30)) = state.signals.values(1:length(quads.signals.values(:,1)),1:30);    
        end
    end
end
%% Save results
display('Sim finished, saving results...')
%save('SimResults2.mat');    

%% Run graphics and create video
display('Creating Sim video...')
graphicx

%%Post processing
display('Performing Postprocess Analysis')
i =1;
start_idx = 1;
stop_idx = 3;
distance = zeros(1,10,'uint32')


while i < str2num(n_quads.InitialValue)
    distance(i) = post_processing.calc_distance(state.signals.values(:,start_idx:stop_idx))
    i = i +1;
    start_idx = start_idx + 3;
    stop_idx =  stop_idx +3;
end

formatSpec = 'Average Distance covered by a quadcopter is: %d';
Average_distance = sum(distance)/ str2num(n_quads.InitialValue);
str = sprintf(formatSpec,Average_distance)
display(str);



%calculate distance covered by each quadcopters