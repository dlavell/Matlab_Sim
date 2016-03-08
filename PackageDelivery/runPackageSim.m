
% Initialize workspace
clear all
warning ('off','all');
display('Initializing Sim...')
%load('initialize_PackageDeliverySim.mat');
T = 10000;
ref.time = 0:.0001:T*3;
%save('initialize_PackageDeliverySim.mat');

warehouses = [37.3154997	-121.8728929;
              37.2912864	-121.9896046;
              37.313593	    -121.7731222;
              37.3217542	-121.9732432;
              37.2554222	-121.8801746;
              37.3310001	-121.860433 ];


R = 6.371*10^6;              % mean radius of earth (meters)

% Initial condition:
    % sets initial condition for integrator
    % sets warehouse origin in destination generator
IC = Simulink.Parameter;
IC.DataType = 'double';

% max Radius:
    % sets maximum radius for deliveries around current warehouse
maxRadius = Simulink.Parameter;
maxRadius.DataType = 'double';

numRuns = length(warehouses(:,1));


for simRun = 1:numRuns
    IC.Value = repmat([warehouses(simRun,1) ; warehouses(simRun,2) ; 0], 100, 1);

    % determine maximum radius of delivery relative to next closest
    % warehouse
    maxDist = inf;
    for w = 1:numRuns
       % skip current warehouse
       if(warehouses(simRun,:) == warehouses(w,:)) continue; end 
       dist = deg2nm(warehouses(simRun,1),warehouses(simRun,2),...
           warehouses(w,1),warehouses(w,2));
       % find closet warehouse
       if(dist < maxDist) maxDist = dist; end
    end
    maxRadius.Value = maxDist/2;

    display('Running Sim...')
    sim('PackageDeliverySim')          % Run Simulation
    
    
    
    
    % Save quadcopter states into one variable quads for video.... NEEDS WORK
    if(simRun == 1)
        quads = state;
    else % voodoo magic
        if(length(quads.signals.values(:,1)) > length(state.signals.values(:,1)))
            quads.signals.values(1:length(state.signals.values(:,1)),((simRun*30)-29):(simRun*30)) = state.signals.values(:,1:30);
        else
            quads.signals.values(:,((simRun*30)-29):(simRun*30)) = state.signals.values(1:length(quads.signals.values(:,1)),1:30);    
        end
    end
end

display('Sim finished, saving results...')
save('SimResults2.mat');         % Save workspace

display('Creating Sim video...')
graphicx                        % Run graphics and create video

clear all