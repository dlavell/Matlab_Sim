
% Initialize workspace
warning ('off','all');
display('Initializing Sim...')
load('initialize_PackageDeliverySim.mat');
T = 10;                        % Change time scale
ref.time = 0:.2:20;             % correct for non-linear time variable

display('Running Sim...')
sim PackageDeliverySim          % Run Simulation

display('Sim finished, saving results...')
save('SimResults.mat');         % Save workspace

display('Creating Sim video...')
graphicx                        % Run graphics and create video

