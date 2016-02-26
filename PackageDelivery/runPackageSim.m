
% Initialize workspace
warning ('off','all');
display('Initializing Sim...')
load('initialize_PackageDeliverySim.mat');
T = 200;
ref.time = 0:.01:T*3;             % correct for non-linear time variable
save('initialize_PackageDeliverySim.mat');

display('Running Sim...')
sim PackageDeliverySim          % Run Simulation

display('Sim finished, saving results...')
save('SimResults.mat');         % Save workspace

display('Creating Sim video...')
graphicx                        % Run graphics and create video