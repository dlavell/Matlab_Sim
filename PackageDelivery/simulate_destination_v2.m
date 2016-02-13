%
% Simulate destination location
%
% Inputs 
%
% lambda_s - Latitude of the origin (degrees)
% tau_s - Longitude of the origin (degrees)
% desired_radius_min - Desired minimum radius (nautical-miles)
% desired_radius_max - Desired maximum radius (nautical-miles)
%
% Outputs
%
% lambda - Latitude of the destination (degrees)
% tau - Longitude of the destination (degrees)
%
function [lambda,tau]=simulate_destination_v2(lambda_s,tau_s,desired_radius_min,desired_radius_max)
%
% Pick a uniform random number between -desired_radius_min and
% desired_radius_max using the Matlab function "rand"
%
    desired_radius = desired_radius_min + (desired_radius_max - desired_radius_min)*rand;
%
% Determine the sign by using the uniform random generator "rand" between -1 and 1.
%
    sign_x_or_y = sign(-1 + 2*rand);
%
if (sign_x_or_y == 1) % If positive, choose x-coordinate
    %
    % Pick a uniform random number between -desired_radius and desired_radius
    % using the Matlab function "rand"
    %
    x = -desired_radius + 2*desired_radius*rand;
    %
    % Determine the y-coordinate using desired_radius as a constraint.
    %
    y = sqrt(desired_radius^2 - x^2);
    %
    % Determine the sign of the y-coordinate by using the uniform random
    % generator "rand" between -1 and 1.
    %
    sign_y = sign(-1 + 2*rand);
    y = sign_y*y;
    %
else % choose y
    %
    % Pick a uniform random number between -desired_radius and desired_radius
    % using the Matlab function "rand"
    %
    y = -desired_radius + 2*desired_radius*rand;
    %
    % Determine the y-coordinate using desired_radius as a constraint.
    %
    x = sqrt(desired_radius^2 - y^2);
    %
    % Determine the sign of the x-coordinate by using the uniform random
    % generator "rand" between -1 and 1.
    %
    sign_x = sign(-1 + 2*rand);
    x = sign_x*x;
    %
end
    
% So now we have both x and y with respect to origin at (0, 0).
%
% We call the inverse oblique perspective projection equation to get the
% coordinates of the destination.
%
[lambda,tau]=inv_oblique_stereographic(lambda_s,tau_s,x,y);
%
% "lambda" and "tau" are the latitude and longitude of the destination location.
%