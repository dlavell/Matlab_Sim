%*********************************************************
%
%
% Inverse oblique stereographic projection equations
%
% Inputs 
%
% lambda_s - Latitude of the origin (degrees)
% tau_s - Longitude of the origin (degrees)
% x - x-coordinate of the projected point (nautical miles)
% y - y-coordinate of the projected point (nautical miles)
%
% Outputs
%
% lambda - Latitude of the destination (degrees)
% tau - Longitude of the destination (degrees)
%
%*********************************************************
%
function [lambda,tau]=inv_oblique_stereographic(lambda_s,tau_s,x,y)
%
%
radius_earth = 3444.046647; % (nautical-miles)
deg_to_rad = pi/180;
rad_90 = 90*deg_to_rad;
%
lambda_s = deg_to_rad*lambda_s;
tau_s = deg_to_rad*tau_s;
%
rho = sqrt(x^2 + y^2);
c = 2*atan2(rho,2*radius_earth);
%
if (rho == 0)
    lambda = lambda_s;
else
	lambda = asin(cos(c)*sin(lambda_s)+(y*sin(c)*cos(lambda_s))/rho);
end
%
if (lambda_s == rad_90)
    tau = tau_s + atan2(x,-y);
elseif(lambda_s == -rad_90)
    tau = tau_s + atan2(x,y);
else
    tau = tau_s + atan2(x*sin(c),(rho*cos(lambda_s)*cos(c) - y*sin(lambda_s)*sin(c)));
end
%
if (tau >= 2*pi)
    tau = tau - 2*pi;
end
%
lambda = lambda/deg_to_rad;
tau = tau/deg_to_rad;
%
