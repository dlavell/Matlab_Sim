%%
% @Function   : calc_time
% @parameter  : trajectory : nx3 matrices
%                speed:   a constant speed for the entire trajectory is assumed for now
% @return     : time
% @brief      : This function takes in a path and a set of way points
%               along with the speed of a quadcopter
%               and computes total time
%               elapsed for given path with a constant speed
% @note      :  None
% @assumtion : lat,lon to x,y conversion equation is acurate
%               on radius of 10 miles if lat lon is more than
%               10 miles better approximation is required
% @Author   :  Quadcopter team(Daniel Lavell, Brandon Luu and Yegeta Zeleke)
% @Revision :  None
% @Date     : 4/25/16

function time = calc_time(trajectory,speed)
R = 6371000 ;% radius
lat = trajectory(:,1);
lon = trajectory(:,2);
alt = trajectory(:,3);
length = size(trajectory,1);
x = zeros(length, 1);
y = zeros(length, 1);
z = alt; % convert longtude to z

for j = 1:1:length
    if(lat(j) == 0) break; end
    x(j) = R *cos(lon(j))*cos(lat(j))*pi/180; % convert latitude to x
    y(j) = R * cos(lat(j))* sin(lon(j))*pi/180; % convert longtude to y
    
end

time=0;
for i = 2:1:length
    if(lat(i) == 0) break; end
    time = time + sqrt((x(i)-x(i-1))^2 +(y(i)-y(i-1))^2 +(z(i)-z(i-1))^2 );
end
time = time./speed;
end