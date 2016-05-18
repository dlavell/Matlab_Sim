function [ dist ] = deg2nm( lat1,long1,lat2,long2 )
%Converts distance between two lat long pairs into nautical miles
R = 6.373*10^6; 

toRadians = pi/180;

dlong = (long2 - long1)* toRadians;
dlat  = (lat2  - lat1) * toRadians;

a = (sin(dlat/2))^2 + cos(lat1) * cos(lat2) * (sin(dlong/2))^2;
c = 2 * atan2( sqrt(a), sqrt(1-a) );
dist = R * c; % meters

toNauticalMiles = .000539957;
dist = dist*toNauticalMiles;

end

