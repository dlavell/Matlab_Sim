function [ elevation ] = getElevation( lat , lng )
% Test function for elevation data
% Returns elevation at given latitude longitude 

lat_range = [36.8 37.2];        % upper and lower bounds of lat
lat_resolution = .05;           % degrees
lat_index = floor((lat - lat_range(1))/lat_resolution) + 1;

lng_range = [-122.3 -121.8];    % upper and lower bounds of lng
lng_resolution = .05;           % degrees
lng_index = floor((lng - lng_range(1))/lng_resolution) + 1;

terrain = repmat([0 .1 .2 .1 0 .1 .2 .1 0], 11, 1);
% terrain = .05*[4 4 4 4 4 4 4 4 4;
%            4 4 4 4 4 4 4 4 4;
%            4 3 3 3 3 3 3 3 4; 
%            4 3 2 2 2 2 2 3 4; 
%            4 3 2 1 0 1 2 3 4; 
%            4 3 2 1 1 1 2 3 4; 
%            4 3 2 2 2 2 2 3 4; 
%            4 3 3 3 3 3 3 3 4; 
%            4 4 4 4 4 4 4 4 4;
%            4 4 4 4 4 4 4 4 4];
if(lng_index > 11) lng_index = 11; end
if(lng_index < 1) lng_index = 1; end
if(lat_index > 9) lat_index = 9; end
if(lat_index < 1) lat_index = 1; end

elevation = terrain(lng_index, lat_index);
%elevation = 0;

end

