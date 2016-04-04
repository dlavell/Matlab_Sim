function [ elevation] = get_elevation(lat , lng, elevation_data)
% Test function for elevation data
% Returns elevation at given latitude longitude 

% Convert lat/lng to elevation_data indexes
lat_range = [37.000 38.000];        % upper and lower bounds of lat
lat_resolution = .001;           % degrees
lat_index = floor(abs((lat - lat_range(1)))/lat_resolution) + 1;

lng_range = [-121.000 -122.000];    % upper and lower bounds of lng
lng_resolution = .001;           % degrees
lng_index = floor(abs((lng - lng_range(1)))/lng_resolution) + 1;

% Access elevation_data at correct index
elevation = elevation_data(lat_index, lng_index);

end
