function [  ] = NFZ_map_generator(lat_min,lng_min,lat_max,lng_max)
% Generates a map of the obstacles based on the given csv file
% 
% Note: Make sure there are no commas any cells in the file
% Remove commas by ctr+f -> replace ',' with ' '


%% Reading NFZ list file
filename = 'gps_sj_NFZ.csv';

file_len = 300;
col2read = 2;
row2start = 1;
col2start = 1;
% NFZList has all latitude longitude pairs of obstacles
NFZ_list = csvread(filename,row2start,col2start,[1,1,file_len,col2read]);

%% Initial obstacle map resolution parameters
map_width = 25;
map_height = 10;

map = zeros(map_height, map_width);

%% Fill map with NFZ represented by '1'
NFZ_marker = 1;
lat_resolution = (lat_max - lat_min)/map_width;
lng_resolution = (lng_max - lng_min)/map_width;
length(NFZ_list)

for NFZ = 1:2%length(NFZ_list)
    lat = NFZ_list(NFZ,1);
    lng = NFZ_list(NFZ,2);
    
    map_lat_index = floor((lat - lat_min)/lat_resolution) + 1;
    map_lng_index = floor((lng - lng_min)/lng_resolution) + 1;
    
    % Check for any out of bounds indexing
    if(map_lat_index > map_width)   continue; end
    if(map_lat_index < 1)           continue; end
    if(map_lng_index > map_height)  continue; end
    if(map_lng_index < 1)           continue; end
    
    map_lat_index
    map_lng_index
    
    map(map_lng_index,map_lat_index) = NFZ_marker
    
end

map;


end

