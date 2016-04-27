function [ path , path_len ] = shortest_path( source, dest )
%coder.extrinsic('dijkstra');
debug = false;
% lat/lng coordinates of destination
lat_d = dest(1);
lng_d = dest(2);
lat_s = source(1);
lng_s = source(2);
% ranges for possible lat/lng pairs
lat_min = 37.15;
lat_max = 37.45;
lng_min = -122.1;
lng_max = -121.65;

% Check bounds of source and destination nodes
if(lat_d < lat_min || lat_d > lat_max || lng_d < lng_min || lng_d > lng_max)
   [ path , path_len ] = exit();
   return;
end
if(lat_s < lat_min || lat_s > lat_max || lng_s < lng_min || lng_s > lng_max)
   [ path , path_len ] = exit();
   return; 
end


obstacles = [
    0     0     0     0     0     0     0     0     0     0     0     0     0     0     0; % 1 - 15
    0     0     0     0     0     0     0     0     0     0     0     0     0     0     0; % 16 - 30
    0     0     0     0     0     0     0     0     0     0     0     0     0     0     0; % 31 - 45
    0     0     0     0     0     0     0     1     0     0     0     0     0     0     0; % 46 - 60
    0     0     0     0     0     0     0     0     0     0     0     0     0     0     0; % 61 - 75
    0     0     0     0     0     0     0     0     0     0     0     0     0     0     0; % 76 - 90
    0     0     0     0     0     0     0     0     1     0     0     1     0     0     0; % 91 - 105
    0     0     0     1     1     1     1     0     0     0     1     0     0     0     0; % 106 - 120
    0     0     1     0     0     0     0     0     1     1     0     0     0     0     0; % 121 - 135
    0     0     0     0     0     0     0     0     0     1     1     1     0     0     0; % 136 - 150
    0     0     0     0     0     0     0     0     0     0     0     0     0     0     0; % 151 - 165
    0     0     0     0     0     0     0     0     0     0     0     0     0     0     0; % 166 - 180
    0     0     0     0     0     0     0     0     0     0     0     0     0     0     0; % 181 - 195
    0     0     0     0     0     0     0     0     0     0     0     0     0     0     0; % 196 - 210
    0     0     0     0     0     0     0     0     0     0     0     0     0     0     0];% 211 - 225


graph = zeros(length(obstacles(1,:))*length(obstacles(:,1)));
rowLength = length(obstacles(1,:));
colLength = length(obstacles(:,1));
lat_resolution = (lat_max - lat_min)/rowLength;
lng_resolution = (lng_max - lng_min)/colLength;

% For each node, label its possible neighbors
graph = LabelNeighbors(graph,obstacles, debug);

% Source node xy coordinates on grid
s_x = floor((lat_s - lat_min)/lat_resolution) + 1;
s_y = floor((lng_s - lng_min)/lng_resolution) + 1;
% Source node index on graph
s_node = mod(s_x,rowLength) + rowLength*(s_y - 1);

% Destination node xy coordinates on grid
d_x = floor((lat_d - lat_min)/lat_resolution) + 1;
d_y = floor((lng_d - lng_min)/lng_resolution) + 1;
% Destination node index on graph
d_node = mod(d_x,rowLength) + rowLength*(d_y - 1);

if(obstacles(s_y,s_x) == 1 || obstacles(d_y,d_x) == 1)
   [ path , path_len ] = exit();
    dest_is_obstacle = 1
   return; 
end

% Dijkstra's shortest path algorithm
[p, c, f] = dijkstra(graph, s_node, d_node);
path_len = length(p);


if(path_len == 0 )
    SHOULDNT_BE_HERE = 1
end

path_ = zeros(path_len,2);
path = zeros(15,2);

% get path in terms of grid...
for i = 1:path_len
   Gridcol = mod(p(i),rowLength);
   Gridrow = ceil(p(i)/rowLength); % was /colLength
   if(Gridcol == 0) Gridcol = rowLength; end
   lat = lat_min + Gridcol*lat_resolution;
   lng = lng_min + Gridrow*lng_resolution;
   path_(i,:) = [lat lng];
   %if(debug)fprintf('[%d %d] ',Gridrow,Gridcol);end
end
%if(debug)fprintf('\nPath length = %d\n',path_len);end

path(1:path_len,:) = path_;

end

function [ path , path_len ] = exit()
    path = zeros(15,2);
    path_len = 0;
    return
end

function graph = LabelNeighbors(G, obstacles, debug)

% Label possible neighbors -> unitary weight for now
NeighborUnitaryWeight = 1;
graph = G;

numberOfNodes = length(G(1,:));
rowLength = length(obstacles(1,:));
colLength = length(obstacles(:,1));

for NodeID = 1:numberOfNodes
    Gridcol = mod(NodeID, rowLength);
    Gridrow = ceil(NodeID/rowLength); % was /colLength
    if(Gridcol == 0) Gridcol = rowLength; end
%     if(debug)
%         fprintf('NodeID : %d\n',NodeID);
%         fprintf('  Location : [%d %d] on the grid\n',Gridrow,Gridcol);
%         fprintf('  Value : %d',obstacles(Gridrow,Gridcol));
%     end
    if(Gridrow+1 <= colLength) % bottom middle neighbor
        if(obstacles(Gridrow+1,Gridcol) ~= 1)% Not an obstacle
            % label as a valid edge
            graph(NodeID+colLength,NodeID) = NeighborUnitaryWeight;
        else
            graph(NodeID+colLength,NodeID) = Inf;
        end
    end
    
    if(Gridrow+1 <= colLength && Gridcol-1 > 0) % bottom left neighbor
        if(obstacles(Gridrow+1,Gridcol-1) ~= 1)% Not an obstacle
            % label as a valid edge
            graph(NodeID+colLength-1,NodeID) = NeighborUnitaryWeight;
        else
            graph(NodeID+colLength-1,NodeID) = Inf;
        end
    end
    
    if(Gridrow+1 <= colLength && Gridcol+1 <= rowLength) % bottom right neighbor
        if(obstacles(Gridrow+1,Gridcol+1) ~= 1)% Not an obstacle
            % label as a valid edge
            graph(NodeID+colLength+1,NodeID) = NeighborUnitaryWeight;
        else
            graph(NodeID+colLength+1,NodeID) = Inf;
        end
    end
    
    if(Gridrow-1 > 0) % Top middle neighbor
        if(obstacles(Gridrow-1,Gridcol) ~= 1)% Not an obstacle
            % label as a valid edge
            graph(NodeID-colLength,NodeID) = NeighborUnitaryWeight;
        else
            graph(NodeID-colLength,NodeID) = Inf;
        end
    end
    
    if(Gridrow-1 > 0 && Gridcol-1 > 0) % Top left neighbor
        if(obstacles(Gridrow-1,Gridcol-1) ~= 1)% Not an obstacle
            % label as a valid edge
            graph(NodeID-colLength-1,NodeID) = NeighborUnitaryWeight;
        else
            graph(NodeID-colLength-1,NodeID) = Inf;
        end
    end
    
    if(Gridrow-1 > 0 && Gridcol+1 <= rowLength) % Top right neighbor
        if(obstacles(Gridrow-1,Gridcol+1) ~= 1)% Not an obstacle
            % label as a valid edge
            graph(NodeID-colLength+1,NodeID) = NeighborUnitaryWeight;
        else
            graph(NodeID-colLength+1,NodeID) = Inf;
        end
    end
    
    if(Gridcol-1 > 0) % Middle Left neighbor
        if(obstacles(Gridrow,Gridcol-1) ~= 1)% Not an obstacle
            % label as a valid edge
            graph(NodeID-1,NodeID) = NeighborUnitaryWeight;
        else
            graph(NodeID-1,NodeID) = Inf;
        end
    end
    
    if(Gridcol+1 <= rowLength) % Middle Right neighbor
        if(obstacles(Gridrow,Gridcol+1) ~= 1)% Not an obstacle
            % label as a valid edge
            graph(NodeID+1,NodeID) = NeighborUnitaryWeight;
        else
            graph(NodeID+1,NodeID) = Inf;
        end
    end
           
    %if(debug)fprintf('\n');end
end



end

