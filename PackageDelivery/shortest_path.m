function [ path ] = shortest_path( )
% obstacles = [
%     0     0     0     0     0     0     0     0     0     0     0     0     0     0     0;
%     0     0     0     0     0     0     0     0     0     0     0     0     0     0     0;
%     0     0     0     0     0     0     0     0     0     0     0     0     0     0     0;
%     0     0     0     0     0     0     0     1     0     0     0     0     0     0     0;
%     0     0     0     0     0     0     0     0     0     0     0     0     0     0     0;
%     0     0     0     0     0     0     0     0     0     0     0     0     0     0     0;
%     0     0     0     0     0     0     0     0     1     0     0     1     0     0     0;
%     0     0     0     1     1     1     1     0     0     0     1     0     0     0     0;
%     0     0     1     0     0     0     0     0     1     1     0     0     0     0     0;
%     0     0     0     0     0     0     0     0     0     1     1     1     0     0     0;
%     0     0     0     0     0     0     0     0     0     0     0     0     0     0     0;
%     0     0     0     0     0     0     0     0     0     0     0     0     0     0     0;
%     0     0     0     0     0     0     0     0     0     0     0     0     0     0     0;
%     0     0     0     0     0     0     0     0     0     0     0     0     0     0     0;
%     0     0     0     0     0     0     0     0     0     0     0     0     0     0     0];

obstacles = [0 0 0; 0 1 0; 0 0 0];
graph = zeros(length(obstacles(1,:))*length(obstacles(:,1)));

% For each node, label its possible neighbors
graph = LabelNeighbors(graph,obstacles);

% Source and Destination nodes
S = 1;
D = 9;


% Dijkstra's shortest path algorithm
[p, c, f] = dijkstra(graph, S, D);


% get path in terms of grid...
rowLength = length(obstacles(1,:));
colLength = length(obstacles(:,1));
for i = 1:length(p)
   Gridcol = mod(p(i),rowLength);
   Gridrow = ceil(p(i)/colLength);
   if(Gridcol == 0) Gridcol = rowLength; end
   fprintf('[%d %d] ',Gridrow,Gridcol);
end
fprintf('\n');




end

function graph = LabelNeighbors(G, obstacles)

% Label possible neighbors -> unitary weight for now
NeighborUnitaryWeight = 1;
graph = G;

numberOfNodes = length(G(1,:));
rowLength = length(obstacles(1,:));
colLength = length(obstacles(:,1));

for NodeID = 1:numberOfNodes
    Gridcol = mod(NodeID, rowLength);
    Gridrow = ceil(NodeID/colLength);
    if(Gridcol == 0) Gridcol = rowLength; end
    fprintf('NodeID : %d\n',NodeID);
    fprintf('  Location : [%d %d] on the grid\n',Gridrow,Gridcol);
    fprintf('  Value : %d',obstacles(Gridrow,Gridcol));

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
           
    fprintf('\n');
end



end

