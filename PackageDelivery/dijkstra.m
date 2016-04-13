%---------------------------------------------------
% Find shortest path from Source to Destination with predefined nodes using
% Dijkstra Algorithm
%
% usage:
% [Path, Cost, Flag] = dijkstra(Graph, Source, Destination)
% or
% [Path, Cost, Flag] = dijkstra(Graph, Source, Destination, restrict2Nodes)
%
% example:
% Inputs:
% G = [0 10 3 0 0;
%      0 0  1 2 0;
%      0 4  0 8 2;
%      0 0  0 0 7;
%      0 0  0 9 0]; % Every element of Graph should be non-negetive
% S = 1; % Starting node of the path
% D = 4; % Ending node of the path
% r2N = [2 4 1]; % If you want to restrict path only to some of your
% distinguished node (other than Source and Destination) in the Graph, you can ignore this if you want to use
% the full Graph
%
% [c, p, f] = dijkstra(G, S, D, r2N); or
% [c, p, f] = dijkstra(G, S, D)
%
% Outputs:
% Path: Shortest path found by the algorithm, =[], if not found any path
% Cost: Total cost for the shortest path, =Inf, if not found any path
% Flag: 'Found' or 'Not Found' string depending upon path found or not found
%
% author : Pramit Biswas
% email  : pramit55555@gmail.com
%---------------------------------------------------
function [shortestPath, Cost, Status] = dijkstra(Graph, Source, Destination, restrict2Nodes)
%% Input Testing
totalNodes = size(Graph);
if totalNodes(1)==totalNodes(2)
    totalNodes=totalNodes(1);
else
    error('Adjacency matrix must be a square Matrix');
end

if Source>totalNodes
    error('Source is not present in the Graph');
elseif Destination>totalNodes
    error('Destination is not present in the Graph');
end

if nargin <3
    error('Missing input parameter(s)!');
elseif nargin ==4
    if find(restrict2Nodes==Source | restrict2Nodes==Destination)~=0
        error('restrict2Nodes should not contain Source and Destination node');
    end
    if find(restrict2Nodes>totalNodes)
        error('one or many nodes in ''restrict2Nodes'' are not present in the Graph');
    end
    restrict2Nodes = sort(restrict2Nodes);
    [~, totalOtherNodes] = size(restrict2Nodes);
    if length(restrict2Nodes)~=length(unique(restrict2Nodes))
        error('There must not be any same node');
    end
    % Modifying the Graph depending upon restriction
    allNodes = 1:totalNodes;
    newSet = [Source restrict2Nodes Destination];
    for i1 = 1:totalOtherNodes+2
        allNodes = allNodes(allNodes~=newSet(i1));
    end
    allNodesLength=length(allNodes);
    if isempty(allNodesLength)==0
        for i1 = 1:allNodesLength
            Graph(allNodes(i1),:)=0;
            Graph(:,allNodes(i1))=0;
        end
    end
    clear allNodes newSet allNodesLength;
elseif nargin>4
    error('Check input parameter(s)...');
end
totalNodes = max(size(Graph));

%% Dijksrta Algorithm
for i1 = 1:totalNodes
    costDijkstra(i1,:) = [Inf, Inf];
end
costDijkstra(Source,:) = [0 0];

foundNodes = Source;
nodesFound=1;
Status='Found';

for i1 = 2:totalNodes+1
    currentNode = foundNodes(nodesFound);
    if currentNode==Destination
        break;
    end
    for i2 = 1:totalNodes
        if Graph(currentNode,i2)~=0
            if costDijkstra(i2,2)> (costDijkstra(currentNode,2)+Graph(currentNode,i2))
                costDijkstra(i2,2)=costDijkstra(currentNode,2)+Graph(currentNode,i2);
                costDijkstra(i2,1)=currentNode;
            end
        end
    end
    shortMatrix=find(costDijkstra(:,2)>=costDijkstra(currentNode,2) & costDijkstra(:,2)<Inf);
    for i2 = 1:nodesFound
        c = shortMatrix==foundNodes(i2);%c = find(shortMatrix==shortestPath(i2));
        shortMatrix(c)=[];
    end
    if isempty(shortMatrix)
        Status='Not Found';
        break;
    end
    lengthShortMatrix=length(shortMatrix);
    minCost=costDijkstra(shortMatrix(1),2);
    minCostPosition=shortMatrix(1);
    for i2 = 2:lengthShortMatrix
        if costDijkstra(shortMatrix(i2),2)<minCost
            minCost=costDijkstra(shortMatrix(i2),2);
            minCostPosition=shortMatrix(i2);
        end
    end
    foundNodes = [foundNodes, minCostPosition];
    nodesFound = nodesFound+1;
end
clearvars -except Status costDijkstra Source Destination totalNodes;

%% Shortest Path
shortestPath=[];
if strcmp(Status, 'Not Found')
    Cost=Inf;
    return;
end
Cost=costDijkstra(Destination,2);
currentNode=Destination;

for i1 = 1:totalNodes
    shortestPath=[currentNode shortestPath];
    if currentNode==Source
        if Source == Destination
            shortestPath = [currentNode shortestPath];
        end
        return;
    end
    currentNode=costDijkstra(currentNode,1);
end
end