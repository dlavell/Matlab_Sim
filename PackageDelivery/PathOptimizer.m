function [ path, path_len ] = PathOptimizer( obstacles, destination )
% Path optimization function used for obstacle avoidance

dir = obstacles;
% source
s = [8 8];
c = [1 1];
route = zeros(10);

map_width = 15;
map_height = 15;
lat_min = 37.15;
lat_max = 37.45;
lng_min = -122.1;
lng_max = -121.65;
lat_resolution = (lat_max - lat_min)/map_width;
lng_resolution = (lng_max - lng_min)/map_width;


lat = destination(1);
lng = destination(2);

map_lat_index = floor((lat - lat_min)/lat_resolution) + 1;
map_lng_index = floor((lng - lng_min)/lng_resolution) + 1;
cont = 0;
% Check for any out of bounds indexing
if(map_lat_index > map_width)   cont = 1; end
if(map_lat_index < 1)           cont = 1; end
if(map_lng_index > map_height)  cont = 1; end
if(map_lng_index < 1)           cont = 1; end
if(cont)
    path_len = 0;
    path = route;
    return
end


cur_value = 2;
dir(map_lng_index,map_lat_index) = cur_value;
cur_value = cur_value+1;
done = 0;

row = 0;
col = 0;
while(~done)
    still_going = 0;
    for x = 1:length(dir(1,:))
        for y = 1:length(dir(:,1))
            if(dir(y,x) == cur_value-1)
                c = [y x];
                for i = 1:8
                    if(i == 1) row =  0; col =  1; end
                    if(i == 2) row =  1; col =  1; end
                    if(i == 3) row =  1; col =  0; end
                    if(i == 4) row =  1; col = -1; end
                    if(i == 5) row =  0; col = -1; end
                    if(i == 6) row = -1; col = -1; end
                    if(i == 7) row = -1; col =  0; end
                    if(i == 8) row = -1; col =  1; end
                    
                    if(c(1)+row > 0 && c(1)+row <= length(dir(:,1)) && ...
                            c(2)+col > 0 && c(2)+col <= length(dir(1,:)) )
                        if(dir(c(1)+row , c(2)+col) == 0)
                            still_going = 1;
                            dir(c(1)+row , c(2)+col) = cur_value;
                        end
                    end
                end % end i
            end
        end % end y
    end % end x
    cur_value = cur_value+1;
    if(~still_going)
        %if(dir(s(1),s(2)) == 0) display('impossible'); end
        path_len = 0;
        path = route;
        %display('no path found');
        return
        done = 1;
    end
end


dir;


start = [5 5];
goal = [destination(1) destination(2)];
done = 0;
c = start;
m_loc = [0 0];

map = dir;

stuck = 0;

route(c(1),c(2)) = inf;
map(c(1),c(2)) = inf;
while(done ~= 1)
    m = inf;
   
    for i = 1:8
        if(i == 1) row =  0; col =  1; end
        if(i == 2) row =  1; col =  1; end
        if(i == 3) row =  1; col =  0; end
        if(i == 4) row =  1; col = -1; end
        if(i == 5) row =  0; col = -1; end
        if(i == 6) row = -1; col = -1; end
        if(i == 7) row = -1; col =  0; end
        if(i == 8) row = -1; col =  1; end
        
        if(c(1)+row > 0 && c(1)+row <= length(map(:,1)) && c(2)+col > 0 && c(2)+col <= length(map(1,:)))
            if(map(c(1)+row , c(2)+col) < m && map(c(1)+row , c(2)+col) ~= 1)
                m = map(c(1)+row , c(2)+col);
                m_loc = [c(1)+row , c(2)+col];
            end
        end
    end
    
    c = m_loc;
    route(c(1),c(2)) = inf;
    map(c(1),c(2)) = inf;
    stuck = stuck + 1;
    % finished finding path
    if(c == goal ) done = 1; end
    if(stuck == 15)
        done = 1;
        path_len = 0;
        path = route;
        %display('no path found');
        return
    end
end % found path

path = route;
path_len = length(path);
end

