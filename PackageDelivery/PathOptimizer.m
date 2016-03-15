function [ path , path_len ] = PathOptimizer( obstacles, lat, lng )
% Path optimization function used for obstacle avoidance
path = zeros(10,2);
path_index = 1;


lat_range = [37 37.5];        % upper and lower bounds of lat
lat_res = .05;             % degrees
lat_index = floor((lat - lat_range(1))/lat_res) + 1;

lng_range = [-122 -121.5];    % upper and lower bounds of lng
lng_res = .05;            % degrees
lng_index = floor((lng - lng_range(1))/lng_res) + 1;


destination = [11-lng_index lat_index];

dir = obstacles;
% source
s = [5 5];
path(path_index,:) = [37.25 -121.75];
path_index = path_index + 1;
c = [1 1];
cur_value = 2;
dir(destination(1),destination(2)) = cur_value;
cur_value = cur_value+1;
done = 0;


while(~done)
    still_going = 0;
    for x = 1:length(dir(1,:))
        for y = 1:length(dir(:,1))
            if(dir(y,x) == cur_value-1)
                c = [y x];
                for i = 1:8
                    if(i == 1) row =  0; col =  1; 
                    elseif(i == 2) row =  1; col =  1; 
                    elseif(i == 3) row =  1; col =  0; 
                    elseif(i == 4) row =  1; col = -1; 
                    elseif(i == 5) row =  0; col = -1; 
                    elseif(i == 6) row = -1; col = -1; 
                    elseif(i == 7) row = -1; col =  0; 
                    elseif(i == 8) row = -1; col =  1;
                    else row = 0; col = 0; end
                    
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
        if(dir(s(1),s(2)) == 0) 
            x = 999999999
            % display('impossible');
        end
        done = 1;
    end
end


route = zeros(10);

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
        if(i == 1) row =  0; col =  1; 
        elseif(i == 2) row =  1; col =  1; 
        elseif(i == 3) row =  1; col =  0; 
        elseif(i == 4) row =  1; col = -1; 
        elseif(i == 5) row =  0; col = -1; 
        elseif(i == 6) row = -1; col = -1; 
        elseif(i == 7) row = -1; col =  0; 
        elseif(i == 8) row = -1; col =  1; 
        else row = 0; col = 0; end
        
        if(c(1)+row > 0 && c(1)+row <= length(map(:,1)) && c(2)+col > 0 && c(2)+col <= length(map(1,:)))
            if(map(c(1)+row , c(2)+col) < m && map(c(1)+row , c(2)+col) ~= 1)
                m = map(c(1)+row , c(2)+col);
                m_loc = [c(1)+row , c(2)+col];
            end
        end
    end
    
    c = m_loc;
    path(path_index,:) = [lat_range(1)+lat_res*m_loc(1) lng_range(1)+lng_res*m_loc(2)];
    path_index = path_index + 1;
    route(c(1),c(2)) = inf;
    map(c(1),c(2)) = inf;
    stuck = stuck + 1;
    % finished finding path
    if(c(1) == goal(1) && c(2) == goal(2) ) done = 1; end
    if(stuck == 15)
        done = 1;
        x = 5555555
        % display('no path found');
    end
end % found path

path;
path_len = path_index-1;
route;
end

