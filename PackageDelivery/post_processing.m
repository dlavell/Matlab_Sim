%%This file is used to computer post- process calculations such as 
% caculating the number of collisions, distance each quad flew and total
% altitude gain
%
classdef post_processing
    methods(Static)
            %%
            % @Function: calc_distance
            % @parameter: state[i]
            % @return: Total_dist
            % @brief: This function takes in each quad workspace and computes and
            %          returns the total distance travelled by the quad
            % @note:None
            % @assumtion :  lat,lon to x,y conversion equation is acurate
            %               on radius of 10 miles if lat lon is more than
            %               10 miles better approximation is required
            % @Author: Quadcopter team(Daniel Lavell, Brandon Luu and Yegeta Zeleke)
            % @Revision: None

            function total_dist = calc_distance(trajectory)
                display('calculating result');
                R = 6371000 ;% radiuth
                lat = trajectory(:,1);
                lon = trajectory(:,2);
                alt = trajectory(:,3);
                length = size(trajectory,1);
              
               % x = zeros(length);
                %y = zeros(length);
                z = alt; % convert longtude to z
                
                for j = 1:1:length
                     x(j) = R *cos(lon(j))*cos(lat(j))*pi/180; % convert latitude to x 
                     y(j) = R * cos(lat(j))* sin(lon(j))*pi/180; % convert longtude to y
                  
                end 
                
                total_dist=0;
                for i = 2:1:length
                    total_dist = total_dist + sqrt((x(i)-x(i-1))^2 +(y(i)-y(i-1))^2 +(z(i)-z(i-1))^2 );
                end
            end

            %%
            % @Function: calc_collision
            % @parameter: state[i]
            % @return: Total_collision
            % @brief: This function takes in each quad workspace and computes and
            %          returns the total number of collision along with the the
            %          possition where the collision occur
            % @note:None
            % @Author: Quadcopter team(Daniel Lavell, Brandon Luu and Yegeta Zeleke)
            % @Revision: None
            function total_collision = calc_collision(quads,quad_num)
                len = length(quads(:,:,1));
                num_collision =0;
                total_collision =[0,0,0];
                for index = 1:1:len
                    for entry = 1:1:quad_num   %get each quadcopters row,:
                        check_row(entry,:) = quads(index,:,entry);
                    end
                    [no_collision_mtx,no_collision_idx] = unique(check_row,'rows','first');  % Finds indices of unique rows
                    collision_index = setdiff(1:size(check_row,1),no_collision_idx);         %Finds indices of repeats
                    
                    if(isempty(collision_index)==0) %if there is collision add it to total collision points
                        for k =1:1:length(collision_index)
                            num_collision = num_collision+1;
                            total_collision(num_collision,:) = check_row(collision_index(k),:);
                        end
                        
                    end
                    
                end
            end

            %%
            % @Function: calc_altGain
            % @parameter: state[i]
            % @return: total_altGain
            % @brief: This function takes in each quad workspace and computes and
            %          returns the total altitude gain of the the quad from bigining of
            %          simulation to end.
            % @note: this function only compute the total altitude gain
            %        throght the entire flight time. if we would like to
            %        make it display the altitude gain after each drop, we
            %        might need to add an extra column on
            %        state.signals.value to indicate dropoff.
            % @Author: Quadcopter team(Daniel Lavell, Brandon Luu and Yegeta Zeleke)
            % @Revision: None
            function total_altGain = calc_altGain(state)
                display('calculating altitude gain');
                R = 6371000 ;% radius of the earth
                total_altGain = 0 ;
                alt = trajectory(:,3);
                length = size(trajectory,1);
                for i = 1:1:length
                    total_altGain = total_altGain+alt(i); 
                end
                
            end
    end
end