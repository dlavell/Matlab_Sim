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
            % @Author: Quadcopter team(Daniel Lavell, Brandon Luu and Yegeta Zeleke)
            % @Revision: None

            function total_dist = calc_distance(trajectory)
                display('calculating result');
                R = 6371000 % radius of the earth
                lat = trajectory(:,1);
                lon = trajectory(:,2);
                alt = trajectory(:,3);
                length = size(trajectory,1)
              
                x = R * cos(lat)* cos(lon) % convert latitude to x 
                y = R * sin(lat)* sin(lon) % convert longtude to y
                z = trajectory(:,3) % convert longtude to z
                  
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
            function total_collision = calc_collision(state)

            end

            %%
            % @Function: calc_altGain
            % @parameter: state[i]
            % @return: total_altGain
            % @brief: This function takes in each quad workspace and computes and
            %          returns the total altitude gain of the the quad from bigining of
            %          simulation to end.
            % @note:None
            % @Author: Quadcopter team(Daniel Lavell, Brandon Luu and Yegeta Zeleke)
            % @Revision: None
            function total_altGain = calc_altGain(state)

            end
    end
end