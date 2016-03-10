function send_quad = Queue_Manager
% *************************************************************************************************
% *                                                  Functions
% *************************************************************************************************


% @Function: request_queue_generator
% @parameter: Population_density
% @return: Total_dist
% @brief: This function takes a population density and generate
%          a request with timestamp, the requestes will be
%          queed on request queue
% @note:None
% @assumtion :  None
% @Author: Quadcopter team(Daniel Lavell, Brandon Luu and Yegeta Zeleke)
% @Revision: None

global  time_stamp current_que;





%current_que = [0 2;1 0;2 4];
current_time = time_stamp ;
send_quad = 0;

%if no quad available at the warehouse no need to try to send a quad
% 
% 
% 
% if(current_time <2)
%     %.
%     if(current_que(1,2)>0) %if there is a package request at time 0 send quad
%         send_quad =1;
%     end
%     
% elseif(current_time < length(current_que))
%     if(current_que(current_time-1,2)>0) %still have package to be sent from previous time stamp
%         %........
%         send_quad = 1;
%         current_que(current_time,2)= current_que(current_time-1,2)+ current_que(current_time,2)-1; % transfer package not send to the current timestamp
%     elseif(current_que(current_time,2)>0)% if there is a package at this time stamp send quad
%         send_quad = 1;
%         current_que(current_time,2)=current_que(current_time,2)-1;    %since a quad has been sent to deliver decrement by one
%     else    %no package waiting
%         ;
%     end
% else
%     ;
%     
% end
time_stamp = time_stamp+1;
%\ assignin('Request_Queue', 'request_queue', current_que) didnt
%work
%Simulink.saveVars('Request_Queue','current_que');
%save('Request_Queue','current_que');
end










