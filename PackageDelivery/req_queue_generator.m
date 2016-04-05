%%
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
%%
function request_queue = req_queue_generator(pop_density,sim_start,step, sim_stop,MAX_NUMREQ)
    get_value = 0;
    total_req = pop_density * MAX_NUMREQ;
    
    %get the length of the array to put the simulation data
     
    request_queue = zeros(ceil((sim_stop-sim_start)/step),2);
    
    req_left = total_req;
    len = length(request_queue);
  
    
    request_queue(1,1) = sim_start;
    for i = 1:1:len
        request_queue(i+1,1) = request_queue(i,1) + step;
    end
    
    while req_left >0
        random_time = ceil(rand*len);
        request_queue(random_time,2) =  request_queue(random_time,2)+1;
        req_left= req_left- 1;
    end


end







