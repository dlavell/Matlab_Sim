%%**************************************************************************
% @ Authors:             Quadcopter team(Daniel Lavell, Brandon Luu 
%                              and Yegeta Zeleke)
% @ Class:               Senior design project ucsc
% @ Date:                02/28/2016
% @ program Discription: This program is a testharnes to test the methods in 
%                           post_processing    
%
%
%
%%**************************************************************************
%{

define 3 matrix as follow 
    quad 1                        quad 2                quad 3
    ---------                   --------               --------
quads(:,:,1) =              quads(:,:,2) =          quads(:,:,3) =       

     0     1     2           2     1     2           21     1     2
     0     2     3           3     2     3           32     2     3
     2     3     5           4     3     5           43     3     5
     4     1     2           6     1     2           64     1     2
     1     2     3          18     2     3           18     2     3
     0     1     2         	 4     1     2            1     2     3
     2     1     2           32     1     2           4     1     2
%}

clc
display('*********************Test Harness for post_processing methods**************')
quads=  [0      1      2; 
        0       2      3; 
        2       3      5; 
        4       1      2;
        1       2      3 ;
        0       1     2;
        2       1     2 ];
    
    
quads(:,:,2)=   [2      1       2; 
                3       2       3;
                4       3       5; 
                6       1       2;
                18      2       3 ;
                4       1       2;
                32      1       2];
            
            
quads(:,:,3)=   [21     1       2; 
                 32     2       3; 
                 43     3       5; 
                 64     1       2;
                 18     2       3 ;
                  1     2       3;
                  4     1       2];
%quads
%consider only quad 1 and 2 and check if they collide
display('check collision between quad 1 and 2...no collision should be reported')
total_collision = post_processing.calc_collision(quads,2)

%consider only all quads
display('check collision with all quads.1 collision at point 18 2 3 should be reported')
total_collision = post_processing.calc_collision(quads,size(quads,3))


%average altitude gain for all quads
total_alt_gain = 0;
max_alt_gain = 0;
min_alt_gain = 0;
n_quads = 10;
for quad_id = 1:n_quads   
   alt_gain = Modified_post_processing.calc_alt_gain(state.signals.values(:,(quad_id*3-2):quad_id*3));   
   total_alt_gain = total_alt_gain + alt_gain;
   
   %max check
   if(max_alt_gain == 0)
      max_alt_gain = alt_gain; 
   elseif(alt_gain > max_alt_gain)
      max_alt_gain = alt_gain; 
   end
   
   %min check
   if(min_alt_gain == 0)
      min_alt_gain = alt_gain; 
   elseif(alt_gain < min_alt_gain)
      min_alt_gain = alt_gain; 
   end
   
   fprintf('quad_id: %2i alt: %f\n', quad_id, alt_gain);   
end

avg_alt_gain = total_alt_gain/n_quads;
fprintf('average: %f max: %f min: %f\n', avg_alt_gain, max_alt_gain, min_alt_gain);


