%read data from sim
%clear all
%load('SimResults2.mat')
Rcam = [1 0 0; 0 1 0; 0 0 1];
% T = 8000;
% ref.time = 0:.0001:T*3;
% t = ref.time;

%draw
vidObj = VideoWriter('sim_testing_live', 'Uncompressed AVI');
step = 1;
vidObj.FrameRate = 1/step;
open(vidObj);
frame = 1;
figure('Units','centimeters','Position',[0 0 34 17])
h = [1, 0.1 ,0.1 , 0.1];
axes
scale = 0.0025; % size of quadcopter


%%

%yegeta added this to test displaying picture on the background
img = imread('San_Jose.JPG');
%imgsec([min_x,maxx],[min_y max_y],img);
%imagesc([36.8 37.2], [-122.3 -121.8],img);
imagesc( [37.15 37.45], [-122.1 -121.65] ,img);
%%

for t2 = 0:step:T
    I = find(t>=t2,1);
    if isempty(I) 
        close(vidObj);
        display('Finished making video.')
        break
    end
    %R(:,:,I) = eye(3);
    R_star = Rcam*eye(3);%R(:,:,I)';
    
    hold on
    grid on
    cla
    
    % cycle through each quad for each frame
    for quad = 1: 60 % length(state.signals.values(1,:))/3
        %pos = state.signals.values(:,3*quad-2:3*quad)';
        pos = quads.signals.values(:,3*quad-2:3*quad)';
        pos_R = Rcam*pos;
        if I >=length(pos_R(1,:)) 
            close(vidObj);
            display('Finished making video.')
            break;
        end
        imagesc( [37.15 37.45], [-122.1 -121.65] ,img); %added
        leg(quad) = plot3(pos_R(1,1:I)',pos_R(2,1:I)',pos_R(3,1:I)','r-');
        p_star = pos_R(:,I);
        quad_plot(p_star,R_star,0,[],0.5,scale);
    end % end for-loop
    
    set(gca,'Xlim',[37.15 37.45], 'YLim', [-122.1 -121.65], 'ZLim', [0 800]);
    %view(37.5,30);
    axis square;
    %legend(leg,{'Quad 1','Quad 2','Quad 3','4'},'location','NorthEast')
    xlabel('Latitude (degrees)')
    ylabel('Longitude (degrees)')
    
    
    if I >=length(pos_R(1,:))
        close(vidObj);
        break;
    end
    F(frame) = getframe(gcf);
    writeVideo(vidObj,F(frame));
    frame = frame+1;
end

hold off;
grid off;
