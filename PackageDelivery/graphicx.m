%read data from sim

load('SimResults.mat')
Rcam = [-1 0 -0; 0 1 0; 0 0 -1];
t = ref.time;

%draw
P0 = zeros(3);
P1 = Rcam;
vidObj = VideoWriter('sim_testing_live', 'Uncompressed AVI');
step = 0.5;% was .05
vidObj.FrameRate = 1/step;
open(vidObj);
frame = 1;
figure('Units','centimeters','Position',[0 0 34 17])
h = [1, 0.1 ,0.1 , 0.1];
axes
scale = 0.005;

for t2 = 0:step:T
    I = find(t>=t2,1);
    if isempty(I) 
        close(vidObj);
        display('Finished making video.')
        break
    end
    if I >=length(p_i_1)
        close(vidObj);
        display('length of internal variable caused the video to end.')
        if I >=length(p_i_1)
            display('p_i was the cause');
            length(p_i_1)
        end
        break
    end
    R(:,:,I) = eye(3);
    R_star = Rcam*R(:,:,I)';
    P1B = R_star*scale;
    
    hold on
    grid on
    cla
    
    % cycle through each quad for each frame
    for quad = 1: 10 % length(state.signals.values(1,:))/3
        pos = state.signals.values(:,3*quad-2:3*quad)';
        pos_R = Rcam*pos;
        pos_R(1,:) = -pos_R(1,:);
        leg(quad) = plot3(pos_R(1,1:I)',pos_R(2,1:I)',pos_R(3,1:I)','r-');
        p_star = pos_R(:,I);
        quad_plot(p_star,R_star,0,[],0.5,scale);
        P0B = repmat(p_star,1,4);
        
    end % end for-loop
    
    set(gca,'Xlim',[36.8 37.2], 'YLim', [-122.3 -121.8], 'ZLim', [0 .5]);
    %view(37.5,30);
    axis square;
    legend(leg,{'Quad 1','Quad 2','Quad 3','4'},'location','NorthEast')
    xlabel('Latitude (degrees)')
    ylabel('Longitude (degrees)')
    
    F(frame) = getframe(gcf);
    writeVideo(vidObj,F(frame));
    frame = frame+1;
end

hold off
hold off;grid off;
