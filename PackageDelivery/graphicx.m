%read data from sim

load('SimResults.mat')
%R = angle2dcm(euler.signals.values(:,1)'*pi/180,euler.signals.values(:,2)'*pi/180,euler.signals.values(:,3)'*pi/180,'ZYX');
%Rcam = angle2dcm(pi,0,0,'YZX');
Rcam = [-1 0 -0; 0 1 0; 0 0 -1];
%p_i = Rcam*state.signals.values';
p_i_1 = Rcam*state.signals.values';
p_i_2 = Rcam*state1.signals.values';
p_i_3 = Rcam*state2.signals.values';

pdt = Rcam*ref.signals.values(:,1:3)';
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
h = [1, 0.1 ,0.1 , 0.1];%create_axis([1, 0.1 ,0.1 , 0.1],[1 0.1 0.1 0.1]);
%T = 100;
axes
for t2 = 0:step:T
    I = find(t>=t2,1);
    if isempty(I) 
        close(vidObj);
        display('Finished making video.')
        break
    end
    if I >=length(p_i_1) || I >=length(pdt)
        close(vidObj);
        display('length of internal variable caused the video to end.')
        if I >=length(pdt)
            display('pdt was the cause');
            length(pdt)
        end
        if I >=length(p_i_1)
            display('p_i was the cause');
            length(p_i_1)
        end
        break
    end
    
    R(:,:,I) = eye(3);
    hold on
    cla
    leg(1) = plot3(pdt(1,1:I)',pdt(2,1:I)',pdt(3,1:I)');
    % leg(2) = plot3(p_i(1,1:I)',p_i(2,1:I)',p_i(3,1:I)','r-'); % single quad
    leg(2) = plot3(p_i_1(1,1:I)',p_i_1(2,1:I)',p_i_1(3,1:I)','r-'); % first quad
    leg(3) = plot3(p_i_2(1,1:I)',p_i_2(2,1:I)',p_i_2(3,1:I)','g-'); % second quad
    leg(4) = plot3(p_i_3(1,1:I)',p_i_3(2,1:I)',p_i_3(3,1:I)','b-'); % third quad
    quiver3(P0(1,:),P0(2,:),P0(3,:),P1(1,:),P1(2,:),P1(3,:),'LineWidth',2,'Color','k');
    grid on
    
    % p_star = p_i(:,I);
    p_star1 = p_i_1(:,I);
    p_star2 = p_i_2(:,I);
    p_star3 = p_i_3(:,I);
    
    R_star = Rcam*R(:,:,I)';
    scale = 0.5;
    
    % P0B = repmat(p_star,1,3);
    P0B1 = repmat(p_star1,1,3);
    P0B2 = repmat(p_star2,1,3);
    P0B3 = repmat(p_star3,1,3);
    
    P1B = R_star*scale;
    
    % quiver3(P0B(1,:),P0B(2,:),P0B(3,:),P1B(1,:),P1B(2,:),P1B(3,:),'LineWidth',1,'Color','k');
    quiver3(P0B1(1,:),P0B1(2,:),P0B1(3,:),P1B(1,:),P1B(2,:),P1B(3,:),'LineWidth',1,'Color','k');
    quiver3(P0B2(1,:),P0B2(2,:),P0B2(3,:),P1B(1,:),P1B(2,:),P1B(3,:),'LineWidth',1,'Color','k');
    quiver3(P0B3(1,:),P0B3(2,:),P0B3(3,:),P1B(1,:),P1B(2,:),P1B(3,:),'LineWidth',1,'Color','k');
    
    % quad_plot(p_star,R_star,0,[],0.5,scale);
    quad_plot(p_star1,R_star,0,[],0.5,scale);
    quad_plot(p_star2,R_star,0,[],0.5,scale);
    quad_plot(p_star3,R_star,0,[],0.5,scale);
    
    % P0B = repmat(p_star,1,4);
    P0B1 = repmat(p_star1,1,4);
    P0B2 = repmat(p_star2,1,4);
    P0B3 = repmat(p_star3,1,4);
    
    % set(gca,'Xlim',[-5 30], 'YLim', [-5 30], 'ZLim', [0 40]);
    % set(gca,'Xlim',[ 30], 'YLim', [-5 30], 'ZLim', [0 40]);
    view(37.5,30);axis square;
    legend(leg,{'Quad 1','Quad 2','Quad 3'},'location','NorthEast')
    
    F(frame) = getframe(gcf);
    writeVideo(vidObj,F(frame));
    frame = frame+1;
end

hold off
hold off;grid off;
