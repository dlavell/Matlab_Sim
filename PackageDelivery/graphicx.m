%read data from sim

load('SimResults.mat')
%R = angle2dcm(euler.signals.values(:,1)'*pi/180,euler.signals.values(:,2)'*pi/180,euler.signals.values(:,3)'*pi/180,'ZYX');
%Rcam = angle2dcm(pi,0,0,'YZX');
Rcam = [-1 0 -0; 0 1 0; 0 0 -1];
p_i = Rcam*state.signals.values';
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
    if I >=length(p_i) || I >=length(pdt)
        close(vidObj);
        display('length of internal variable caused the video to end.')
        if I >=length(pdt)
            display('pdt was the cause');
            length(pdt)
        end
        if I >=length(p_i)
            display('p_i was the cause');
            length(p_i)
        end
        break
    end
    
    R(:,:,I) = eye(3);
    hold on
    cla
    leg(1) = plot3(pdt(1,1:I)',pdt(2,1:I)',pdt(3,1:I)');
    leg(2) = plot3(p_i(1,1:I)',p_i(2,1:I)',p_i(3,1:I)','r-');
    quiver3(P0(1,:),P0(2,:),P0(3,:),P1(1,:),P1(2,:),P1(3,:),'LineWidth',2,'Color','k');
    grid on
    p_star = p_i(:,I);
    R_star = Rcam*R(:,:,I)';
    scale = 0.5;
    P0B = repmat(p_star,1,3);
    P1B = R_star*scale;
    
    quiver3(P0B(1,:),P0B(2,:),P0B(3,:),P1B(1,:),P1B(2,:),P1B(3,:),'LineWidth',1,'Color','k');
    quad_plot(p_star,R_star,0,[],0.5,scale);
    P0B = repmat(p_star,1,4);
    set(gca,'Xlim',[-5 30], 'YLim', [-5 30], 'ZLim', [0 40]);
    view(37.5,30);axis square;
    legend(leg,{'Reference Trajectory','Actual Trajectory'},'location','NorthEast')
    
    F(frame) = getframe(gcf);
    writeVideo(vidObj,F(frame));
    frame = frame+1;
end

hold off
hold off;grid off;
