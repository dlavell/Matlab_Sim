%read data from sim
%known working
%load('teste_bom_h0_1_ganhos_3_6_03_001_3_40')

load('SimResults.mat')
%load('20131030_h0_minus1_ganhos_casau.mat')
R = angle2dcm(euler.signals.values(:,1)'*pi/180,euler.signals.values(:,2)'*pi/180,euler.signals.values(:,3)'*pi/180,'ZYX');
R_ = R;
R = cat(3,R(:,:,:),R_(:,:,:));   
R = cat(3,R(:,:,:),R_(:,:,:));   % Allows longer graphic simulation
Rcam = angle2dcm(pi,0,0,'YZX');
p_i = Rcam*p.signals.values';
pdt = Rcam*ref.signals.values(:,1:3)';
t = ref.time;
%draw
P0 = zeros(3);
P1 = Rcam;
vidObj = VideoWriter('sim_testing_live', 'Uncompressed AVI');
step = 0.5;% was .05
vidObj.FrameRate = 1/step;
open(vidObj);
%define cube (rigid body)
frame = 1;
figure('Units','centimeters','Position',[0 0 34 17])
%h = create_axis([2, 0.1 ,0.1 , 0.1],[1 0.1 0.1 0.1]);
h = create_axis([1, 0.1 ,0.1 , 0.1],[1 0.1 0.1 0.1]);
%T = 100;
R0= quat2dcm(q0.signals.values);
for t2 = 0:step:T
    I = find(t>=t2,1);
    if I >=length(R)
        close(vidObj);
        %display('I >=length(R) caused the break.')
        break
    end
    if isempty(I) 
        close(vidObj);
        %display('isempty(I)  caused the break.')
        break
    end
    %axes(h(1))
    axes(h)
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
    %
%     axes(h(2))
%     hold on
%     grid on
%     cla
%     scale = 1.5;
%     P0B = zeros(3);
%     P1B = R_star*scale;
%     P2B = Rcam*R0(:,:,I)*scale;
%     leg2(1) = quiver3(P0B(1,:),P0B(2,:),P0B(3,:),P1B(1,:),P1B(2,:),P1B(3,:),'LineWidth',1,'Color','k');
%     leg2(2) = quiver3(P0B(1,:),P0B(2,:),P0B(3,:),P2B(1,:),P2B(2,:),P2B(3,:),'LineWidth',2,'Color','r');
%     legend(leg2,{'R(q)','R(q_0)'},'location','NorthEast','fontsize',16)
%     quad_plot(zeros(3,1),R_star,0,[],0.5,1);
%     set(gca,'Xlim',[-1 1], 'YLim', [-1 1], 'ZLim', [-1 1]);
%     view(37.5,30);axis square;
    %
    F(frame) = getframe(gcf);
    writeVideo(vidObj,F(frame));
    frame = frame+1;
end
%movie(F,1,10)
%close(vidObj);
hold off
hold off;grid off;
%set(gca,'XTickLabel','','YTickLabel','','ZTickLabel','')
%xlabel('x [m]','FontSize',16)
%ylabel('y [m]','FontSize',16)
%zlabel('z [m]','FontSize',16)
%legend(leg,{'Reference Trajectory','Actual Trajectory'},'location','NorthEast')