%read data from sim
%clear all
%load('SimResults2.mat')
Rcam = [1 0 0; 0 1 0; 0 0 1];
numQuads = 10;

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
%img = imread('San_Jose.JPG');
%imgsec([min_x,maxx],[min_y max_y],img);
%imagesc([36.8 37.2], [-122.3 -121.8],img);

% Terrain Graph
x = 37:.001:38;
y = 121:.001:122.5;
y = y * -1;
z = csvread('SJ-lat-lng-3decimal.csv');
z = transpose(z(:,1:1501));


%source lat and long
lat_s = 37.3154997;
lng_s = -121.8728929;
%imagesc( [lat_s-1.5 lat_s+1.5], [lng_s-2.25 lng_s+2.25] ,img);
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
    
    % Terrain Graph    
    %mesh(x,y,z);
    %shading interp;
    
    % cycle through each quad for each frame
    for quad = 1: numQuads % length(state.signals.values(1,:))/3
        %pos = state.signals.values(:,3*quad-2:3*quad)';
        pos = quads.signals.values(:,3*quad-2:3*quad)';
        pos_R = Rcam*pos;
        if I >=length(pos_R(1,:)) 
            close(vidObj);
            display('Finished making video.')
            break;
        end
        %imagesc( [lat_s-.15 lat_s+.15], [lng_s-.225 lng_s+.225] ,img); %added
        leg(quad) = plot3(pos_R(1,1:I)',pos_R(2,1:I)',pos_R(3,1:I)','r-');
        p_star = pos_R(:,I);
        quad_plot(p_star,R_star,0,[],0.5,scale);
    end % end for-loop
    
    set(gca,'Xlim',[lat_s-.15 lat_s+.15], 'YLim', [lng_s-.225 lng_s+.225], 'ZLim', [0 800]);
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
