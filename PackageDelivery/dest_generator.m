function [dest_lat_hist, dest_lng_hist] = ...
    dest_generator(lat_s, lng_s, radius_min, radius_max, n_dest)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% proc_simulate_destination
%

% lat_s = 37.0000; % Source latitude(degrees)
% lng_s = -122.0600; % Source longitude(degrees)
% radius_min = 2; % nautical-miles
% radius_max = 10; %nautical-miles
% n_dest = 10;
print_on = 0; % 0 turns it off. 1 turns it on
plot_on = 0;  % 0 turns it off. 1 turns it on
%
dest_lat_hist = zeros(n_dest,1);
dest_lng_hist = zeros(n_dest,1);
for i = 1:n_dest
    [lat,lng]=simulate_destination_v2(lat_s,lng_s,radius_min,radius_max);
    if (print_on == 1)
        fprintf('Latitude = %f (degrees) Longitude = %f (degrees)\n',lat, lng);
    end
    dest_lat_hist(i) = lat;
    dest_lng_hist(i) = lng;
end

if(plot_on)
    plot(dest_lng_hist,dest_lat_hist,'xb','MarkerSize',10);
    hold on;
    plot([lng_s,lng_s],[lat_s,lat_s],'or','MarkerSize',10);
    xlabel('Longitude (degrees)','FontSize',18);
    ylabel('Latitude (degrees)','FontSize',18);
    set(gca,'Fontsize',18,'LineWidth',2);
    grid on;
end

end

