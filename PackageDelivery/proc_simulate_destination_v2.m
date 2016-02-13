% proc_simulate_destination
%
lambda_s = 37.0000; %(degrees)
tau_s = -122.0600; %(degrees)
desired_radius_min = 2; % nautical-miles
desired_radius_max = 10; %nautical-miles
number_of_destinations = 100000;
print_on = 0; % 0 turns it off. 1 turns it on
%
lambda_hist = zeros(number_of_destinations,1);
tau_hist = zeros(number_of_destinations,1);
for i = 1:number_of_destinations
    [lambda,tau]=simulate_destination_v2(lambda_s,tau_s,desired_radius_min,desired_radius_max);
    if (print_on == 1)
        fprintf('Latitude = %f (degrees) Longitude = %f (degrees)\n',lambda, tau);
    end
    lambda_hist(i) = lambda;
    tau_hist(i) = tau;
end
%
plot(tau_hist,lambda_hist,'xb','MarkerSize',10);
hold on;
plot([tau_s,tau_s],[lambda_s,lambda_s],'or','MarkerSize',10);
xlabel('Longitude (degrees)','FontSize',18);
ylabel('Latitude (degrees)','FontSize',18);
set(gca,'Fontsize',18,'LineWidth',2);
grid on;