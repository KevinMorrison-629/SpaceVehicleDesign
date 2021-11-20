clc;
clear;




%% Orbit Parameters

% the distance from the sun every month (in au)
sun_dist = [4.087 3.964 3.818 3.667 3.499 3.326 3.136 2.932 2.723 ...
            2.493 2.258 2.003 1.740 1.495 1.258 1.098 1.069 1.186 1.403];

x = 1:19;
P = polyfit(x, sun_dist, 5);
yfit = P(1).*x.^5 + P(2).*x.^4 + P(3).*x.^3 + P(4).*x.^2 + P(5).*x + P(6);

figure(101)
scatter(x, sun_dist)
hold on
plot(x, yfit)

sun_dist_func = @(x) P(1).*x.^5 + P(2).*x.^4 + P(3).*x.^3 + P(4).*x.^2 + P(5).*x + P(6);
        
timesteps = 12960;
        
        
figure(1)
plot(sun_dist)
xlabel('Time (months)')
ylabel('Distance from Sun (au)')
title('Solar Distance')


%% Solar Power
T0 = 28; % degrees C
P_rated = 1353; % W/m^2

H_I = @(R) (1./R).^2;
N_T = @(T) 1 - (1.6e-3).*(T - T0);
N_plf = @(alpha) cos(alpha);

N_rad = linspace(0.99, 0.9, timesteps);
N_uv = linspace(1, 0.98, timesteps);
N_cy = linspace(0.99, 0.97, timesteps);
N_m = 0.975;
N_i = 0.98;
N_con = 0.98;
N_s = 1; % assume negligible shadowing

P_array = @(ts, T, R, alpha) P_rated.*N_rad(ts).*N_uv(ts).*N_cy(ts).*N_m.*N_i.*N_con.*N_T(T).*H_I(R).*N_plf(alpha);

T_arr = @(i) 20 + 20.*sin((i).*pi/120);
alpha_arr = @(i) 0*(pi/180);

Power_per_square_meter = [];
Temperature_hist = [];

for i = 1:timesteps
    Power_per_square_meter(i) = P_array(i, T_arr(i), sun_dist_func(i/720), alpha_arr(i));
    Temperature_hist(i) = T_arr(i);
end



figure(2)
plot(Power_per_square_meter)
xlabel('Time (hours)')
ylabel('Power Generation per Square Meter')
title('Power Generation Througout Mission')

figure(3)
plot(Temperature_hist)
xlabel('Time (hours)')
ylabel('Temperature')
title('Temperature History')
ylim([-20, 60])



%% 

x = 1:19;
sun_dist = [4.087 3.964 3.818 3.667 3.499 3.326 3.136 2.932 2.723 ...
            2.493 2.258 2.003 1.740 1.495 1.258 1.098 1.069 1.186 1.403];

P = polyfit(x, sun_dist, 5);
yfit = P(1).*x.^5 + P(2).*x.^4 + P(3).*x.^3 + P(4).*x.^2 + P(5).*x + P(6);

figure(101)
scatter(x, sun_dist)
hold on
plot(x, yfit)

sun_dist_func = @(x) P(1).*x.^5 + P(2).*x.^4 + P(3).*x.^3 + P(4).*x.^2 + P(5).*x + P(6);
