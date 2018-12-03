clear;
clc;

%% Data

% load('enginedata.mat');
% % Data from (TOYOTA PRIUS): https://media.toyota.co.uk/wp-content/files_mf/1329489972120216MTOYOTAPRIUSTECHNICALSPECIFICATIONS.pdf

Vehicle_length = 4.48; % meter
Vehicle_mass = 1805; % kg
Gear_ratio = 2.683; % Forward Gear Ratio
Differential_drive_ratio = 3.267; % Differential Gear Ratio
Wheel_radius = 0.3175; % m => 12.5 inch tire radius (195/65r15 tire);
Max_power = 73000; % watt

% Assumptions:
time = 1:100;
random_values = 0.2*rand(length(time),1) + 0.6; % limiting range of random variables
Road_length = 500; % meter
Vehicle_safe_distance = 3.0 ; % meter
Vehicle_max_velocity = 29.057; % m/s => 65 mph
Engine_efficiency = 0.80;

%% PDE model for Vehicle Density

Road_density_max = Road_length / (Vehicle_length + Vehicle_safe_distance);
Road_density_random = random_values * Road_density_max;
velocity_vehicle = Vehicle_max_velocity * (1 - (Road_density_random / Road_density_max));

%% Vehicle Modeling

Net_Gear_ratio = Gear_ratio * Differential_drive_ratio;
Wheel_rpm = (velocity_vehicle*60)./(2*pi*Wheel_radius);
Engine_rpm = Wheel_rpm * Net_Gear_ratio;
Engine_torque = Max_power ./ Wheel_rpm;
Wheel_torque = Engine_torque .* Net_Gear_ratio * 0.5;

%% Optimization

% Polynomial derived from curve fitting app using given engine data
% X axis: eng_consum_spd
% Y axis: eng_consum_trq
% Z axis: eng_fuel_map_gpkWh
% Polynomial order: (5,5)

p00 = 976.7;
p10 = -12.14;
p01 = -3.574;
p20 = 0.1646;
p11 = -0.02367;
p02 = 0.01739;
p30 = -0.0009203;
p21 = 0.000117;
p12 = 4.708e-06;
p03 = -2.651e-05;
p40 = 2.344e-06;
p31 = -2.78e-07;
p22 = -4.89e-09;
p13 = 6.132e-10;
p04 = 1.604e-08;
p50 = -2.234e-09;
p41 = 2.812e-10;
p32 = -4.378e-11;
p23 = 2.251e-11;
p14 = -6.75e-12;
p05 = -1.881e-12;

% x0 = [Engine_rpm(1) Engine_torque(1)];
x0 =[100 100];

bsfc = @(x) (1.5*(p00 + p10*x(1) + p01*x(2) + p20*x(1)^2 + p11*x(1)*x(2) + p02*x(2)^2 + p30*x(1)^3 + p21*x(1)^2*x(2) ...
                + p12*x(1)*x(2)^2 + p03*x(2)^3 + p40*x(1)^4 + p31*x(1)^3*x(2) + p22*x(1)^2*x(2)^2 ...
                + p13*x(1)*x(2)^3 + p04*x(2)^4 + p50*x(1)^5 + p41*x(1)^4*x(2) + p32*x(1)^3*x(2)^2 ...
                + p23*x(1)^2*x(2)^3 + p14*x(1)*x(2)^4 + p05*x(2)^5));
dl_0 = 0;
df_0 = 0;
dt = 1;

for i = time
    
   dl(i) = velocity_vehicle(i)*dt + dl_0; % Distance(position) of Leading Vehicle
   dl_0 = dl(i);
   
end

for j = time

LB = [min(Engine_rpm) min(Engine_torque)];
UB = 8*[Engine_rpm(j) Engine_torque(j)];
A = [-dl(j) df_0; dl(j) -df_0];
B = [-3; 5]; %[Safe_Distance; max_allowed_distance_for_following_purpose]

x = fmincon(bsfc,x0,A,B,[],[],LB,UB);

x1(j) = x(1);
x2(j) = x(2);

mf(j) = abs(bsfc(x))/(3.6e9*Engine_efficiency)*0.8;
vel(j) = ((x2(j)./Net_Gear_ratio)*(2*pi/60))*(Wheel_radius);

df(j) = vel(j)*dt + df_0; % Distance(position) of following vehicle
df_0 = df(j);

x0=x;

end

figure(1);
hold on;
% subplot(2,1,1);
plot(time,mf)
xlabel("Time (sec)");
ylabel("Mass flow rate (Kg/s)");
hold off;

figure(2)
hold on;
% subplot(2,1,2);
plot(time,vel)
xlabel("Time (sec)");
ylabel("Our Vehicle Optimized Velocity (m/s)");
hold off;

cyc_mph(:,1) = time;
cyc_mph(:,2) = vel;
save('cyc_mph.mat','cyc_mph')

% ans = fMPG(4);