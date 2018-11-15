clear;
clc;

%% Data

load('enginedata.mat');

% % Data from (TOYOTA PRIUS): https://media.toyota.co.uk/wp-content/files_mf/1329489972120216MTOYOTAPRIUSTECHNICALSPECIFICATIONS.pdf

for j = 1:100
    
Vehicle_length = 4.48; % meter
Vehicle_weight = 1805; % kg
Max_Engine_torque = 142; % Nm @ 4000 rpm
Gear_ratio = 2.683; % Forward Gear Ratio
Final_drive_ratio = 3.267; % Differential Gear Ratio
Wheel_radius = 0.3175; % m => 12.5 inch tire radius (195/65r15 tire);
Max_power = 73000; % watt
Calorific_Value_Gasoline = 45.8e6; % J/kg

% Assumptions:
iter = 100;
Road_length = 500; % meter
Vehicle_safe_distance = 3.0 ; % meter
Vehicle_max_velocity = 29.0576; % m/s => 65 mph
Engine_efficiency = 0.80;

%% PDE model for Vehicle Density

Road_density_max = Road_length / (Vehicle_length + Vehicle_safe_distance);
random_values = 0.4 .* rand(iter,1) + 0.5;
Road_density_random = random_values .* Road_density_max;
velocity_vehicle = Vehicle_max_velocity .* (1 - (Road_density_random ./ Road_density_max));

%% Vehicle Modeling

Net_Gear_ratio = Gear_ratio * Final_drive_ratio;
Wheel_rpm = (velocity_vehicle*60)./(2*pi*Wheel_radius);
Engine_rpm = Wheel_rpm * Net_Gear_ratio;
Engine_torque = Max_power ./ Engine_rpm;
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

x0 = [Engine_rpm(1) Engine_torque(1)];

LB = [min(Engine_rpm) min(Engine_torque)];
UB = [max(Engine_rpm) max(Engine_torque)];

spd = x0(1);
trq = x0(2);

bsfc =@(x) (p00 + p10*spd + p01*trq + p20*spd^2 + p11*spd*trq + p02*trq^2 + p30*spd^3 + p21*spd^2*trq ...
                + p12*spd*trq^2 + p03*trq^3 + p40*spd^4 + p31*spd^3*trq + p22*spd^2*trq^2 ...
                + p13*spd*trq^3 + p04*trq^4 + p50*spd^5 + p41*spd^4*trq + p32*spd^3*trq^2 ...
                + p23*spd^2*trq^3 + p14*spd*trq^4 + p05*trq^5);

x = fmincon(bsfc,x0,[],[],[],[],LB,UB);
x1(j,1)=x(1);
x1(j,2)=x(2);
mf(j) = abs(bsfc(x))/(3.6e9*Engine_efficiency);

end

figure(1);
hold on;
xlabel("time (sec)");
ylabel("Engine Speed (rpm)");
plot(1:100,x1(:,1))
hold off;

figure(2);
hold on;
xlabel("time (sec)");
ylabel("Engine Torque (Nm)");
plot(1:100,x1(:,2))
hold off;

figure(3);
hold on;
xlabel("time (sec)");
ylabel("Mass flow rate (Kg/s)");
plot(1:100,mf)
hold off;