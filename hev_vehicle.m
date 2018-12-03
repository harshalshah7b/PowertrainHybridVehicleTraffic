% HEV Vehicle Architecture Exploration Research
% Optimal Design Laboratory
% University of Michigan
% ***********************************************
% Vehicle Model File v2.0
% Created on 10/18/2012 by Alparslan Emrah Bayrak
% ***********************************************
% Parameter Definitions:
% Tshft: Driving torque at front wheel shaft [Nm]
% Trear: Driving torque at rear wheel shaft [Nm]
% Vwind: Wind speed [m/s]
% alpha_road: Road inclination angle [rad]
% Vveh_old: In simulation vehicle speed [m/s]
% Fext: External force applied to the vehicle from front side [N]
% dt: Simulation time step
function [Wshft, Tshft, Pshft] = hev_vehicle(Vwind, alpha_road, Vveh, Mveh, dt)
in2m=0.0254;            %inches to meter
rps2rpm = 30/pi;        % rad/s to rev/sec
% Vehicle Parameters
Drim = 12.5*in2m;         % Rim Diameter [m]
width_tyre = 195/1000;  % Tyre Width [m]
height_tyre = 65;       % Tyre Height [%]
Jwheel = 0.5;           % Wheel Inertia [kg.m^2]
Mveh = 1805;          % Static Vehicle Mass [kg]
f_fric = 0.01;          % Coulomb Friction Coefficient
k_fric = 0;             % Viscous Friction Coefficient
wind_fric = 0;          % Windage Coefficient
Cdrag = 0.29;           % Aerodynamic Drag Ceofficient
% Cdrag = 0.3;
Sveh = 2;               % Vehicle Frontal Area [m^2]
% Sveh = 3.58;
g = 9.81;               % Gravitational Acceleration [m/s^2]
rho_air = 1.205;        % Air density [kg/m^3]
% rho_air = 1.2;
Cstic = 1.2;            % Stiction Coefficient
if (Vveh ~= 0)
    Cstic = 1;
end
FR = 3.9;

% Internal variables
Rwheel = 0.5*Drim+width_tyre*height_tyre/100;   % Wheel radius
Rwheel_dyn = 0.97*Rwheel;                       % Dynamic wheel radius
Mveh_dyn = Mveh + 4*Jwheel/Rwheel^2;            % Dynamic vehicle mass
Fcl = Mveh*g*sin(atan(alpha_road*0.01));        % Climbing Resistance Force
Faero = 0.5*rho_air*Cdrag*Sveh*(Vveh+Vwind).^2;  % Areodynamic Drag Force
Froll = Mveh*g*(f_fric+k_fric*Vveh+wind_fric*Vveh.^2).*sign(Vveh);   % Rolling Friction Force
Fres = Fcl+Faero+Froll;                         % Total Resistive Force
Vveh_new = [Vveh(2:(length(Vveh))) 0];
aveh = (Vveh_new - Vveh)/dt;                    % a = dV/dt
Fnet = Mveh_dyn*aveh;                           % F = m*a
Fdr = Fnet + Fres*Cstic;                        % Force balance
Tshft = Fdr*Rwheel_dyn;                         % T = F*R
Wshft = Vveh/Rwheel_dyn*rps2rpm;                % Rotary Velocity of the Front Wheel
Pshft = Fdr.*(Vveh+Vveh_new)/2;
end