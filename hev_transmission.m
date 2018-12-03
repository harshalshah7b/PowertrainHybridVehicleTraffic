% HEV Vehicle Architecture Exploration Research
% Optimal Design Laboratory
% University of Michigan
% ***********************************************
% Transmission Model File v1.0
% Created on 11/09/2012 by Alparslan Emrah Bayrak
% ***********************************************
% (A Quasi-stationary model)
% ***********************************************
% Parameter Definitions:
% Wtrans: Transmission shaft speed [rpm]
% Weng: Engine speed [rpm]
% Ttrans: Transmission shaft torque [Nm]
% Teng: Engine torque [Nm]
% Cmode_array: Cell array of transmission matrices of size 2x2 (Coming from Bond Graph Study)
% mode: Mode signal coming from the controller
function [Tmg1,Wmg1] = hev_transmission(Wtrans, Weng, Ttrans, Teng, Cmode)
MinCmodeTrans = -Cmode';
DetMinCmodeTrans = det(MinCmodeTrans);
InvMinCmodeTrans = 1/DetMinCmodeTrans*[MinCmodeTrans(4), -MinCmodeTrans(3); -MinCmodeTrans(2), MinCmodeTrans(1)];

% Quasi-stationary kinematic equations.
Wmg1 = Cmode(1,1)*Weng + Cmode(1,2)*Wtrans;
% Wmg2 = Cmode(2,1)*Weng + Cmode(2,2)*Wtrans;
% Wmg1 = Cmode(1,:)*[Weng;Wtrans];
% Wmg2 = Cmode(2,:)*[Weng;Wtrans];

Tmg1 = InvMinCmodeTrans(1,1)*Teng - InvMinCmodeTrans(1,2)*Ttrans;
% Tmg2 = InvMinCmodeTrans(2,1)*Teng - InvMinCmodeTrans(2,2)*Ttrans;
% Tmg1 = InvMinCmodeTrans(1,:)*[Teng;-Ttrans];
% Tmg2 = InvMinCmodeTrans(2,:)*[Teng;-Ttrans];
end