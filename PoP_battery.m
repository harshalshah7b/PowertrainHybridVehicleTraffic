function [Pbatt] = PoP_battery(Vbatt, Ibatt)
% % Battery Parameters
% Sbank = 6;              % # of banks in series 
% Ncell = 40;             % # of cell in series
% Pbank = 1;              % # of banks in parallel
% Rcell = 0.00051;
% Rint = Rcell*Ncell*Sbank/Pbank;
eta_batt = 0.92;
Pbatt = (Ibatt*Vbatt.*eta_batt.^(-sign(Ibatt)));
% Pbatt = Vbatt*Ibatt+Ibatt.^2*Rint;
end