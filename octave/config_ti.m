
% System parameters from TI example

Vhs = 10;               % voltage at high-side (input of buck converter)
Chs = 800e-6;           % capacitor at low-side
Rhs = 2e-3;             % ESR of low-side capacitor

L = 1e-6;               % inductor
Rind = 8.6e-3;          % inductor series resistance

Vls = 1;                % voltage at low-side (output of buck converter)
Cls = 800e-6;           % capacitor at low-side
Rls = 2e-3;             % ESR of low-side capacitor
Rload = 10;             % load resistance in power plant

fs = 500e3;             % switching frequency
Ts = 1/fs;              % switiching period

D = Vls/Vhs;            % steady state duty cycle
td = 0.25*Ts + D*Ts;    % delay in ADC + compensator + modulator (DTs < td < Ts)
