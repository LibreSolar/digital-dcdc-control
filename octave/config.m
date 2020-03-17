
% System parameters for MPPT charge controller

Vhs = 28;               % voltage at high-side (input of buck converter)
Chs = 470e-6;           % capacitor at low-side
Rhs = 2e-3;             % ESR of low-side capacitor

L = 22e-6;              % inductor
Rind = 2.6e-3;          % inductor series resistance

Vls = 12;               % voltage at low-side (output of buck converter)
Cls = 820e-6;           % capacitor at low-side
Rls = 4.2e-3;           % ESR of low-side capacitor
Rload = 12;             % load resistance in power plant

fs = 70e3;              % switching frequency
Ts = 1/fs;              % switiching period

D = Vls/Vhs;            % steady state duty cycle
td = 0.25*Ts + D*Ts;    % delay in ADC + compensator + modulator (DTs < td < Ts)

