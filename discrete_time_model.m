%% Load necessary packages

pkg load control;
pkg load signal;
clc;
clear;
close all;
grid on;

%% Plant transfer function (Continous time domain):

% notations:
    % x' = A*x + B*Vg
    % vout = C*x + D*Vg
    % d    - control signal
    % vout - ouput of the plant
    % x    - state variable matrix

% Our System:

    Vout = 12;      % outputof the power stage
    Vg = 28;        % input to power stage
    fs = 50e3;      % switcing freq
    Ts = 1/fs;      % switiching period
    Cap = 820e-6;   % Cap in power plant
    Resr = 0.042;   % equivalent series resistance of cap
    L = 22e-6;      % Ind in power plant
    RL = 0.0026;    % series resistance of inductor
    R = 12;         % Load resistance in power plant
    Duty = 12/28;   % steady state Duty cycle
    td = 0.25*Ts + Duty*Ts;     % delay in A/D + Compenstor + Modulator (DTs < td < Ts)

% Plant state space modelling:

    Rpar = (R*Resr)/(R+Resr);
    p1 = -(1/((R+Resr)*Cap));
    p2 = ( R /((R+Resr)*Cap));
    p3 = -(R/((R+Resr)*L));
    p4 = -((RL + Rpar)/L);
    p5 = (R/(R+Resr));
    A1 = [p1 p2 ; p3 p4];       % A    - State matrix A.
    A2 = [0];
    B1 = [0;(1/L)];             % B    - Input-to-state matrix B
    B2 = [0];
    C  = [p5, Rpar];            % C    - State-to-output matrix C
    D  = 0;                     % D    - Feedthrough matrix D
    I  = eye(2);
    plant = ss(A1,B1,C,D);
    [n_plant,d_plant] = ss2tf(A1,B1,C,D);
    plant_tf = tf(n_plant,d_plant);


    %% Exponentials are not handled by Octave control package to include a delay
    %% Using the Pade approximation for delay seems to be the best work around
    %% Pade (Octave): https://octave.sourceforge.io/octave/function/padecoef.html
    %% Pade (Wiki): https://en.wikipedia.org/wiki/Pad%C3%A9_table

% TO BE IMPLEMENTED


% plotting Step Response of Plant:

    figure(1);
    step(plant);
    title('Step Response of the Plant only');

% Calculation of Bandwidth of the Plant:

    %Stepinfo function now available based on funtion forum input here: https://lists.gnu.org/archive/html/help-octave/2015-02/msg00023.html
    % Function is not verified as working.
%    plant_info = stepinfo(plant);
%    [Wn_plant,DR_plant,Poles_plant] = damp(plant);

 % BEWARE 0 Stepinfo function is providing incorrect information.
%    temp1 = sqrt((4*(DR_plant(1))^4) - (4*(DR_plant(1))^2) +2);
%    temp2 =  1 - (2*(DR_plant(1))^2);
%
%    %%Depends on plant info - using current stepinfo command (not tested)
%    temp3 = 4/(plant_info.SettlingTime*(DR_plant(1)));
%    BW_plant = temp3*sqrt(temp1 + temp2)

% plotting Bode plot for Plant:

    %% Use bode (Octave) vs bodeplot (Matlab) , similar functionality
    figure(2)
    bode(plant);
    title('Bode diagram of the Plant only (continous model)');
    [Gm_Plant,Pm_Plant,Wcg_Plant,Wcp_Plant] = margin(plant);



%% Divider circuitry:

% Documentation here: https://learn.libre.solar/b/dc-control/development/digital_control.html#modelling-the-digital-controller
% Voltage is divided to ADC full scale by divider circuit:
     R1 = 100e3;
     R2 = 5.6e3;
     Cz = 0;%ORIGIRNAL CODE:%Cz = 1e-6;
     %Typically not populated as this is not a programmable compensation zero
     %Hardware currently does not have Cz.
     Rp = (R1*R2)/(R1+R2);
     Cp = 10e-9; %ORIGINAL CODE: Cp = 1/(2*pi*fs*Rp);
     % Cp = 10nF is as implemented in hardware

% Gain of the Divider:

     Kdiv = R2/(R1+R2);
     Vsense = Vout * Kdiv; %The sensed voltage in for control loop. Max voltage is 3.3V.

% Transfer function of the Divider circuit:

     G1 = tf([R1*Cz 1],[Kdiv*R1*(Cz+Cp) 1]);     % continuous time domain (S Domain)

%Plant Plus Divider Transfer Function:

    plant_plus_divider = plant_tf * G1;
    [y1, t1, x1] = step(plant_plus_divider); % Get info for multiple plotting on the same axis
    [mag1, pha1, w1] = bode(plant_plus_divider); % Get info for multiple plotting on the same axis

    figure(3);
    step(plant_plus_divider);
    title('Step Response of the Plant Plus Divider');
    legend("Plant Plus Divider")
    [Gm_plant_plus_divider,Pm_plant_plus_divider,Wcg_plant_plus_divider,Wcp_plant_plus_divider] = margin(plant_plus_divider);

%Check system controlability:

%TO BE IMPLEMENTED
%    [A B C D] = tf2ss(plant_plus_divider)
%    rank_system = rank(ctrb(A,B))   %Ensure



% Compare bode plots with different divider circuits
% Plot to show minimal difference in Cp values

    Cp2 = 1/(2*pi*fs*Rp); %New Cp value to compare with hardware value of 10e-9F

% Transfer function of the Divider circuit:

    G2 = tf([R1*Cz 1],[Kdiv*R1*(Cz+Cp2) 1]);
    plant_plus_divider_2 = plant_tf * G2;
    [y2, t2, x2] = step(plant_plus_divider_2);
    [mag2, pha2, w2] = bode(plant_plus_divider_2);

%Plot comparison of step responses
    figure(4)
    plot(t1,y1, 'r:');
    hold on;
    plot(t2,y2, 'k--');
    hold off;
    title('Comparison of divider circuits')
    legend("Cp = 10e-9 (Hardware)", "Cp = 0.6e-9 (Theory)")

% Check plant_plus_divider poles and zeros to simplify the model
% Dominant pole approximation to simplify model
% https://lpsa.swarthmore.edu/PZXferStepBode/DomPole.html

    [zeros, poles, k] = tf2zp(plant_plus_divider);


%%--- OUTPUT --- %%
%{
    zeros = -29036.00465
    poles =

    -18857.14286 +     0.00000i
    -1060.94289 +  7356.99698i
    -1060.94289 -  7356.99698i

    k =  35874439.46188
%}

%>> plant_plus_divider
%{
    Transfer function 'plant_plus_divider' from input 'u1' to output ...

                        1902 s + 5.524e+07
     y1:  ----------------------------------------------
          5.303e-05 s^3 + 1.113 s^2 + 5052 s + 5.525e+07
%}

%Use output to determine a simpler model
% Since the second and third poles are dominant, the first pole at s = 188857 can be ignored.

% New second order model form: K/(A*s^2 + B*s + C)
    K = 1902 * 29036;   %Set gain to match the previous transfer function
    A = 1;
    B = 2*1060.94289;
    C =  7356.99698^2;

    second_order = tf([K],[A B C])

%Assess new second order model vs thrid order

    [y3, t3, x3] = step(second_order); % Get info for multiple plotting on the same axis
    [mag3, pha3, w3] = bode(second_order); % Get info for multiple plotting on the same axis

%Plot comparison of step responses

    figure(5)
    plot(t1,y1, 'r:');
    hold on;
    plot(t3,y3, 'k--');
    hold off;
    title('Comparison second order and third order model')
    legend("Third order model", "Second order model")

%% Multiple bode plots

    figure(6)
    grid on;

    subplot(2,1,1)
    hold on;
    semilogx(w1,20*log10(abs(mag1)));
    semilogx(w3,20*log10(abs(mag3)));
    hold off;
    legend("Third order model", "Second order model");
    title("Constructed magnitude plot for comparison of second and third order models");

    subplot(2,1,2)
    hold on;
    semilogx(w1, pha1);
    semilogx(w3, pha3);
    hold off;
    legend("Third order model", "Second order model");
    title("Constructed phase plot for comparison of second and third order models");

% Sense check that bode plots are plotting correctly.

%{
    figure(10)
    bode(plant_plus_divider)
    title("Actual bode plot for third order model")
    figure(11)
    bode(second_order)
    title("Actual bode plot for second order model")
%}

%%--- TUNING  METHODS --- %%

% http://faculty.mercer.edu/jenkins_he/documents/TuningforPIDControllers.pdf

%From inspection values of L (dead time) and T (time constant) are:
% L = 0.00006s
% T = 0.00017s

% Ziegler-Nichls Tuning Rules give

%% P Controller (1)
% P_1 = T/L
% I_1 = Inf.
% D_1 = 0

%% PI COntroller (2)
% P_2 = 0.9*T/L
% I_2 = L/0.3
% D_2 = 0

%% PID Controller (3)
% P_3 = 1.2*T/L
% I_3 = 2*L
% D_3 = 0.5*L

% Read parameters

   L = 0.00006;
   T = 0.00017;

%Define PID Parameters

   P_1 = T/L;  %P control only
   I_1 = 1e30;% approximately Inf
   D_1 = 0;

   P_2 = 0.9*T/L;  %PI control
   I_2 = L/0.3;
   D_2 = 0;

   P_3 = 1.2*T/L;   %PID control
   I_3 = 2*L;
   D_3 = 0.5*L;


% Define feedback transfer functions
    s = tf('s');
    FB_P = tf(P_1 + I_1/s + D_1*s);
    FB_PI = tf(P_2 + I_2/s + D_2*s);
    FB_PID = tf(P_3 + I_3/s + D_3*s);

% Apply feedback loop

%% CHOICE TO APPLY FEEDBACK LOOP TO SECOND OR THIRD ORDER SYSTEM
    %Second order
%    PID_P = feedback(second_order,FB_P);
%    PID_PI = feedback(second_order,FB_PI);
%    PID_PID = feedback(second_order,FB_PID);

    %Third order
    PID_P = feedback(plant_plus_divider,FB_P);
    PID_PI = feedback(plant_plus_divider,FB_PI);
    PID_PID = feedback(plant_plus_divider,FB_PID);
% Get step data

    [y_P, t_P, x_P] = step(PID_P);
    [y_PI, t_PI, x_PI] = step(PID_PI);
    [y_PID, t_PID, x_PID] = step(PID_PID);

% Get bode data

    [mag_P, pha_P, w_P] = bode(PID_P);
    [mag_PI, pha_PI, w_PI] = bode(PID_PI);
    [mag_PID, pha_PID, w_PID] = bode(PID_PID);

%Plot comparison of step responses

    figure(7)
    plot(t_P,y_P, 'r:');
    hold on;
    plot(t_PI,y_PI, 'k--');
    plot(t_PID,y_PID, 'b-.');
    hold off;
    title('Comparison P, PI and PID control')
    legend("P control", "PI control", "PID control")


%% -- OTHER FEEDBACK METHODS --
%% Find ultimate gain with feedback
%% Pol placement method


%% Additional Notes (legacy code) for discrete time modelling of the converter

%% Control-to-output-voltage transfer function (Discrete time domain):
% Literature : Applying Digital Technology to PWM Control-Loop Designs
% Discrete time modelling (Trailing Edge DPWM and sampling at interval 2(DTs < td < Ts)):
%{
    phi   = (exp(A2*(Ts-td)))*(exp(A1*Duty*Ts))*exp(A2*(td-Duty*Ts));
    alpha = (B1 - B2)*Vg;
    gamma = (exp(A2*(Ts-td)))*alpha*Ts;
    z     = tf('z');
    Gvd   = (C* inv(I-((1/z)*phi))*gamma)/z
%}

%% Literature (Small-Signal Discrete-Time Modeling of Digitally Controlled PWM Converters)
% Direct equation:
%{
    z      = tf('z');
    Num    = z + (Ts/(Ts- td + Cap*Resr))*((Resr/R)-((Cap*Resr)/Ts)- (((Ts-td)*Resr)/L)+(td/Ts));
    Dem    = z^2 - (2 - (Ts/(R*Cap)))*z + (1 - Ts/(R*Cap) + Ts^2/(L*Cap));
    Gvd2   = ((Vg*Ts*(Ts- td + Cap*Resr))*Num)/(L*Cap*Dem)
%}

pause
