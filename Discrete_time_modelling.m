clc;
clear;
grid on;

%% Plant transfer function (Continous time domain):

% notations:
    % x' = A*x + B*Vg    
    % vout = C*x + D*Vg  
    % d    — control signal
    % vout — ouput of the plant
    % x    — state variable matrix


% example:

    % Vout = 1;      % outputof the power stage    
    % Vg = 10;        % input to power stage
    % fs = 500e3;      % switcing freq
    % Ts = 1/fs;      % switiching period
    % Cap = 800e-6;   % Cap in power plant
    % Resr = 2e-3;   % equivalent series resistance of cap
    % L = 1e-6;      % Ind in power plant  
    % RL = 8.6e-3;     % series resistance of inductor
    % R = 10;         % Load resistance in power plant
    % Duty = Vout/Vg;    % steady state Duty cycle
    % td = 0.25*Ts + Duty*Ts; % not sure why need to be checked 

    
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
    A1 = [p1 p2 ; p3 p4];       % A    — State matrix A. 
    A2 = [0];
    B1 = [0;(1/L)];             % B    — Input-to-state matrix B
    B2 = [0];
    C  = [p5, Rpar];            % C    — State-to-output matrix C
    D  = 0;                     % D    — Feedthrough matrix D
    I  = eye(2);
    plant = ss(A1,B1,C,D);
    [n_plant,d_plant] = ss2tf(A1,B1,C,D);
    plant_tf = tf(n_plant,d_plant)
     
% plotting Step Response of Plant:

    figure(1);
    subplot(2,1,1);
    step(plant); 
    title('Step Response of the Plant');
    plant_info = stepinfo(plant);
    [Wn_plant,DR_plant,Poles_plant] = damp(plant);


% plotting Bode plot for Plant:

    subplot(2,1,2);
    plant_plot = bodeplot(plant);
    setoptions(plant_plot,'FreqUnits','Hz','PhaseVisible','on');
    title('Bode diagram of the Plant(continous model)');
    [Gm_Plant,Pm_Plant,Wcg_Plant,Wcp_Plant] = margin(plant);
    
 % Calculation of Bandwidth of the Plant:

    temp1 = sqrt((4*(DR_plant(1))^4) - (4*(DR_plant(1))^2) +2);
    temp2 =  1 - (2*(DR_plant(1))^2);
    temp3 = 4/(plant_info.SettlingTime*(DR_plant(1)));
    BW_plant = temp3*sqrt(temp1 + temp2)
  
  
% Checking stability condition:

    % eig(A);
    % if(isstable(plant))
    %     disp('Plant is stable');
    % else
    %     disp('Plant is unstable');
    % end   
    
%% Direct conversion to Discrete time domain:
    
    dis = c2d(plant_tf,Ts);
    
% plotting Step Response of Direct discrete conversion:
    
    figure(2);
    subplot(2,1,1);
    step(dis);
    dis_info = stepinfo(dis);
    title('Step Response of the Plant(Direct discrete conversion)');
   
%plotting Bode plot for Plant(Direct discrete conversion);
    
    subplot(2,1,2);
    dis_plot = bodeplot(dis);
    setoptions(dis_plot,'FreqUnits','Hz','PhaseVisible','on');
    title('Bode diagram of the Plant(Direct discrete conversion)');
    
    
% Calculation of Bandwidth of the Plant:

    t1 = sqrt((4*(DR_plant(1))^4) - (4*(DR_plant(1))^2) +2);
    t2 =  1 - (2*(DR_plant(1))^2);
    t3 = 4/(dis_info.SettlingTime*(DR_plant(1)));
    BW_dis = t3*sqrt(t1 + t2)
        
%% Control-to-output-voltage transfer function (Discrete time domain):

% Literature : Applying Digital Technology to PWM Control-Loop Designs
    % Discrete time modelling (Trailing Edge DPWM and sampling at interval 2(DTs < td < Ts)):

        % phi   = (exp(A2*(Ts-td)))*(exp(A1*Duty*Ts))*exp(A2*(td-Duty*Ts)); 
        % alpha = (B1 - B2)*Vg; 
        % gamma = (exp(A2*(Ts-td)))*alpha*Ts;
        % z     = tf('z');
        % Gvd   = (C* inv(I-((1/z)*phi))*gamma)/z
     
  
%Literature(Small-Signal Discrete-Time Modeling of Digitally Controlled PWM Converters)
    % Direct equation: 
     
     z      = tf('z');
     Num    = z + (Ts/(Ts- td + Cap*Resr))*((Resr/R)-((Cap*Resr)/Ts)- (((Ts-td)*Resr)/L)+(td/Ts));
     Dem    = z^2 - (2 - (Ts/(R*Cap)))*z + (1 - Ts/(R*Cap) + Ts^2/(L*Cap));
     Gvd2   = ((Vg*Ts*(Ts- td + Cap*Resr))*Num)/(L*Cap*Dem)
     

% plotting Step Response of Plant(Gvd2):

    figure(3);
    %subplot(2,1,1);
    step(Gvd2)
    title('Step Response of the control-to-output-voltage transfer function');
    % Gvd_info = stepinfo(Gvd2);
    %[Wn_Gvd,DR_Gvd,Poles_Gvd] = damp(Gvd2);
    
% plotting Bode plot for Gvd2:

%     subplot(2,1,2);
%     Gvd2_plot = bodeplot(Gvd2);
%     %setoptions(Gvd2_plot,'FreqUnits','Hz','PhaseVisible','on');
%     title('Bode diagram of the control-to-output-voltage transfer function');
%     %[Gm_Gvd,Pm_Gvd,Wcg_Gvd,Wcp_Gvd] = margin(Gvd2);

%% Divider circuitry:
% Voltage is divided to ADC full scale by divider circuit:
    
    % R1 = 100e3; 
    % R2 = 5.6e3;
    % Cz = 1e-6;
    % Rp = (R1*R2)/(R1+R2);
    % Cp = 1/(2*pi*fs*Rp);

% Gain of the Divider:
 
    % Kdiv = R2/(R1+R2);
     
% Transfer function of the Divider circuit:

    % G1 = tf([R1*Cz 1],[Kdiv*R1*(Cz+Cp) 1]);     % continuous time domain (S Domain)
    % G2 = G1*Kdiv;
    % Gdiv = c2d(G2,Ts,'tustin')                  % Discrete time domain (Z Domain)

%% Multplication of Plant and Divider Transfer function to get overall G(s):

%  G = Gvd2 * Gdiv;

% figure(5);
% step(G);
 
%% PID controller:
% rootlocus method or PID tuner method (yet to decide):