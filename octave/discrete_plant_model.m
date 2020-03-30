%% Control-to-output-voltage transfer function (Discrete time domain):

pkg load control;
pkg load signal;

% See http://www.ti.com/download/trng/docs/seminar/Topic_7_Hagen.pdf

source config_ti.m
source plant_model_plot.m %allow for use of     function plot_model_plot(plant_1, plant_2)
source plant_model.m %to allow plotting of continuous time model

% As in continious model, define the values for A,B,C,D.
%   1: on state
%   2: off state

Rpar = (Rload * Rls) / (Rload + Rls);
p1 = -(1 / ((Rload + Rls) * Cls));
p2 =  (Rload / ((Rload + Rls) * Cls));
p3 = -(Rload / ((Rload + Rls) * L));
p4 = -((Rind + Rpar) / L);
p5 = (Rload / (Rload + Rls));
A1 = [p1 p2; p3 p4];        % State matrix A.
A2 = A1;
B1 = [0; (1 / L)];            % Input-to-state matrix B
B2 = [0];
C1 = [p5, Rpar];            % State-to-output matrix C
C2 = C1;
D1 = 0;                     % Feedthrough matrix D
D2 = D1;


% Literature : Applying Digital Technology to PWM Control-Loop Designs
    % Discrete time modelling (Trailing Edge DPWM and sampling at interval 2(DTs < td < Ts)):

%Buck converter model

sampling_time = Ts/10

         % phi   = (exp(A2*(Ts-td)))*(exp(A1*D*Ts))*exp(A2*(td-D *Ts));
         % alpha = (B1 - B2)*Vhs;  %Needs to change to be (A1-A2)*X_p + (B1 - B2)*Vhs; where X_p = [V_c I_l]^T
         % gamma = (exp(A2*(Ts-td)))*alpha*Ts;
          z     = tf('z',sampling_time);
         % Gvd   = (Cls* inv(I-((1/z)*phi))*gamma)/z


% Literature (Small-Signal Discrete-Time Modeling of Digitally Controlled PWM Converters)
    % Direct equation:


     Num    = z + (Ts/(Ts- td + Cls*Rls))*((Rls/Rload)-((Cls*Rls)/Ts)- (((Ts-td)*Rls)/L)+(td/Ts));
     Dem    = z^2 - (2 - (Ts/(Rload*Cls)))*z + (1 - Ts/(Rload*Cls) + Ts^2/(L*Cls));
     Gvd2   = ((Vhs*Ts*(Ts- td + Cls*Rls))*Num)/(L*Cls*Dem)
     Gvd2.name = "Discrete Time Model - Direct"


plot_model_plot(Gvd2, G_plant, "Discrete vs Cont.")

%% Divider circuitry:
% Voltage is divided to ADC full scale by divider circuit:

     R1 = 100e3;
     R2 = 5.6e3;
     Cz = 1e-6;
     Rp = (R1*R2)/(R1+R2);
     Cp = 1/(2*pi*fs*Rp);

% Gain of the Divider:

     Kdiv = R2/(R1+R2);

% Transfer function of the Divider circuit:

     G1 = tf([R1*Cz 1],[Kdiv*R1*(Cz+Cp) 1]);     % continuous time domain (S Domain)
     G2 = G1*Kdiv;

     Gdiv = c2d(G2,Ts,'tustin')                  % Discrete time domain (Z Domain)
