% Plant transfer function (continous time domain)

pkg load control;
pkg load signal;

% See http://www.ti.com/download/trng/docs/seminar/Topic_7_Hagen.pdf

source config_ti.m

%% Plant state space model

%   x  = [vC, iL]           state vector
%   u  = Vg                 input vector
%   x' = A*x + B*u          derivative of state vector
%   y  = C*x + D*u          output vector

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

A_avg = A1 * D + A2 * (1 - D);
B_avg = B1 * D + B2 * (1 - D);
C_avg = C1 * D + C2 * (1 - D);
D_avg = D1 * D + D2 * (1 - D);

plant = ss(A_avg, B_avg, C_avg, D_avg);

#[n_plant, d_plant] = ss2tf(A1, B1, C1, D1);
[n_plant, d_plant] = ss2tf(A_avg, B_avg, C_avg, D_avg);

G_plant = tf(n_plant, d_plant);

%% ADC delay

% Exponentials are not handled by Octave control package to include a delay
% Using the Pade approximation for delay seems to be the best work around
% Pade (Octave): https://octave.sourceforge.io/octave/function/padecoef.html
% Pade (Wiki): https://en.wikipedia.org/wiki/Pad%C3%A9_table

% TO BE IMPLEMENTED


% Calculation of Bandwidth of the Plant:

% Stepinfo function now available based on funtion forum input here: https://lists.gnu.org/archive/html/help-octave/2015-02/msg00023.html
% Function is not verified as working.
%plant_info = stepinfo(plant);
%[Wn_plant,DR_plant,Poles_plant] = damp(plant);

% BEWARE 0 Stepinfo function is providing incorrect information.
%    temp1 = sqrt((4*(DR_plant(1))^4) - (4*(DR_plant(1))^2) +2);
%    temp2 =  1 - (2*(DR_plant(1))^2);
%
%    %%Depends on plant info - using current stepinfo command (not tested)
%    temp3 = 4/(plant_info.SettlingTime*(DR_plant(1)));
%    BW_plant = temp3*sqrt(temp1 + temp2)

% plotting Bode plot for Plant:

    %% Use bode (Octave) vs bodeplot (Matlab) , similar functionality
%    figure(2)
%    bode(plant);
%    title('Bode diagram of the Plant only (continous model)');
%    [Gm_Plant,Pm_Plant,Wcg_Plant,Wcp_Plant] = margin(plant);


%% Divider circuitry

% Documentation here: https://learn.libre.solar/b/dc-control/development/digital_control.html#modelling-the-digital-controller
% Voltage is divided to ADC full scale by divider circuit:

R1 = 100e3;
R2 = 5.6e3;
Cz = 0;         % Previously: Cz = 1e-6;
% Typically not populated as this is not a programmable compensation zero
% Hardware currently does not have Cz.
Rp = (R1 * R2) / (R1 + R2);
Cp = 1 / (2*pi*fs*Rp);      % good attenuation according to paper
Cp = 10e-9;                 % overwrite with actual value in hardware (10nF)

% Gain of the divider

Kdiv = R2 / (R1 + R2);
Vsense = Vls * Kdiv; % The sensed voltage in for control loop. Max voltage is 3.3V.

% Transfer function of the divider circuit:

G_div = tf([R1*Cz 1], [Kdiv*R1*(Cz+Cp) 1]);     % continuous time domain (S domain)

% Plant + divider transfer function

G_plant_div = G_plant * G_div;


#mag_db = cellfun (@mag2db, mag, "uniformoutput", false);

#mag_args = horzcat (cellfun (@horzcat, f, mag_db, sty, "uniformoutput", false){:});
#pha_args = horzcat (cellfun (@horzcat, f, pha, sty, "uniformoutput", false){:});

% Check system controlability:

%TO BE IMPLEMENTED
%    [A B C D] = tf2ss(G_plant_div)
%    rank_system = rank(ctrb(A,B))   %Ensure


% Check G_plant_div poles and zeros to simplify the model
% Dominant pole approximation to simplify model
% https://lpsa.swarthmore.edu/PZXferStepBode/DomPole.html

[find_zeros, poles, k] = tf2zp(G_plant_div);


%Use output to determine a simpler model
% Since the second and third poles are dominant, the first pole at s = 188857 can be ignored.

% New second order model form: K/(A*s^2 + B*s + C)

K = 1902 * 29036;   %Set gain to match the previous transfer function
A = 1;
B = 2*1060.94289;
C =  7356.99698^2;

second_order = tf([K],[A B C]);

%Assess new second order model vs third order

[y3, t3, x3] = step(second_order); % Get info for multiple plotting on the same axis
[mag3, pha3, w3] = bode(second_order); % Get info for multiple plotting on the same axis

%Plot comparison of step responses

%    figure(5)
%    plot(t1,y1, 'r:');
%    hold on;
%    plot(t3,y3, 'k--');
%    hold off;
%    title('Comparison second order and third order model')
%    legend("Third order model", "Second order model")

%% Multiple bode plots

%% Comment to save computation time

%    figure(6)
%    grid on;
%
%    subplot(2,1,1)
%    hold on;
%    semilogx(w1,20*log10(abs(mag1)));
%    semilogx(w3,20*log10(abs(mag3)));
%    hold off;
%    legend("Third order model", "Second order model");
%    title("Constructed magnitude plot for comparison of second and third order models");
%
%    subplot(2,1,2)
%    hold on;
%    semilogx(w1, pha1);
%    semilogx(w3, pha3);
%    hold off;
%    legend("Third order model", "Second order model");
%    title("Constructed phase plot for comparison of second and third order models");

% Sense check that bode plots are plotting correctly.

%{
    figure(10)
    bode(G_plant_div)
    title("Actual bode plot for third order model")
    figure(11)
    bode(second_order)
    title("Actual bode plot for second order model")
%}