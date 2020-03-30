
source plant_model.m

%% Bode plot

% The Octave bode function returns omega (rad/s), so we need to convert to frequency

function plot_model_plot(plant_1, plant_2, title_str = "Bode plot")


[mag1, pha1, w1] = bode(plant_1);
f1 = w1 ./ (2*pi);

[mag2, pha2, w2] = bode(plant_2);
f2 = w2 ./ (2*pi);

% Magnitude

figure()
subplot (2, 1, 1)
hold on
box on
semilogx (f1/1000, mag2db(mag1))
semilogx (f2/1000, mag2db(mag2))
hold off
legend(plant_1.name, plant_2.name)
axis ("tight")
ylim ([-50, 20])
xlim ([0.1, 300])
set (gca, "xticklabel", num2str (get (gca, "xtick"), '%g|'))
grid ("on")
title (title_str)
ylabel ("Magnitude (dB)")
set (gca, 'fontsize', 14);
h = legend ("location", "southwest");
set (h, "fontsize", 14);

% Phase

subplot (2, 1, 2)
hold on
box on
semilogx (f1/1000, pha1)
semilogx (f2/1000, pha2)
hold off
yticks ([45 0 -45 -90 -135 -180 -225 -270])
ylim ([-270, 45])
xlim ([0.1, 300])
set (gca, "xticklabel", num2str (get (gca, "xtick"), '%g|'))
grid ("on")
xlabel ("Frequency (kHz)")
ylabel ("Phase (deg)")
set (gca, 'fontsize', 14);

endfunction


% Step response of plant

%{
[y1, t1, x1] = step(G_plant);
[y2, t2, x2] = step(G_plant_div);

figure()
hold on;
plot(t1, y1, 'r:');
plot(t2, y2, 'k--');
hold off;
title('Comparison second order and third order model')
legend("Third order model", "Second order model")
%}

%{

figure(3);
step(G_plant_div);
title('Step Response of the Plant Plus Divider');
legend("Plant Plus Divider")
#[Gm_G_plant_div,Pm_G_plant_div,Wcg_G_plant_div,Wcp_G_plant_div] = margin(G_plant_div);




figure();
step(plant);
title('Step Response of the Plant only');


% Comparison of step responses

figure()
plot(t1,y1, 'r:');
hold on;
plot(t3,y3, 'k--');
hold off;
title('Comparison second order and third order model')
legend("Third order model", "Second order model")

%}

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
    bode(plant_plus_divider)
    title("Actual bode plot for third order model")
    figure(11)
    bode(second_order)
    title("Actual bode plot for second order model")
%}

pause
