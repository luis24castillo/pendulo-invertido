%% Datos de los ensayos en el prototipo

datospert   = readtable('datos_pert.xlsx');
datosss     = readtable('datos_ss.xlsx');
datoslima   = readtable('datos_lima.xlsx');
datosinclin = readtable('datos_inclin.xlsx');
datossen    = readtable('datos_sen.xlsx');

theta_offset = -4;
omega_w_offset = -276;

% % Test en estado estacionario
% time    = datosss.time;
% phi     = (datosss.phi/pi)*180;
% omega   = datosss.omega_c;
% omega_w = datosss.omega_w*30/pi;
% torque  = datosss.torque*1000;

% Test con perturbaciones en ambos lados
% time    = datospert.time;
% ref     = (datospert.ref/pi)*180*0;
% theta   = (datospert.phi/pi)*180;
% omega   = datospert.omega_c;
% omega_w = datospert.omega_w*30/pi;
% torque  = datospert.torque*1000;

% % Test con cambio de centro de masa (la lima)
% time    = datoslima.time;
% ref     = (datoslima.ref/pi)*180;
% theta   = (datoslima.phi/pi)*180;
% omega   = datoslima.omega_c;
% omega_w = datoslima.omega_w*30/pi;
% torque  = datoslima.torque*1000;

% % Test de plano inclinado
% time    = datosinclin.time;
% ref     = (datosinclin.ref/pi)*180*0;
% theta   = (datosinclin.phi/pi)*180;
% omega   = datosinclin.omega_c;
% omega_w = datosinclin.omega_w*30/pi;
% torque  = datosinclin.torque*1000;

% Test de seguimiento de referencia senoidal
time    = datossen.time;
ref     = (datossen.ref/pi)*180;
theta   = (datossen.phi/pi)*180+theta_offset; %
omega   = datossen.omega_c;
omega_w = datossen.omega_w*30/pi+omega_w_offset;
torque  = datossen.torque*1000;


% Create a 2x2 layout
figure (2);
t2 = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'tight');

% primer plot
ax1=nexttile;
plot(time, ref, 'r', 'LineWidth', 1.0);
hold on;
plot(time, theta, 'b', 'LineWidth', 1.0);
ylabel(ax1, 'Ángulo (grados)');
ax1.YAxis.FontSize = 11;
ax1.XAxis.FontSize = 11;
ax1.XTickLabel = [];
legend('referencia ($\theta_{ref}$)', '\''angulo del cubo ($\theta_{b}$)', 'Interpreter', 'latex', 'FontSize', 12, 'Location', 'best');
grid on;
hold off;

% segundo plot
ax2=nexttile;
plot(time, omega, 'b', 'LineWidth', 1.0);
ylabel(ax2, 'Velocidad del cubo (rad/s)');
ax2.YAxis.FontSize = 11;
ax2.XAxis.FontSize = 11;
ax2.XTickLabel = [];
legend('\boldmath{$\dot{\theta}_b$}', 'Interpreter', 'latex', 'FontSize', 12, 'Location', 'best');
grid on;

% tercer plot
ax3=nexttile;
plot(time, omega_w, 'k', 'LineWidth', 1.0);
ylabel(ax3, 'Velocidad de la rueda (rpm)');
ax3.YAxis.FontSize = 11;
ax3.XAxis.FontSize = 11;
ax3.XTickLabel = [];
legend('\boldmath{$\dot{\theta}_w$}', 'Interpreter', 'latex', 'FontSize', 12, 'Location', 'best');
grid on;

% cuarto plot
ax4=nexttile;
plot(time, torque, 'm', 'LineWidth', 1.0);
ylabel(ax4, 'Par del motor (mNm)');
xlabel(ax4, 'Tiempo (s)');
ax4.YAxis.FontSize = 11;
ax4.XAxis.FontSize = 11;
legend('\boldmath{$\tau_m$}', 'Interpreter', 'latex', 'FontSize', 12, 'Location', 'best');
grid on;


