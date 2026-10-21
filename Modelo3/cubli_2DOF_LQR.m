% Ganancias de bloques
rad2rpm = 60/(2*pi);

% Constantes
lw = 0.106;         % distancia del pivote al CM de la rueda (m)
lb = 0.112;         % distancia del pivote al CM del cuerpo del pendulo (m)
mb = 0.714;         % masa del chasis o cuerpo del pendulo (kg)
mw = 0.217;         % masa de la rueda (kg)
Ib = 0.01154;       % momento de inercia del cuerpo del pendulo respecto al pivote (kg-m^2)
Iw = 4.85*10^-4;    % momento de inercia de la rueda respecto al eje del motor (kg-m^2)
Cb = 0;             % friccion viscosa en el pivote (Nms/rad)
Cw = 1.5*10^-5;     % friccion viscosa del eje del motor (Nms/rad)
g = 9.81;           % acel. de la gravedad (m/s^2)
Km = 24.7*10^-3;    % constante de par del motor (Nm/A)
thau_max = 0.604;   % par maximo del motor (Nm)

% Matrices del sistema
A= [            0                  ,        1         ,    0   ,                  0                         ;
    (g*(mb*lb+mw*lw))/(Ib+mw*lw^2) , -Cb/(Ib+mw*lw^2) ,    0   ,            Cw/(Ib+mw*lw^2)                 ;
                0                  ,        0         ,    0   ,                   1                        ; 
   -(g*(mb*lb+mw*lw))/(Ib+mw*lw^2) , Cb/(Ib+mw*lw^2)  ,    0   ,  -(Cw*(Ib+Iw+mw*lw^2))/(Iw*(Ib+mw*lw^2))  ];

B= [                0                    ;
            -Km/(Ib+mw*lw^2)             ;
                    0                    ;
    Km*(Ib+Iw+mw*lw^2)/(Iw*(Ib+mw*lw^2)) ];

C= eye(4);
D=zeros(4,1);


% Matrices de ponderacion
Q=diag([132 1 0.01 0.01]);  % penalizacion al error
R=0.25;                     % penalizacion al esfuerzo

[K, S, E] = lqr(A, B, Q, R);

%% CONTROLABILIDAD

% Calcular la matriz de controlabilidad
Co = ctrb(A, B);

% Obtener el número de estados (tamaño de A)
n = length(A); 

% Calcular el rango de la matriz de controlabilidad
rango_Co = rank(Co);

% Mostrar los resultados
disp('Matriz de Controlabilidad (Co):');
disp(Co);

fprintf('Número de estados (n): %d\n', n);
fprintf('Rango de la matriz de controlabilidad: %d\n\n', rango_Co);

%% OBSERVABILIDAD

% Calcular la matriz de controlabilidad
Ob = obsv(A, C);

% Obtener el número de estados (tamaño de A)
n = length(A); 

% Calcular el rango de la matriz de controlabilidad
rango_Ob = rank(Ob);

% Mostrar los resultados
disp('Matriz de Observabilidad (Ob):');
disp(Ob);

fprintf('Número de estados (n): %d\n', n);
fprintf('Rango de la matriz de observabilidad: %d\n\n', rango_Ob);

% Tiempo de entrada señal senoidal
switch_time = 10;


%% GRAFICAS

% Crea layout 2x2
figure (1);
t = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'tight');

% primer plot
ax1=nexttile;
plot(out.sen_ref.time , (out.sen_ref.signals.values/pi)*180, 'r', 'LineWidth', 1.0); % multiplicar señales por 0 para referencia nula
hold on;
plot(out.theta_b.time, (out.theta_b.signals.values/pi)*180, 'b', 'LineWidth', 1.0);
ylabel(ax1, 'Ángulo (grados)' , 'FontSize', 11);
ax1.YAxis.FontSize = 11;
ax1.XTickLabel = [];
legend('referencia ($\theta_{ref}$)', '\''angulo del cubo ($\theta_{b}$)', 'Interpreter', 'latex', 'FontSize', 12, 'Location', 'best');
grid on;
hold off;

% segundo plot
ax2=nexttile;
plot(out.omega_b.time, out.omega_b.signals.values, 'b', 'LineWidth', 1.0);
ylabel(ax2, 'Velocidad del cubo (rad/s)', 'FontSize', 11);
ax2.YAxis.FontSize = 11;
ax2.XTickLabel = []
legend('\boldmath{$\dot{\theta}_b$}', 'Interpreter', 'latex', 'FontSize', 12, 'Location', 'best');
grid on;

% tercer plot
ax3=nexttile;
plot(out.omega_w.time, -out.omega_w.signals.values*rad2rpm, 'k', 'LineWidth', 1.0);
ylabel(ax3, 'Velocidad de la rueda (rpm)', 'FontSize', 11);
ax3.YAxis.FontSize = 11;
ax3.XTickLabel = []
legend('\boldmath{$\dot{\theta}_w$}', 'Interpreter', 'latex', 'FontSize', 12, 'Location', 'best');
grid on;

% cuarto plot
ax4=nexttile;
plot(out.torque.time, -out.torque.signals.values, 'm', 'LineWidth', 1.0);
ylabel(ax4, 'Par del motor (mNm)', 'FontSize', 11);
xlabel(ax4, 'Tiempo (s)', 'FontSize', 11);
ax4.YAxis.FontSize = 11;
ax4.XAxis.FontSize = 11;
legend('\boldmath{$\tau_m$}', 'Interpreter', 'latex', 'FontSize', 12, 'Location', 'best');
grid on;
