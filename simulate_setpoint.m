%--------------------------------------------------------------------------
% simulate_setpoint.m (CORREGIDO)
% Simulación de la planta controlada ante un único Set-Point
%--------------------------------------------------------------------------
clear; clc; close all;

%% 1) Cargar sistema y parámetros
try
    % Cargar el archivo .mat que ahora contiene todo lo necesario
    load('sistema_controlado.mat');
catch
    error('No se encontró "sistema_controlado.mat". Asegúrate de generarlo con el script de optimización modificado.');
end

%% 2) Definir Set-Point y tiempo de simulación
SP     = 210;       % Set-Point deseado [mm]
Tfinal = 50;        % Tiempo total de simulación [s] (usa un tiempo suficiente)
t      = (0:Ts:Tfinal)';

%% 3) Simular lazo cerrado con la función NO LINEAL
% Se usan los parámetros cargados (Kp, Ki, Kd) y la planta original.
% La función 'simulate_pid' debe estar al final de este script o en la ruta de MATLAB.
y_real = simulate_pid(planta, [Kp, Ki, Kd], t, SP, v_op, u_op, Ts, U_MIN, U_MAX);

%% 4) Graficar resultados
figure('Color','w');
plot(t, y_real, 'LineWidth', 2);
hold on;
yline(SP, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo [s]');
ylabel('Posición [mm]');
title(sprintf('Respuesta NO LINEAL al escalón de Set-Point = %d mm', SP));
legend('Salida Real', 'Set-Point', 'Location', 'Best');

%% 5) Métricas de desempeño (opcional, pero ahora sobre datos realistas)
% Tiempo de asentamiento (banda del 2%)
y_ss_estimado = SP; % Asumimos que el controlador puede llegar al SP
banda_superior = y_ss_estimado * 1.02;
banda_inferior = y_ss_estimado * 0.98;

% Encuentra el último punto fuera de la banda
indice_fuera_banda = find(y_real > banda_superior | y_real < banda_inferior, 1, 'last');

if isempty(indice_fuera_banda)
    % Si nunca salió de la banda, el tiempo de asentamiento es casi 0
    t_settle = t(1);
else
    % El tiempo de asentamiento es el siguiente punto en el tiempo
    t_settle = t(indice_fuera_banda + 1);
end

fprintf('Tiempo de asentamiento (2%%): %.2f s\n', t_settle);


%% -------------------------------------------------------------------------
%  FUNCIÓN DE SIMULACIÓN (copiada del script original)
%  -------------------------------------------------------------------------
function y = simulate_pid(planta, params, t, SP, v_op, u_op, Ts, umin, umax)
    Kp = params(1); Ki = params(2); Kd = params(3);
    sys_d = c2d(ss(planta), Ts, 'zoh');
    A = sys_d.A; B = sys_d.B; Cc = sys_d.C; D = sys_d.D;
    
    x = zeros(size(A,1),1);
    y = v_op * ones(size(t));
    
    integral = 0;
    prev_err = 0;
    deriv_state = 0;
    Tf = 0.05;  % Filtro derivativo (debe ser el mismo que en la optimización)
    
    for k = 1:length(t)-1
        % Error
        err = SP - y(k);
        
        % Término integral con anti-windup
        integral = integral + err * Ts;
        
        % Término derivativo con filtro
        a = Tf/(Tf+Ts); 
        b = Kd/(Tf+Ts);
        deriv = a * deriv_state + b * (err - prev_err);
        deriv_state = deriv;
        
        % Salida del controlador PID (en desviación)
        u_pid = Kp*err + Ki*integral + deriv;
        
        % Señal de control total con saturación
        u_total = u_op + u_pid;
        u_saturada = min(max(u_total, umin), umax);
        
        % Anti-windup: se ajusta el integrador si hubo saturación
        % Este es un método de "back-calculation" simplificado
        if Ki ~= 0
            windup_error = u_saturada - u_total;
            integral = integral + (windup_error / Kp) * Ts; % Kt=1/Kp es una heurística común
        end
        
        % Simulación de la planta con la señal saturada
        u_plant_dev = u_saturada - u_op; % Entrada a la planta (en desviación)
        x = A*x + B*u_plant_dev;
        y(k+1) = v_op + Cc*x + D*u_plant_dev;
        
        % Actualizar estado para el siguiente ciclo
        prev_err = err;
    end
end