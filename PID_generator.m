%--------------------------------------------------------------------------
% OPTIMIZACIÓN AUTOMÁTICA DE PID PARA SISTEMA BARRA-BOLA
%--------------------------------------------------------------------------
% Este script busca el mejor controlador PID para el sistema
% identificado "sys_con_retardo_aprox" mediante:
%  1) Múltiples métodos de diseño inicial (pidtune con distintos focos válidos)
%  2) Refinamiento numérico con fminsearch minimizando el IAE
%  3) Simulación y selección del PID que minimiza el error acumulado
%  4) Guardado del sistema en lazo cerrado discreto para uso en otros scripts
%--------------------------------------------------------------------------
clear; clc; close all;

%% CARGA DE MODELO
try
    load('modelo_identificado.mat');  % Debe contener sys_con_retardo_aprox, u_op, v_op
catch
    error('No se encontró "modelo_identificado.mat"');
end
planta = sys_con_retardo_aprox;

%% PARÁMETROS DE SIMULACIÓN
Ts       = 0.01;             % Tiempo de muestreo
T_final  = 50;               % Tiempo total de simulación [s]
t        = (0:Ts:T_final)';  % Vector tiempo
dt       = Ts;
U_MIN    = 68; U_MAX = 80;   % Límites de señal de control [°]
SP       = 220;              % Setpoint [mm]

%% MÉTODOS INICIALES CON pidtune
focos = {'reference-tracking', 'disturbance-rejection', 'balanced'};
init_set = [];
for i = 1:length(focos)
    opts = pidtuneOptions('DesignFocus', focos{i}, 'PhaseMargin', 60);
    Ctmp = pidtune(planta, 'PIDF', opts);
    init_set = [init_set; Ctmp.Kp, Ctmp.Ki, Ctmp.Kd];
end

%% REFINAMIENTO CON fminsearch
eval_metric = @(params) objective_pid(params, planta, t, SP, v_op, u_op, Ts, U_MIN, U_MAX);
best_cost = inf;
best_params = [];
options_fmin = optimset('Display', 'iter', 'TolX', 1e-3, 'TolFun', 1e-2);

for idx = 1:size(init_set, 1)
    x0 = init_set(idx, :);  % [Kp, Ki, Kd]
    [x_opt, cost] = fminsearch(eval_metric, x0, options_fmin);
    if cost < best_cost
        best_cost = cost;
        best_params = x_opt;
    end
end

Kp = best_params(1);
Ki = best_params(2);
Kd = best_params(3);

fprintf('\nMejor PID encontrado: Kp=%.3f, Ki=%.3f, Kd=%.3f (Costo IAE=%.2f)\n', Kp, Ki, Kd, best_cost);

%% SIMULACIÓN FINAL CON MEJOR PID
y = simulate_pid(planta, [Kp, Ki, Kd], t, SP, v_op, u_op, Ts, U_MIN, U_MAX);

figure('Name', 'PID Optimizado', 'Color', 'white');
plot(t, y, 'LineWidth', 2);
hold on; yline(SP, 'r--', 'LineWidth', 1.5);
grid on;
title('Respuesta con PID Optimizado'); xlabel('Tiempo [s]'); ylabel('Posición [mm]');
legend('Salida', 'Setpoint', 'Location', 'best');

%% CREACIÓN Y GUARDADO DEL SISTEMA CONTROLADO DISCRETO
% Discretizar la planta
sys_d = c2d(ss(planta), Ts, 'zoh');
% Crear controlador PID continuo y discretizarlo
C_pid = pid(Kp, Ki, Kd);
C_d   = c2d(C_pid, Ts, 'tustin');
% Construir lazo cerrado discreto
sistema_controlado = feedback(C_d * sys_d, 1);
% Guardar en .mat
save('sistema_controlado.mat', 'sistema_controlado', 'planta', ...
     'Ts', 'Kp', 'Ki', 'Kd', 'u_op', 'v_op', 'U_MIN', 'U_MAX');
disp('Archivo "sistema_controlado.mat" creado con todas las variables necesarias.');


%% FUNCIONES AUXILIARES
function J = objective_pid(params, planta, t, SP, v_op, u_op, Ts, umin, umax)
    y = simulate_pid(planta, params, t, SP, v_op, u_op, Ts, umin, umax);
    J = trapz(t, abs(y - SP));
end

function y = simulate_pid(planta, params, t, SP, v_op, u_op, Ts, umin, umax)
    Kp = params(1); Ki = params(2); Kd = params(3);
    sys_d = c2d(ss(planta), Ts, 'zoh');
    A = sys_d.A; B = sys_d.B; Cc = sys_d.C; D = sys_d.D;
    x = zeros(size(A,1),1);
    y = v_op * ones(size(t));
    integral = 0; prev_err = 0; deriv_state = 0;
    Tf = 0.05;  % filtro derivativo
    for k = 1:length(t)-1
        err = SP - y(k);
        a = Tf/(Tf+Ts); b = Kd/(Tf+Ts);
        deriv = a*deriv_state + b*(err - prev_err);
        deriv_state = deriv;
        u_pid = Kp*err + Ki*integral + deriv;
        u = min(max(u_op + u_pid, umin), umax);
        if Ki ~= 0
            windup = (u - (u_op + u_pid)) / Ki;
        else
            windup = 0;
        end
        integral = integral + (err + windup)*Ts;
        u_plant = u - u_op;
        x = A*x + B*u_plant;
        y(k+1) = v_op + Cc*x + D*u_plant;
        prev_err = err;
    end
end
