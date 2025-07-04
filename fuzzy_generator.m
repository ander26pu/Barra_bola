%--------------------------------------------------------------------------
% CONTROLADOR DIFUSO OPTIMIZADO PARA PLANTA NO LINEAL (BARRA-BOLA)
%--------------------------------------------------------------------------
% Este script diseña un controlador difuso optimizado con:
%  1. Estructura PD difusa + acción integral externa
%  2. Optimización de ganancias (Ke, Kd, Ku, Ki)
%  3. Mecanismo anti-windup mejorado
%  4. Selección automática de mejor conjunto de parámetros
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
Ts = 0.01;                  % Tiempo de muestreo
T_final = 500;               % Tiempo total de simulación [s]
t = (0:Ts:T_final)';        % Vector tiempo
U_MIN = 68; U_MAX = 80;     % Límites de señal de control [°]
SP = 200;                   % Setpoint [mm]

%% DISEÑO DE LA ESTRUCTURA DIFUSA
fis = mamfis('Name', 'fuzzy_pid');

% Variables de entrada/salida
fis = addInput(fis, [-1 1], 'Name', 'e_norm');      % Error normalizado
fis = addInput(fis, [-1 1], 'Name', 'de_norm');     % Derivada del error normalizada
fis = addOutput(fis, [-1 1], 'Name', 'du_norm');    % Incremento de control normalizado

% Funciones de membresía - Triangular para mejor comportamiento lineal
names = {'NB', 'NS', 'ZE', 'PS', 'PB'};
centers_e = [-1, -0.5, 0, 0.5, 1];
centers_de = [-1, -0.5, 0, 0.5, 1];
centers_du = [-1, -0.5, 0, 0.5, 1];

for i = 1:length(names)
    fis = addMF(fis, 'e_norm', 'trimf', [centers_e(i)-0.5, centers_e(i), centers_e(i)+0.5], 'Name', names{i});
    fis = addMF(fis, 'de_norm', 'trimf', [centers_de(i)-0.5, centers_de(i), centers_de(i)+0.5], 'Name', names{i});
    fis = addMF(fis, 'du_norm', 'trimf', [centers_du(i)-0.5, centers_du(i), centers_du(i)+0.5], 'Name', names{i});
end

% Base de reglas (25 reglas)
ruleList = [
    1 1 5 1 1;   % Si e=NB y de=NB -> du=PB
    1 2 5 1 1;
    1 3 4 1 1;
    1 4 3 1 1;
    1 5 2 1 1;
    
    2 1 5 1 1;
    2 2 4 1 1;
    2 3 3 1 1;
    2 4 2 1 1;
    2 5 1 1 1;
    
    3 1 4 1 1;
    3 2 3 1 1;
    3 3 3 1 1;
    3 4 2 1 1;
    3 5 1 1 1;
    
    4 1 3 1 1;
    4 2 2 1 1;
    4 3 2 1 1;
    4 4 1 1 1;
    4 5 1 1 1;
    
    5 1 2 1 1;
    5 2 1 1 1;
    5 3 1 1 1;
    5 4 1 1 1;
    5 5 1 1 1;
];
fis = addRule(fis, ruleList);

% Visualización de reglas
figure('Name', 'Reglas Difusas', 'Color', 'white');
plotfis(fis);

%% OPTIMIZACIÓN DE GANANCIAS
% Función objetivo
eval_metric = @(params) objective_fuzzy(params, fis, planta, t, SP, v_op, u_op, Ts, U_MIN, U_MAX);

% Parámetros a optimizar: [Ke, Kd, Ku, Ki]
% Ke = Ganancia error, Kd = Ganancia derivada
% Ku = Ganancia salida difusa, Ki = Ganancia integral

% Conjuntos iniciales de parámetros
init_params = [
    0.01, 0.1, 1.0, 0.001;    % Valores conservadores
    0.02, 0.05, 2.0, 0.005;   % Mayor ganancia
    0.005, 0.2, 0.5, 0.002;   % Menor ganancia, mayor derivada
    0.03, 0.01, 1.5, 0.01     % Mayor acción integral
];

% Optimización
best_cost = inf;
best_params = [];
options_fmin = optimset('Display', 'iter', 'TolX', 1e-4, 'TolFun', 1e-3, 'MaxIter', 30);

for i = 1:size(init_params, 1)
    fprintf('\nOptimizando conjunto inicial %d/%d...\n', i, size(init_params, 1));
    [params_opt, cost] = fminsearch(eval_metric, init_params(i, :), options_fmin);
    
    if cost < best_cost
        best_cost = cost;
        best_params = params_opt;
        fprintf('Nuevo mejor encontrado: Costo=%.2f\n', best_cost);
    end
end

Ke = best_params(1);
Kd = best_params(2);
Ku = best_params(3);
Ki = best_params(4);

fprintf('\nMejores ganancias optimizadas:\n');
fprintf('Ke = %.4f, Kd = %.4f, Ku = %.4f, Ki = %.4f\n', Ke, Kd, Ku, Ki);
fprintf('Costo IAE = %.2f\n', best_cost);

%% SIMULACIÓN FINAL CON PARÁMETROS OPTIMIZADOS
sys_d = c2d(ss(planta), Ts, 'zoh');
A = sys_d.A; B = sys_d.B; C = sys_d.C; D = sys_d.D;

y = v_op * ones(size(t));     % Salida del sistema
u = u_op * ones(size(t));     % Señal de control
e = zeros(size(t));           % Error
de = zeros(size(t));          % Derivada del error
ie = 0;                       % Integral del error
prev_err = 0;                 % Error anterior
x = zeros(size(A,1),1);       % Estado del sistema

for k = 2:length(t)
    % Cálculo de errores
    e(k) = SP - y(k-1);
    de(k) = (e(k) - prev_err) / Ts;
    
    % Acción difusa
    e_norm = Ke * e(k);
    de_norm = Kd * de(k);
    e_norm = min(max(e_norm, -1), 1);   % Saturación
    de_norm = min(max(de_norm, -1), 1); % Saturación
    
    du_norm = evalfis(fis, [e_norm, de_norm]);
    u_fuzzy = Ku * du_norm;
    
    % Acción integral con anti-windup
    ie = ie + Ki * e(k) * Ts;
    
    % Señal de control total
    u_total = u_fuzzy + ie;
    u(k) = u_op + u_total;
    
    % Anti-windup condicional
    if (u(k) >= U_MAX && e(k) > 0) || (u(k) <= U_MIN && e(k) < 0)
        ie = ie - Ki * e(k) * Ts;  % Retroceder la integral
    end
    
    % Saturación
    u(k) = min(max(u(k), U_MIN), U_MAX);
    
    % Aplicar a la planta
    u_plant = u(k) - u_op;
    x = A*x + B*u_plant;
    y(k) = v_op + C*x + D*u_plant;
    prev_err = e(k);
end

%% GRÁFICOS DE RESULTADOS
figure('Name', 'Control Difuso Optimizado', 'Color', 'white', 'Position', [100 100 800 600]);

% Respuesta del sistema
subplot(3,1,1);
plot(t, y, 'LineWidth', 1.5);
hold on; 
yline(SP, 'r--', 'LineWidth', 1.5);
grid on; 
title('Respuesta del Sistema'); 
xlabel('Tiempo [s]'); 
ylabel('Posición [mm]');
legend('Salida', 'Setpoint', 'Location', 'best');
ylim([min(y)-5, max(y)+5]);

% Señal de control
subplot(3,1,2);
plot(t, u, 'LineWidth', 1.5);
hold on; 
yline([U_MIN, U_MAX], '--', 'LineWidth', 1);
grid on; 
title('Señal de Control'); 
xlabel('Tiempo [s]'); 
ylabel('Ángulo [°]');
ylim([U_MIN-2, U_MAX+2]);

% Error
subplot(3,1,3);
plot(t, e, 'LineWidth', 1.5);
grid on; 
title('Error de Seguimiento'); 
xlabel('Tiempo [s]'); 
ylabel('Error [mm]');

%% ANÁLISIS DE DESEMPEÑO
% Cálculo de métricas
IAE = trapz(t, abs(e));          % Integral del error absoluto
ISE = trapz(t, e.^2);            % Integral del error cuadrático
ITAE = trapz(t, t.*abs(e));      % Integral del tiempo por error absoluto

% Mostrar métricas
fprintf('\nMétricas de desempeño:\n');
fprintf('IAE: %.2f\n', IAE);
fprintf('ISE: %.2f\n', ISE);
fprintf('ITAE: %.2f\n', ITAE);

% Guardar controlador
save('fuzzy_controller_opt.mat', 'fis', 'Ke', 'Kd', 'Ku', 'Ki', 'u_op', 'v_op');
writeFIS(fis, 'fuzzy_controller_opt'); 
disp('Controlador difuso optimizado guardado en "fuzzy_controller_opt.fis"');

%% FUNCIONES AUXILIARES
function J = objective_fuzzy(params, fis, planta, t, SP, v_op, u_op, Ts, umin, umax)
    % Parámetros
    Ke = params(1);
    Kd = params(2);
    Ku = params(3);
    Ki = params(4);
    
    % Discretizar planta
    sys_d = c2d(ss(planta), Ts, 'zoh');
    A = sys_d.A; B = sys_d.B; C = sys_d.C; D = sys_d.D;
    
    % Variables de simulación
    y = v_op * ones(size(t));     % Salida
    u = u_op * ones(size(t));     % Control
    e = zeros(size(t));           % Error
    de = zeros(size(t));          % Derivada del error
    ie = 0;                      % Integral del error
    prev_err = 0;                % Error anterior
    x = zeros(size(A,1),1);      % Estado
    
    for k = 2:length(t)-1
        % Cálculo de errores
        e(k) = SP - y(k-1);
        de(k) = (e(k) - prev_err) / Ts;
        
        % Normalización con saturación
        e_norm = Ke * e(k);
        de_norm = Kd * de(k);
        e_norm = min(max(e_norm, -1), 1);
        de_norm = min(max(de_norm, -1), 1);
        
        % Evaluación difusa
        du_norm = evalfis(fis, [e_norm, de_norm]);
        u_fuzzy = Ku * du_norm;
        
        % Acción integral
        ie = ie + Ki * e(k) * Ts;
        u_total = u_fuzzy + ie;
        u(k) = u_op + u_total;
        
        % Anti-windup condicional
        if (u(k) >= umax && e(k) > 0) || (u(k) <= umin && e(k) < 0)
            ie = ie - Ki * e(k) * Ts;
        end
        
        % Saturación
        u(k) = min(max(u(k), umin), umax);
        
        % Aplicar a la planta
        u_plant = u(k) - u_op;
        x = A*x + B*u_plant;
        y(k) = v_op + C*x + D*u_plant;
        prev_err = e(k);
    end
    
    % Cálculo del costo (IAE + penalización por sobrepico)
    J = trapz(t, abs(SP - y));
    
    % Penalización por sobrepico excesivo (>10%)
    overshoot = max(0, (max(y) - SP)/SP * 100 - 10);
    J = J + overshoot * 10;
end