% =========== FILE 1: run_fuzzy_control.m (VERSIÓN FINAL) ===========
clear; clc; close all;

%% 1. CARGA DE MODELO
try
    load('modelo_identificado.mat');
catch
    error('No se encontró "modelo_identificado.mat". Asegúrese de que esté en la misma carpeta.');
end

%% 2. CREACIÓN DEL CONTROLADOR DIFUSO (FIS)
% Esta sección está confirmada que funciona en tu sistema
fis = sugfis('Name', 'controlador_PD_difuso');
fis = addInput(fis, [-1 1], 'Name', 'e');
fis = addInput(fis, [-1 1], 'Name', 'de');
nombres_mf = {'NB', 'NS', 'Z', 'PS', 'PB'};
mf_puntos = [-1.3 -1 -0.5; -1 -0.5 0; -0.5 0 0.5; 0 0.5 1; 0.5 1 1.3];
for i = 1:5
    fis = addMF(fis, 'e', 'trimf', mf_puntos(i,:), 'Name', nombres_mf{i});
    fis = addMF(fis, 'de', 'trimf', mf_puntos(i,:), 'Name', nombres_mf{i});
end
fis = addOutput(fis, [-1 1], 'Name', 'u');
output_levels = [-1, -0.5, 0, 0.5, 1];
for i = 1:5
    fis = addMF(fis, 'u', 'constant', output_levels(i), 'Name', nombres_mf{i});
end
rule_matrix = [1 1 1 2 3; 1 2 2 3 4; 1 2 3 4 5; 2 3 4 5 5; 3 4 5 5 5];
[e_grid, de_grid] = meshgrid(1:5, 1:5);
rule_list = [e_grid(:), de_grid(:), rule_matrix.'(:), ones(25, 2)];
fis = addRule(fis, rule_list);
fprintf('Controlador FIS creado exitosamente.\n');

%% 3. PREPARACIÓN PARA LA OPTIMIZACIÓN
% --- CAMBIO PRINCIPAL: Agrupar parámetros en una 'struct' ---
data.planta = planta;
data.fis = fis;
data.t = (0:0.01:25)'; % Vector de tiempo
data.SP = 220;
data.u_op = u_op;
data.v_op = v_op;
data.Ts = 0.01;
data.U_MIN = 68;
data.U_MAX = 80;

%% 4. OPTIMIZACIÓN CON FMINSEARCH (MÉTODO CLÁSICO)
init_params = [1/data.SP, 0.1, 1.0];
options_fmin = optimset('Display', 'iter', 'TolX', 1e-4, 'MaxFunEvals', 200);

% --- CAMBIO PRINCIPAL: Llamada directa a fminsearch ---
% Se pasa el manejador @objective_fuzzy y la struct 'data' por separado.
% Esto evita la función anónima compleja que podría estar causando el error.
[best_params, best_cost] = fminsearch(@objective_fuzzy, init_params, options_fmin, data);

Ke = best_params(1);
Kde = best_params(2);
Ku = best_params(3);
fprintf('\nOptimización completada.\n');
fprintf('Mejores parámetros: Ke=%.4f, Kde=%.4f, Ku=%.3f (Costo IAE=%.2f)\n', Ke, Kde, Ku, best_cost);

%% 5. SIMULACIÓN Y GRÁFICA FINAL
t_final_sim = (0:data.Ts:50)';
y = simulate_fuzzy(data.planta, data.fis, best_params, t_final_sim, data.SP, data.u_op, data.v_op, data.Ts, data.U_MIN, data.U_MAX);

figure('Name', 'Control Difuso - Solución Robusta', 'Color', 'white');
plot(t_final_sim, y, 'b-', 'LineWidth', 2);
hold on;
yline(data.SP, 'r--', 'LineWidth', 1.5, 'Label', 'Setpoint');
grid on;
title('Respuesta del Sistema (Método Robusto)');
xlabel('Tiempo [s]');
ylabel('Posición [mm]');
legend('Salida del Sistema', 'Setpoint', 'Location', 'SouthEast');
ylim([min(y)-20, max(y)+20]);