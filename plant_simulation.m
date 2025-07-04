%--------------------------------------------------------------------------
% Script para simular la RESPUESTA COMPLETA del sistema identificado
%--------------------------------------------------------------------------
clear; clc; close all;

%% 1) Cargar el modelo identificado
% Supongamos que guardaste el modelo y los puntos de operación
load('modelo_identificado.mat'); % Esto contendría sys_estimado, u_op, v_op

% Si no lo guardaste, define los valores aquí para el ejemplo:
% sys_estimado = ... (tu modelo G(s))
% u_op = 74.5; % Valor aproximado de (69+80)/2
% v_op = 200.0; % El offset que observaste en la salida

%% 2) Generar la misma señal de entrada sinusoidal
sineFreq = 0.48;
minAngle = 69.0;
maxAngle = 80.0;
Tfinal = 20;
Ts = 0.01;
t = 0:Ts:Tfinal;

amplitud = (maxAngle - minAngle) / 2;
offset_entrada   = (maxAngle + minAngle) / 2; % Este es tu u_real
omega    = 2 * pi * sineFreq;
u_real = amplitud * sin(omega * t) + offset_entrada;

%% 3) Simular la respuesta del modelo USANDO LOS PUNTOS DE OPERACIÓN
% Primero, centramos la entrada restando el offset con el que se identificó el modelo
u_sim_detrend = u_real - u_op;

% Luego, simulamos la respuesta dinámica
y_sim_detrend = lsim(sys_estimado, u_sim_detrend, t);

% Finalmente, añadimos el offset de salida para obtener la respuesta completa
y_sim_completa = y_sim_detrend + v_op;

%% 4) Graficar los resultados
figure('Color','white', 'Name', 'Respuesta Corregida del Modelo Identificado');

% Gráfica de la señal de entrada
subplot(2, 1, 1);
plot(t, u_real, 'k', 'LineWidth', 1.5);
title('Señal de Entrada (Acción de Control)');
ylabel('Ángulo servo (°)');
grid on;

% Gráfica de la señal de salida simulada
subplot(2, 1, 2);
plot(t, y_sim_completa, 'r--', 'LineWidth', 2);
title('Respuesta Simulada del Modelo (Corregida con Offset)');
ylabel('Distancia (mm)');
xlabel('Tiempo (s)');
grid on;
ylim([min(y_sim_completa)-20 max(y_sim_completa)+20]); % Ajustar límites