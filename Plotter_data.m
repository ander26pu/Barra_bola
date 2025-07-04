  % -------------------------------------------------------------------------
% SCRIPT PARA LEER Y GRAFICAR DATOS DE IDENTIFICACIÓN DE PLANTA DESDE CSV
% -------------------------------------------------------------------------
% Descripción:
% Este script abre un cuadro de diálogo para que el usuario seleccione un
% archivo .csv (generado por el script de Python). Luego, lee los datos
% y grafica la señal de entrada 'u' y la señal de salida 'v' en función
% del tiempo en dos subplots separados.
%
% Ideal para la visualización previa antes de usar iddata en el System
% Identification Toolbox de MATLAB.
% -------------------------------------------------------------------------

% --- Limpieza del entorno ---
% Limpia el workspace, cierra todas las figuras y limpia la ventana de comandos.
clear;
close all;
clc;

% --- Selección del archivo de datos ---
% Abre una ventana para que el usuario elija el archivo .csv
[fileName, pathName] = uigetfile('*.csv', 'Selecciona el archivo de datos CSV');

% Comprobar si el usuario canceló la selección
if isequal(fileName, 0)
    disp('Selección de archivo cancelada por el usuario.');
    return; % Termina el script si no se seleccionó archivo
end

% Construir la ruta completa al archivo
fullFilePath = fullfile(pathName, fileName);
fprintf('Cargando datos desde: %s\n', fullFilePath);

% --- Lectura de los datos del archivo CSV ---
try
    % Usar readtable para leer el CSV, es robusto y maneja los encabezados
    dataTable = readtable(fullFilePath);
    
    % Extraer las columnas de datos
    tiempo = dataTable.tiempo;         % Columna de tiempo
    u_entrada = dataTable.u_entrada_;  % Columna de entrada (ángulo)
    v_salida = dataTable.v_salida_;    % Columna de salida (distancia)
    
    fprintf('Datos cargados correctamente.\n');
    
catch ME
    % Manejo de errores en caso de que el archivo no se pueda leer
    fprintf('Error al leer el archivo: %s\n', ME.message);
    return;
end

% --- Creación de las gráficas ---
% Crear una nueva figura para mostrar los plots
figure('Name', 'Datos de Entrada y Salida de la Planta', 'NumberTitle', 'off');

% 1. Subplot para la señal de entrada u(t)
subplot(2, 1, 1); % Divide la figura en 2 filas, 1 columna, y selecciona el 1er plot
plot(tiempo, u_entrada, 'b-', 'LineWidth', 1.5); % Graficar tiempo vs entrada
title('Señal de Entrada (u) vs. Tiempo');
xlabel('Tiempo (s)');
ylabel('Ángulo (grados)');
grid on;
legend('u(t) - Ángulo del servo');
axis tight; % Ajustar los ejes al rango de los datos

% 2. Subplot para la señal de salida v(t)
subplot(2, 1, 2); % Selecciona el 2do plot
plot(tiempo, v_salida, 'r-', 'LineWidth', 1.5); % Graficar tiempo vs salida
title('Señal de Salida (v) vs. Tiempo');
xlabel('Tiempo (s)');
ylabel('Distancia (mm)');
grid on;
legend('v(t) - Distancia medida');
axis tight; % Ajustar los ejes al rango de los datos

% Mensaje final en la consola
disp('Gráficas generadas exitosamente.');

% --- NOTA PARA IDENTIFICACIÓN ---
% Para usar estos datos en el System Identification Toolbox:
%
% Fs = 1 / mean(diff(tiempo)); % Calcular frecuencia de muestreo
% data = iddata(v_salida, u_entrada, 1/Fs);
% systemIdentification(data);
%
% -------------------------------------------------------------------------
