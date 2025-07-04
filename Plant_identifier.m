% -------------------------------------------------------------------------
% SCRIPT AVANZADO PARA IDENTIFICACIÓN DE SISTEMAS (BARRA Y BOLA)
% -------------------------------------------------------------------------
% Descripción:
% Versión final que incluye:
% 1. Carga de datos y selección de rango.
% 2. Selección de estructura de modelo (TF, SS).
% 3. Búsqueda automática del retardo óptimo en un rango.
% 4. Muestra el modelo como F. de Transferencia y en formato ZPK.
% 5. **Genera dos LGR: uno sin retardo y otro con retardo aproximado.**
% 6. Validación completa del modelo.
% -------------------------------------------------------------------------

% --- Limpieza del entorno ---
clear;
close all;
clc;

% --- 1. Selección y Lectura de Datos ---
[fileName, pathName] = uigetfile('*.csv', 'Selecciona el archivo de datos CSV');
if isequal(fileName, 0), disp('Selección cancelada.'); return; end
fullFilePath = fullfile(pathName, fileName);
fprintf('Cargando datos desde: %s\n', fullFilePath);
try
    dataTable = readtable(fullFilePath);
    tiempo_completo = dataTable.tiempo;
    u_completo = dataTable.u_entrada_; % Entrada (ángulo)
    v_completo = dataTable.v_salida_;  % Salida (distancia)
    fprintf('Datos cargados correctamente.\n');
catch ME
    fprintf('Error al leer el archivo: %s\n', ME.message);
    return;
end

% --- 2. Selección del Rango de Tiempo ---
prompt = {'Tiempo inicial (s):', 'Tiempo final (s):'};
dlgtitle = 'Rango para Identificación';
definput = {'2', '14'};
answer = inputdlg(prompt, dlgtitle, [1 50], definput);
if isempty(answer), disp('Proceso cancelado.'); return; end
startTime = str2double(answer{1});
endTime = str2double(answer{2});

% --- 3. Preparación de Datos ---
indices = find(tiempo_completo >= startTime & tiempo_completo <= endTime);
if isempty(indices), disp('Rango de tiempo sin datos.'); return; end
tiempo = tiempo_completo(indices);
u_entrada = u_completo(indices);
v_salida = v_completo(indices);
Ts = mean(diff(tiempo));
data = iddata(v_salida - mean(v_salida), u_entrada - mean(u_entrada), Ts, 'Name', 'Barra-Bola');
data.InputName = 'Ángulo';
data.OutputName = 'Distancia';
u_op = mean(u_entrada);
v_op = mean(v_salida);
fprintf('Datos recortados. Usando %d muestras con Ts = %.4f s.\n', length(tiempo), Ts);
fprintf('Puntos de operación (Offsets) U_op: %.2f, V_op: %.2f\n', u_op, v_op);

% --- 4. Selección Interactiva del Modelo ---
modelOptions = {
    'Función de Transferencia (4 polos, 0 ceros)', ...
    'Función de Transferencia (Orden Personalizado)', ...
    'Modelo de Espacio de Estados (Orden 2)', ...
    'Modelo de Espacio de Estados (Orden Personalizado)'
    };
[selection, ok] = listdlg('PromptString', 'Seleccione la estructura del modelo:', ...
    'SelectionMode', 'single', 'ListString', modelOptions, 'ListSize', [350 150]);
if ~ok, disp('Selección de modelo cancelada.'); return; end

% --- Preguntar por RANGO de búsqueda de retardo ---
prompt_delay = {'Retraso Mínimo a probar (s):', 'Retraso Máximo a probar (s):'};
dlgtitle_delay = 'Rango de Búsqueda de Retardo';
definput_delay = {'0.0', '2.0'};
answer_delay = inputdlg(prompt_delay, dlgtitle_delay, [1 60], definput_delay);
if isempty(answer_delay), disp('Proceso cancelado.'); return; end
minDelay = str2double(answer_delay{1});
maxDelay = str2double(answer_delay{2});
delayVector = minDelay:Ts:maxDelay;
fprintf('\nIniciando búsqueda del mejor retardo en el rango [%.3f s, %.3f s]...\n', minDelay, maxDelay);

% --- 5. Estimación del Modelo con Búsqueda de Retardo ---
mejor_ajuste = -inf;
mejor_retraso = NaN;
sys_estimado = [];
h_waitbar = waitbar(0, 'Iniciando búsqueda de retardo...');

% Obtener parámetros de modelo si es personalizado
np = []; nz = []; order = [];
switch selection
    case 2
        order_ans = inputdlg({'Polos (np):', 'Ceros (nz):'}, 'Orden TF', [1 40], {'3', '1'});
        if isempty(order_ans), disp('Cancelado.'); close(h_waitbar); return; end
        np = str2double(order_ans{1}); nz = str2double(order_ans{2});
    case 4
        order_ans = inputdlg({'Orden del sistema:'}, 'Orden SS', [1 40], {'3'});
        if isempty(order_ans), disp('Cancelado.'); close(h_waitbar); return; end
        order = str2double(order_ans{1});
end

% Bucle de búsqueda
for i = 1:length(delayVector)
    currentDelay = delayVector(i);
    waitbar(i / length(delayVector), h_waitbar, sprintf('Probando retraso: %.3f s', currentDelay));
    temp_sys = [];
    try
        switch selection
            case 1, temp_sys = tfest(data, 4, 0, 'InputDelay', currentDelay);
            case 2, temp_sys = tfest(data, np, nz, 'InputDelay', currentDelay);
            case 3, temp_sys = ssest(data, 2, 'InputDelay', currentDelay);
            case 4, temp_sys = ssest(data, order, 'InputDelay', currentDelay);
        end
    catch, continue; end
    
    if ~isempty(temp_sys) && temp_sys.Report.Fit.FitPercent > mejor_ajuste
        mejor_ajuste = temp_sys.Report.Fit.FitPercent;
        mejor_retraso = currentDelay;
        sys_estimado = temp_sys;
    end
end
close(h_waitbar);

% --- 6. Mostrar Resultados ---
if isempty(sys_estimado), disp('No se pudo estimar un modelo válido.'); return; end

fprintf('\nBúsqueda finalizada.\n');
fprintf('=========================================\n');
fprintf('MEJOR RETRASO ENCONTRADO: %.3f segundos\n', mejor_retraso);
fprintf('MEJOR AJUSTE OBTENIDO:    %.2f%%\n', mejor_ajuste);
fprintf('=========================================\n');

disp('-----------------------------------------');
disp('MEJOR MODELO IDENTIFICADO:');
fprintf('\n>> Función de Transferencia (Formato Estándar):\n');
disp(tf(sys_estimado));
fprintf('>> Función de Transferencia (Formato Cero-Polo-Ganancia):\n');
disp(zpk(sys_estimado));
disp('-----------------------------------------');

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INICIO DE LA SECCIÓN DE ANÁLISIS GRÁFICO
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --- 7. Análisis Gráfico Adicional ---

% --- 7A. Graficar el LGR del sistema SIN retardo ---
figure('Name', 'LGR (Sistema Puro, SIN Retardo)', 'NumberTitle', 'off');
fprintf('\nGraficando LGR del sistema puro (ignorando el retardo de tiempo)...\n');
sys_sin_retardo = sys_estimado;
sys_sin_retardo.InputDelay = 0; % Anulamos el retardo para este análisis
rlocus(sys_sin_retardo);
grid on;
title({'Lugar Geométrico de las Raíces (LGR)', 'del Sistema Puro (SIN Retardo)'});
sgrid;

% --- 7B. Graficar el LGR del sistema CON retardo aproximado ---
figure('Name', 'LGR (Sistema con Retardo Aproximado)', 'NumberTitle', 'off');
sys_con_retardo_aprox = sys_estimado;
if sys_con_retardo_aprox.InputDelay > 0
    fprintf('Graficando LGR del sistema completo, aproximando el retardo (%.3f s) con Padé (2do orden)...\n', sys_con_retardo_aprox.InputDelay);
    sys_con_retardo_aprox = pade(sys_estimado, 2);
    rlocus(sys_con_retardo_aprox);
    title({'Lugar Geométrico de las Raíces (LGR)', 'del Sistema con Retardo Aproximado (Padé)'});
else
    fprintf('El sistema no tiene retardo, el segundo LGR no es necesario.\n');
    title('Lugar Geométrico de las Raíces (LGR) - Sin retardo');
    rlocus(sys_con_retardo_aprox); % Se grafica de todos modos para consistencia
end
grid on;
sgrid;

% --- 8. Validación del Modelo ---
% --- 8A. Gráfica de Comparación
figure('Name', 'Validación: Comparación de Salidas', 'NumberTitle', 'off');
compare(data, sys_estimado);
grid on;
title(sprintf('Comparación Salida Real vs. Modelo (Fit: %.2f%%)', sys_estimado.Report.Fit.FitPercent));
legend('Salida Real Medida', 'Salida del Modelo Estimado');

% --- 8B. Análisis de Residuos
figure('Name', 'Validación: Análisis de Residuos', 'NumberTitle', 'off');
resid(data, sys_estimado);
disp('Análisis de Residuos realizado. Revise la figura correspondiente.');

% --- 9. Guardar Resultados ---
save('modelo_identificado.mat', 'sys_estimado', 'sys_con_retardo_aprox', 'u_op', 'v_op', 'mejor_retraso');
fprintf('\n✅ ¡Éxito! El modelo y datos de operación han sido guardados en "modelo_identificado.mat"\n');
disp('Script de identificación finalizado.');