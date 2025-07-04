% =========== FILE 2: objective_fuzzy.m (VERSIÓN FINAL) ===========
function J = objective_fuzzy(params, data)
    % Esta función calcula el costo (IAE) para una simulación.
    
    % Desempaquetar los datos necesarios de la struct 'data'
    planta = data.planta;
    fis = data.fis;
    t = data.t;
    SP = data.SP;
    u_op = data.u_op;
    v_op = data.v_op;
    Ts = data.Ts;
    umin = data.U_MIN;
    umax = data.U_MAX;

    % Llamar a la función de simulación (esta no cambia)
    y = simulate_fuzzy(planta, fis, params, t, SP, u_op, v_op, Ts, umin, umax);

    % Penalización por inestabilidad
    if any(isnan(y)) || any(abs(y) > 1e4)
        J = Inf;
        return;
    end

    % Cálculo del IAE (Integral Absolute Error)
    J = trapz(t, abs(y - SP));
end