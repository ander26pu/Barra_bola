% =========== FILE 3: simulate_fuzzy.m ===========
function y = simulate_fuzzy(planta, fis, params, t, SP, u_op, v_op, Ts, umin, umax)
    % Esta función simula la respuesta del sistema en lazo cerrado.
    Ke = params(1); Kde = params(2); Ku = params(3);

    sys_d = c2d(ss(planta), Ts, 'zoh');
    A = sys_d.A; B = sys_d.B; C = sys_d.C; D = sys_d.D;

    x = zeros(size(A,1),1);
    y = v_op * ones(size(t));
    e_prev = SP - y(1);

    for k = 1:length(t)-1
        error_actual = SP - y(k);
        derivada_error = (error_actual - e_prev) / Ts;
        e_prev = error_actual;

        e_norm = max(-1, min(1, Ke * error_actual));
        de_norm = max(-1, min(1, Kde * derivada_error));

        u_norm = evalfis(fis, [e_norm, de_norm]);

        u_fuzzy = Ku * u_norm;

        u_total = u_op + u_fuzzy;
        u_total = max(umin, min(umax, u_total));

        u_plant = u_total - u_op;
        x = A*x + B*u_plant;
        y(k+1) = v_op + C*x + D*u_plant;
    end
end