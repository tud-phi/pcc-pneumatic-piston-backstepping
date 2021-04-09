function x = euler_forward_int(dx,t,t_a,dt, t_b)
    % dx: symbolix expression to integrate
    % t: symbolic variable used for integration
    % t_a: lower bound for integration
    % dt: step size of integration
    % t_b: high bound for integration
    %   Detailed explanation goes here
    x = 0;
    t_dash = t_a;
    while t_dash < t_b
        dx_tdash = subs(dx,t,t_dash);
        x = x + dx_tdash*dt;
        t_dash = t_dash + dt;
    end
end

