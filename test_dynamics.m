% parameters
l = [1; 1; 1];
alpha = 0;
rho = [1; 1; 1];
g = [9.81; 0];
k = [1; 1; 1];

q = [0.2; -0.2; 0.1];
qdot = [0; 0; 0];

n_b = length(q);
tol = 0.001;

Bi = zeros(n_b, n_b, n_b);
for i=1:n_b
    s_dash = tol;
    ds = l(i) / 50;
    while s_dash < l(1)
        if i == 1
            dBi_sdash = B0_ds_fun(s_dash, q, alpha, l, rho);
        elseif i == 2
            dBi_sdash = B1_ds_fun(s_dash, q, alpha, l, rho);
        else
            dBi_sdash = B2_ds_fun(s_dash, q, alpha, l, rho);
        end
        Bi(i, :, :) = squeeze(Bi(i, :, :)) + dBi_sdash*ds;
        s_dash = s_dash + ds;
    end
end
B = squeeze(sum(Bi, 1))

C = C_fun(q, qdot, alpha, l, rho)
G = G_fun(q, alpha, l, rho, k, g)