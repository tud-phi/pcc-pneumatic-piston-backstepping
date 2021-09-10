%% Configuration
n_b = 3; % dimension of configuration space
n_C = 6; % number of chambers
% q_max = pi/2; % maximum assumed generalized coordinates

% parameters
l = 0.110*[1;1;1]; % length of every segment
rho = 0.99*[1;1;1]; % mass density (per length) of every segment

% equilibrium configuration (experimentally determined)
q_eq = [-108; 21.5; -3.4]/180*pi;
mu_p_eq = [0.12135; 0.12866; 0.12574; 0.12426; 0.12489; 0.12511];

% minimal q for stability of B, C and G matrices
tol_q_stable = 3/180*pi;