%% Configuration
n_b = 3; % dimension of configuration space
n_C = 6; % number of chambers
% q_max = pi/2; % maximum assumed generalized coordinates

% parameters
l = 0.110*[1;1;1]; % length of every segment
rho = 0.99*[1;1;1]; % mass density (per length) of every segment

% equilibrium configuration (experimentally determined)
q_eq = [-108; 21.5; -3.4]/180*pi;

% minimal q for stability of B, C and G matrices
tol_q_stable = 1/180*pi;