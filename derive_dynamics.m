%% Initialisation
syms q0 q1 q2;
syms qdot0 qdot1 qdot2;
syms s;
syms alpha;
syms l0 l1 l2;
syms rho0 rho1 rho2;
syms gx gy;
syms k0 k1 k2;

q = [q0; q1; q2];
qdot = [qdot0; qdot1; qdot2];
l = [l0; l1; l2];
rho = [rho0; rho1; rho2];
g = [gx; gy];
k = [k0; k1; k2];

% integration of weight per length rho to mass
m0 = int(rho0,s,0,l(1));
m1 = int(rho1,s,0,l(2));
m2 = int(rho2,s,0,l(3));
m = [m0; m1; m2];

%% Kinematics
kappa0 = q0/l0;
kappa1 = q1/l1;
kappa2 = q2/l2;
kappa = [kappa0; kappa1; kappa2];

theta0 = alpha + s*kappa0;
theta1 = subs(theta0,s,l0) + s*kappa1;
theta2 = subs(theta1,s,l1) + s*kappa2;
theta = [theta0; theta1; theta2];

x0 = [(sin(theta0)-sin(alpha))/kappa0;
        -(cos(theta0)-cos(alpha))/kappa0];
x1 = subs(x0,s,l0) + [(sin(theta1)-sin(subs(theta0,s,l0)))/kappa1;
                      -(cos(theta1)-cos(subs(theta0,s,l0)))/kappa1];
x2 = subs(x1,s,l1) + [(sin(theta2)-sin(subs(theta1,s,l1)))/kappa2;
                      -(cos(theta2)-cos(subs(theta1,s,l1)))/kappa2];
                  
% temporary formulation for forward kinematics of point masses
x_m0 = subs(x0,s,l0);
x_m1 = subs(x1,s,l1);
x_m2 = subs(x2,s,l2);
           
% Jacobians
J_m0_P = jacobian(x_m0, q);
J_m1_P = jacobian(x_m1, q);
J_m2_P = jacobian(x_m2, q);

% parametrisized Jacobians
J_0P = jacobian(x0, q)
J_1P = jacobian(x1, q)
J_2P = jacobian(x2, q);
return;

% time derivatives of Jacobians
Jdot_0P = simplify(dAdt(J_0P, q, qdot));
Jdot_1P = simplify(dAdt(J_1P, q, qdot));
Jdot_2P = simplify(dAdt(J_2P, q, qdot));

%% Dynamics
% B(q) matrix in EOM
fprintf('Computing mass matrix B ... ');
test1 = J_0P'*rho(1)*J_0P
B0 = int(J_0P'*rho(1)*J_0P, s, 0, l(1))
test2 = J_1P'*rho(2)*J_1P
B1 = int(J_1P'*rho(2)*J_1P, s, 0, l(2))
B2 = int(J_2P'*rho(3)*J_2P, s, 0, l(3));
B = simplify(B0 + B1 + B2);
fprintf('done!\n');

% Kinetic energy of the system
T = 1/2*qdot'*B*qdot;

% Gravitational potential energy
U_g0 = int(rho(1)*g'*x0, s, 0, l(1));
U_g1 = int(rho(2)*g'*x1, s, 0, l(2));
U_g2 = int(rho(3)*g'*x2, s, 0, l(3));
U_g = U_g0 + U_g1 + U_g2;

% Elastic potential energy
U_k0 = int(1/2*k0*kappa0^2, s, 0, l(1));
U_k1 = int(1/2*k1*kappa1^2, s, 0, l(2));
U_k2 = int(1/2*k2*kappa2^2, s, 0, l(3));
U_k = U_k0 + U_k1 + U_k2;

% Total potential energy
U = simplify(U_g + U_k);

% Langrangian
L = T - U;

% EOM: B(q)*qddot + C(q,qdot)*qdot + G(q) = tau

% C(q, qdot) matrix in EoM
fprintf('Computing coriolis and centrifugal vector b and simplifying... ');
C0 = int(J_0P'*rho(1)*Jdot_0P, s, 0, l(1));
C1 = int(J_1P'*rho(2)*Jdot_1P, s, 0, l(2));
C2 = int(J_2P'*rho(3)*Jdot_2P, s, 0, l(3));
C = simplify(C0 + C1 + C2);
fprintf('done!\n');

% G(q) vector in EoM
fprintf('Computing gravity vector G... ');
G = simplify(jacobian(U, q)');
fprintf('done!\n');

%% Generate matlab functions
fname = mfilename;
fpath = mfilename('fullpath');
dpath = strrep(fpath, fname, '');

fprintf('Generating absolute angles theta\n');
matlabFunction(theta0, 'vars', {s, q, alpha, l}, 'file', strcat(dpath,'/q2theta0_fun'), 'Optimize', false);
matlabFunction(theta1, 'vars', {s, q, alpha, l}, 'file', strcat(dpath,'/q2theta1_fun'), 'Optimize', false);
matlabFunction(theta2, 'vars', {s, q, alpha, l}, 'file', strcat(dpath,'/q2theta2_fun'), 'Optimize', false);


fprintf('Generating forward kinematics\n');
matlabFunction(x0, 'vars', {s, q, alpha, l}, 'file', strcat(dpath,'/q2x0_fun'), 'Optimize', false);
matlabFunction(x1, 'vars', {s, q, alpha, l}, 'file', strcat(dpath,'/q2x1_fun'), 'Optimize', false);
matlabFunction(x2, 'vars', {s, q, alpha, l}, 'file', strcat(dpath,'/q2x2_fun'), 'Optimize', false);

fprintf('Generating eom scripts... ');
fprintf('B... ');
matlabFunction(B, 'vars', {q, alpha, l, rho}, 'file', strcat(dpath,'/B_fun'), 'Optimize', false);
fprintf('C... ');
matlabFunction(C, 'vars', {q, qdot, alpha, l, rho}, 'file', strcat(dpath,'/C_fun'), 'Optimize', false);
fprintf('G... ');
matlabFunction(G, 'vars', {q, alpha, l, rho, k, g}, 'file', strcat(dpath,'/G_fun'), 'Optimize', false);
fprintf('\n');