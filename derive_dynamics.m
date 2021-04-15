%% Configuration
n_b = 3;

%% Initialisation
syms q0 q1 q2 real;
syms qdot0 qdot1 qdot2 real;
syms s real positive;
syms alpha real;
syms l0 l1 l2 real positive;
syms rho0 rho1 rho2 real positive;
syms gx gy real;
syms k0 k1 k2 real positive;

assume(s > 0)
assume(l0 > 0)
assume(l1 > 0)
assume(l2 > 0)
assume(rho0 > 0)
assume(rho1 > 0)
assume(rho2 > 0)

q = [q0; q1; q2];
qdot = [qdot0; qdot1; qdot2];
l = [l0; l1; l2];
rho = [rho0; rho1; rho2];
g = [gx; gy];
k = [k0; k1; k2];

% integration of weight per length rho to mass
m0 = int(rho(1),s,0,l(1));
m1 = int(rho(2),s,0,l(2));
m2 = int(rho(3),s,0,l(3));
m = [m0; m1; m2];

%% Kinematics
kappa0 = q0/l(1);
kappa1 = q1/l(2);
kappa2 = q2/l(3);
kappa = [kappa0; kappa1; kappa2];

theta0 = alpha + s*kappa0;
theta1 = subs(theta0,s,l(1)) + s*kappa1;
theta2 = subs(theta1,s,l(2)) + s*kappa2;
theta = [theta0; theta1; theta2];

x0 = [(sin(theta(1))-sin(alpha))/kappa0;
        -(cos(theta(1))-cos(alpha))/kappa0];
x1 = subs(x0,s,l(1)) + [(sin(theta(2))-sin(subs(theta(1),s,l(1))))/kappa1;
                      -(cos(theta(2))-cos(subs(theta(1),s,l(1))))/kappa1];
x2 = subs(x1,s,l(2)) + [(sin(theta(3))-sin(subs(theta(2),s,l(2))))/kappa2;
                      -(cos(theta(3))-cos(subs(theta(2),s,l(2))))/kappa2];
                  
% temporary formulation for forward kinematics of point masses
x_m0 = subs(x0,s,l(1));
x_m1 = subs(x1,s,l(2));
x_m2 = subs(x2,s,l(3));
           
% point mass Jacobians
J_m0_P = jacobian(x_m0, q);
J_m1_P = jacobian(x_m1, q);
J_m2_P = jacobian(x_m2, q);

% parametrisized Jacobians
J_0P = jacobian(x0, q);
J_1P = jacobian(x1, q);
J_2P = jacobian(x2, q);

% time derivatives of point mass Jacobians
Jdot_m0_P = simplify(dAdt(J_m0_P, q, qdot));
Jdot_m1_P = simplify(dAdt(J_m1_P, q, qdot));
Jdot_m2_P = simplify(dAdt(J_m2_P, q, qdot));

% time derivatives of point mass Jacobians
Jdot_0P = simplify(dAdt(J_0P, q, qdot));
Jdot_1P = simplify(dAdt(J_1P, q, qdot));
Jdot_2P = simplify(dAdt(J_2P, q, qdot));

%% Dynamics
% B(q) matrix in EOM
% tolerance to prevent divisions-by-zero during integration
tol = 0.001;
fprintf('Computing mass matrix B ... ');
B0_ds = simplify(J_0P'*rho(1)*J_0P);
fprintf('int B0 ... ');
B0 = int(B0_ds, s, tol, l(1));
B1_ds = simplify(J_1P'*rho(2)*J_1P);
fprintf('int B1 ... ');
B1 = int(B1_ds, s, tol, l(2));
B2_ds = simplify(J_2P'*rho(3)*J_2P);
fprintf('int B2 ... ');
B2 = int(B2_ds, s, tol, l(3));
fprintf('simplify B ... ');
% B = simplify(J_m0_P'*m(1)*J_m0_P + J_m1_P'*m(2)*J_m1_P + J_m2_P'*m(3)*J_m2_P);
B = simplify(B0+B1+B2);
fprintf('done!\n');

% Kinetic energy of the system
T = 1/2*qdot'*B*qdot;

% EOM: B(q)*qddot + C(q,qdot)*qdot + G(q) = tau

% C(q, qdot) matrix in EoM
fprintf('Computing coriolis and centrifugal matrix C and simplifying... ');
% Christoffel symbols
Gamma = sym(zeros(n_b, n_b, n_b));
for i=1:n_b
   for j=1:n_b
       for k_it=1:n_b
           Gamma(i,j,k_it) = 1/2*(diff(B(i,j),q(k_it))+diff(B(i,k_it),q(j))-diff(B(j,k_it),q(i)));
       end
   end
end
C = sym(zeros(n_b,n_b));
for i=1:n_b
   C(i,:,:) = qdot'*squeeze(Gamma(i,:,:));
end
C = simplify(C);

% C(q, qdot) computed using projected Newton-Euler approach (RSL)
% C0_ds = simplify(J_0P'*rho(1)*Jdot_0P);
% fprintf('int C0 ... ');
% C0 = int(C0_ds, s, tol, l(1));
% C1_ds = simplify(J_1P'*rho(2)*Jdot_1P);
% fprintf('int C1 ... ');
% C1 = int(C1_ds, s, tol, l(2));
% C2_ds = simplify(J_2P'*rho(3)*Jdot_2P);
% fprintf('int C2 ... ');
% B2 = int(C2_ds, s, tol, l(3));
% fprintf('simplify C ... ');
% C = simplify(C0+C1+C2);
% only point masses
% C_pne = simplify(J_m0_P'*m(1)*Jdot_m0_P + J_m1_P'*m(2)*Jdot_m1_P + J_m2_P'*m(3)*Jdot_m2_P)
fprintf('done!\n');

% Gravitational potential energy
fprintf('Computing gravitational potential energy... ');
U_g0 = int(rho(1)*g'*x0, s, 0, l(1));
U_g1 = int(rho(2)*g'*x1, s, 0, l(2));
U_g2 = int(rho(3)*g'*x2, s, 0, l(3));
U_g = simplify(U_g0 + U_g1 + U_g2);
fprintf('done!\n');

% Elastic potential energy
fprintf('Computing elastic potential energy... ');
U_k0 = int(1/2*k0*kappa0^2, s, 0, l(1));
U_k1 = int(1/2*k1*kappa1^2, s, 0, l(2));
U_k2 = int(1/2*k2*kappa2^2, s, 0, l(3));
U_k = simplify(U_k0 + U_k1 + U_k2);
fprintf('done!\n');

% Total potential energy
fprintf('Simplifying potential energy... ');
U = simplify(U_g + U_k);
fprintf('done!\n');

% G(q) vector in EoM
fprintf('Computing gravity vector G... ');
G = simplify(jacobian(U_g, q)');
fprintf('done!\n');

% Langrangian
L = T - U;

%% Derive actuator dynamics
syms m_p0 m_p1 m_p2 m_p3 m_p4 m_p5 real positive;
syms A_p0 A_p1 A_p2 A_p3 A_p4 A_p5 real positive;
syms mu_p0 mu_p1 mu_p2 mu_p3 mu_p4 mu_p5 real positive;
syms d_Ca d_Cb real positive;
syms b_C real positive; % thickness of planar soft robot chamber 

assume(d_Ca < d_Cb);

m_p = [m_p0; m_p1; m_p2; m_p3; m_p4; m_p5];
A_p = [A_p0; A_p1; A_p2; A_p3; A_p4; A_p5];
mu_p = [mu_p0; mu_p1; mu_p2; mu_p3; mu_p4; mu_p5];
d_C = [d_Ca; d_Cb];

% mass matrix
M_p = diag(m_p);

% volume in pistons vector
V_p = A_p.*mu_p;

% volume in chambers vector
V_C = sym(zeros(length(V_p), 1));
for i=1:length(q)
    V_C(2*i-1) = b_C*(d_Cb-d_Ca)*(l(i)-q(i)/2*(d_Cb-d_Ca));
    V_C(2*i) = b_C*(-d_Cb+d_Ca)*(l(i)-q(i)/2*(-d_Cb+d_Ca));
end

% total volume of fluid stored in system
V = simplify(V_p + V_C);

% potential energy of fluid
% TODO insert alpha constant
U_fluid_j = log(V);
U_fluid = simplify(sum(U_fluid_j, 1));

% force acting on piston
G_p_mu = simplify(jacobian(U_fluid, mu_p)');

% force acting on the PCC soft robot
G_p_q = simplify(jacobian(U_fluid, q)');

%% Generate matlab functions
% fname = mfilename;
% fpath = mfilename('fullpath');
% dpath = strrep(fpath, fname, '');
dpath = pwd;

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
matlabFunction(B0_ds, 'vars', {s, q, alpha, l, rho}, 'file', strcat(dpath,'/B0_ds_fun'), 'Optimize', false);
matlabFunction(B1_ds, 'vars', {s, q, alpha, l, rho}, 'file', strcat(dpath,'/B1_ds_fun'), 'Optimize', false);
matlabFunction(B2_ds, 'vars', {s, q, alpha, l, rho}, 'file', strcat(dpath,'/B2_ds_fun'), 'Optimize', false);
matlabFunction(B, 'vars', {q, alpha, l, rho}, 'file', strcat(dpath,'/B_fun'), 'Optimize', true);
fprintf('C... ');
for i=1:1:n_b
    for j=1:1:n_b
        c_fun_name = sprintf('/C%d%d_fun', i, j);
        matlabFunction(C(i,j), 'vars', {q, qdot, alpha, l, rho}, 'file', strcat(dpath,c_fun_name), 'Optimize', true);
    end
end
% matlabFunction(C, 'vars', {q, qdot, alpha, l, rho}, 'file', strcat(dpath,'/C_fun'), 'Optimize', false);

fprintf('G ... ');
matlabFunction(G, 'vars', {q, alpha, l, rho, g}, 'file', strcat(dpath,'/G_fun'), 'Optimize', false);
fprintf('\n');

% actuator dynamics
fprintf('Generating actuator forces...');
fprintf('G_p_mu ... ')
matlabFunction(G_p_mu, 'vars', {q, mu_p, l, A_p, b_C, d_C}, 'file', strcat(dpath,'/G_p_mu_fun'), 'Optimize', false);
fprintf('G_p_q ... ');
matlabFunction(G_p_q, 'vars', {q, mu_p, l, A_p, b_C, d_C}, 'file', strcat(dpath,'/G_p_q_fun'), 'Optimize', false);
fprintf('\n');