%% Run init
init

%% Initialisation
syms q0 q1 q2 real;
syms q_ref0 q_ref1 q_ref2 real;
syms qdot0 qdot1 qdot2 real;
syms qddot0 qddot1 qddot2 real;
syms s real positive;
syms alpha real;
syms gx gy real;
syms k0 k1 k2 real positive;

q = [q0; q1; q2];
q_ref = [q_ref0; q_ref1; q_ref2];
qdot = [qdot0; qdot1; qdot2];
qddot = [qddot0; qddot1; qddot2];
g = [gx; gy];
k = [k0; k1; k2];
K = diag(k);

% assume(q ~= 0)
% assume(abs(q) < q_max)
% assume(abs(q_ref) < q_max)
assume(s > 0)


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
fprintf('B_dot ... ');
B_dot = simplify(dAdt(B, q, qdot));
fprintf('done!\n');

% Kinetic energy of the system
T = 1/2*qdot'*B*qdot;

% EOM: B(q)*qddot + C(q,qdot)*qdot + G(q) = tau

% C(q, qdot) matrix in EoM
fprintf('Computing coriolis and centrifugal matrix C and simplifying... ');
% Christoffel symbols
Gamma_C = sym(zeros(n_b, n_b, n_b));
for i=1:n_b
   for j=1:n_b
       for k_it=1:n_b
           Gamma_C(i,j,k_it) = 1/2*(diff(B(i,j),q(k_it))+diff(B(i,k_it),q(j))-diff(B(j,k_it),q(i)));
       end
   end
end
C = sym(zeros(n_b,n_b));
for i=1:n_b
   C(i,:,:) = qdot'*squeeze(Gamma_C(i,:,:));
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
U_k0 = int(1/2*k(1)*kappa0^2, s, 0, l(1));
U_k1 = int(1/2*k(2)*kappa1^2, s, 0, l(2));
U_k2 = int(1/2*k(3)*kappa2^2, s, 0, l(3));
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

% set point control using gravity compensation
tau_ref_spc = K*q_ref + G; % controller
H_spc = simplify(0.5*qdot'*B*qdot + 0.5*q'*K*q); % lyapunov function
dH_spc_dqdot = simplify(jacobian(H_spc, qdot));
dH_spc_dqdot_dot = simplify(dAdt(dH_spc_dqdot',q,qdot) + dAdt(dH_spc_dqdot',qdot,qddot));

%% Derive actuator dynamics
syms m_p0 m_p1 m_p2 m_p3 m_p4 m_p5 real positive;
syms A_p0 A_p1 A_p2 A_p3 A_p4 A_p5 real positive;
syms mu_p0 mu_p1 mu_p2 mu_p3 mu_p4 mu_p5 real positive;
syms l_p0 l_p1 l_p2 l_p3 l_p4 l_p5 real positive;
syms d_Ca d_Cb real positive;
syms b_C real positive; % thickness of planar soft robot chamber
% commanded torques on the soft robot in configuration space
syms tau_ref0 tau_ref1 tau_ref2 real;
% initial position of the pistons at t=0
syms mu_p0_t0 mu_p1_t0 mu_p2_t0 mu_p3_t0 mu_p4_t0 mu_p5_t0 real positive;
% commanded piston position delta
syms Delta_mu_p0 Delta_mu_p1 Delta_mu_p2 Delta_mu_p3 Delta_mu_p4 Delta_mu_p5 real;

assume(d_Ca < d_Cb);

m_p = [m_p0; m_p1; m_p2; m_p3; m_p4; m_p5];
A_p = [A_p0; A_p1; A_p2; A_p3; A_p4; A_p5];
mu_p = [mu_p0; mu_p1; mu_p2; mu_p3; mu_p4; mu_p5];
l_p = [l_p0; l_p1; l_p2; l_p3; l_p4; l_p5];
d_C = [d_Ca; d_Cb];
tau_ref = [tau_ref0; tau_ref1; tau_ref2];
mu_p_t0 = [mu_p0_t0; mu_p1_t0; mu_p2_t0; mu_p3_t0; mu_p4_t0; mu_p5_t0];
Delta_mu_p = [Delta_mu_p0; Delta_mu_p1; Delta_mu_p2; Delta_mu_p3; Delta_mu_p4; Delta_mu_p5];

assume(mu_p <= l_p);
assume(mu_p_t0 <= l_p);
assume(abs(Delta_mu_p) <= l_p);

% mass matrix
M_p = diag(m_p);

% volume in pistons vector
V_p = A_p.*mu_p;

% volume in chambers vector
V_C = sym(zeros(length(V_p), 1));
dV_C_dq = sym(zeros(length(V_p), 1));
for i=1:length(q)
    V_C(2*i-1) = b_C*(d_Cb-d_Ca)*(l(i)-q(i)/2*(d_Cb-d_Ca));
    V_C(2*i) = b_C*(d_Cb-d_Ca)*(l(i)+q(i)/2*(d_Cb-d_Ca));
    dV_C_dq(2*i-1) = simplify(diff(V_C(2*i-1), q(i)));
    dV_C_dq(2*i) = simplify(diff(V_C(2*i), q(i)));
end

% total volume of fluid stored in system
V = simplify(V_p + V_C);

% initial conditions and alpha_air = n*R*T = p*V
p_atm = 10^5;
% we assume the piston at initial condition to be fully retracted (e.g. no
% applied pressure and at mu_p = l_p)
V0 = subs(V, mu_p, l_p);
% we assume the robot at initial condition to be at neutral (straight)
% configuration for purposes of modelling the initial chamber volume
V0 = subs(V0, q, zeros(length(q), 1));
% instead of using alpha_air=n*R*T, we use initial volume and atmospheric
% pressure to determin alpha_air
alpha_air = p_atm * V0;

% potential energy of fluid
U_fluid_j = -alpha_air .* (log(V) - log(V0) - V./V0 + 1);
U_fluid = simplify(sum(U_fluid_j, 1));

% force acting on piston
G_p_mu = simplify(jacobian(U_fluid, mu_p)');

% force acting on the PCC soft robot
G_p_q = simplify(jacobian(U_fluid, q)');
G_p_q_j = sym(zeros(length(V_C), 1));
for i=1:length(q)
    G_p_q_j(2*i-1) = simplify(diff(U_fluid_j(2*i-1), q(i)));
    G_p_q_j(2*i) = simplify(diff(U_fluid_j(2*i), q(i)));
end

% initial force by fluid on system by every piston
G_p_q_j_t0 = simplify(subs(G_p_q_j, cat(1,q,mu_p), cat(1,[0;0;0],mu_p_t0)));

%% Actuation control
% balance of force
% distribute commanded tau of segment $i$ between left (j) and right (j+1)
% pistons leading to commanded force by fluid on system by each chamber
Delta_G_p_q_j_ref = sym(zeros(length(G_p_q_j),1));
for i=1:length(q)
    Delta_G_p_q_j_ref(2*i-1) = -tau_ref(i)/2;
    Delta_G_p_q_j_ref(2*i) = -tau_ref(i)/2;
end

% inverse of G_p_q to compute mu_p_ref from tau_ref
G_p_q_j_ref = simplify(G_p_q_j_t0 + Delta_G_p_q_j_ref);
% mu_p_ref = simplify(1./A_p .* (dV_C_dq./G_p_q_j_ref - V_C));
mu_p_ref = simplify(1./A_p.*(1./(1./V0-1./alpha_air.*1./dV_C_dq.*G_p_q_j_ref)-V_C));

%% Backstepping
% Controller for piston position using set point control with gravity
% compensation
Gamma = simplify(subs(mu_p_ref, tau_ref, tau_ref_spc));
fprintf('Computing Gamma_dot ... ');
dGamma_dq = jacobian(Gamma, q);
Gamma_dot = dGamma_dq * qdot;
dGamma_dot_dq = jacobian(Gamma_dot, q);
fprintf('done!\n');

% Lemma 1
S = sym(zeros(length(q), length(mu_p)));
for i=1:length(q)
   for j=1:length(mu_p)
       S(i,j) = alpha_air(j)*dV_C_dq(j)*A_p(j) / ...
                ((V_C(j)+A_p(j)*mu_p(j))*(V_C(j)+A_p(j)*Gamma(j)));
   end
end
S = simplify(S);
S_dot = dAdt(S,q,qdot);


%% Generate matlab functions
% fname = mfilename;
% fpath = mfilename('fullpath');
% dpath = strrep(fpath, fname, '');
dpath = fullfile(pwd, 'funs');

fprintf('Generating absolute angles theta\n');
matlabFunction(theta0, 'vars', {s, q, alpha}, 'file', strcat(dpath,'/q2theta0_fun'), 'Optimize', false);
matlabFunction(theta1, 'vars', {s, q, alpha}, 'file', strcat(dpath,'/q2theta1_fun'), 'Optimize', false);
matlabFunction(theta2, 'vars', {s, q, alpha}, 'file', strcat(dpath,'/q2theta2_fun'), 'Optimize', false);


fprintf('Generating forward kinematics\n');
matlabFunction(x0, 'vars', {s, q, alpha}, 'file', strcat(dpath,'/q2x0_fun'), 'Optimize', false);
matlabFunction(x1, 'vars', {s, q, alpha}, 'file', strcat(dpath,'/q2x1_fun'), 'Optimize', false);
matlabFunction(x2, 'vars', {s, q, alpha}, 'file', strcat(dpath,'/q2x2_fun'), 'Optimize', false);

fprintf('Generating eom scripts... ');
fprintf('B... ');
% matlabFunction(B0_ds, 'vars', {s, q, alpha}, 'file', strcat(dpath,'/B0_ds_fun'), 'Optimize', false);
% matlabFunction(B1_ds, 'vars', {s, q, alpha}, 'file', strcat(dpath,'/B1_ds_fun'), 'Optimize', false);
% matlabFunction(B2_ds, 'vars', {s, q, alpha}, 'file', strcat(dpath,'/B2_ds_fun'), 'Optimize', false);
matlabFunction(B, 'vars', {q, alpha}, 'file', strcat(dpath,'/B_fun'), 'Optimize', true);
matlabFunction(B_dot, 'vars', {q, qdot, alpha}, 'file', strcat(dpath,'/B_dot_fun'), 'Optimize', true);
fprintf('C... ');
% for i=1:1:n_b
%     for j=1:1:n_b
%         c_fun_name = sprintf('/C%d%d_fun', i, j);
%         matlabFunction(C(i,j), 'vars', {q, qdot, alpha}, 'file', strcat(dpath,c_fun_name), 'Optimize', true);
%     end
% end
matlabFunction(C, 'vars', {q, qdot, alpha}, 'file', strcat(dpath,'/C_fun'), 'Optimize', false);

fprintf('G ... ');
matlabFunction(G, 'vars', {q, alpha, g}, 'file', strcat(dpath,'/G_fun'), 'Optimize', false);
fprintf('\n');

% PCC controllers
fprintf('Generating set point controller fun ... ');
matlabFunction(tau_ref_spc, 'vars', {q, q_ref, alpha, g, k}, 'file', strcat(dpath,'/tau_ref_spc_fun'), 'Optimize', false);
fprintf('Lyapunov candidate fun ... ');
matlabFunction(H_spc, 'vars', {q, qdot, alpha, k}, 'file', strcat(dpath,'/H_spc_fun'), 'Optimize', false);
matlabFunction(dH_spc_dqdot, 'vars', {q, qdot, alpha}, 'file', strcat(dpath,'/dH_spc_dqdot_fun'), 'Optimize', false);
matlabFunction(dH_spc_dqdot_dot, 'vars', {q, qdot, qddot, alpha}, 'file', strcat(dpath,'/dH_spc_dqdot_dot_fun'), 'Optimize', true);
fprintf('\n');

% actuator dynamics
fprintf('Generating actuator dynamics ... ');
fprintf('Volumes ... ')
matlabFunction(V, 'vars', {q, mu_p, A_p, b_C, d_C}, 'file', strcat(dpath,'/V_fun'), 'Optimize', false);
matlabFunction(V_p, 'vars', {mu_p, A_p}, 'file', strcat(dpath,'/V_p_fun'), 'Optimize', false);
matlabFunction(V_C, 'vars', {q, b_C, d_C}, 'file', strcat(dpath,'/V_C_fun'), 'Optimize', false);
fprintf('alpha air ... ')
matlabFunction(alpha_air, 'vars', {l_p, A_p, b_C, d_C}, 'file', strcat(dpath,'/alpha_air_fun'), 'Optimize', false);
fprintf('G_p_mu ... ')
matlabFunction(G_p_mu, 'vars', {q, mu_p, l_p, A_p, b_C, d_C}, 'file', strcat(dpath,'/G_p_mu_fun'), 'Optimize', false);
fprintf('G_p_q ... ');
matlabFunction(G_p_q, 'vars', {q, mu_p, l_p, A_p, b_C, d_C}, 'file', strcat(dpath,'/G_p_q_fun'), 'Optimize', false);
fprintf('G_p_q_j_ref ... ');
matlabFunction(G_p_q_j_ref, 'vars', {tau_ref, mu_p_t0, l_p, A_p, b_C, d_C}, 'file', strcat(dpath,'/G_p_q_j_ref_fun'), 'Optimize', false);
fprintf('mu_p_ref ... ');
matlabFunction(mu_p_ref, 'vars', {q, tau_ref, mu_p_t0, l_p, A_p, b_C, d_C}, 'file', strcat(dpath,'/mu_p_ref_fun'), 'Optimize', false);
fprintf('\n');

% backstepping controller
fprintf('Generating backstepping controller ... ');
fprintf('Gamma ... ');
matlabFunction(Gamma, 'vars', {q, q_ref, mu_p_t0, alpha, l_p, A_p, b_C, d_C, g, k}, 'file', strcat(dpath,'/Gamma_fun'), 'Optimize', false);
fprintf('dGamma_dq ... ');
matlabFunction(dGamma_dq, 'vars', {q, q_ref, mu_p_t0, alpha, l_p, A_p, b_C, d_C, g, k}, 'file', strcat(dpath,'/dGamma_dq_fun'), 'Optimize', false);
fprintf('Gamma_dot ... ');
matlabFunction(Gamma_dot, 'vars', {q, qdot, q_ref, mu_p_t0, alpha, l_p, A_p, b_C, d_C, g, k}, 'file', strcat(dpath,'/Gamma_dot_fun'), 'Optimize', false);
fprintf('dGamma_dot_dq ... ');
matlabFunction(dGamma_dot_dq, 'vars', {q, qdot, q_ref, mu_p_t0, alpha, l_p, A_p, b_C, d_C, g, k}, 'file', strcat(dpath,'/dGamma_dot_dq_fun'), 'Optimize', false);
fprintf('S ... ');
matlabFunction(S, 'vars', {q, q_ref, mu_p, mu_p_t0, alpha, l_p, A_p, b_C, d_C, g, k}, 'file', strcat(dpath,'/S_fun'), 'Optimize', false);
fprintf('S_dot ... ');
matlabFunction(S_dot, 'vars', {q, qdot, q_ref, mu_p, mu_p_t0, alpha, l_p, A_p, b_C, d_C, g, k}, 'file', strcat(dpath,'/S_dot_fun'), 'Optimize', false);
fprintf('\n');