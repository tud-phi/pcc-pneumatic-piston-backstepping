syms q0 q1 q2;
syms qdot0 qdot1 qdot2;
syms alpha;
syms l0 l1 l2;

q = [q0; q1; q2];
qdot = [qdot0; qdot1; qdot2];
l = [l0, l1, l2];

theta0 = alpha + q0;
theta1 = theta0 + q1;
theta2 = theta1 + q2;
theta = [theta0; theta1; theta2];

kappa0 = q0/l0;
kappa1 = q1/l1;
kappa2 = q2/l2;
kappa = [kappa0; kappa1; kappa2];

x_m0 = [(sin(theta0)-sin(alpha))/kappa0;
        -(cos(theta0)-cos(alpha))/kappa0];
x_m1 = x_m0 + [(sin(theta1)-sin(theta0))/kappa1;
               -(cos(theta1)-cos(theta0))/kappa1];
x_m2 = x_m1 + [(sin(theta2)-sin(theta1))/kappa2;
               -(cos(theta2)-cos(theta1))/kappa2];
           
J_m0_P = jacobian(x_m0, q)
J_m1_P = jacobian(x_m1, q)
J_m2_P = jacobian(x_m2, q)