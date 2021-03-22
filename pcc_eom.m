function [B, C, g]  = pcc_eom(m, k, q, qdot)
    J_m0_P = [cos(q(1))/k(1), 0, 0;
              sin(q(1))/k(1), 0, 0];
    J_m1_P = [cos(q(1))*(1/k(1)-1/k(2)), cos(q(2))/k(2), 0;
              sin(q(1))*(1/k(1)-1/k(2)), sin(q(2))/k(2), 0];
    J_m2_P = [cos(q(1))*(1/k(1)-1/k(2)), cos(q(2))*(1/k(2)-1/k(3)), cos(q(3))/k(3);
              sin(q(1))*(1/k(1)-1/k(2)), sin(q(2))*(1/k(2)-1/k(3)), sin(q(3))/k(3)];
    Jdot_m0_P = [-sin(q(1))/k(1)*qdot(1), 0, 0;
                 cos(q(1))/k(1)*qdot(1), 0, 0];
    Jdot_m1_P = [-sin(q(1))*(1/k(1)-1/k(2))*qdot(1), cos(q(2))/k(2)*qdot(2), 0;
                 cos(q(1))*(1/k(1)-1/k(2)), sin(q(2))/k(2)*qdot(2), 0];
    Jdot_m2_P = [-sin(q(1))*(1/k(1)-1/k(2))*qdot(1), -sin(q(2))*(1/k(2)-1/k(3))*qdot(2), cos(q(3))/k(3)*qdot(3);
                 cos(q(1))*(1/k(1)-1/k(2))*qdot(1), cos(q(2))*(1/k(2)-1/k(3))*qdot(2), sin(q(3))/k(3)*qdot(3)];
    B = J_m0_P'*m(1)*J_m0_P + J_m1_P'*m(2)*J_m1_P + J_m2_P'*m(3)*J_m2_P;
    C = (Jdot_m0_P'*m(1)*J_m0_P + Jdot_m1_P'*m(2)*J_m1_P + Jdot_m2_P'*m(3)*J_m2_P)*qdot;
    g = [0; 0; 0];
end
