function G_p_mu = G_p_mu_fun(in1,in2,in3,in4,b_C,in6)
%G_P_MU_FUN
%    G_P_MU = G_P_MU_FUN(IN1,IN2,IN3,IN4,B_C,IN6)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    07-Sep-2021 10:55:58

A_p0 = in4(1,:);
A_p1 = in4(2,:);
A_p2 = in4(3,:);
A_p3 = in4(4,:);
A_p4 = in4(5,:);
A_p5 = in4(6,:);
d_Ca = in6(1,:);
d_Cb = in6(2,:);
l_p0 = in3(1,:);
l_p1 = in3(2,:);
l_p2 = in3(3,:);
l_p3 = in3(4,:);
l_p4 = in3(5,:);
l_p5 = in3(6,:);
mu_p0 = in2(1,:);
mu_p1 = in2(2,:);
mu_p2 = in2(3,:);
mu_p3 = in2(4,:);
mu_p4 = in2(5,:);
mu_p5 = in2(6,:);
q0 = in1(1,:);
q1 = in1(2,:);
q2 = in1(3,:);
mt1 = [(A_p0./(A_p0.*l_p0-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))-A_p0./(A_p0.*conj(mu_p0)+b_C.*(d_Ca.*(-1.1e+1./1.0e+2)+d_Cb.*(1.1e+1./1.0e+2)+(q0.*(d_Ca.^2-d_Cb.^2))./2.0))).*(A_p0.*l_p0.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5)];
mt2 = [(A_p1./(A_p1.*l_p1-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))-A_p1./(A_p1.*conj(mu_p1)-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)+(q0.*(d_Ca.^2-d_Cb.^2))./2.0))).*(A_p1.*l_p1.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5)];
mt3 = [(A_p2./(A_p2.*l_p2-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))-A_p2./(A_p2.*conj(mu_p2)+b_C.*(d_Ca.*(-1.1e+1./1.0e+2)+d_Cb.*(1.1e+1./1.0e+2)+(q1.*(d_Ca.^2-d_Cb.^2))./2.0))).*(A_p2.*l_p2.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5)];
mt4 = [(A_p3./(A_p3.*l_p3-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))-A_p3./(A_p3.*conj(mu_p3)-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)+(q1.*(d_Ca.^2-d_Cb.^2))./2.0))).*(A_p3.*l_p3.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5)];
mt5 = [(A_p4./(A_p4.*l_p4-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))-A_p4./(A_p4.*conj(mu_p4)+b_C.*(d_Ca.*(-1.1e+1./1.0e+2)+d_Cb.*(1.1e+1./1.0e+2)+(q2.*(d_Ca.^2-d_Cb.^2))./2.0))).*(A_p4.*l_p4.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5)];
mt6 = [(A_p5./(A_p5.*l_p5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))-A_p5./(A_p5.*conj(mu_p5)-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)+(q2.*(d_Ca.^2-d_Cb.^2))./2.0))).*(A_p5.*l_p5.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5)];
G_p_mu = reshape([mt1,mt2,mt3,mt4,mt5,mt6],6,1);
