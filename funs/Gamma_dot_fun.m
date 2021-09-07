function Gamma_dot = Gamma_dot_fun(in1,in2,in3,in4,alpha,in6,in7,b_C,in9,in10,in11)
%GAMMA_DOT_FUN
%    GAMMA_DOT = GAMMA_DOT_FUN(IN1,IN2,IN3,IN4,ALPHA,IN6,IN7,B_C,IN9,IN10,IN11)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    07-Sep-2021 10:56:00

A_p0 = in7(1,:);
A_p1 = in7(2,:);
A_p2 = in7(3,:);
A_p3 = in7(4,:);
A_p4 = in7(5,:);
A_p5 = in7(6,:);
d_Ca = in9(1,:);
d_Cb = in9(2,:);
gx = in10(1,:);
gy = in10(2,:);
k0 = in11(1,:);
k1 = in11(2,:);
k2 = in11(3,:);
l_p0 = in6(1,:);
l_p1 = in6(2,:);
l_p2 = in6(3,:);
l_p3 = in6(4,:);
l_p4 = in6(5,:);
l_p5 = in6(6,:);
mu_p0_t0 = in4(1,:);
mu_p1_t0 = in4(2,:);
mu_p2_t0 = in4(3,:);
mu_p3_t0 = in4(4,:);
mu_p4_t0 = in4(5,:);
mu_p5_t0 = in4(6,:);
q0 = in1(1,:);
q1 = in1(2,:);
q2 = in1(3,:);
q_ref0 = in3(1,:);
q_ref1 = in3(2,:);
q_ref2 = in3(3,:);
qdot0 = in2(1,:);
qdot1 = in2(2,:);
qdot2 = in2(3,:);
et1 = k2.*q_ref2.*(-1.0./2.0)+gy.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3-gx.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3+gy.*1.0./q2.^2.*cos(alpha+q0+q1).*5.9895e-3-gx.*1.0./q2.^2.*sin(alpha+q0+q1).*5.9895e-3+gx.*1.0./q2.^3.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*1.1979e-2+gy.*1.0./q2.^3.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*1.1979e-2;
et2 = (A_p5.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p5-mu_p5_t0).*5.0e+6)./(A_p5.*mu_p5_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1);
et3 = gx.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3+gy.*1.0./q2.^3.*cos(alpha+q0+q1+q2).*2.3958e-2-gx.*1.0./q2.^3.*sin(alpha+q0+q1+q2).*2.3958e-2+gy.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3+gy.*1.0./q2.^3.*cos(alpha+q0+q1).*1.1979e-2-gx.*1.0./q2.^3.*sin(alpha+q0+q1).*1.1979e-2+gx.*1.0./q2.^4.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*3.5937e-2;
et4 = gy.*1.0./q2.^4.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*3.5937e-2;
et5 = k2.*q_ref2.*(-1.0./2.0)+gy.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3-gx.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3+gy.*1.0./q2.^2.*cos(alpha+q0+q1).*5.9895e-3-gx.*1.0./q2.^2.*sin(alpha+q0+q1).*5.9895e-3+gx.*1.0./q2.^3.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*1.1979e-2+gy.*1.0./q2.^3.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*1.1979e-2;
et6 = (A_p5.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p5-mu_p5_t0).*5.0e+6)./(A_p5.*mu_p5_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1);
et7 = (qdot0.*1.0./(1.0./(A_p5.*l_p5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+((et5+et6).*2.0)./(b_C.*(A_p5.*l_p5.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2))).^2)./(A_p5.*b_C.*(A_p5.*l_p5.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2));
et8 = (gx.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3+gy.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3+gx.*1.0./q2.^2.*cos(alpha+q0+q1).*5.9895e-3+gy.*1.0./q2.^2.*sin(alpha+q0+q1).*5.9895e-3-gy.*1.0./q2.^3.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*1.1979e-2+gx.*1.0./q2.^3.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*1.1979e-2).*2.0;
et9 = k2.*q_ref2.*(-1.0./2.0)+gy.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3-gx.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3+gy.*1.0./q2.^2.*cos(alpha+q0+q1).*5.9895e-3-gx.*1.0./q2.^2.*sin(alpha+q0+q1).*5.9895e-3+gx.*1.0./q2.^3.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*1.1979e-2+gy.*1.0./q2.^3.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*1.1979e-2;
et10 = (A_p5.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p5-mu_p5_t0).*5.0e+6)./(A_p5.*mu_p5_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1);
et11 = (qdot1.*1.0./(1.0./(A_p5.*l_p5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+((et9+et10).*2.0)./(b_C.*(A_p5.*l_p5.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2))).^2)./(A_p5.*b_C.*(A_p5.*l_p5.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2));
et12 = (gx.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3+gy.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3+gx.*1.0./q2.^2.*cos(alpha+q0+q1).*5.9895e-3+gy.*1.0./q2.^2.*sin(alpha+q0+q1).*5.9895e-3-gy.*1.0./q2.^3.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*1.1979e-2+gx.*1.0./q2.^3.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*1.1979e-2).*2.0;
et13 = (k2.*q_ref2)./2.0-gy.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3+gx.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3-gy.*1.0./q2.^2.*cos(alpha+q0+q1).*5.9895e-3+gx.*1.0./q2.^2.*sin(alpha+q0+q1).*5.9895e-3-gx.*1.0./q2.^3.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*1.1979e-2-gy.*1.0./q2.^3.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*1.1979e-2;
et14 = (A_p4.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p4-mu_p4_t0).*5.0e+6)./(A_p4.*mu_p4_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1);
et15 = gx.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3+gy.*1.0./q2.^3.*cos(alpha+q0+q1+q2).*2.3958e-2-gx.*1.0./q2.^3.*sin(alpha+q0+q1+q2).*2.3958e-2+gy.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3+gy.*1.0./q2.^3.*cos(alpha+q0+q1).*1.1979e-2-gx.*1.0./q2.^3.*sin(alpha+q0+q1).*1.1979e-2+gx.*1.0./q2.^4.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*3.5937e-2;
et16 = gy.*1.0./q2.^4.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*3.5937e-2;
et17 = (k2.*q_ref2)./2.0-gy.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3+gx.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3-gy.*1.0./q2.^2.*cos(alpha+q0+q1).*5.9895e-3+gx.*1.0./q2.^2.*sin(alpha+q0+q1).*5.9895e-3-gx.*1.0./q2.^3.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*1.1979e-2-gy.*1.0./q2.^3.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*1.1979e-2;
et18 = (A_p4.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p4-mu_p4_t0).*5.0e+6)./(A_p4.*mu_p4_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1);
et19 = (qdot0.*1.0./(1.0./(A_p4.*l_p4-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+(et17+et18)./(b_C.*(A_p4.*l_p4.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0))).^2)./(A_p4.*b_C.*(A_p4.*l_p4.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0));
et20 = -(gx.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3+gy.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3+gx.*1.0./q2.^2.*cos(alpha+q0+q1).*5.9895e-3+gy.*1.0./q2.^2.*sin(alpha+q0+q1).*5.9895e-3-gy.*1.0./q2.^3.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*1.1979e-2+gx.*1.0./q2.^3.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*1.1979e-2);
et21 = (k2.*q_ref2)./2.0-gy.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3+gx.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3-gy.*1.0./q2.^2.*cos(alpha+q0+q1).*5.9895e-3+gx.*1.0./q2.^2.*sin(alpha+q0+q1).*5.9895e-3-gx.*1.0./q2.^3.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*1.1979e-2-gy.*1.0./q2.^3.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*1.1979e-2;
et22 = (A_p4.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p4-mu_p4_t0).*5.0e+6)./(A_p4.*mu_p4_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1);
et23 = (qdot1.*1.0./(1.0./(A_p4.*l_p4-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+(et21+et22)./(b_C.*(A_p4.*l_p4.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0))).^2)./(A_p4.*b_C.*(A_p4.*l_p4.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0));
et24 = -(gx.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3+gy.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3+gx.*1.0./q2.^2.*cos(alpha+q0+q1).*5.9895e-3+gy.*1.0./q2.^2.*sin(alpha+q0+q1).*5.9895e-3-gy.*1.0./q2.^3.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*1.1979e-2+gx.*1.0./q2.^3.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*1.1979e-2);
et25 = gy.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4-gx.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4+gx.*q2.^2.*cos(alpha+q0).*2.3958e+4+gy.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gy.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4+gx.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gy.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gy.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et26 = gy.*q1.*q2.^2.*cos(alpha+q0).*2.3958e+4-gx.*q1.*q2.^2.*sin(alpha+q0).*2.3958e+4;
et27 = gx.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4-gy.*q1.^2.*cos(alpha+q0+q1+q2).*3.5937e+4+gx.*q1.^2.*sin(alpha+q0+q1+q2).*3.5937e+4+gy.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4-gy.*q2.^2.*cos(alpha+q0).*2.3958e+4+gx.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gy.*q1.^2.*cos(alpha+q0+q1).*3.5937e+4+gy.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gx.*q1.^2.*sin(alpha+q0+q1).*3.5937e+4-gx.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4;
et28 = gx.*q1.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gx.*q1.^2.*q2.*cos(alpha+q0+q1).*3.5937e+4-gy.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4+gy.*q1.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4-gy.*q1.^2.*q2.*sin(alpha+q0+q1).*3.5937e+4+gy.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et29 = gy.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4-gx.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4+gx.*q2.^2.*cos(alpha+q0).*2.3958e+4+gy.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gy.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4+gx.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gy.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gy.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et30 = gy.*q1.*q2.^2.*cos(alpha+q0).*2.3958e+4-gx.*q1.*q2.^2.*sin(alpha+q0).*2.3958e+4;
et31 = 1.0./(b_C.*(A_p3.*l_p3.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2));
et32 = 1.0./(1.0./(A_p3.*l_p3-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+((k1.*q_ref1.*(-1.0./2.0)+(1.0./q1.^3.*1.0./q2.^2.*(et25+et26))./2.0e+6+(A_p3.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p3-mu_p3_t0).*5.0e+6)./(A_p3.*mu_p3_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1)).*2.0)./(b_C.*(A_p3.*l_p3.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2))).^2;
et33 = ((1.0./q1.^3.*1.0./q2.^2.*(et27+et28))./2.0e+6+1.0./q1.^4.*1.0./q2.^2.*(et29+et30).*1.5e-6).*2.0;
et34 = gy.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4-gx.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4+gx.*q2.^2.*cos(alpha+q0).*2.3958e+4+gy.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gy.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4+gx.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gy.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gy.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et35 = gy.*q1.*q2.^2.*cos(alpha+q0).*2.3958e+4-gx.*q1.*q2.^2.*sin(alpha+q0).*2.3958e+4;
et36 = (1.0./q1.^3.*1.0./q2.^2.*(gx.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4+gy.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4+gx.*q2.*cos(alpha+q0+q1).*4.7916e+4+gy.*q2.*sin(alpha+q0+q1).*4.7916e+4-gx.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4-gy.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gx.*q2.*cos(alpha+q0).*4.7916e+4-gy.*q2.*sin(alpha+q0).*4.7916e+4+gx.*q1.^2.*q2.*cos(alpha+q0+q1).*2.3958e+4+gy.*q1.^2.*q2.*sin(alpha+q0+q1).*2.3958e+4-gy.*q1.*q2.*cos(alpha+q0).*4.7916e+4+gx.*q1.*q2.*sin(alpha+q0).*4.7916e+4))./2.0e+6;
et37 = (1.0./q1.^3.*1.0./q2.^3.*(et34+et35))./1.0e+6;
et38 = gy.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4-gx.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4+gx.*q2.^2.*cos(alpha+q0).*2.3958e+4+gy.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gy.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4+gx.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gy.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gy.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et39 = gy.*q1.*q2.^2.*cos(alpha+q0).*2.3958e+4-gx.*q1.*q2.^2.*sin(alpha+q0).*2.3958e+4;
et40 = (qdot2.*(et36+et37))./(A_p3.*b_C.*(A_p3.*l_p3.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2));
et41 = 1.0./(1.0./(A_p3.*l_p3-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+((k1.*q_ref1.*(-1.0./2.0)+(1.0./q1.^3.*1.0./q2.^2.*(et38+et39))./2.0e+6+(A_p3.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p3-mu_p3_t0).*5.0e+6)./(A_p3.*mu_p3_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1)).*2.0)./(b_C.*(A_p3.*l_p3.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2))).^2.*2.0;
et42 = gy.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4-gx.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4+gx.*q2.^2.*cos(alpha+q0).*2.3958e+4+gy.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gy.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4+gx.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gy.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gy.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et43 = gy.*q1.*q2.^2.*cos(alpha+q0).*2.3958e+4-gx.*q1.*q2.^2.*sin(alpha+q0).*2.3958e+4;
et44 = gx.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4+gy.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4-gy.*q2.^2.*cos(alpha+q0).*2.3958e+4+gx.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gy.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gx.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gy.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4+gy.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et45 = gx.*q1.*q2.^2.*cos(alpha+q0).*2.3958e+4+gy.*q1.*q2.^2.*sin(alpha+q0).*2.3958e+4;
et46 = (1.0./q1.^3.*1.0./q2.^2.*qdot0)./(A_p3.*b_C.*(A_p3.*l_p3.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2));
et47 = (1.0./(1.0./(A_p3.*l_p3-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+((k1.*q_ref1.*(-1.0./2.0)+(1.0./q1.^3.*1.0./q2.^2.*(et42+et43))./2.0e+6+(A_p3.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p3-mu_p3_t0).*5.0e+6)./(A_p3.*mu_p3_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1)).*2.0)./(b_C.*(A_p3.*l_p3.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2))).^2.*(et44+et45))./1.0e+6;
et48 = gx.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4-gy.*q1.^2.*cos(alpha+q0+q1+q2).*3.5937e+4+gx.*q1.^2.*sin(alpha+q0+q1+q2).*3.5937e+4+gy.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4-gy.*q2.^2.*cos(alpha+q0).*2.3958e+4+gx.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gy.*q1.^2.*cos(alpha+q0+q1).*3.5937e+4+gy.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gx.*q1.^2.*sin(alpha+q0+q1).*3.5937e+4-gx.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4;
et49 = gx.*q1.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gx.*q1.^2.*q2.*cos(alpha+q0+q1).*3.5937e+4-gy.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4+gy.*q1.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4-gy.*q1.^2.*q2.*sin(alpha+q0+q1).*3.5937e+4+gy.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et50 = gy.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4-gx.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4+gx.*q2.^2.*cos(alpha+q0).*2.3958e+4+gy.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gy.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4+gx.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gy.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gy.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et51 = gy.*q1.*q2.^2.*cos(alpha+q0).*2.3958e+4-gx.*q1.*q2.^2.*sin(alpha+q0).*2.3958e+4;
et52 = gy.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4-gx.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4+gx.*q2.^2.*cos(alpha+q0).*2.3958e+4+gy.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gy.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4+gx.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gy.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gy.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et53 = gy.*q1.*q2.^2.*cos(alpha+q0).*2.3958e+4-gx.*q1.*q2.^2.*sin(alpha+q0).*2.3958e+4;
et54 = ((1.0./q1.^3.*1.0./q2.^2.*(et48+et49))./2.0e+6+1.0./q1.^4.*1.0./q2.^2.*(et50+et51).*1.5e-6)./(b_C.*(A_p2.*l_p2.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0));
et55 = 1.0./(1.0./(A_p2.*l_p2-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+((k1.*q_ref1)./2.0-(1.0./q1.^3.*1.0./q2.^2.*(et52+et53))./2.0e+6+(A_p2.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p2-mu_p2_t0).*5.0e+6)./(A_p2.*mu_p2_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1))./(b_C.*(A_p2.*l_p2.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0))).^2;
et56 = gy.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4-gx.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4+gx.*q2.^2.*cos(alpha+q0).*2.3958e+4+gy.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gy.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4+gx.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gy.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gy.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et57 = gy.*q1.*q2.^2.*cos(alpha+q0).*2.3958e+4-gx.*q1.*q2.^2.*sin(alpha+q0).*2.3958e+4;
et58 = (1.0./q1.^3.*1.0./q2.^2.*(gx.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4+gy.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4+gx.*q2.*cos(alpha+q0+q1).*4.7916e+4+gy.*q2.*sin(alpha+q0+q1).*4.7916e+4-gx.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4-gy.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gx.*q2.*cos(alpha+q0).*4.7916e+4-gy.*q2.*sin(alpha+q0).*4.7916e+4+gx.*q1.^2.*q2.*cos(alpha+q0+q1).*2.3958e+4+gy.*q1.^2.*q2.*sin(alpha+q0+q1).*2.3958e+4-gy.*q1.*q2.*cos(alpha+q0).*4.7916e+4+gx.*q1.*q2.*sin(alpha+q0).*4.7916e+4))./2.0e+6;
et59 = (1.0./q1.^3.*1.0./q2.^3.*(et56+et57))./1.0e+6;
et60 = gy.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4-gx.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4+gx.*q2.^2.*cos(alpha+q0).*2.3958e+4+gy.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gy.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4+gx.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gy.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gy.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et61 = gy.*q1.*q2.^2.*cos(alpha+q0).*2.3958e+4-gx.*q1.*q2.^2.*sin(alpha+q0).*2.3958e+4;
et62 = (qdot2.*(et58+et59))./(A_p2.*b_C.*(A_p2.*l_p2.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0));
et63 = -1.0./(1.0./(A_p2.*l_p2-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+((k1.*q_ref1)./2.0-(1.0./q1.^3.*1.0./q2.^2.*(et60+et61))./2.0e+6+(A_p2.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p2-mu_p2_t0).*5.0e+6)./(A_p2.*mu_p2_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1))./(b_C.*(A_p2.*l_p2.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0))).^2;
et64 = gy.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4-gx.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4+gx.*q2.^2.*cos(alpha+q0).*2.3958e+4+gy.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gy.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4+gx.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gy.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gy.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et65 = gy.*q1.*q2.^2.*cos(alpha+q0).*2.3958e+4-gx.*q1.*q2.^2.*sin(alpha+q0).*2.3958e+4;
et66 = gx.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4+gy.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4-gy.*q2.^2.*cos(alpha+q0).*2.3958e+4+gx.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gy.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gx.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gy.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4+gy.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et67 = gx.*q1.*q2.^2.*cos(alpha+q0).*2.3958e+4+gy.*q1.*q2.^2.*sin(alpha+q0).*2.3958e+4;
et68 = (1.0./q1.^3.*1.0./q2.^2.*qdot0)./(A_p2.*b_C.*(A_p2.*l_p2.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0));
et69 = 1.0./(1.0./(A_p2.*l_p2-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+((k1.*q_ref1)./2.0-(1.0./q1.^3.*1.0./q2.^2.*(et64+et65))./2.0e+6+(A_p2.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p2-mu_p2_t0).*5.0e+6)./(A_p2.*mu_p2_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1))./(b_C.*(A_p2.*l_p2.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0))).^2.*(et66+et67).*(-5.0e-7);
et70 = (k0.*q_ref0)./2.0+gx.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3+(gy.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3)./q1+(gx.*cos(alpha+q0).*1.1979e-2)./q0-(gx.*cos(alpha+q0).*5.9895e-3)./q1-gy.*1.0./q0.^2.*cos(alpha+q0).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha+q0).*5.9895e-3+(gy.*sin(alpha+q0).*1.1979e-2)./q0;
et71 = (gy.*sin(alpha+q0).*(-5.9895e-3))./q1-gy.*1.0./q0.^2.*cos(alpha).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha).*5.9895e-3-(gx.*cos(alpha+q0+q1).*5.9895e-3)./q2-(gy.*sin(alpha+q0+q1).*5.9895e-3)./q2+gx.*1.0./q0.^3.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2+gy.*1.0./q0.^2.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2;
et72 = gx.*1.0./q0.^2.*(sin(alpha+q0)-sin(alpha)).*(-1.1979e-2)+gy.*1.0./q0.^3.*(sin(alpha+q0)-sin(alpha)).*1.1979e-2+gy.*1.0./q2.^2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*5.9895e-3+(gx.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3)./q1-gy.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3-gx.*1.0./q2.^2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*5.9895e-3;
et73 = (A_p1.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p1-mu_p1_t0).*-5.0e+6)./(A_p1.*mu_p1_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1);
et74 = (gx.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3)./q1-gy.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3+gx.*1.0./q0.^2.*cos(alpha+q0).*1.79685e-2-(gy.*cos(alpha+q0).*1.1979e-2)./q0+(gy.*cos(alpha+q0).*5.9895e-3)./q1-gy.*1.0./q0.^3.*cos(alpha+q0).*2.3958e-2+(gx.*sin(alpha+q0).*1.1979e-2)./q0;
et75 = (gx.*sin(alpha+q0).*(-5.9895e-3))./q1+gx.*1.0./q0.^3.*sin(alpha+q0).*2.3958e-2+gy.*1.0./q0.^2.*sin(alpha+q0).*1.79685e-2-gy.*1.0./q0.^3.*cos(alpha).*1.1979e-2+gx.*1.0./q0.^3.*sin(alpha).*1.1979e-2+(gy.*cos(alpha+q0+q1).*5.9895e-3)./q2-(gx.*sin(alpha+q0+q1).*5.9895e-3)./q2+gx.*1.0./q0.^4.*(cos(alpha+q0)-cos(alpha)).*3.5937e-2;
et76 = gy.*1.0./q0.^3.*(cos(alpha+q0)-cos(alpha)).*2.3958e-2-gx.*1.0./q0.^3.*(sin(alpha+q0)-sin(alpha)).*2.3958e-2+gy.*1.0./q0.^4.*(sin(alpha+q0)-sin(alpha)).*3.5937e-2+gx.*1.0./q2.^2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*5.9895e-3-gx.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3-(gy.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3)./q1;
et77 = gy.*1.0./q2.^2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*5.9895e-3;
et78 = (k0.*q_ref0)./2.0+gx.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3+(gy.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3)./q1+(gx.*cos(alpha+q0).*1.1979e-2)./q0-(gx.*cos(alpha+q0).*5.9895e-3)./q1-gy.*1.0./q0.^2.*cos(alpha+q0).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha+q0).*5.9895e-3+(gy.*sin(alpha+q0).*1.1979e-2)./q0;
et79 = (gy.*sin(alpha+q0).*(-5.9895e-3))./q1-gy.*1.0./q0.^2.*cos(alpha).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha).*5.9895e-3-(gx.*cos(alpha+q0+q1).*5.9895e-3)./q2-(gy.*sin(alpha+q0+q1).*5.9895e-3)./q2+gx.*1.0./q0.^3.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2+gy.*1.0./q0.^2.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2;
et80 = gx.*1.0./q0.^2.*(sin(alpha+q0)-sin(alpha)).*(-1.1979e-2)+gy.*1.0./q0.^3.*(sin(alpha+q0)-sin(alpha)).*1.1979e-2+gy.*1.0./q2.^2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*5.9895e-3+(gx.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3)./q1-gy.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3-gx.*1.0./q2.^2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*5.9895e-3;
et81 = (A_p1.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p1-mu_p1_t0).*-5.0e+6)./(A_p1.*mu_p1_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1);
et82 = (qdot2.*1.0./(1.0./(A_p1.*l_p1-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))-((et78+et79+et80+et81).*2.0)./(b_C.*(A_p1.*l_p1.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2))).^2)./(A_p1.*b_C.*(A_p1.*l_p1.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2));
et83 = (gx.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3+gy.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3+gx.*1.0./q2.^2.*cos(alpha+q0+q1).*5.9895e-3+gy.*1.0./q2.^2.*sin(alpha+q0+q1).*5.9895e-3-gy.*1.0./q2.^3.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*1.1979e-2+gx.*1.0./q2.^3.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*1.1979e-2).*2.0;
et84 = (k0.*q_ref0)./2.0+gx.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3+(gy.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3)./q1+(gx.*cos(alpha+q0).*1.1979e-2)./q0-(gx.*cos(alpha+q0).*5.9895e-3)./q1-gy.*1.0./q0.^2.*cos(alpha+q0).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha+q0).*5.9895e-3+(gy.*sin(alpha+q0).*1.1979e-2)./q0;
et85 = (gy.*sin(alpha+q0).*(-5.9895e-3))./q1-gy.*1.0./q0.^2.*cos(alpha).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha).*5.9895e-3-(gx.*cos(alpha+q0+q1).*5.9895e-3)./q2-(gy.*sin(alpha+q0+q1).*5.9895e-3)./q2+gx.*1.0./q0.^3.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2+gy.*1.0./q0.^2.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2;
et86 = gx.*1.0./q0.^2.*(sin(alpha+q0)-sin(alpha)).*(-1.1979e-2)+gy.*1.0./q0.^3.*(sin(alpha+q0)-sin(alpha)).*1.1979e-2+gy.*1.0./q2.^2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*5.9895e-3+(gx.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3)./q1-gy.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3-gx.*1.0./q2.^2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*5.9895e-3;
et87 = (A_p1.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p1-mu_p1_t0).*-5.0e+6)./(A_p1.*mu_p1_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1);
et88 = gx.*1.0./q1.^3.*(sin(alpha+q0+q1)-sin(alpha+q0)).*1.1979e-2+gy.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3-gx.*1.0./q1.^2.*cos(alpha+q0).*5.9895e-3-gy.*1.0./q1.^2.*sin(alpha+q0).*5.9895e-3-gx.*1.0./q1.^2.*cos(alpha+q0+q1).*5.9895e-3-(gy.*cos(alpha+q0+q1).*5.9895e-3)./q1+(gy.*cos(alpha+q0+q1).*5.9895e-3)./q2;
et89 = (gx.*sin(alpha+q0+q1).*5.9895e-3)./q1-(gx.*sin(alpha+q0+q1).*5.9895e-3)./q2-gy.*1.0./q1.^2.*sin(alpha+q0+q1).*5.9895e-3+gx.*1.0./q2.^2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*5.9895e-3+gx.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3-gy.*1.0./q1.^3.*(cos(alpha+q0+q1)-cos(alpha+q0)).*1.1979e-2;
et90 = gy.*1.0./q2.^2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*5.9895e-3;
et91 = (qdot0.*(b_C.*(d_Ca.^2./2.0-d_Cb.^2./2.0)-(1.0./(1.0./(A_p1.*l_p1-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))-((et70+et71+et72+et73).*2.0)./(b_C.*(A_p1.*l_p1.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2))).^2.*(et74+et75+et76+et77).*2.0)./(b_C.*(A_p1.*l_p1.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2))))./A_p1+et82.*et83;
et92 = (qdot1.*1.0./(1.0./(A_p1.*l_p1-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))-((et84+et85+et86+et87).*2.0)./(b_C.*(A_p1.*l_p1.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2))).^2.*(et88+et89+et90).*-2.0)./(A_p1.*b_C.*(A_p1.*l_p1.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2));
et93 = (k0.*q_ref0)./2.0+gx.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3+(gy.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3)./q1+(gx.*cos(alpha+q0).*1.1979e-2)./q0-(gx.*cos(alpha+q0).*5.9895e-3)./q1-gy.*1.0./q0.^2.*cos(alpha+q0).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha+q0).*5.9895e-3+(gy.*sin(alpha+q0).*1.1979e-2)./q0;
et94 = (gy.*sin(alpha+q0).*(-5.9895e-3))./q1-gy.*1.0./q0.^2.*cos(alpha).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha).*5.9895e-3-(gx.*cos(alpha+q0+q1).*5.9895e-3)./q2-(gy.*sin(alpha+q0+q1).*5.9895e-3)./q2+gx.*1.0./q0.^3.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2+gy.*1.0./q0.^2.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2;
et95 = gx.*1.0./q0.^2.*(sin(alpha+q0)-sin(alpha)).*(-1.1979e-2)+gy.*1.0./q0.^3.*(sin(alpha+q0)-sin(alpha)).*1.1979e-2+gy.*1.0./q2.^2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*5.9895e-3+(gx.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3)./q1-gy.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3-gx.*1.0./q2.^2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*5.9895e-3;
et96 = (A_p0.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p0-mu_p0_t0).*5.0e+6)./(A_p0.*mu_p0_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1);
et97 = (gx.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3)./q1-gy.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3+gx.*1.0./q0.^2.*cos(alpha+q0).*1.79685e-2-(gy.*cos(alpha+q0).*1.1979e-2)./q0+(gy.*cos(alpha+q0).*5.9895e-3)./q1-gy.*1.0./q0.^3.*cos(alpha+q0).*2.3958e-2+(gx.*sin(alpha+q0).*1.1979e-2)./q0;
et98 = (gx.*sin(alpha+q0).*(-5.9895e-3))./q1+gx.*1.0./q0.^3.*sin(alpha+q0).*2.3958e-2+gy.*1.0./q0.^2.*sin(alpha+q0).*1.79685e-2-gy.*1.0./q0.^3.*cos(alpha).*1.1979e-2+gx.*1.0./q0.^3.*sin(alpha).*1.1979e-2+(gy.*cos(alpha+q0+q1).*5.9895e-3)./q2-(gx.*sin(alpha+q0+q1).*5.9895e-3)./q2+gx.*1.0./q0.^4.*(cos(alpha+q0)-cos(alpha)).*3.5937e-2;
et99 = gy.*1.0./q0.^3.*(cos(alpha+q0)-cos(alpha)).*2.3958e-2-gx.*1.0./q0.^3.*(sin(alpha+q0)-sin(alpha)).*2.3958e-2+gy.*1.0./q0.^4.*(sin(alpha+q0)-sin(alpha)).*3.5937e-2+gx.*1.0./q2.^2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*5.9895e-3-gx.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3-(gy.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3)./q1;
et100 = gy.*1.0./q2.^2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*5.9895e-3;
et101 = (k0.*q_ref0)./2.0+gx.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3+(gy.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3)./q1+(gx.*cos(alpha+q0).*1.1979e-2)./q0-(gx.*cos(alpha+q0).*5.9895e-3)./q1-gy.*1.0./q0.^2.*cos(alpha+q0).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha+q0).*5.9895e-3+(gy.*sin(alpha+q0).*1.1979e-2)./q0;
et102 = (gy.*sin(alpha+q0).*(-5.9895e-3))./q1-gy.*1.0./q0.^2.*cos(alpha).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha).*5.9895e-3-(gx.*cos(alpha+q0+q1).*5.9895e-3)./q2-(gy.*sin(alpha+q0+q1).*5.9895e-3)./q2+gx.*1.0./q0.^3.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2+gy.*1.0./q0.^2.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2;
et103 = gx.*1.0./q0.^2.*(sin(alpha+q0)-sin(alpha)).*(-1.1979e-2)+gy.*1.0./q0.^3.*(sin(alpha+q0)-sin(alpha)).*1.1979e-2+gy.*1.0./q2.^2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*5.9895e-3+(gx.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3)./q1-gy.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3-gx.*1.0./q2.^2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*5.9895e-3;
et104 = (A_p0.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p0-mu_p0_t0).*5.0e+6)./(A_p0.*mu_p0_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1);
et105 = gx.*1.0./q1.^3.*(sin(alpha+q0+q1)-sin(alpha+q0)).*1.1979e-2+gy.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3-gx.*1.0./q1.^2.*cos(alpha+q0).*5.9895e-3-gy.*1.0./q1.^2.*sin(alpha+q0).*5.9895e-3-gx.*1.0./q1.^2.*cos(alpha+q0+q1).*5.9895e-3-(gy.*cos(alpha+q0+q1).*5.9895e-3)./q1+(gy.*cos(alpha+q0+q1).*5.9895e-3)./q2;
et106 = (gx.*sin(alpha+q0+q1).*5.9895e-3)./q1-(gx.*sin(alpha+q0+q1).*5.9895e-3)./q2-gy.*1.0./q1.^2.*sin(alpha+q0+q1).*5.9895e-3+gx.*1.0./q2.^2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*5.9895e-3+gx.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3-gy.*1.0./q1.^3.*(cos(alpha+q0+q1)-cos(alpha+q0)).*1.1979e-2;
et107 = gy.*1.0./q2.^2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*5.9895e-3;
et108 = (k0.*q_ref0)./2.0+gx.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3+(gy.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3)./q1+(gx.*cos(alpha+q0).*1.1979e-2)./q0-(gx.*cos(alpha+q0).*5.9895e-3)./q1-gy.*1.0./q0.^2.*cos(alpha+q0).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha+q0).*5.9895e-3+(gy.*sin(alpha+q0).*1.1979e-2)./q0;
et109 = (gy.*sin(alpha+q0).*(-5.9895e-3))./q1-gy.*1.0./q0.^2.*cos(alpha).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha).*5.9895e-3-(gx.*cos(alpha+q0+q1).*5.9895e-3)./q2-(gy.*sin(alpha+q0+q1).*5.9895e-3)./q2+gx.*1.0./q0.^3.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2+gy.*1.0./q0.^2.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2;
et110 = gx.*1.0./q0.^2.*(sin(alpha+q0)-sin(alpha)).*(-1.1979e-2)+gy.*1.0./q0.^3.*(sin(alpha+q0)-sin(alpha)).*1.1979e-2+gy.*1.0./q2.^2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*5.9895e-3+(gx.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3)./q1-gy.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3-gx.*1.0./q2.^2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*5.9895e-3;
et111 = (A_p0.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p0-mu_p0_t0).*5.0e+6)./(A_p0.*mu_p0_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1);
et112 = (qdot2.*1.0./(1.0./(A_p0.*l_p0-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+(et108+et109+et110+et111)./(b_C.*(A_p0.*l_p0.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0))).^2)./(A_p0.*b_C.*(A_p0.*l_p0.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0));
et113 = -(gx.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3+gy.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3+gx.*1.0./q2.^2.*cos(alpha+q0+q1).*5.9895e-3+gy.*1.0./q2.^2.*sin(alpha+q0+q1).*5.9895e-3-gy.*1.0./q2.^3.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*1.1979e-2+gx.*1.0./q2.^3.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*1.1979e-2);
et114 = -(qdot0.*(b_C.*(d_Ca.^2./2.0-d_Cb.^2./2.0)-(1.0./(1.0./(A_p0.*l_p0-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+(et93+et94+et95+et96)./(b_C.*(A_p0.*l_p0.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0))).^2.*(et97+et98+et99+et100))./(b_C.*(A_p0.*l_p0.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0))))./A_p0;
et115 = (qdot1.*1.0./(1.0./(A_p0.*l_p0-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+(et101+et102+et103+et104)./(b_C.*(A_p0.*l_p0.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0))).^2.*(et105+et106+et107))./(A_p0.*b_C.*(A_p0.*l_p0.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0))+et112.*et113;
mt1 = [et114+et115,et91+et92,et62.*et63+et68.*et69-(qdot1.*(et54.*et55+b_C.*(d_Ca.^2./2.0-d_Cb.^2./2.0)))./A_p2,et40.*et41+et46.*et47+(qdot1.*(b_C.*(d_Ca.^2./2.0-d_Cb.^2./2.0)+et31.*et32.*et33))./A_p3];
mt2 = [et19.*et20+et23.*et24-(qdot2.*(b_C.*(d_Ca.^2./2.0-d_Cb.^2./2.0)+((et15+et16).*1.0./(1.0./(A_p4.*l_p4-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+(et13+et14)./(b_C.*(A_p4.*l_p4.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0))).^2)./(b_C.*(A_p4.*l_p4.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0))))./A_p4];
mt3 = [et7.*et8+et11.*et12+(qdot2.*(b_C.*(d_Ca.^2./2.0-d_Cb.^2./2.0)+((et3+et4).*1.0./(1.0./(A_p5.*l_p5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+((et1+et2).*2.0)./(b_C.*(A_p5.*l_p5.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2))).^2.*2.0)./(b_C.*(A_p5.*l_p5.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2))))./A_p5];
Gamma_dot = reshape([mt1,mt2,mt3],6,1);
