function Gamma = Gamma_fun(in1,in2,in3,alpha,in5,in6,b_C,in8,in9,in10)
%GAMMA_FUN
%    GAMMA = GAMMA_FUN(IN1,IN2,IN3,ALPHA,IN5,IN6,B_C,IN8,IN9,IN10)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    07-Sep-2021 10:55:59

A_p0 = in6(1,:);
A_p1 = in6(2,:);
A_p2 = in6(3,:);
A_p3 = in6(4,:);
A_p4 = in6(5,:);
A_p5 = in6(6,:);
d_Ca = in8(1,:);
d_Cb = in8(2,:);
gx = in9(1,:);
gy = in9(2,:);
k0 = in10(1,:);
k1 = in10(2,:);
k2 = in10(3,:);
l_p0 = in5(1,:);
l_p1 = in5(2,:);
l_p2 = in5(3,:);
l_p3 = in5(4,:);
l_p4 = in5(5,:);
l_p5 = in5(6,:);
mu_p0_t0 = in3(1,:);
mu_p1_t0 = in3(2,:);
mu_p2_t0 = in3(3,:);
mu_p3_t0 = in3(4,:);
mu_p4_t0 = in3(5,:);
mu_p5_t0 = in3(6,:);
q0 = in1(1,:);
q1 = in1(2,:);
q2 = in1(3,:);
q_ref0 = in2(1,:);
q_ref1 = in2(2,:);
q_ref2 = in2(3,:);
et1 = k2.*q_ref2.*(-1.0./2.0)+gy.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3-gx.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3+gy.*1.0./q2.^2.*cos(alpha+q0+q1).*5.9895e-3-gx.*1.0./q2.^2.*sin(alpha+q0+q1).*5.9895e-3+gx.*1.0./q2.^3.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*1.1979e-2+gy.*1.0./q2.^3.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*1.1979e-2;
et2 = (A_p5.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p5-mu_p5_t0).*5.0e+6)./(A_p5.*mu_p5_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1);
et3 = (k2.*q_ref2)./2.0-gy.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3+gx.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3-gy.*1.0./q2.^2.*cos(alpha+q0+q1).*5.9895e-3+gx.*1.0./q2.^2.*sin(alpha+q0+q1).*5.9895e-3-gx.*1.0./q2.^3.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*1.1979e-2-gy.*1.0./q2.^3.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*1.1979e-2;
et4 = (A_p4.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p4-mu_p4_t0).*5.0e+6)./(A_p4.*mu_p4_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1);
et5 = gy.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4-gx.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4+gx.*q2.^2.*cos(alpha+q0).*2.3958e+4+gy.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gy.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4+gx.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gy.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gy.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et6 = gy.*q1.*q2.^2.*cos(alpha+q0).*2.3958e+4-gx.*q1.*q2.^2.*sin(alpha+q0).*2.3958e+4;
et7 = b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)+(q1.*(d_Ca.^2-d_Cb.^2))./2.0);
et8 = 1.0./(1.0./(A_p3.*l_p3-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+((k1.*q_ref1.*(-1.0./2.0)+(1.0./q1.^3.*1.0./q2.^2.*(et5+et6))./2.0e+6+(A_p3.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p3-mu_p3_t0).*5.0e+6)./(A_p3.*mu_p3_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1)).*2.0)./(b_C.*(A_p3.*l_p3.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2)));
et9 = gy.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4-gx.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4+gx.*q2.^2.*cos(alpha+q0).*2.3958e+4+gy.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gy.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4+gx.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gy.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gy.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et10 = gy.*q1.*q2.^2.*cos(alpha+q0).*2.3958e+4-gx.*q1.*q2.^2.*sin(alpha+q0).*2.3958e+4;
et11 = 1.0./(1.0./(A_p2.*l_p2-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+((k1.*q_ref1)./2.0-(1.0./q1.^3.*1.0./q2.^2.*(et9+et10))./2.0e+6+(A_p2.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p2-mu_p2_t0).*5.0e+6)./(A_p2.*mu_p2_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1))./(b_C.*(A_p2.*l_p2.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0)));
et12 = -b_C.*(d_Ca.*(-1.1e+1./1.0e+2)+d_Cb.*(1.1e+1./1.0e+2)+(q1.*(d_Ca.^2-d_Cb.^2))./2.0);
et13 = (k0.*q_ref0)./2.0+gx.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3+(gy.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3)./q1+(gx.*cos(alpha+q0).*1.1979e-2)./q0-(gx.*cos(alpha+q0).*5.9895e-3)./q1-gy.*1.0./q0.^2.*cos(alpha+q0).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha+q0).*5.9895e-3+(gy.*sin(alpha+q0).*1.1979e-2)./q0;
et14 = (gy.*sin(alpha+q0).*(-5.9895e-3))./q1-gy.*1.0./q0.^2.*cos(alpha).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha).*5.9895e-3-(gx.*cos(alpha+q0+q1).*5.9895e-3)./q2-(gy.*sin(alpha+q0+q1).*5.9895e-3)./q2+gx.*1.0./q0.^3.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2+gy.*1.0./q0.^2.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2;
et15 = gx.*1.0./q0.^2.*(sin(alpha+q0)-sin(alpha)).*(-1.1979e-2)+gy.*1.0./q0.^3.*(sin(alpha+q0)-sin(alpha)).*1.1979e-2+gy.*1.0./q2.^2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*5.9895e-3+(gx.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3)./q1-gy.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3-gx.*1.0./q2.^2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*5.9895e-3;
et16 = (A_p1.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p1-mu_p1_t0).*-5.0e+6)./(A_p1.*mu_p1_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1);
et17 = (k0.*q_ref0)./2.0+gx.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3+(gy.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3)./q1+(gx.*cos(alpha+q0).*1.1979e-2)./q0-(gx.*cos(alpha+q0).*5.9895e-3)./q1-gy.*1.0./q0.^2.*cos(alpha+q0).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha+q0).*5.9895e-3+(gy.*sin(alpha+q0).*1.1979e-2)./q0;
et18 = (gy.*sin(alpha+q0).*(-5.9895e-3))./q1-gy.*1.0./q0.^2.*cos(alpha).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha).*5.9895e-3-(gx.*cos(alpha+q0+q1).*5.9895e-3)./q2-(gy.*sin(alpha+q0+q1).*5.9895e-3)./q2+gx.*1.0./q0.^3.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2+gy.*1.0./q0.^2.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2;
et19 = gx.*1.0./q0.^2.*(sin(alpha+q0)-sin(alpha)).*(-1.1979e-2)+gy.*1.0./q0.^3.*(sin(alpha+q0)-sin(alpha)).*1.1979e-2+gy.*1.0./q2.^2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*5.9895e-3+(gx.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3)./q1-gy.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3-gx.*1.0./q2.^2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*5.9895e-3;
et20 = (A_p0.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p0-mu_p0_t0).*5.0e+6)./(A_p0.*mu_p0_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1);
mt1 = [-(b_C.*(d_Ca.*(-1.1e+1./1.0e+2)+d_Cb.*(1.1e+1./1.0e+2)+(q0.*(d_Ca.^2-d_Cb.^2))./2.0)-1.0./(1.0./(A_p0.*l_p0-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+(et17+et18+et19+et20)./(b_C.*(A_p0.*l_p0.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0))))./A_p0];
mt2 = [(1.0./(1.0./(A_p1.*l_p1-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))-((et13+et14+et15+et16).*2.0)./(b_C.*(A_p1.*l_p1.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2)))+b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)+(q0.*(d_Ca.^2-d_Cb.^2))./2.0))./A_p1,(et11+et12)./A_p2,(et7+et8)./A_p3];
mt3 = [-(b_C.*(d_Ca.*(-1.1e+1./1.0e+2)+d_Cb.*(1.1e+1./1.0e+2)+(q2.*(d_Ca.^2-d_Cb.^2))./2.0)-1.0./(1.0./(A_p4.*l_p4-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+(et3+et4)./(b_C.*(A_p4.*l_p4.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0))))./A_p4];
mt4 = [(1.0./(1.0./(A_p5.*l_p5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+((et1+et2).*2.0)./(b_C.*(A_p5.*l_p5.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2)))+b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)+(q2.*(d_Ca.^2-d_Cb.^2))./2.0))./A_p5];
Gamma = reshape([mt1,mt2,mt3,mt4],6,1);
