function dGamma_dq = dGamma_dq_fun(in1,in2,in3,alpha,in5,in6,b_C,in8,in9,in10)
%DGAMMA_DQ_FUN
%    DGAMMA_DQ = DGAMMA_DQ_FUN(IN1,IN2,IN3,ALPHA,IN5,IN6,B_C,IN8,IN9,IN10)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    07-Sep-2021 10:56:00

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
et3 = 1.0./(1.0./(A_p5.*l_p5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+((et1+et2).*2.0)./(b_C.*(A_p5.*l_p5.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2))).^2./(A_p5.*b_C.*(A_p5.*l_p5.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2));
et4 = (gx.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3+gy.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3+gx.*1.0./q2.^2.*cos(alpha+q0+q1).*5.9895e-3+gy.*1.0./q2.^2.*sin(alpha+q0+q1).*5.9895e-3-gy.*1.0./q2.^3.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*1.1979e-2+gx.*1.0./q2.^3.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*1.1979e-2).*2.0;
et5 = (k2.*q_ref2)./2.0-gy.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3+gx.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3-gy.*1.0./q2.^2.*cos(alpha+q0+q1).*5.9895e-3+gx.*1.0./q2.^2.*sin(alpha+q0+q1).*5.9895e-3-gx.*1.0./q2.^3.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*1.1979e-2-gy.*1.0./q2.^3.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*1.1979e-2;
et6 = (A_p4.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p4-mu_p4_t0).*5.0e+6)./(A_p4.*mu_p4_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1);
et7 = 1.0./(1.0./(A_p4.*l_p4-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+(et5+et6)./(b_C.*(A_p4.*l_p4.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0))).^2./(A_p4.*b_C.*(A_p4.*l_p4.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0));
et8 = -(gx.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3+gy.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3+gx.*1.0./q2.^2.*cos(alpha+q0+q1).*5.9895e-3+gy.*1.0./q2.^2.*sin(alpha+q0+q1).*5.9895e-3-gy.*1.0./q2.^3.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*1.1979e-2+gx.*1.0./q2.^3.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*1.1979e-2);
et9 = gy.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4-gx.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4+gx.*q2.^2.*cos(alpha+q0).*2.3958e+4+gy.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gy.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4+gx.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gy.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gy.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et10 = gy.*q1.*q2.^2.*cos(alpha+q0).*2.3958e+4-gx.*q1.*q2.^2.*sin(alpha+q0).*2.3958e+4;
et11 = gx.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4+gy.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4-gy.*q2.^2.*cos(alpha+q0).*2.3958e+4+gx.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gy.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gx.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gy.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4+gy.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et12 = gx.*q1.*q2.^2.*cos(alpha+q0).*2.3958e+4+gy.*q1.*q2.^2.*sin(alpha+q0).*2.3958e+4;
et13 = (1.0./q1.^3.*1.0./q2.^2)./(A_p3.*b_C.*(A_p3.*l_p3.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2));
et14 = (1.0./(1.0./(A_p3.*l_p3-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+((k1.*q_ref1.*(-1.0./2.0)+(1.0./q1.^3.*1.0./q2.^2.*(et9+et10))./2.0e+6+(A_p3.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p3-mu_p3_t0).*5.0e+6)./(A_p3.*mu_p3_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1)).*2.0)./(b_C.*(A_p3.*l_p3.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2))).^2.*(et11+et12))./1.0e+6;
et15 = gy.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4-gx.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4+gx.*q2.^2.*cos(alpha+q0).*2.3958e+4+gy.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gy.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4+gx.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gy.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gy.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et16 = gy.*q1.*q2.^2.*cos(alpha+q0).*2.3958e+4-gx.*q1.*q2.^2.*sin(alpha+q0).*2.3958e+4;
et17 = gx.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4+gy.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4-gy.*q2.^2.*cos(alpha+q0).*2.3958e+4+gx.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gy.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gx.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gy.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4+gy.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et18 = gx.*q1.*q2.^2.*cos(alpha+q0).*2.3958e+4+gy.*q1.*q2.^2.*sin(alpha+q0).*2.3958e+4;
et19 = (1.0./q1.^3.*1.0./q2.^2)./(A_p2.*b_C.*(A_p2.*l_p2.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0));
et20 = 1.0./(1.0./(A_p2.*l_p2-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+((k1.*q_ref1)./2.0-(1.0./q1.^3.*1.0./q2.^2.*(et15+et16))./2.0e+6+(A_p2.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p2-mu_p2_t0).*5.0e+6)./(A_p2.*mu_p2_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1))./(b_C.*(A_p2.*l_p2.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0))).^2.*(et17+et18).*(-5.0e-7);
et21 = (k0.*q_ref0)./2.0+gx.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3+(gy.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3)./q1+(gx.*cos(alpha+q0).*1.1979e-2)./q0-(gx.*cos(alpha+q0).*5.9895e-3)./q1-gy.*1.0./q0.^2.*cos(alpha+q0).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha+q0).*5.9895e-3+(gy.*sin(alpha+q0).*1.1979e-2)./q0;
et22 = (gy.*sin(alpha+q0).*(-5.9895e-3))./q1-gy.*1.0./q0.^2.*cos(alpha).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha).*5.9895e-3-(gx.*cos(alpha+q0+q1).*5.9895e-3)./q2-(gy.*sin(alpha+q0+q1).*5.9895e-3)./q2+gx.*1.0./q0.^3.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2+gy.*1.0./q0.^2.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2;
et23 = gx.*1.0./q0.^2.*(sin(alpha+q0)-sin(alpha)).*(-1.1979e-2)+gy.*1.0./q0.^3.*(sin(alpha+q0)-sin(alpha)).*1.1979e-2+gy.*1.0./q2.^2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*5.9895e-3+(gx.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3)./q1-gy.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3-gx.*1.0./q2.^2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*5.9895e-3;
et24 = (A_p1.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p1-mu_p1_t0).*-5.0e+6)./(A_p1.*mu_p1_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1);
et25 = (gx.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3)./q1-gy.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3+gx.*1.0./q0.^2.*cos(alpha+q0).*1.79685e-2-(gy.*cos(alpha+q0).*1.1979e-2)./q0+(gy.*cos(alpha+q0).*5.9895e-3)./q1-gy.*1.0./q0.^3.*cos(alpha+q0).*2.3958e-2+(gx.*sin(alpha+q0).*1.1979e-2)./q0;
et26 = (gx.*sin(alpha+q0).*(-5.9895e-3))./q1+gx.*1.0./q0.^3.*sin(alpha+q0).*2.3958e-2+gy.*1.0./q0.^2.*sin(alpha+q0).*1.79685e-2-gy.*1.0./q0.^3.*cos(alpha).*1.1979e-2+gx.*1.0./q0.^3.*sin(alpha).*1.1979e-2+(gy.*cos(alpha+q0+q1).*5.9895e-3)./q2-(gx.*sin(alpha+q0+q1).*5.9895e-3)./q2+gx.*1.0./q0.^4.*(cos(alpha+q0)-cos(alpha)).*3.5937e-2;
et27 = gy.*1.0./q0.^3.*(cos(alpha+q0)-cos(alpha)).*2.3958e-2-gx.*1.0./q0.^3.*(sin(alpha+q0)-sin(alpha)).*2.3958e-2+gy.*1.0./q0.^4.*(sin(alpha+q0)-sin(alpha)).*3.5937e-2+gx.*1.0./q2.^2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*5.9895e-3-gx.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3-(gy.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3)./q1;
et28 = gy.*1.0./q2.^2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*5.9895e-3;
et29 = (k0.*q_ref0)./2.0+gx.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3+(gy.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3)./q1+(gx.*cos(alpha+q0).*1.1979e-2)./q0-(gx.*cos(alpha+q0).*5.9895e-3)./q1-gy.*1.0./q0.^2.*cos(alpha+q0).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha+q0).*5.9895e-3+(gy.*sin(alpha+q0).*1.1979e-2)./q0;
et30 = (gy.*sin(alpha+q0).*(-5.9895e-3))./q1-gy.*1.0./q0.^2.*cos(alpha).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha).*5.9895e-3-(gx.*cos(alpha+q0+q1).*5.9895e-3)./q2-(gy.*sin(alpha+q0+q1).*5.9895e-3)./q2+gx.*1.0./q0.^3.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2+gy.*1.0./q0.^2.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2;
et31 = gx.*1.0./q0.^2.*(sin(alpha+q0)-sin(alpha)).*(-1.1979e-2)+gy.*1.0./q0.^3.*(sin(alpha+q0)-sin(alpha)).*1.1979e-2+gy.*1.0./q2.^2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*5.9895e-3+(gx.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3)./q1-gy.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3-gx.*1.0./q2.^2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*5.9895e-3;
et32 = (A_p0.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p0-mu_p0_t0).*5.0e+6)./(A_p0.*mu_p0_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1);
et33 = (gx.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3)./q1-gy.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3+gx.*1.0./q0.^2.*cos(alpha+q0).*1.79685e-2-(gy.*cos(alpha+q0).*1.1979e-2)./q0+(gy.*cos(alpha+q0).*5.9895e-3)./q1-gy.*1.0./q0.^3.*cos(alpha+q0).*2.3958e-2+(gx.*sin(alpha+q0).*1.1979e-2)./q0;
et34 = (gx.*sin(alpha+q0).*(-5.9895e-3))./q1+gx.*1.0./q0.^3.*sin(alpha+q0).*2.3958e-2+gy.*1.0./q0.^2.*sin(alpha+q0).*1.79685e-2-gy.*1.0./q0.^3.*cos(alpha).*1.1979e-2+gx.*1.0./q0.^3.*sin(alpha).*1.1979e-2+(gy.*cos(alpha+q0+q1).*5.9895e-3)./q2-(gx.*sin(alpha+q0+q1).*5.9895e-3)./q2+gx.*1.0./q0.^4.*(cos(alpha+q0)-cos(alpha)).*3.5937e-2;
et35 = gy.*1.0./q0.^3.*(cos(alpha+q0)-cos(alpha)).*2.3958e-2-gx.*1.0./q0.^3.*(sin(alpha+q0)-sin(alpha)).*2.3958e-2+gy.*1.0./q0.^4.*(sin(alpha+q0)-sin(alpha)).*3.5937e-2+gx.*1.0./q2.^2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*5.9895e-3-gx.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3-(gy.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3)./q1;
et36 = gy.*1.0./q2.^2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*5.9895e-3;
et37 = k2.*q_ref2.*(-1.0./2.0)+gy.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3-gx.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3+gy.*1.0./q2.^2.*cos(alpha+q0+q1).*5.9895e-3-gx.*1.0./q2.^2.*sin(alpha+q0+q1).*5.9895e-3+gx.*1.0./q2.^3.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*1.1979e-2+gy.*1.0./q2.^3.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*1.1979e-2;
et38 = (A_p5.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p5-mu_p5_t0).*5.0e+6)./(A_p5.*mu_p5_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1);
et39 = 1.0./(1.0./(A_p5.*l_p5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+((et37+et38).*2.0)./(b_C.*(A_p5.*l_p5.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2))).^2./(A_p5.*b_C.*(A_p5.*l_p5.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2));
et40 = (gx.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3+gy.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3+gx.*1.0./q2.^2.*cos(alpha+q0+q1).*5.9895e-3+gy.*1.0./q2.^2.*sin(alpha+q0+q1).*5.9895e-3-gy.*1.0./q2.^3.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*1.1979e-2+gx.*1.0./q2.^3.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*1.1979e-2).*2.0;
et41 = (k2.*q_ref2)./2.0-gy.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3+gx.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3-gy.*1.0./q2.^2.*cos(alpha+q0+q1).*5.9895e-3+gx.*1.0./q2.^2.*sin(alpha+q0+q1).*5.9895e-3-gx.*1.0./q2.^3.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*1.1979e-2-gy.*1.0./q2.^3.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*1.1979e-2;
et42 = (A_p4.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p4-mu_p4_t0).*5.0e+6)./(A_p4.*mu_p4_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1);
et43 = 1.0./(1.0./(A_p4.*l_p4-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+(et41+et42)./(b_C.*(A_p4.*l_p4.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0))).^2./(A_p4.*b_C.*(A_p4.*l_p4.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0));
et44 = -(gx.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3+gy.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3+gx.*1.0./q2.^2.*cos(alpha+q0+q1).*5.9895e-3+gy.*1.0./q2.^2.*sin(alpha+q0+q1).*5.9895e-3-gy.*1.0./q2.^3.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*1.1979e-2+gx.*1.0./q2.^3.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*1.1979e-2);
et45 = gy.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4-gx.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4+gx.*q2.^2.*cos(alpha+q0).*2.3958e+4+gy.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gy.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4+gx.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gy.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gy.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et46 = gy.*q1.*q2.^2.*cos(alpha+q0).*2.3958e+4-gx.*q1.*q2.^2.*sin(alpha+q0).*2.3958e+4;
et47 = gx.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4-gy.*q1.^2.*cos(alpha+q0+q1+q2).*3.5937e+4+gx.*q1.^2.*sin(alpha+q0+q1+q2).*3.5937e+4+gy.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4-gy.*q2.^2.*cos(alpha+q0).*2.3958e+4+gx.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gy.*q1.^2.*cos(alpha+q0+q1).*3.5937e+4+gy.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gx.*q1.^2.*sin(alpha+q0+q1).*3.5937e+4-gx.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4;
et48 = gx.*q1.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gx.*q1.^2.*q2.*cos(alpha+q0+q1).*3.5937e+4-gy.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4+gy.*q1.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4-gy.*q1.^2.*q2.*sin(alpha+q0+q1).*3.5937e+4+gy.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et49 = gy.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4-gx.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4+gx.*q2.^2.*cos(alpha+q0).*2.3958e+4+gy.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gy.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4+gx.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gy.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gy.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et50 = gy.*q1.*q2.^2.*cos(alpha+q0).*2.3958e+4-gx.*q1.*q2.^2.*sin(alpha+q0).*2.3958e+4;
et51 = 1.0./(b_C.*(A_p3.*l_p3.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2));
et52 = 1.0./(1.0./(A_p3.*l_p3-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+((k1.*q_ref1.*(-1.0./2.0)+(1.0./q1.^3.*1.0./q2.^2.*(et45+et46))./2.0e+6+(A_p3.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p3-mu_p3_t0).*5.0e+6)./(A_p3.*mu_p3_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1)).*2.0)./(b_C.*(A_p3.*l_p3.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2))).^2;
et53 = ((1.0./q1.^3.*1.0./q2.^2.*(et47+et48))./2.0e+6+1.0./q1.^4.*1.0./q2.^2.*(et49+et50).*1.5e-6).*2.0;
et54 = gx.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4-gy.*q1.^2.*cos(alpha+q0+q1+q2).*3.5937e+4+gx.*q1.^2.*sin(alpha+q0+q1+q2).*3.5937e+4+gy.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4-gy.*q2.^2.*cos(alpha+q0).*2.3958e+4+gx.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gy.*q1.^2.*cos(alpha+q0+q1).*3.5937e+4+gy.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gx.*q1.^2.*sin(alpha+q0+q1).*3.5937e+4-gx.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4;
et55 = gx.*q1.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gx.*q1.^2.*q2.*cos(alpha+q0+q1).*3.5937e+4-gy.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4+gy.*q1.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4-gy.*q1.^2.*q2.*sin(alpha+q0+q1).*3.5937e+4+gy.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et56 = gy.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4-gx.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4+gx.*q2.^2.*cos(alpha+q0).*2.3958e+4+gy.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gy.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4+gx.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gy.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gy.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et57 = gy.*q1.*q2.^2.*cos(alpha+q0).*2.3958e+4-gx.*q1.*q2.^2.*sin(alpha+q0).*2.3958e+4;
et58 = gy.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4-gx.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4+gx.*q2.^2.*cos(alpha+q0).*2.3958e+4+gy.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gy.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4+gx.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gy.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gy.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et59 = gy.*q1.*q2.^2.*cos(alpha+q0).*2.3958e+4-gx.*q1.*q2.^2.*sin(alpha+q0).*2.3958e+4;
et60 = ((1.0./q1.^3.*1.0./q2.^2.*(et54+et55))./2.0e+6+1.0./q1.^4.*1.0./q2.^2.*(et56+et57).*1.5e-6)./(b_C.*(A_p2.*l_p2.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0));
et61 = 1.0./(1.0./(A_p2.*l_p2-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+((k1.*q_ref1)./2.0-(1.0./q1.^3.*1.0./q2.^2.*(et58+et59))./2.0e+6+(A_p2.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p2-mu_p2_t0).*5.0e+6)./(A_p2.*mu_p2_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1))./(b_C.*(A_p2.*l_p2.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0))).^2;
et62 = (k0.*q_ref0)./2.0+gx.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3+(gy.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3)./q1+(gx.*cos(alpha+q0).*1.1979e-2)./q0-(gx.*cos(alpha+q0).*5.9895e-3)./q1-gy.*1.0./q0.^2.*cos(alpha+q0).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha+q0).*5.9895e-3+(gy.*sin(alpha+q0).*1.1979e-2)./q0;
et63 = (gy.*sin(alpha+q0).*(-5.9895e-3))./q1-gy.*1.0./q0.^2.*cos(alpha).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha).*5.9895e-3-(gx.*cos(alpha+q0+q1).*5.9895e-3)./q2-(gy.*sin(alpha+q0+q1).*5.9895e-3)./q2+gx.*1.0./q0.^3.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2+gy.*1.0./q0.^2.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2;
et64 = gx.*1.0./q0.^2.*(sin(alpha+q0)-sin(alpha)).*(-1.1979e-2)+gy.*1.0./q0.^3.*(sin(alpha+q0)-sin(alpha)).*1.1979e-2+gy.*1.0./q2.^2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*5.9895e-3+(gx.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3)./q1-gy.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3-gx.*1.0./q2.^2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*5.9895e-3;
et65 = (A_p1.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p1-mu_p1_t0).*-5.0e+6)./(A_p1.*mu_p1_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1);
et66 = gx.*1.0./q1.^3.*(sin(alpha+q0+q1)-sin(alpha+q0)).*1.1979e-2+gy.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3-gx.*1.0./q1.^2.*cos(alpha+q0).*5.9895e-3-gy.*1.0./q1.^2.*sin(alpha+q0).*5.9895e-3-gx.*1.0./q1.^2.*cos(alpha+q0+q1).*5.9895e-3-(gy.*cos(alpha+q0+q1).*5.9895e-3)./q1+(gy.*cos(alpha+q0+q1).*5.9895e-3)./q2;
et67 = (gx.*sin(alpha+q0+q1).*5.9895e-3)./q1-(gx.*sin(alpha+q0+q1).*5.9895e-3)./q2-gy.*1.0./q1.^2.*sin(alpha+q0+q1).*5.9895e-3+gx.*1.0./q2.^2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*5.9895e-3+gx.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3-gy.*1.0./q1.^3.*(cos(alpha+q0+q1)-cos(alpha+q0)).*1.1979e-2;
et68 = gy.*1.0./q2.^2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*5.9895e-3;
et69 = (k0.*q_ref0)./2.0+gx.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3+(gy.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3)./q1+(gx.*cos(alpha+q0).*1.1979e-2)./q0-(gx.*cos(alpha+q0).*5.9895e-3)./q1-gy.*1.0./q0.^2.*cos(alpha+q0).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha+q0).*5.9895e-3+(gy.*sin(alpha+q0).*1.1979e-2)./q0;
et70 = (gy.*sin(alpha+q0).*(-5.9895e-3))./q1-gy.*1.0./q0.^2.*cos(alpha).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha).*5.9895e-3-(gx.*cos(alpha+q0+q1).*5.9895e-3)./q2-(gy.*sin(alpha+q0+q1).*5.9895e-3)./q2+gx.*1.0./q0.^3.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2+gy.*1.0./q0.^2.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2;
et71 = gx.*1.0./q0.^2.*(sin(alpha+q0)-sin(alpha)).*(-1.1979e-2)+gy.*1.0./q0.^3.*(sin(alpha+q0)-sin(alpha)).*1.1979e-2+gy.*1.0./q2.^2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*5.9895e-3+(gx.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3)./q1-gy.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3-gx.*1.0./q2.^2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*5.9895e-3;
et72 = (A_p0.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p0-mu_p0_t0).*5.0e+6)./(A_p0.*mu_p0_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1);
et73 = gx.*1.0./q1.^3.*(sin(alpha+q0+q1)-sin(alpha+q0)).*1.1979e-2+gy.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3-gx.*1.0./q1.^2.*cos(alpha+q0).*5.9895e-3-gy.*1.0./q1.^2.*sin(alpha+q0).*5.9895e-3-gx.*1.0./q1.^2.*cos(alpha+q0+q1).*5.9895e-3-(gy.*cos(alpha+q0+q1).*5.9895e-3)./q1+(gy.*cos(alpha+q0+q1).*5.9895e-3)./q2;
et74 = (gx.*sin(alpha+q0+q1).*5.9895e-3)./q1-(gx.*sin(alpha+q0+q1).*5.9895e-3)./q2-gy.*1.0./q1.^2.*sin(alpha+q0+q1).*5.9895e-3+gx.*1.0./q2.^2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*5.9895e-3+gx.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3-gy.*1.0./q1.^3.*(cos(alpha+q0+q1)-cos(alpha+q0)).*1.1979e-2;
et75 = gy.*1.0./q2.^2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*5.9895e-3;
et76 = k2.*q_ref2.*(-1.0./2.0)+gy.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3-gx.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3+gy.*1.0./q2.^2.*cos(alpha+q0+q1).*5.9895e-3-gx.*1.0./q2.^2.*sin(alpha+q0+q1).*5.9895e-3+gx.*1.0./q2.^3.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*1.1979e-2+gy.*1.0./q2.^3.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*1.1979e-2;
et77 = (A_p5.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p5-mu_p5_t0).*5.0e+6)./(A_p5.*mu_p5_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1);
et78 = gx.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3+gy.*1.0./q2.^3.*cos(alpha+q0+q1+q2).*2.3958e-2-gx.*1.0./q2.^3.*sin(alpha+q0+q1+q2).*2.3958e-2+gy.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3+gy.*1.0./q2.^3.*cos(alpha+q0+q1).*1.1979e-2-gx.*1.0./q2.^3.*sin(alpha+q0+q1).*1.1979e-2+gx.*1.0./q2.^4.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*3.5937e-2;
et79 = gy.*1.0./q2.^4.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*3.5937e-2;
et80 = (k2.*q_ref2)./2.0-gy.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3+gx.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3-gy.*1.0./q2.^2.*cos(alpha+q0+q1).*5.9895e-3+gx.*1.0./q2.^2.*sin(alpha+q0+q1).*5.9895e-3-gx.*1.0./q2.^3.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*1.1979e-2-gy.*1.0./q2.^3.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*1.1979e-2;
et81 = (A_p4.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p4-mu_p4_t0).*5.0e+6)./(A_p4.*mu_p4_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1);
et82 = gx.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3+gy.*1.0./q2.^3.*cos(alpha+q0+q1+q2).*2.3958e-2-gx.*1.0./q2.^3.*sin(alpha+q0+q1+q2).*2.3958e-2+gy.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3+gy.*1.0./q2.^3.*cos(alpha+q0+q1).*1.1979e-2-gx.*1.0./q2.^3.*sin(alpha+q0+q1).*1.1979e-2+gx.*1.0./q2.^4.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*3.5937e-2;
et83 = gy.*1.0./q2.^4.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*3.5937e-2;
et84 = gy.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4-gx.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4+gx.*q2.^2.*cos(alpha+q0).*2.3958e+4+gy.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gy.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4+gx.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gy.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gy.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et85 = gy.*q1.*q2.^2.*cos(alpha+q0).*2.3958e+4-gx.*q1.*q2.^2.*sin(alpha+q0).*2.3958e+4;
et86 = (1.0./q1.^3.*1.0./q2.^2.*(gx.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4+gy.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4+gx.*q2.*cos(alpha+q0+q1).*4.7916e+4+gy.*q2.*sin(alpha+q0+q1).*4.7916e+4-gx.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4-gy.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gx.*q2.*cos(alpha+q0).*4.7916e+4-gy.*q2.*sin(alpha+q0).*4.7916e+4+gx.*q1.^2.*q2.*cos(alpha+q0+q1).*2.3958e+4+gy.*q1.^2.*q2.*sin(alpha+q0+q1).*2.3958e+4-gy.*q1.*q2.*cos(alpha+q0).*4.7916e+4+gx.*q1.*q2.*sin(alpha+q0).*4.7916e+4))./2.0e+6;
et87 = (1.0./q1.^3.*1.0./q2.^3.*(et84+et85))./1.0e+6;
et88 = gy.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4-gx.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4+gx.*q2.^2.*cos(alpha+q0).*2.3958e+4+gy.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gy.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4+gx.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gy.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gy.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et89 = gy.*q1.*q2.^2.*cos(alpha+q0).*2.3958e+4-gx.*q1.*q2.^2.*sin(alpha+q0).*2.3958e+4;
et90 = (et86+et87)./(A_p3.*b_C.*(A_p3.*l_p3.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2));
et91 = 1.0./(1.0./(A_p3.*l_p3-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+((k1.*q_ref1.*(-1.0./2.0)+(1.0./q1.^3.*1.0./q2.^2.*(et88+et89))./2.0e+6+(A_p3.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p3-mu_p3_t0).*5.0e+6)./(A_p3.*mu_p3_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1)).*2.0)./(b_C.*(A_p3.*l_p3.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2))).^2.*2.0;
et92 = gy.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4-gx.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4+gx.*q2.^2.*cos(alpha+q0).*2.3958e+4+gy.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gy.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4+gx.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gy.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gy.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et93 = gy.*q1.*q2.^2.*cos(alpha+q0).*2.3958e+4-gx.*q1.*q2.^2.*sin(alpha+q0).*2.3958e+4;
et94 = (1.0./q1.^3.*1.0./q2.^2.*(gx.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4+gy.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4+gx.*q2.*cos(alpha+q0+q1).*4.7916e+4+gy.*q2.*sin(alpha+q0+q1).*4.7916e+4-gx.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4-gy.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gx.*q2.*cos(alpha+q0).*4.7916e+4-gy.*q2.*sin(alpha+q0).*4.7916e+4+gx.*q1.^2.*q2.*cos(alpha+q0+q1).*2.3958e+4+gy.*q1.^2.*q2.*sin(alpha+q0+q1).*2.3958e+4-gy.*q1.*q2.*cos(alpha+q0).*4.7916e+4+gx.*q1.*q2.*sin(alpha+q0).*4.7916e+4))./2.0e+6;
et95 = (1.0./q1.^3.*1.0./q2.^3.*(et92+et93))./1.0e+6;
et96 = gy.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4-gx.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4+gx.*q2.^2.*cos(alpha+q0).*2.3958e+4+gy.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gy.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4+gx.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gy.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gy.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et97 = gy.*q1.*q2.^2.*cos(alpha+q0).*2.3958e+4-gx.*q1.*q2.^2.*sin(alpha+q0).*2.3958e+4;
et98 = (et94+et95)./(A_p2.*b_C.*(A_p2.*l_p2.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0));
et99 = -1.0./(1.0./(A_p2.*l_p2-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+((k1.*q_ref1)./2.0-(1.0./q1.^3.*1.0./q2.^2.*(et96+et97))./2.0e+6+(A_p2.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p2-mu_p2_t0).*5.0e+6)./(A_p2.*mu_p2_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1))./(b_C.*(A_p2.*l_p2.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0))).^2;
et100 = (k0.*q_ref0)./2.0+gx.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3+(gy.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3)./q1+(gx.*cos(alpha+q0).*1.1979e-2)./q0-(gx.*cos(alpha+q0).*5.9895e-3)./q1-gy.*1.0./q0.^2.*cos(alpha+q0).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha+q0).*5.9895e-3+(gy.*sin(alpha+q0).*1.1979e-2)./q0;
et101 = (gy.*sin(alpha+q0).*(-5.9895e-3))./q1-gy.*1.0./q0.^2.*cos(alpha).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha).*5.9895e-3-(gx.*cos(alpha+q0+q1).*5.9895e-3)./q2-(gy.*sin(alpha+q0+q1).*5.9895e-3)./q2+gx.*1.0./q0.^3.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2+gy.*1.0./q0.^2.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2;
et102 = gx.*1.0./q0.^2.*(sin(alpha+q0)-sin(alpha)).*(-1.1979e-2)+gy.*1.0./q0.^3.*(sin(alpha+q0)-sin(alpha)).*1.1979e-2+gy.*1.0./q2.^2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*5.9895e-3+(gx.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3)./q1-gy.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3-gx.*1.0./q2.^2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*5.9895e-3;
et103 = (A_p1.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p1-mu_p1_t0).*-5.0e+6)./(A_p1.*mu_p1_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1);
et104 = 1.0./(1.0./(A_p1.*l_p1-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))-((et100+et101+et102+et103).*2.0)./(b_C.*(A_p1.*l_p1.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2))).^2./(A_p1.*b_C.*(A_p1.*l_p1.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2));
et105 = (gx.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3+gy.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3+gx.*1.0./q2.^2.*cos(alpha+q0+q1).*5.9895e-3+gy.*1.0./q2.^2.*sin(alpha+q0+q1).*5.9895e-3-gy.*1.0./q2.^3.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*1.1979e-2+gx.*1.0./q2.^3.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*1.1979e-2).*2.0;
et106 = (k0.*q_ref0)./2.0+gx.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3+(gy.*(sin(alpha+q0+q1)-sin(alpha+q0)).*5.9895e-3)./q1+(gx.*cos(alpha+q0).*1.1979e-2)./q0-(gx.*cos(alpha+q0).*5.9895e-3)./q1-gy.*1.0./q0.^2.*cos(alpha+q0).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha+q0).*5.9895e-3+(gy.*sin(alpha+q0).*1.1979e-2)./q0;
et107 = (gy.*sin(alpha+q0).*(-5.9895e-3))./q1-gy.*1.0./q0.^2.*cos(alpha).*5.9895e-3+gx.*1.0./q0.^2.*sin(alpha).*5.9895e-3-(gx.*cos(alpha+q0+q1).*5.9895e-3)./q2-(gy.*sin(alpha+q0+q1).*5.9895e-3)./q2+gx.*1.0./q0.^3.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2+gy.*1.0./q0.^2.*(cos(alpha+q0)-cos(alpha)).*1.1979e-2;
et108 = gx.*1.0./q0.^2.*(sin(alpha+q0)-sin(alpha)).*(-1.1979e-2)+gy.*1.0./q0.^3.*(sin(alpha+q0)-sin(alpha)).*1.1979e-2+gy.*1.0./q2.^2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*5.9895e-3+(gx.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3)./q1-gy.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0)).*5.9895e-3-gx.*1.0./q2.^2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*5.9895e-3;
et109 = (A_p0.*b_C.*(d_Ca.^2-d_Cb.^2).*(l_p0-mu_p0_t0).*5.0e+6)./(A_p0.*mu_p0_t0.*1.0e+2-b_C.*d_Ca.*1.1e+1+b_C.*d_Cb.*1.1e+1);
et110 = 1.0./(1.0./(A_p0.*l_p0-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+(et106+et107+et108+et109)./(b_C.*(A_p0.*l_p0.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0))).^2./(A_p0.*b_C.*(A_p0.*l_p0.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0));
et111 = -(gx.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*5.9895e-3+gy.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*5.9895e-3+gx.*1.0./q2.^2.*cos(alpha+q0+q1).*5.9895e-3+gy.*1.0./q2.^2.*sin(alpha+q0+q1).*5.9895e-3-gy.*1.0./q2.^3.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*1.1979e-2+gx.*1.0./q2.^3.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*1.1979e-2);
mt1 = [-(b_C.*(d_Ca.^2./2.0-d_Cb.^2./2.0)-(1.0./(1.0./(A_p0.*l_p0-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+(et29+et30+et31+et32)./(b_C.*(A_p0.*l_p0.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0))).^2.*(et33+et34+et35+et36))./(b_C.*(A_p0.*l_p0.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0)))./A_p0];
mt2 = [(b_C.*(d_Ca.^2./2.0-d_Cb.^2./2.0)-(1.0./(1.0./(A_p1.*l_p1-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))-((et21+et22+et23+et24).*2.0)./(b_C.*(A_p1.*l_p1.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2))).^2.*(et25+et26+et27+et28).*2.0)./(b_C.*(A_p1.*l_p1.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2)))./A_p1,et19.*et20,et13.*et14,et7.*et8,et3.*et4];
mt3 = [(1.0./(1.0./(A_p0.*l_p0-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+(et69+et70+et71+et72)./(b_C.*(A_p0.*l_p0.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0))).^2.*(et73+et74+et75))./(A_p0.*b_C.*(A_p0.*l_p0.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0))];
mt4 = [(1.0./(1.0./(A_p1.*l_p1-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))-((et62+et63+et64+et65).*2.0)./(b_C.*(A_p1.*l_p1.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2))).^2.*(et66+et67+et68).*-2.0)./(A_p1.*b_C.*(A_p1.*l_p1.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2)),-(et60.*et61+b_C.*(d_Ca.^2./2.0-d_Cb.^2./2.0))./A_p2];
mt5 = [(b_C.*(d_Ca.^2./2.0-d_Cb.^2./2.0)+et51.*et52.*et53)./A_p3,et43.*et44,et39.*et40,et110.*et111,et104.*et105,et98.*et99,et90.*et91];
mt6 = [-(b_C.*(d_Ca.^2./2.0-d_Cb.^2./2.0)+((et82+et83).*1.0./(1.0./(A_p4.*l_p4-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+(et80+et81)./(b_C.*(A_p4.*l_p4.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0))).^2)./(b_C.*(A_p4.*l_p4.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2./2.0-d_Cb.^2./2.0)))./A_p4];
mt7 = [(b_C.*(d_Ca.^2./2.0-d_Cb.^2./2.0)+((et78+et79).*1.0./(1.0./(A_p5.*l_p5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)))+((et76+et77).*2.0)./(b_C.*(A_p5.*l_p5.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2))).^2.*2.0)./(b_C.*(A_p5.*l_p5.*1.0e+5-b_C.*(d_Ca.*(1.1e+1./1.0e+2)-d_Cb.*(1.1e+1./1.0e+2)).*1.0e+5).*(d_Ca.^2-d_Cb.^2)))./A_p5];
dGamma_dq = reshape([mt1,mt2,mt3,mt4,mt5,mt6,mt7],6,3);
