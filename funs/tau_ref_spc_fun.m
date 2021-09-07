function tau_ref_spc = tau_ref_spc_fun(in1,in2,alpha,in4,in5)
%TAU_REF_SPC_FUN
%    TAU_REF_SPC = TAU_REF_SPC_FUN(IN1,IN2,ALPHA,IN4,IN5)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    07-Sep-2021 10:55:53

gx = in4(1,:);
gy = in4(2,:);
k0 = in5(1,:);
k1 = in5(2,:);
k2 = in5(3,:);
q0 = in1(1,:);
q1 = in1(2,:);
q2 = in1(3,:);
q_ref0 = in2(1,:);
q_ref1 = in2(2,:);
q_ref2 = in2(3,:);
et1 = gy.*q1.^3.*cos(alpha+q0+q1+q2).*1.1979e+4-gx.*q1.^3.*sin(alpha+q0+q1+q2).*1.1979e+4+gx.*q2.^2.*cos(alpha+q0).*2.3958e+4+gy.*q2.^2.*sin(alpha+q0).*2.3958e+4-gx.*q2.^2.*cos(alpha+q0+q1).*2.3958e+4-gy.*q1.^3.*cos(alpha+q0+q1).*1.1979e+4+gx.*q1.^3.*sin(alpha+q0+q1).*1.1979e+4-gy.*q2.^2.*sin(alpha+q0+q1).*2.3958e+4+gx.*q1.^3.*q2.*cos(alpha+q0+q1).*1.1979e+4+gy.*q1.^3.*q2.*sin(alpha+q0+q1).*1.1979e+4-gx.*q1.^2.*q2.^2.*cos(alpha+q0+q1).*1.1979e+4-gy.*q1.^2.*q2.^2.*sin(alpha+q0+q1).*1.1979e+4;
et2 = gy.*q1.*q2.^2.*cos(alpha+q0).*2.3958e+4-gx.*q1.*q2.^2.*sin(alpha+q0).*2.3958e+4;
et3 = k0.*q_ref0+gx.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0)).*1.1979e-2+(gy.*(sin(alpha+q0+q1)-sin(alpha+q0)).*1.1979e-2)./q1+(gx.*cos(alpha+q0).*2.3958e-2)./q0-(gx.*cos(alpha+q0).*1.1979e-2)./q1-gy.*1.0./q0.^2.*cos(alpha+q0).*1.1979e-2+gx.*1.0./q0.^2.*sin(alpha+q0).*1.1979e-2+(gy.*sin(alpha+q0).*2.3958e-2)./q0;
et4 = (gy.*sin(alpha+q0).*(-1.1979e-2))./q1-gy.*1.0./q0.^2.*cos(alpha).*1.1979e-2+gx.*1.0./q0.^2.*sin(alpha).*1.1979e-2-(gx.*cos(alpha+q0+q1).*1.1979e-2)./q2-(gy.*sin(alpha+q0+q1).*1.1979e-2)./q2+gx.*1.0./q0.^3.*(cos(alpha+q0)-cos(alpha)).*2.3958e-2+gy.*1.0./q0.^2.*(cos(alpha+q0)-cos(alpha)).*2.3958e-2;
et5 = gx.*1.0./q0.^2.*(sin(alpha+q0)-sin(alpha)).*(-2.3958e-2)+gy.*1.0./q0.^3.*(sin(alpha+q0)-sin(alpha)).*2.3958e-2+gy.*1.0./q2.^2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*1.1979e-2+(gx.*(cos(alpha+q0+q1)-cos(alpha+q0)).*1.1979e-2)./q1-gy.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0)).*1.1979e-2-gx.*1.0./q2.^2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*1.1979e-2;
mt1 = [et3+et4+et5,k1.*q_ref1-(1.0./q1.^3.*1.0./q2.^2.*(et1+et2))./1.0e+6];
mt2 = [k2.*q_ref2-gy.*1.0./q2.^2.*cos(alpha+q0+q1+q2).*1.1979e-2+gx.*1.0./q2.^2.*sin(alpha+q0+q1+q2).*1.1979e-2-gy.*1.0./q2.^2.*cos(alpha+q0+q1).*1.1979e-2+gx.*1.0./q2.^2.*sin(alpha+q0+q1).*1.1979e-2-gx.*1.0./q2.^3.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)).*2.3958e-2-gy.*1.0./q2.^3.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)).*2.3958e-2];
tau_ref_spc = reshape([mt1,mt2],3,1);
