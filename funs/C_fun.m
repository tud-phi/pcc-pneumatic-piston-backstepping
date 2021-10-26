function C = C_fun(in1,in2)
%C_fun
%    C = C_fun(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    26-Oct-2021 11:02:38

q0 = in1(1,:);
q1 = in1(2,:);
q2 = in1(3,:);
qdot0 = in2(1,:);
qdot1 = in2(2,:);
qdot2 = in2(3,:);
et1 = q0.^3.*q2.*qdot1.*cos(q1+q2./1.1e+2).*2.63538e+6+q1.^2.*q2.^2.*qdot0.*cos(q0+q1).*2.611422e+6-q1.^2.*q2.^2.*qdot0.*cos(q1+q2).*2.63538e+6-q1.^2.*q2.*qdot0.*sin(q1+q2./1.1e+2).*5.27076e+6-q0.^3.*q2.^2.*qdot1.*sin(q1+q2).*1.31769e+6-q1.^2.*q2.*qdot0.*sin(q0+q1+q2).*5.27076e+6-q0.^3.*q2.*qdot1.*cos(q2./1.1e+2).*2.63538e+6-q1.^2.*q2.^2.*qdot0.*cos(q1).*2.611422e+6+q0.^3.*q1.^2.*qdot0.*sin(q2).*3.95307e+6+q0.^3.*q1.^2.*qdot1.*sin(q2).*3.95307e+6;
et2 = q0.^3.*q2.^2.*qdot1.*sin(q1).*-1.305711e+6+q0.^3.*q2.^2.*qdot1.*sin(q2).*1.31769e+6+q1.^2.*q2.^2.*qdot0.*cos(q1+q2./1.1e+2).*2.3958e+4+q0.^3.*q2.^2.*qdot1.*sin(q1+q2./1.1e+2).*1.1979e+4+q1.^2.*q2.^2.*qdot0.*cos(q0+q1+q2).*2.63538e+6+q1.^2.*q2.*qdot0.*sin(q0+q1+q2./1.1e+2).*5.27076e+6-q0.^3.*q1.^2.*qdot0.*sin(q2./1.1e+2).*3.95307e+6-q0.^3.*q1.^2.*qdot1.*sin(q2./1.1e+2).*3.95307e+6-q0.^3.*q2.^2.*qdot1.*sin(q2./1.1e+2).*1.1979e+4;
et3 = q0.^3.*q2.*qdot1.*cos(q1+q2).*-2.63538e+6+q1.^2.*q2.*qdot0.*sin(q1+q2).*5.27076e+6+q0.^3.*q1.*q2.^2.*qdot0.*1.305711e+6-q0.^3.*q1.^2.*q2.*qdot0.*2.611422e+6+q0.^3.*q1.*q2.^2.*qdot1.*1.305711e+6-q0.^3.*q1.^2.*q2.*qdot1.*2.611422e+6+q0.^3.*q2.*qdot1.*cos(q2).*2.63538e+6-q1.^2.*q2.^2.*qdot0.*cos(q0+q1+q2./1.1e+2).*2.3958e+4-q0.^3.*q1.*q2.^2.*qdot0.*cos(q2./1.1e+2).*1.1979e+4+q0.^3.*q1.^2.*q2.*qdot0.*cos(q2./1.1e+2).*1.1979e+4;
et4 = q0.^3.*q1.*q2.^2.*qdot1.*cos(q2./1.1e+2).*-1.1979e+4+q0.^3.*q1.^2.*q2.*qdot1.*cos(q2./1.1e+2).*1.1979e+4+q0.^2.*q1.^2.*q2.^2.*qdot0.*cos(q1).*1.305711e+6+q0.*q1.^2.*q2.*qdot0.*cos(q1+q2).*5.27076e+6+q0.^3.*q1.*q2.*qdot0.*sin(q1+q2).*2.63538e+6-q0.^2.*q1.^2.*q2.^2.*qdot0.*cos(q1+q2./1.1e+2).*1.1979e+4-q0.^3.*q1.*q2.*qdot0.*sin(q2).*2.63538e+6-q0.^3.*q1.*q2.*qdot1.*sin(q2).*2.63538e+6-q0.*q1.^2.*q2.*qdot0.*cos(q1+q2./1.1e+2).*5.27076e+6;
et5 = q0.^3.*q1.*q2.^2.*qdot0.*cos(q1+q2).*-1.31769e+6-q0.^3.*q1.*q2.*qdot0.*sin(q1+q2./1.1e+2).*2.63538e+6+q0.*q1.^2.*q2.^2.*qdot0.*sin(q1+q2).*2.63538e+6-q0.^2.*q1.^2.*q2.*qdot0.*sin(q1+q2).*2.63538e+6-q0.^3.*q1.*q2.^2.*qdot0.*cos(q1).*1.305711e+6+q0.^3.*q1.*q2.^2.*qdot0.*cos(q2).*1.31769e+6-q0.^3.*q1.^2.*q2.*qdot0.*cos(q2).*1.31769e+6+q0.^3.*q1.*q2.^2.*qdot1.*cos(q2).*1.31769e+6-q0.^3.*q1.^2.*q2.*qdot1.*cos(q2).*1.31769e+6;
et6 = q0.^3.*q1.*q2.*qdot0.*sin(q2./1.1e+2).*2.63538e+6+q0.^3.*q1.*q2.*qdot1.*sin(q2./1.1e+2).*2.63538e+6+q0.*q1.^2.*q2.^2.*qdot0.*sin(q1).*2.611422e+6+q0.^3.*q1.*q2.^2.*qdot0.*cos(q1+q2./1.1e+2).*1.1979e+4+q0.^2.*q1.^2.*q2.^2.*qdot0.*cos(q1+q2).*1.31769e+6-q0.*q1.^2.*q2.^2.*qdot0.*sin(q1+q2./1.1e+2).*2.3958e+4+q0.^2.*q1.^2.*q2.*qdot0.*sin(q1+q2./1.1e+2).*2.63538e+6;
et7 = q1.^2.*q2.^4.*qdot0.*-5.222844e+6+q1.^2.*q2.^4.*qdot0.*cos(q0+q1).*2.3958e+4-q1.^4.*q2.^2.*qdot0.*cos(q1+q2).*2.63538e+6+q1.*q2.^4.*qdot0.*sin(q0+q1./1.1e+2).*5.27076e+6-q1.^3.*q2.^4.*qdot0.*sin(q0+q1).*2.611422e+6+q1.^4.*q2.^3.*qdot0.*sin(q0+q1).*2.611422e+6+q0.^2.*q1.^2.*q2.^4.*qdot0.*2.611422e+6+q0.^3.*q1.^2.*q2.^3.*qdot0.*1.305711e+6-q0.^3.*q1.^3.*q2.^2.*qdot2.*1.305711e+6+q1.^2.*q2.^4.*qdot0.*cos(q0).*5.222844e+6;
et8 = q1.^2.*q2.^4.*qdot0.*cos(q1).*-2.3958e+4-q1.*q2.^4.*qdot0.*sin(q1./1.1e+2).*5.27076e+6+q0.^3.*q2.^4.*qdot0.*sin(q1).*3.95307e+6+q1.^3.*q2.^4.*qdot0.*sin(q1).*2.611422e+6-q1.^4.*q2.^3.*qdot0.*sin(q1).*2.611422e+6-q0.^3.*q1.^4.*qdot2.*sin(q2).*3.95307e+6-q1.^2.*q2.^4.*qdot0.*cos(q0+q1./1.1e+2).*2.3958e+4+q1.^4.*q2.^2.*qdot0.*cos(q1+q2./1.1e+2).*2.63538e+6+q1.^4.*q2.^2.*qdot0.*cos(q0+q1+q2).*2.63538e+6+q1.^2.*q2.^4.*qdot0.*cos(q1./1.1e+2).*2.3958e+4;
et9 = q0.^3.*q2.^4.*qdot0.*sin(q1./1.1e+2).*-3.95307e+6+q0.^3.*q1.^4.*qdot2.*sin(q2./1.1e+2).*3.95307e+6-q1.*q2.^4.*qdot0.*sin(q0+q1).*5.27076e+6-q0.^3.*q1.*q2.^4.*qdot0.*5.222844e+6+q0.^3.*q1.^4.*q2.*qdot2.*2.611422e+6-q1.^4.*q2.^2.*qdot0.*cos(q0+q1+q2./1.1e+2).*2.63538e+6+q1.*q2.^4.*qdot0.*sin(q1).*5.27076e+6+q0.^3.*q1.*q2.^4.*qdot0.*cos(q1./1.1e+2).*1.1979e+4+q0.^3.*q1.^2.*q2.*qdot2.*cos(q2./1.1e+2).*2.63538e+6;
et10 = q0.^3.*q1.^4.*q2.*qdot2.*cos(q2./1.1e+2).*-1.1979e+4+q0.^2.*q1.^2.*q2.^4.*qdot0.*cos(q1).*1.1979e+4-q0.^3.*q1.^2.*q2.^3.*qdot0.*cos(q1).*1.305711e+6-q0.^3.*q1.^3.*q2.^2.*qdot2.*cos(q2).*1.31769e+6-q0.*q1.^2.*q2.^4.*qdot0.*sin(q1./1.1e+2).*2.3958e+4+q0.^2.*q1.*q2.^4.*qdot0.*sin(q1./1.1e+2).*2.63538e+6-q0.^3.*q1.^3.*q2.*qdot2.*sin(q2./1.1e+2).*2.63538e+6-q0.^3.*q1.^2.*q2.^2.*qdot0.*sin(q2).*1.31769e+6-q0.^2.*q1.^3.*q2.^4.*qdot0.*sin(q1).*1.305711e+6;
et11 = q0.^2.*q1.^4.*q2.^3.*qdot0.*sin(q1).*1.305711e+6+q0.^3.*q1.^2.*q2.^2.*qdot2.*sin(q1).*1.305711e+6+q0.^3.*q1.^2.*q2.^4.*qdot0.*sin(q1).*1.305711e+6-q0.^3.*q1.^3.*q2.^3.*qdot0.*sin(q1).*1.305711e+6-q0.^3.*q1.^2.*q2.^2.*qdot2.*sin(q2).*1.31769e+6-q0.^2.*q1.^4.*q2.^2.*qdot0.*cos(q1+q2./1.1e+2).*1.31769e+6+q0.^3.*q1.^3.*q2.^2.*qdot0.*cos(q1+q2./1.1e+2).*1.31769e+6-q0.^3.*q1.^2.*q2.^2.*qdot0.*sin(q1+q2./1.1e+2).*1.31769e+6;
et12 = q0.^3.*q1.^2.*q2.^2.*qdot2.*sin(q1+q2./1.1e+2).*-1.1979e+4+q0.*q1.*q2.^4.*qdot0.*cos(q1).*5.27076e+6-q0.^2.*q1.^2.*q2.^4.*qdot0.*cos(q1./1.1e+2).*1.1979e+4+q0.^3.*q1.^3.*q2.^2.*qdot2.*cos(q2./1.1e+2).*1.1979e+4+q0.^3.*q1.^2.*q2.^2.*qdot0.*sin(q2./1.1e+2).*1.31769e+6+q0.^3.*q1.^2.*q2.^2.*qdot2.*sin(q2./1.1e+2).*1.1979e+4+q0.^3.*q1.^2.*q2.*qdot2.*cos(q1+q2).*2.63538e+6+q0.*q1.^4.*q2.^2.*qdot0.*sin(q1+q2).*2.63538e+6;
et13 = q0.*q1.*q2.^4.*qdot0.*cos(q1./1.1e+2).*-5.27076e+6+q0.*q1.^3.*q2.^4.*qdot0.*cos(q1).*2.611422e+6-q0.*q1.^4.*q2.^3.*qdot0.*cos(q1).*2.611422e+6+q0.^3.*q1.*q2.^4.*qdot0.*cos(q1).*1.293732e+6-q0.^3.*q1.^2.*q2.*qdot2.*cos(q2).*2.63538e+6+q0.^3.*q1.^4.*q2.*qdot2.*cos(q2).*1.31769e+6+q0.*q1.^2.*q2.^4.*qdot0.*sin(q1).*2.3958e+4-q0.^2.*q1.*q2.^4.*qdot0.*sin(q1).*2.63538e+6+q0.^3.*q1.^3.*q2.*qdot2.*sin(q2).*2.63538e+6-q0.^3.*q1.^2.*q2.*qdot2.*cos(q1+q2./1.1e+2).*2.63538e+6;
et14 = q0.^2.*q1.^4.*q2.^2.*qdot0.*cos(q1+q2).*1.31769e+6-q0.^3.*q1.^3.*q2.^2.*qdot0.*cos(q1+q2).*1.31769e+6-q0.*q1.^4.*q2.^2.*qdot0.*sin(q1+q2./1.1e+2).*2.63538e+6+q0.^3.*q1.^2.*q2.^2.*qdot0.*sin(q1+q2).*1.31769e+6+q0.^3.*q1.^2.*q2.^2.*qdot2.*sin(q1+q2).*1.31769e+6;
et15 = q0.^3.*q1.^4.*q2.^4.*qdot0.*-3.05065167e+8+q0.^4.*q1.^3.*q2.^4.*qdot0.*2.611422e+8+q0.^5.*q1.^2.*q2.^4.*qdot1.*2.611422e+8+q0.^6.*q1.^2.*q2.^3.*qdot1.*1.305711e+8+q0.^6.*q1.^3.*q2.^2.*qdot2.*1.305711e+8+q1.^4.*q2.^4.*qdot0.*sin(q0).*1.31769e+9+q0.^6.*q2.^4.*qdot1.*sin(q1).*3.95307e+8+q0.^6.*q1.^4.*qdot2.*sin(q2).*3.95307e+8-q1.^4.*q2.^4.*qdot0.*sin(q0./1.1e+2).*1.31769e+9;
et16 = q0.^6.*q2.^4.*qdot1.*sin(q1./1.1e+2).*-3.95307e+8-q0.^6.*q1.^4.*qdot2.*sin(q2./1.1e+2).*3.95307e+8-q0.*q1.^4.*q2.^4.*qdot0.*1.5668532e+9-q0.^6.*q1.*q2.^4.*qdot1.*5.222844e+8-q0.^6.*q1.^4.*q2.*qdot2.*2.611422e+8+q0.*q1.^4.*q2.^4.*qdot0.*cos(q0./1.1e+2).*7.1874e+6-q0.^4.*q1.*q2.^4.*qdot1.*cos(q1./1.1e+2).*2.63538e+8+q0.^6.*q1.*q2.^4.*qdot1.*cos(q1./1.1e+2).*1.1979e+6;
et17 = q0.^6.*q1.^4.*q2.*qdot2.*cos(q2./1.1e+2).*1.1979e+6+q0.^3.*q1.^2.*q2.^4.*qdot0.*cos(q1).*2.63538e+8-q0.^3.*q1.^4.*q2.^4.*qdot0.*cos(q0).*2.611422e+8+q0.^4.*q1.^3.*q2.^4.*qdot0.*cos(q0).*2.611422e+8-q0.^4.*q1.^3.*q2.^4.*qdot0.*cos(q1).*1.305711e+8+q0.^4.*q1.^4.*q2.^3.*qdot0.*cos(q1).*1.305711e+8+q0.^4.*q1.^3.*q2.^4.*qdot1.*cos(q1).*1.305711e+8-q0.^4.*q1.^4.*q2.^3.*qdot1.*cos(q1).*1.305711e+8;
et18 = q0.^5.*q1.^2.*q2.^4.*qdot1.*cos(q1).*1.1979e+6-q0.^6.*q1.^2.*q2.^3.*qdot1.*cos(q1).*1.305711e+8+q0.^5.*q1.^4.*q2.^2.*qdot2.*cos(q1).*1.305711e+8-q0.^6.*q1.^3.*q2.^2.*qdot2.*cos(q1).*1.305711e+8+q0.^6.*q1.^3.*q2.^2.*qdot2.*cos(q2).*1.31769e+8+q0.^5.*q1.*q2.^4.*qdot1.*sin(q1./1.1e+2).*2.63538e+8+q0.^6.*q1.^3.*q2.*qdot2.*sin(q2./1.1e+2).*2.63538e+8+q0.^2.*q1.^4.*q2.^4.*qdot0.*sin(q0).*9.127998e+8;
et19 = q0.^3.*q1.^3.*q2.^4.*qdot0.*sin(q0).*-5.222844e+8-q0.^3.*q1.^3.*q2.^4.*qdot0.*sin(q1).*2.611422e+8+q0.^3.*q1.^4.*q2.^3.*qdot0.*sin(q1).*2.611422e+8-q0.^4.*q1.^2.*q2.^4.*qdot0.*sin(q1).*1.31769e+8-q0.^4.*q1.^2.*q2.^4.*qdot1.*sin(q0).*2.611422e+8+q0.^4.*q1.^2.*q2.^4.*qdot1.*sin(q1).*1.1979e+6+q0.^4.*q1.^4.*q2.^2.*qdot2.*sin(q1).*1.305711e+8-q0.^6.*q1.^2.*q2.^2.*qdot1.*sin(q2).*1.31769e+8;
et20 = q0.^5.*q1.^3.*q2.^4.*qdot1.*sin(q1).*-1.305711e+8+q0.^5.*q1.^4.*q2.^3.*qdot1.*sin(q1).*1.305711e+8+q0.^6.*q1.^2.*q2.^4.*qdot1.*sin(q1).*1.305711e+8-q0.^6.*q1.^3.*q2.^3.*qdot1.*sin(q1).*1.305711e+8+q0.^3.*q1.^2.*q2.^4.*qdot0.*cos(q0+q1./1.1e+2).*2.63538e+8-q0.^3.*q1.^4.*q2.^2.*qdot0.*cos(q1+q2./1.1e+2).*2.63538e+8-q0.^5.*q1.^4.*q2.^2.*qdot1.*cos(q1+q2./1.1e+2).*1.31769e+8;
et21 = q0.^6.*q1.^3.*q2.^2.*qdot1.*cos(q1+q2./1.1e+2).*1.31769e+8-q0.^5.*q1.^4.*q2.^2.*qdot2.*cos(q1+q2./1.1e+2).*1.1979e+6+q0.^6.*q1.^3.*q2.^2.*qdot2.*cos(q1+q2./1.1e+2).*1.1979e+6+q0.^4.*q1.^2.*q2.^4.*qdot0.*sin(q0+q1./1.1e+2).*1.31769e+8+q0.^4.*q1.^2.*q2.^4.*qdot1.*sin(q0+q1./1.1e+2).*1.1979e+6+q0.^4.*q1.^4.*q2.^2.*qdot0.*sin(q1+q2./1.1e+2).*1.31769e+8;
et22 = q0.^4.*q1.^4.*q2.^2.*qdot1.*sin(q1+q2./1.1e+2).*-1.31769e+8-q0.^6.*q1.^2.*q2.^2.*qdot1.*sin(q1+q2./1.1e+2).*1.31769e+8-q0.^4.*q1.^4.*q2.^2.*qdot2.*sin(q1+q2./1.1e+2).*1.1979e+6+q0.^4.*q1.^4.*q2.*qdot2.*cos(q0+q1+q2./1.1e+2).*2.63538e+8-q0.^3.*q1.^4.*q2.^2.*qdot0.*cos(q0+q1+q2).*2.63538e+8-q0.^4.*q1.^4.*q2.^2.*qdot0.*sin(q0+q1+q2).*1.31769e+8-q0.^4.*q1.^4.*q2.^2.*qdot1.*sin(q0+q1+q2).*1.31769e+8;
et23 = q0.^4.*q1.^4.*q2.^2.*qdot2.*sin(q0+q1+q2).*-1.31769e+8-q0.^3.*q1.^2.*q2.^4.*qdot0.*cos(q1./1.1e+2).*2.63538e+8-q0.^5.*q1.^2.*q2.^4.*qdot1.*cos(q1./1.1e+2).*1.1979e+6-q0.^6.*q1.^3.*q2.^2.*qdot2.*cos(q2./1.1e+2).*1.1979e+6+q0.^2.*q1.^4.*q2.^4.*qdot0.*sin(q0./1.1e+2).*1.089e+4+q0.^4.*q1.^2.*q2.^4.*qdot0.*sin(q1./1.1e+2).*1.31769e+8-q0.^4.*q1.^2.*q2.^4.*qdot1.*sin(q1./1.1e+2).*1.1979e+6;
et24 = q0.^6.*q1.^2.*q2.^2.*qdot1.*sin(q2./1.1e+2).*1.31769e+8-q0.^4.*q1.*q2.^4.*qdot1.*cos(q0+q1).*2.63538e+8+q0.^4.*q1.^4.*q2.*qdot2.*cos(q1+q2).*2.63538e+8-q0.^5.*q1.^4.*q2.*qdot2.*sin(q1+q2).*2.63538e+8+q0.^6.*q1.^3.*q2.*qdot2.*sin(q1+q2).*2.63538e+8+q0.*q1.^4.*q2.^4.*qdot0.*cos(q0).*2.539548e+8+q0.^4.*q1.*q2.^4.*qdot1.*cos(q1).*2.63538e+8+q0.^6.*q1.*q2.^4.*qdot1.*cos(q1).*1.293732e+8-q0.^6.*q1.^4.*q2.*qdot2.*cos(q2).*1.31769e+8;
et25 = q0.^3.*q1.^4.*q2.^2.*qdot0.*cos(q0+q1+q2./1.1e+2).*2.63538e+8-q0.^5.*q1.*q2.^4.*qdot1.*sin(q1).*2.63538e+8-q0.^6.*q1.^3.*q2.*qdot2.*sin(q2).*2.63538e+8+q0.^4.*q1.^4.*q2.^2.*qdot0.*sin(q0+q1+q2./1.1e+2).*1.31769e+8+q0.^4.*q1.^4.*q2.^2.*qdot1.*sin(q0+q1+q2./1.1e+2).*1.31769e+8+q0.^4.*q1.^4.*q2.^2.*qdot2.*sin(q0+q1+q2./1.1e+2).*1.1979e+6+q0.^4.*q1.*q2.^4.*qdot1.*cos(q0+q1./1.1e+2).*2.63538e+8;
et26 = q0.^4.*q1.^4.*q2.*qdot2.*cos(q1+q2./1.1e+2).*-2.63538e+8-q0.^3.*q1.^2.*q2.^4.*qdot0.*cos(q0+q1).*2.63538e+8+q0.^3.*q1.^4.*q2.^2.*qdot0.*cos(q1+q2).*2.63538e+8-q0.^4.*q1.^3.*q2.^4.*qdot0.*cos(q0+q1).*1.305711e+8+q0.^4.*q1.^4.*q2.^3.*qdot0.*cos(q0+q1).*1.305711e+8-q0.^4.*q1.^3.*q2.^4.*qdot1.*cos(q0+q1).*1.305711e+8+q0.^4.*q1.^4.*q2.^3.*qdot1.*cos(q0+q1).*1.305711e+8+q0.^5.*q1.^4.*q2.^2.*qdot1.*cos(q1+q2).*1.31769e+8;
et27 = q0.^6.*q1.^3.*q2.^2.*qdot1.*cos(q1+q2).*-1.31769e+8+q0.^5.*q1.^4.*q2.^2.*qdot2.*cos(q1+q2).*1.31769e+8-q0.^6.*q1.^3.*q2.^2.*qdot2.*cos(q1+q2).*1.31769e+8+q0.^5.*q1.^4.*q2.*qdot2.*sin(q1+q2./1.1e+2).*2.63538e+8-q0.^6.*q1.^3.*q2.*qdot2.*sin(q1+q2./1.1e+2).*2.63538e+8+q0.^3.*q1.^3.*q2.^4.*qdot0.*sin(q0+q1).*2.611422e+8-q0.^3.*q1.^4.*q2.^3.*qdot0.*sin(q0+q1).*2.611422e+8;
et28 = q0.^4.*q1.^2.*q2.^4.*qdot0.*sin(q0+q1).*-1.31769e+8-q0.^4.*q1.^2.*q2.^4.*qdot1.*sin(q0+q1).*1.1979e+6-q0.^4.*q1.^4.*q2.^2.*qdot0.*sin(q1+q2).*1.31769e+8-q0.^4.*q1.^4.*q2.^2.*qdot2.*sin(q0+q1).*1.305711e+8+q0.^4.*q1.^4.*q2.^2.*qdot1.*sin(q1+q2).*1.31769e+8+q0.^6.*q1.^2.*q2.^2.*qdot1.*sin(q1+q2).*1.31769e+8+q0.^4.*q1.^4.*q2.^2.*qdot2.*sin(q1+q2).*1.31769e+8-q0.^4.*q1.^4.*q2.*qdot2.*cos(q0+q1+q2).*2.63538e+8;
et29 = q2.^2.*qdot1.*2.611422e+6+q1.^3.*q2.*qdot0.*2.611422e+6+q1.^3.*q2.*qdot1.*2.611422e+6-q2.*qdot1.*sin(q2).*5.27076e+6-q2.^2.*qdot1.*cos(q1+q2).*2.63538e+6-q2.*qdot1.*sin(q1+q2./1.1e+2).*5.27076e+6-q1.^2.*q2.^2.*qdot0.*1.305711e+6-q1.^2.*q2.^2.*qdot1.*1.305711e+6-q2.^2.*qdot1.*cos(q1).*2.611422e+6+q2.^2.*qdot1.*cos(q2).*2.63538e+6+q2.*qdot1.*sin(q2./1.1e+2).*5.27076e+6-q1.^3.*qdot0.*sin(q2).*3.95307e+6-q1.^3.*qdot1.*sin(q2).*3.95307e+6;
et30 = q2.^2.*qdot1.*cos(q1+q2./1.1e+2).*2.3958e+4-q2.^2.*qdot1.*cos(q2./1.1e+2).*2.3958e+4+q1.^3.*qdot0.*sin(q2./1.1e+2).*3.95307e+6+q1.^3.*qdot1.*sin(q2./1.1e+2).*3.95307e+6+q2.*qdot1.*sin(q1+q2).*5.27076e+6-q1.*q2.^2.*qdot0.*sin(q1+q2./1.1e+2).*1.1979e+4-q1.^3.*q2.*qdot0.*cos(q2./1.1e+2).*1.1979e+4-q1.^3.*q2.*qdot1.*cos(q2./1.1e+2).*1.1979e+4-q1.^2.*q2.^2.*qdot0.*cos(q2).*1.31769e+6-q1.^2.*q2.^2.*qdot1.*cos(q2).*1.31769e+6;
et31 = q1.*q2.^2.*qdot0.*sin(q2./1.1e+2).*1.1979e+4-q1.^2.*q2.*qdot0.*sin(q2./1.1e+2).*2.63538e+6+q1.*q2.^2.*qdot1.*sin(q2./1.1e+2).*2.3958e+4-q1.^2.*q2.*qdot1.*sin(q2./1.1e+2).*2.63538e+6+q1.*q2.*qdot0.*cos(q1+q2).*2.63538e+6-q1.*q2.*qdot0.*cos(q2).*2.63538e+6-q1.*q2.*qdot1.*cos(q2).*5.27076e+6+q1.^2.*q2.^2.*qdot0.*cos(q2./1.1e+2).*1.1979e+4+q1.^2.*q2.^2.*qdot1.*cos(q2./1.1e+2).*1.1979e+4-q1.*q2.*qdot0.*cos(q1+q2./1.1e+2).*2.63538e+6;
et32 = q1.*q2.^2.*qdot0.*sin(q1+q2).*1.31769e+6+q1.*q2.*qdot0.*cos(q2./1.1e+2).*2.63538e+6+q1.*q2.*qdot1.*cos(q2./1.1e+2).*5.27076e+6+q1.^3.*q2.*qdot0.*cos(q2).*1.31769e+6+q1.^3.*q2.*qdot1.*cos(q2).*1.31769e+6+q1.*q2.^2.*qdot0.*sin(q1).*1.305711e+6-q1.*q2.^2.*qdot0.*sin(q2).*1.31769e+6+q1.^2.*q2.*qdot0.*sin(q2).*2.63538e+6-q1.*q2.^2.*qdot1.*sin(q2).*2.63538e+6+q1.^2.*q2.*qdot1.*sin(q2).*2.63538e+6;
et33 = q1.*q2.^4.*qdot1.*1.0445688e+9+q1.^6.*q2.*qdot2.*2.611422e+8+q1.^3.*q2.^4.*qdot1.*1.74494067e+8-q1.^4.*q2.^3.*qdot1.*1.305711e+8-q1.^5.*q2.^2.*qdot2.*1.305711e+8-q2.^4.*qdot1.*sin(q1).*1.31769e+9-q1.^6.*qdot2.*sin(q2).*3.95307e+8+q2.^4.*qdot1.*sin(q1./1.1e+2).*1.31769e+9+q1.^6.*qdot2.*sin(q2./1.1e+2).*3.95307e+8-q1.^4.*q2.*qdot2.*cos(q1+q2./1.1e+2).*2.63538e+8;
et34 = q1.^3.*q2.^2.*qdot1.*cos(q1+q2).*2.63538e+8+q1.^4.*q2.^2.*qdot1.*sin(q1+q2).*1.31769e+8+q1.^4.*q2.^2.*qdot2.*sin(q1+q2).*1.31769e+8-q1.*q2.^4.*qdot1.*cos(q1./1.1e+2).*7.1874e+6+q1.^4.*q2.*qdot2.*cos(q2./1.1e+2).*2.63538e+8-q1.^6.*q2.*qdot2.*cos(q2./1.1e+2).*1.1979e+6-q1.^3.*q2.^2.*qdot1.*cos(q2).*2.63538e+8+q1.^3.*q2.^4.*qdot1.*cos(q1).*1.305711e+8-q1.^4.*q2.^3.*qdot1.*cos(q1).*1.305711e+8;
et35 = q1.^5.*q2.^2.*qdot2.*cos(q2).*-1.31769e+8-q1.^5.*q2.*qdot2.*sin(q2./1.1e+2).*2.63538e+8-q1.^2.*q2.^4.*qdot1.*sin(q1).*3.905154e+8+q1.^3.*q2.^3.*qdot1.*sin(q1).*2.611422e+8+q1.^4.*q2.^2.*qdot1.*sin(q2).*1.31769e+8+q1.^4.*q2.^2.*qdot2.*sin(q1).*1.305711e+8-q1.^4.*q2.^2.*qdot2.*sin(q2).*1.31769e+8-q1.^3.*q2.^2.*qdot1.*cos(q1+q2./1.1e+2).*2.63538e+8-q1.^4.*q2.^2.*qdot1.*sin(q1+q2./1.1e+2).*1.31769e+8;
et36 = q1.^4.*q2.^2.*qdot2.*sin(q1+q2./1.1e+2).*-1.1979e+6+q1.^3.*q2.^2.*qdot1.*cos(q2./1.1e+2).*2.63538e+8+q1.^5.*q2.^2.*qdot2.*cos(q2./1.1e+2).*1.1979e+6-q1.^2.*q2.^4.*qdot1.*sin(q1./1.1e+2).*1.089e+4-q1.^4.*q2.^2.*qdot1.*sin(q2./1.1e+2).*1.31769e+8+q1.^4.*q2.^2.*qdot2.*sin(q2./1.1e+2).*1.1979e+6+q1.^4.*q2.*qdot2.*cos(q1+q2).*2.63538e+8+q1.*q2.^4.*qdot1.*cos(q1).*2.683296e+8-q1.^4.*q2.*qdot2.*cos(q2).*2.63538e+8;
et37 = q1.^6.*q2.*qdot2.*cos(q2).*1.31769e+8+q1.^5.*q2.*qdot2.*sin(q2).*2.63538e+8;
et38 = q1.^2.*q2.^4.*qdot1.*5.222844e+7-q1.^5.*q2.*qdot2.*cos(q1+q2./1.1e+2).*2.63538e+7-q1.^2.*q2.^4.*qdot0.*cos(q0+q1).*2.63538e+7-q1.^2.*q2.^4.*qdot1.*cos(q0+q1).*2.659338e+7-q1.^4.*q2.^4.*qdot0.*cos(q0+q1).*1.305711e+7+q1.^5.*q2.^3.*qdot0.*cos(q0+q1).*1.305711e+7-q1.^4.*q2.^4.*qdot1.*cos(q0+q1).*1.305711e+7+q1.^5.*q2.^3.*qdot1.*cos(q0+q1).*1.305711e+7-q1.*q2.^4.*qdot1.*sin(q0+q1./1.1e+2).*7.90614e+7;
et39 = q1.^3.*q2.^4.*qdot0.*sin(q0+q1).*-1.1979e+5+q1.^3.*q2.^4.*qdot1.*sin(q0+q1).*1.293732e+7+q1.^5.*q2.^2.*qdot0.*sin(q1+q2).*1.31769e+7-q1.^5.*q2.^2.*qdot2.*sin(q0+q1).*1.305711e+7+q1.^5.*q2.^2.*qdot1.*sin(q1+q2).*1.31769e+7+q1.^5.*q2.^2.*qdot2.*sin(q1+q2).*1.31769e+7-q1.^5.*q2.*qdot2.*cos(q0+q1+q2).*2.63538e+7-q0.^2.*q1.^2.*q2.^4.*qdot0.*5.222844e+7+q0.^2.*q1.^3.*q2.^3.*qdot0.*1.305711e+7-q0.^2.*q1.^2.*q2.^4.*qdot1.*3.9290031e+7;
et40 = q0.^2.*q1.^3.*q2.^3.*qdot1.*2.611422e+7+q0.^2.*q1.^4.*q2.^2.*qdot2.*1.305711e+7+q0.^2.*q2.^4.*qdot1.*cos(q1).*5.27076e+7+q1.^2.*q2.^4.*qdot0.*cos(q1).*2.63538e+7-q1.^2.*q2.^4.*qdot1.*cos(q0).*5.222844e+7+q1.^2.*q2.^4.*qdot1.*cos(q1).*2.659338e+7+q1.^4.*q2.^4.*qdot0.*cos(q1).*1.305711e+7-q1.^5.*q2.^3.*qdot0.*cos(q1).*1.305711e+7+q1.^4.*q2.^4.*qdot1.*cos(q1).*1.305711e+7-q1.^5.*q2.^3.*qdot1.*cos(q1).*1.305711e+7;
et41 = q1.*q2.^4.*qdot1.*sin(q1./1.1e+2).*7.90614e+7-q1.^3.*q2.^4.*qdot0.*sin(q0).*2.611422e+7+q1.^3.*q2.^4.*qdot0.*sin(q1).*1.1979e+5+q0.^2.*q1.^5.*qdot2.*sin(q2).*3.95307e+7-q1.^3.*q2.^4.*qdot1.*sin(q1).*1.293732e+7+q1.^5.*q2.^2.*qdot2.*sin(q1).*1.305711e+7+q1.^2.*q2.^4.*qdot0.*cos(q0+q1./1.1e+2).*2.63538e+7+q1.^2.*q2.^4.*qdot1.*cos(q0+q1./1.1e+2).*4.7916e+5+q1.^3.*q2.^4.*qdot0.*sin(q0+q1./1.1e+2).*1.1979e+5;
et42 = q1.^3.*q2.^4.*qdot1.*sin(q0+q1./1.1e+2).*1.089e+3-q1.^5.*q2.^2.*qdot0.*sin(q1+q2./1.1e+2).*1.31769e+7-q1.^5.*q2.^2.*qdot1.*sin(q1+q2./1.1e+2).*1.31769e+7-q1.^5.*q2.^2.*qdot2.*sin(q1+q2./1.1e+2).*1.1979e+5+q1.^5.*q2.*qdot2.*cos(q0+q1+q2./1.1e+2).*2.63538e+7-q1.^5.*q2.^2.*qdot0.*sin(q0+q1+q2).*1.31769e+7-q1.^5.*q2.^2.*qdot1.*sin(q0+q1+q2).*1.31769e+7-q1.^5.*q2.^2.*qdot2.*sin(q0+q1+q2).*1.31769e+7;
et43 = q0.^2.*q2.^4.*qdot1.*cos(q1./1.1e+2).*-5.27076e+7-q1.^2.*q2.^4.*qdot0.*cos(q1./1.1e+2).*2.63538e+7-q1.^2.*q2.^4.*qdot1.*cos(q1./1.1e+2).*4.7916e+5-q1.^3.*q2.^4.*qdot0.*sin(q1./1.1e+2).*1.1979e+5-q0.^2.*q1.^5.*qdot2.*sin(q2./1.1e+2).*3.95307e+7-q1.^3.*q2.^4.*qdot1.*sin(q1./1.1e+2).*1.089e+3+q1.^5.*q2.*qdot2.*cos(q1+q2).*2.63538e+7+q1.*q2.^4.*qdot1.*sin(q0+q1).*7.90614e+7+q0.*q1.^3.*q2.^4.*qdot0.*2.611422e+7;
et44 = q0.^2.*q1.^5.*q2.*qdot2.*-2.611422e+7-q1.*q2.^4.*qdot1.*sin(q1).*7.90614e+7+q1.^5.*q2.^2.*qdot0.*sin(q0+q1+q2./1.1e+2).*1.31769e+7+q1.^5.*q2.^2.*qdot1.*sin(q0+q1+q2./1.1e+2).*1.31769e+7+q1.^5.*q2.^2.*qdot2.*sin(q0+q1+q2./1.1e+2).*1.1979e+5-q0.*q1.^3.*q2.^4.*qdot0.*cos(q1./1.1e+2).*1.1979e+5-q0.*q1.^3.*q2.^4.*qdot1.*cos(q1./1.1e+2).*1.089e+3+q0.^2.*q1.^5.*q2.*qdot2.*cos(q2./1.1e+2).*1.1979e+5;
et45 = q0.^2.*q1.^2.*q2.^2.*qdot1.*cos(q2).*2.63538e+7+q0.^2.*q1.^2.*q2.^4.*qdot0.*cos(q1).*1.293732e+7-q0.^2.*q1.^3.*q2.^3.*qdot0.*cos(q1).*1.305711e+7+q0.^2.*q1.^2.*q2.^4.*qdot1.*cos(q1).*1.293732e+7-q0.^2.*q1.^4.*q2.^2.*qdot2.*cos(q1).*1.305711e+7+q0.^2.*q1.^4.*q2.^2.*qdot2.*cos(q2).*1.31769e+7+q0.*q1.^2.*q2.^4.*qdot0.*sin(q1./1.1e+2).*2.63538e+7-q0.^2.*q1.*q2.^4.*qdot0.*sin(q1./1.1e+2).*3.95307e+7;
et46 = q0.*q1.^2.*q2.^4.*qdot1.*sin(q1./1.1e+2).*4.7916e+5-q0.^2.*q1.*q2.^4.*qdot1.*sin(q1./1.1e+2).*4.7916e+5+q0.^2.*q1.^4.*q2.*qdot2.*sin(q2./1.1e+2).*2.63538e+7-q0.^2.*q1.^2.*q2.^3.*qdot1.*sin(q1).*2.611422e+7-q0.^2.*q1.^3.*q2.^2.*qdot0.*sin(q2).*1.31769e+7-q0.^2.*q1.^3.*q2.^2.*qdot1.*sin(q2).*2.63538e+7+q0.^2.*q1.^3.*q2.^4.*qdot0.*sin(q1).*1.305711e+7-q0.^2.*q1.^4.*q2.^3.*qdot0.*sin(q1).*1.305711e+7;
et47 = q0.^2.*q1.^3.*q2.^4.*qdot1.*sin(q1).*1.305711e+7-q0.^2.*q1.^4.*q2.^3.*qdot1.*sin(q1).*1.305711e+7-q0.*q1.^5.*q2.*qdot2.*sin(q1+q2).*2.63538e+7+q0.^2.*q1.^2.*q2.^2.*qdot1.*cos(q1+q2./1.1e+2).*2.63538e+7+q0.^2.*q1.^4.*q2.^2.*qdot0.*cos(q1+q2./1.1e+2).*1.31769e+7+q0.^2.*q1.^4.*q2.^2.*qdot1.*cos(q1+q2./1.1e+2).*1.31769e+7+q0.^2.*q1.^4.*q2.^2.*qdot2.*cos(q1+q2./1.1e+2).*1.1979e+5;
et48 = q0.^2.*q1.^3.*q2.^2.*qdot0.*sin(q1+q2./1.1e+2).*-1.31769e+7-q0.*q1.*q2.^4.*qdot1.*cos(q1).*7.90614e+7-q0.^2.*q1.^2.*q2.^2.*qdot1.*cos(q2./1.1e+2).*2.63538e+7+q0.^2.*q1.^2.*q2.^4.*qdot0.*cos(q1./1.1e+2).*1.1979e+5+q0.^2.*q1.^2.*q2.^4.*qdot1.*cos(q1./1.1e+2).*1.089e+3-q0.^2.*q1.^4.*q2.^2.*qdot2.*cos(q2./1.1e+2).*1.1979e+5+q0.^2.*q1.^3.*q2.^2.*qdot0.*sin(q2./1.1e+2).*1.31769e+7;
et49 = q0.^2.*q1.^3.*q2.^2.*qdot1.*sin(q2./1.1e+2).*2.63538e+7+q0.*q1.^5.*q2.^2.*qdot0.*cos(q1+q2).*1.31769e+7+q0.*q1.^5.*q2.^2.*qdot1.*cos(q1+q2).*1.31769e+7+q0.*q1.^5.*q2.^2.*qdot2.*cos(q1+q2).*1.31769e+7+q0.*q1.^5.*q2.*qdot2.*sin(q1+q2./1.1e+2).*2.63538e+7+q0.^2.*q1.^4.*q2.*qdot2.*sin(q1+q2).*2.63538e+7+q0.*q1.*q2.^4.*qdot1.*cos(q1./1.1e+2).*7.90614e+7+q0.*q1.^3.*q2.^4.*qdot0.*cos(q1).*1.1979e+5-q0.*q1.^3.*q2.^4.*qdot1.*cos(q1).*1.293732e+7;
et50 = q0.*q1.^5.*q2.^2.*qdot2.*cos(q1).*1.305711e+7-q0.^2.*q1.^5.*q2.*qdot2.*cos(q2).*1.31769e+7-q0.*q1.^2.*q2.^4.*qdot0.*sin(q1).*2.63538e+7+q0.^2.*q1.*q2.^4.*qdot0.*sin(q1).*3.95307e+7-q0.*q1.^2.*q2.^4.*qdot1.*sin(q1).*2.659338e+7+q0.^2.*q1.*q2.^4.*qdot1.*sin(q1).*5.27076e+7-q0.*q1.^4.*q2.^4.*qdot0.*sin(q1).*1.305711e+7+q0.*q1.^5.*q2.^3.*qdot0.*sin(q1).*1.305711e+7-q0.*q1.^4.*q2.^4.*qdot1.*sin(q1).*1.305711e+7+q0.*q1.^5.*q2.^3.*qdot1.*sin(q1).*1.305711e+7;
et51 = q0.^2.*q1.^4.*q2.*qdot2.*sin(q2).*-2.63538e+7-q0.*q1.^5.*q2.^2.*qdot0.*cos(q1+q2./1.1e+2).*1.31769e+7-q0.*q1.^5.*q2.^2.*qdot1.*cos(q1+q2./1.1e+2).*1.31769e+7-q0.*q1.^5.*q2.^2.*qdot2.*cos(q1+q2./1.1e+2).*1.1979e+5-q0.^2.*q1.^2.*q2.^2.*qdot1.*cos(q1+q2).*2.63538e+7-q0.^2.*q1.^4.*q2.^2.*qdot0.*cos(q1+q2).*1.31769e+7-q0.^2.*q1.^4.*q2.^2.*qdot1.*cos(q1+q2).*1.31769e+7-q0.^2.*q1.^4.*q2.^2.*qdot2.*cos(q1+q2).*1.31769e+7;
et52 = q0.^2.*q1.^4.*q2.*qdot2.*sin(q1+q2./1.1e+2).*-2.63538e+7+q0.^2.*q1.^3.*q2.^2.*qdot0.*sin(q1+q2).*1.31769e+7;
et53 = q2.^2.*qdot2.*-2.611422e+7-q1.*q2.^3.*qdot0.*1.305711e+7-q1.*q2.^3.*qdot1.*1.305711e+7+q2.*qdot2.*sin(q2).*7.90614e+7+q2.^2.*qdot0.*cos(q1+q2).*2.63538e+7+q2.^2.*qdot1.*cos(q1+q2).*2.63538e+7+q2.^2.*qdot2.*cos(q1+q2).*5.27076e+7+q2.*qdot2.*sin(q1+q2./1.1e+2).*7.90614e+7+q2.^3.*qdot0.*sin(q1+q2).*1.31769e+7+q2.^3.*qdot1.*sin(q1+q2).*1.31769e+7+q2.^3.*qdot2.*sin(q1+q2).*1.31769e+7+q1.^2.*q2.^2.*qdot0.*2.611422e+7;
et54 = q1.^2.*q2.^2.*qdot1.*2.611422e+7+q1.^2.*q2.^2.*qdot2.*1.3175811e+7-q2.^2.*qdot0.*cos(q2).*2.63538e+7-q1.^2.*qdot2.*cos(q2).*5.27076e+7-q2.^2.*qdot1.*cos(q2).*2.63538e+7+q2.^2.*qdot2.*cos(q1).*2.611422e+7-q2.^2.*qdot2.*cos(q2).*5.27076e+7-q2.*qdot2.*sin(q2./1.1e+2).*7.90614e+7+q2.^3.*qdot0.*sin(q1).*1.305711e+7-q2.^3.*qdot0.*sin(q2).*1.31769e+7+q2.^3.*qdot1.*sin(q1).*1.305711e+7-q2.^3.*qdot1.*sin(q2).*1.31769e+7;
et55 = q2.^3.*qdot2.*sin(q2).*-1.31769e+7-q2.^2.*qdot0.*cos(q1+q2./1.1e+2).*2.63538e+7-q2.^2.*qdot1.*cos(q1+q2./1.1e+2).*2.63538e+7-q2.^2.*qdot2.*cos(q1+q2./1.1e+2).*4.7916e+5-q2.^3.*qdot0.*sin(q1+q2./1.1e+2).*1.1979e+5-q2.^3.*qdot1.*sin(q1+q2./1.1e+2).*1.1979e+5-q2.^3.*qdot2.*sin(q1+q2./1.1e+2).*1.089e+3+q2.^2.*qdot0.*cos(q2./1.1e+2).*2.63538e+7+q1.^2.*qdot2.*cos(q2./1.1e+2).*5.27076e+7;
et56 = q2.^2.*qdot1.*cos(q2./1.1e+2).*2.63538e+7+q2.^2.*qdot2.*cos(q2./1.1e+2).*4.7916e+5+q2.^3.*qdot0.*sin(q2./1.1e+2).*1.1979e+5+q2.^3.*qdot1.*sin(q2./1.1e+2).*1.1979e+5+q2.^3.*qdot2.*sin(q2./1.1e+2).*1.089e+3-q2.*qdot2.*sin(q1+q2).*7.90614e+7+q1.*q2.^3.*qdot0.*cos(q2./1.1e+2).*1.1979e+5+q1.*q2.^3.*qdot1.*cos(q2./1.1e+2).*1.1979e+5+q1.*q2.^3.*qdot2.*cos(q2./1.1e+2).*1.089e+3+q1.^2.*q2.^2.*qdot0.*cos(q2).*1.31769e+7;
et57 = q1.^2.*q2.^2.*qdot1.*cos(q2).*1.31769e+7+q1.^2.*q2.^2.*qdot2.*cos(q2).*1.31769e+7-q1.*q2.^2.*qdot0.*sin(q2./1.1e+2).*2.63538e+7+q1.^2.*q2.*qdot0.*sin(q2./1.1e+2).*3.95307e+7-q1.*q2.^2.*qdot1.*sin(q2./1.1e+2).*2.63538e+7+q1.^2.*q2.*qdot1.*sin(q2./1.1e+2).*3.95307e+7-q1.*q2.^2.*qdot2.*sin(q2./1.1e+2).*4.7916e+5+q1.^2.*q2.*qdot2.*sin(q2./1.1e+2).*4.7916e+5+q1.*q2.*qdot2.*cos(q2).*7.90614e+7;
et58 = q1.^2.*q2.^2.*qdot0.*cos(q2./1.1e+2).*-1.1979e+5-q1.^2.*q2.^2.*qdot1.*cos(q2./1.1e+2).*1.1979e+5-q1.^2.*q2.^2.*qdot2.*cos(q2./1.1e+2).*1.089e+3-q1.*q2.*qdot2.*cos(q2./1.1e+2).*7.90614e+7-q1.*q2.^3.*qdot0.*cos(q2).*1.31769e+7-q1.*q2.^3.*qdot1.*cos(q2).*1.31769e+7-q1.*q2.^3.*qdot2.*cos(q2).*1.31769e+7+q1.*q2.^2.*qdot0.*sin(q2).*2.63538e+7-q1.^2.*q2.*qdot0.*sin(q2).*3.95307e+7+q1.*q2.^2.*qdot1.*sin(q2).*2.63538e+7;
et59 = q1.^2.*q2.*qdot1.*sin(q2).*-3.95307e+7+q1.*q2.^2.*qdot2.*sin(q2).*5.27076e+7-q1.^2.*q2.*qdot2.*sin(q2).*5.27076e+7;
et60 = q0.^2.*q2.^3.*qdot0.*-1.305711e+7-q0.^2.*q2.^3.*qdot1.*1.305711e+7+q1.*q2.^2.*qdot0.*cos(q1+q2./1.1e+2).*2.63538e+7+q1.*q2.^2.*qdot1.*cos(q1+q2./1.1e+2).*2.63538e+7+q0.^2.*q2.*qdot2.*cos(q1+q2./1.1e+2).*7.90614e+7+q1.*q2.^2.*qdot2.*cos(q1+q2./1.1e+2).*4.7916e+5+q0.^2.*q2.^3.*qdot0.*cos(q1+q2).*1.31769e+7+q0.^2.*q2.^3.*qdot1.*cos(q1+q2).*1.31769e+7+q0.^2.*q2.^3.*qdot2.*cos(q1+q2).*1.31769e+7;
et61 = q1.*q2.^3.*qdot0.*sin(q1+q2./1.1e+2).*1.1979e+5+q1.*q2.^3.*qdot1.*sin(q1+q2./1.1e+2).*1.1979e+5+q1.*q2.^3.*qdot2.*sin(q1+q2./1.1e+2).*1.089e+3-q0.^2.*q2.^2.*qdot0.*sin(q1+q2).*2.63538e+7-q0.^2.*q2.^2.*qdot1.*sin(q1+q2).*2.63538e+7-q0.^2.*q2.^2.*qdot2.*sin(q1+q2).*5.27076e+7+q1.*q2.^2.*qdot0.*cos(q0+q1+q2).*2.63538e+7+q1.*q2.^2.*qdot1.*cos(q0+q1+q2).*2.63538e+7+q1.*q2.^2.*qdot2.*cos(q0+q1+q2).*5.27076e+7;
et62 = q1.*q2.*qdot2.*sin(q0+q1+q2./1.1e+2).*7.90614e+7+q1.*q2.^3.*qdot0.*sin(q0+q1+q2).*1.31769e+7+q1.*q2.^3.*qdot1.*sin(q0+q1+q2).*1.31769e+7+q1.*q2.^3.*qdot2.*sin(q0+q1+q2).*1.31769e+7+q0.^2.*q1.*qdot2.*cos(q2./1.1e+2).*5.27076e+7-q0.^2.*q2.*qdot2.*cos(q2./1.1e+2).*7.90614e+7+q0.^2.*q2.^3.*qdot0.*cos(q1).*1.305711e+7-q0.^2.*q2.^3.*qdot0.*cos(q2).*1.31769e+7+q0.^2.*q2.^3.*qdot1.*cos(q1).*1.305711e+7;
et63 = q0.^2.*q2.^3.*qdot1.*cos(q2).*-1.31769e+7-q0.^2.*q2.^3.*qdot2.*cos(q2).*1.31769e+7+q0.^2.*q2.^2.*qdot0.*sin(q2).*2.63538e+7+q0.^2.*q2.^2.*qdot1.*sin(q2).*2.63538e+7-q0.^2.*q2.^2.*qdot2.*sin(q1).*2.611422e+7+q0.^2.*q2.^2.*qdot2.*sin(q2).*5.27076e+7+q1.*q2.*qdot2.*sin(q1+q2).*7.90614e+7-q0.^2.*q2.^3.*qdot0.*cos(q1+q2./1.1e+2).*1.1979e+5-q0.^2.*q2.^3.*qdot1.*cos(q1+q2./1.1e+2).*1.1979e+5;
et64 = q0.^2.*q2.^3.*qdot2.*cos(q1+q2./1.1e+2).*-1.089e+3+q0.^2.*q2.^2.*qdot0.*sin(q1+q2./1.1e+2).*2.63538e+7+q0.^2.*q2.^2.*qdot1.*sin(q1+q2./1.1e+2).*2.63538e+7+q0.^2.*q2.^2.*qdot2.*sin(q1+q2./1.1e+2).*4.7916e+5-q1.*q2.^2.*qdot0.*cos(q0+q1+q2./1.1e+2).*2.63538e+7-q1.*q2.^2.*qdot1.*cos(q0+q1+q2./1.1e+2).*2.63538e+7-q1.*q2.^2.*qdot2.*cos(q0+q1+q2./1.1e+2).*4.7916e+5-q1.*q2.^3.*qdot0.*sin(q0+q1+q2./1.1e+2).*1.1979e+5;
et65 = q1.*q2.^3.*qdot1.*sin(q0+q1+q2./1.1e+2).*-1.1979e+5-q1.*q2.^3.*qdot2.*sin(q0+q1+q2./1.1e+2).*1.089e+3+q0.^2.*q2.^3.*qdot0.*cos(q2./1.1e+2).*1.1979e+5+q0.^2.*q2.^3.*qdot1.*cos(q2./1.1e+2).*1.1979e+5+q0.^2.*q2.^3.*qdot2.*cos(q2./1.1e+2).*1.089e+3-q0.^2.*q2.^2.*qdot0.*sin(q2./1.1e+2).*2.63538e+7-q0.^2.*q2.^2.*qdot1.*sin(q2./1.1e+2).*2.63538e+7-q0.^2.*q2.^2.*qdot2.*sin(q2./1.1e+2).*4.7916e+5;
et66 = q1.*q2.^2.*qdot0.*cos(q1+q2).*-2.63538e+7+q1.*q2.^2.*qdot2.*cos(q0+q1).*2.611422e+7-q1.*q2.^2.*qdot1.*cos(q1+q2).*2.63538e+7-q0.^2.*q2.*qdot2.*cos(q1+q2).*7.90614e+7-q1.*q2.^2.*qdot2.*cos(q1+q2).*5.27076e+7-q1.*q2.*qdot2.*sin(q1+q2./1.1e+2).*7.90614e+7+q1.*q2.^3.*qdot0.*sin(q0+q1).*1.305711e+7+q1.*q2.^3.*qdot1.*sin(q0+q1).*1.305711e+7-q1.*q2.^3.*qdot0.*sin(q1+q2).*1.31769e+7-q1.*q2.^3.*qdot1.*sin(q1+q2).*1.31769e+7;
et67 = q1.*q2.^3.*qdot2.*sin(q1+q2).*-1.31769e+7-q1.*q2.*qdot2.*sin(q0+q1+q2).*7.90614e+7+q0.^2.*q1.*q2.^2.*qdot0.*2.611422e+7+q0.^2.*q1.*q2.^2.*qdot1.*2.611422e+7+q0.^2.*q1.*q2.^2.*qdot2.*1.3175811e+7-q0.^2.*q1.*qdot2.*cos(q2).*5.27076e+7-q1.*q2.^2.*qdot2.*cos(q1).*2.611422e+7+q0.^2.*q2.*qdot2.*cos(q2).*7.90614e+7-q1.*q2.^3.*qdot0.*sin(q1).*1.305711e+7-q1.*q2.^3.*qdot1.*sin(q1).*1.305711e+7-q0.^2.*q1.*q2.^2.*qdot0.*cos(q2./1.1e+2).*1.1979e+5;
et68 = q0.^2.*q1.*q2.^2.*qdot1.*cos(q2./1.1e+2).*-1.1979e+5-q0.^2.*q1.*q2.^2.*qdot2.*cos(q2./1.1e+2).*1.089e+3-q0.*q1.*q2.*qdot2.*cos(q1+q2./1.1e+2).*7.90614e+7-q0.*q1.*q2.^3.*qdot0.*cos(q1+q2).*1.31769e+7-q0.*q1.*q2.^3.*qdot1.*cos(q1+q2).*1.31769e+7-q0.*q1.*q2.^3.*qdot2.*cos(q1+q2).*1.31769e+7+q0.*q1.*q2.^2.*qdot0.*sin(q1+q2).*2.63538e+7+q0.*q1.*q2.^2.*qdot1.*sin(q1+q2).*2.63538e+7+q0.*q1.*q2.^2.*qdot2.*sin(q1+q2).*5.27076e+7;
et69 = q0.*q1.*q2.^3.*qdot0.*cos(q1).*-1.305711e+7-q0.*q1.*q2.^3.*qdot1.*cos(q1).*1.305711e+7-q0.^2.*q1.*q2.*qdot0.*sin(q2).*3.95307e+7+q0.*q1.*q2.^2.*qdot2.*sin(q1).*2.611422e+7-q0.^2.*q1.*q2.*qdot1.*sin(q2).*3.95307e+7-q0.^2.*q1.*q2.*qdot2.*sin(q2).*5.27076e+7+q0.*q1.*q2.^3.*qdot0.*cos(q1+q2./1.1e+2).*1.1979e+5+q0.*q1.*q2.^3.*qdot1.*cos(q1+q2./1.1e+2).*1.1979e+5+q0.*q1.*q2.^3.*qdot2.*cos(q1+q2./1.1e+2).*1.089e+3-q0.*q1.*q2.^2.*qdot0.*sin(q1+q2./1.1e+2).*2.63538e+7;
et70 = q0.*q1.*q2.^2.*qdot1.*sin(q1+q2./1.1e+2).*-2.63538e+7-q0.*q1.*q2.^2.*qdot2.*sin(q1+q2./1.1e+2).*4.7916e+5+q0.^2.*q1.*q2.^2.*qdot0.*cos(q2).*1.31769e+7+q0.^2.*q1.*q2.^2.*qdot1.*cos(q2).*1.31769e+7+q0.^2.*q1.*q2.^2.*qdot2.*cos(q2).*1.31769e+7+q0.^2.*q1.*q2.*qdot0.*sin(q2./1.1e+2).*3.95307e+7+q0.^2.*q1.*q2.*qdot1.*sin(q2./1.1e+2).*3.95307e+7+q0.^2.*q1.*q2.*qdot2.*sin(q2./1.1e+2).*4.7916e+5+q0.*q1.*q2.*qdot2.*cos(q1+q2).*7.90614e+7;
if (q2 ~= 0.0)
    t0 = (1.0./q0.^6.*1.0./q1.^4.*1.0./q2.^4.*(et15+et16+et17+et18+et19+et20+et21+et22+et23+et24+et25+et26+et27+et28))./1.0e+11;
else
    t0 = NaN;
end
if (q2 ~= 0.0)
    t1 = 1.0./q0.^3.*1.0./q1.^4.*1.0./q2.^4.*(et7+et8+et9+et10+et11+et12+et13+et14).*(-1.0e-9);
else
    t1 = NaN;
end
if (q2 ~= 0.0)
    t2 = 1.0./q0.^3.*1.0./q1.^2.*1.0./q2.^4.*(et1+et2+et3+et4+et5+et6).*(-1.0e-9);
else
    t2 = NaN;
end
if (q2 ~= 0.0)
    t3 = (1.0./q0.^2.*1.0./q1.^5.*1.0./q2.^4.*(et38+et39+et40+et41+et42+et43+et44+et45+et46+et47+et48+et49+et50+et51+et52))./1.0e+10;
else
    t3 = NaN;
end
mt1 = [t0,t1,t2,t3];
if (q2 ~= 0.0)
    t0 = (1.0./q1.^3.*1.0./q2.^4.*(et29+et30+et31+et32))./1.0e+9;
else
    t0 = NaN;
end
if (q2 ~= 0.0)
    t1 = (1.0./q0.^2.*1.0./q2.^5.*(et60+et61+et62+et63+et64+et65+et66+et67+et68+et69+et70).*(-1.0e-10))./q1;
else
    t1 = NaN;
end
if (q2 ~= 0.0)
    t2 = 1.0./q1.^2.*1.0./q2.^5.*(et53+et54+et55+et56+et57+et58+et59).*(-1.0e-10);
else
    t2 = NaN;
end
mt2 = [1.0./q1.^6.*1.0./q2.^4.*(et33+et34+et35+et36+et37).*(-1.0e-11),t0,t1,t2];
if (q2 ~= 0.0)
    t0 = 1.0./q2.^6.*qdot2.*(q2.*1.58268e+7+sin(q2./1.1e+2).*3.993e+7-sin(q2).*3.993e+7-q2.*cos(q2./1.1e+2).*2.178e+5+q2.^2.*sin(q2).*3.993e+6-q2.^2.*sin(q2./1.1e+2).*3.3e+2+q2.*cos(q2).*2.3958e+7+q2.^3.*1.330999e+6).*(-3.3e-10);
else
    t0 = NaN;
end
mt3 = [t0];
C = reshape([mt1,mt2,mt3],3,3);
