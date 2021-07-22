function x2 = q2x2_fun(s,in2,alpha)
%Q2X2_FUN
%    X2 = Q2X2_FUN(S,IN2,ALPHA)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    24-May-2021 16:04:28

q0 = in2(1,:);
q1 = in2(2,:);
q2 = in2(3,:);
x2 = [((sin(alpha+q0+q1)-sin(alpha+q0+q1+q2.*s.*(1.0e+2./1.1e+1))).*(-1.1e+1./1.0e+2))./q2+((sin(alpha+q0)-sin(alpha)).*(1.1e+1./1.0e+2))./q0+((sin(alpha+q0+q1)-sin(alpha+q0)).*(1.1e+1./1.0e+2))./q1;((cos(alpha+q0+q1)-cos(alpha+q0+q1+q2.*s.*(1.0e+2./1.1e+1))).*(1.1e+1./1.0e+2))./q2-((cos(alpha+q0)-cos(alpha)).*(1.1e+1./1.0e+2))./q0-((cos(alpha+q0+q1)-cos(alpha+q0)).*(1.1e+1./1.0e+2))./q1];