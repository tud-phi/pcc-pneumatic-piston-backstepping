function theta0 = q2theta0_fun(s,in2,alpha)
%Q2THETA0_FUN
%    THETA0 = Q2THETA0_FUN(S,IN2,ALPHA)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    26-Oct-2021 11:02:30

q0 = in2(1,:);
theta0 = alpha+q0.*s.*(1.0e+2./1.1e+1);
