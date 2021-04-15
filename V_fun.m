function V = V_fun(in1,in2,in3,in4,b_C,in6)
%V_FUN
%    V = V_FUN(IN1,IN2,IN3,IN4,B_C,IN6)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    15-Apr-2021 21:38:21

A_p0 = in4(1,:);
A_p1 = in4(2,:);
A_p2 = in4(3,:);
A_p3 = in4(4,:);
A_p4 = in4(5,:);
A_p5 = in4(6,:);
d_Ca = in6(1,:);
d_Cb = in6(2,:);
l0 = in3(1,:);
l1 = in3(2,:);
l2 = in3(3,:);
mu_p0 = in2(1,:);
mu_p1 = in2(2,:);
mu_p2 = in2(3,:);
mu_p3 = in2(4,:);
mu_p4 = in2(5,:);
mu_p5 = in2(6,:);
q0 = in1(1,:);
q1 = in1(2,:);
q2 = in1(3,:);
V = [A_p0.*mu_p0-b_C.*(l0+(q0.*(d_Ca-d_Cb))./2.0).*(d_Ca-d_Cb);A_p1.*mu_p1-b_C.*(l0-(q0.*(d_Ca-d_Cb))./2.0).*(d_Ca-d_Cb);A_p2.*mu_p2-b_C.*(l1+(q1.*(d_Ca-d_Cb))./2.0).*(d_Ca-d_Cb);A_p3.*mu_p3-b_C.*(l1-(q1.*(d_Ca-d_Cb))./2.0).*(d_Ca-d_Cb);A_p4.*mu_p4-b_C.*(l2+(q2.*(d_Ca-d_Cb))./2.0).*(d_Ca-d_Cb);A_p5.*mu_p5-b_C.*(l2-(q2.*(d_Ca-d_Cb))./2.0).*(d_Ca-d_Cb)];
