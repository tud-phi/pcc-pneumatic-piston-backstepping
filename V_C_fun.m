function V_C = V_C_fun(in1,in2,b_C,in4)
%V_C_FUN
%    V_C = V_C_FUN(IN1,IN2,B_C,IN4)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    15-Apr-2021 21:38:21

d_Ca = in4(1,:);
d_Cb = in4(2,:);
l0 = in2(1,:);
l1 = in2(2,:);
l2 = in2(3,:);
q0 = in1(1,:);
q1 = in1(2,:);
q2 = in1(3,:);
V_C = [-b_C.*(l0+(q0.*(d_Ca-d_Cb))./2.0).*(d_Ca-d_Cb);-b_C.*(l0-(q0.*(d_Ca-d_Cb))./2.0).*(d_Ca-d_Cb);-b_C.*(l1+(q1.*(d_Ca-d_Cb))./2.0).*(d_Ca-d_Cb);-b_C.*(l1-(q1.*(d_Ca-d_Cb))./2.0).*(d_Ca-d_Cb);-b_C.*(l2+(q2.*(d_Ca-d_Cb))./2.0).*(d_Ca-d_Cb);-b_C.*(l2-(q2.*(d_Ca-d_Cb))./2.0).*(d_Ca-d_Cb)];
