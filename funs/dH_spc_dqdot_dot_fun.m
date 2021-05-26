function dH_spc_dqdot_dot = dH_spc_dqdot_dot_fun(in1,in2,in3,alpha)
%DH_SPC_DQDOT_DOT_FUN
%    DH_SPC_DQDOT_DOT = DH_SPC_DQDOT_DOT_FUN(IN1,IN2,IN3,ALPHA)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    24-May-2021 16:04:35

q0 = in1(1,:);
q1 = in1(2,:);
q2 = in1(3,:);
qddot0 = in3(1,:);
qddot1 = in3(2,:);
qddot2 = in3(3,:);
qdot0 = in2(1,:);
qdot1 = in2(2,:);
qdot2 = in2(3,:);
t2 = cos(q0);
t3 = cos(q1);
t4 = cos(q2);
t5 = sin(q0);
t6 = sin(q1);
t7 = sin(q2);
t8 = q0+q1;
t9 = q1+q2;
t10 = q0.^2;
t11 = q0.^3;
t12 = q1.^2;
t14 = q1.^3;
t15 = q2.^2;
t16 = q0.^5;
t18 = q2.^3;
t20 = q1.^5;
t23 = q2.^5;
t24 = (q2 ~= 0.0);
t25 = qdot0.^2;
t26 = qdot1.^2;
t27 = qdot2.^2;
t37 = q0./1.1e+2;
t38 = q1./1.1e+2;
t39 = q2./1.1e+2;
t13 = t10.^2;
t17 = t12.^2;
t19 = t10.^3;
t21 = t15.^2;
t22 = t12.^3;
t28 = cos(t8);
t29 = cos(t9);
t30 = q2+t8;
t31 = sin(t8);
t32 = sin(t9);
t33 = 1.0./t11;
t34 = 1.0./t23;
t40 = cos(t37);
t41 = cos(t38);
t42 = cos(t39);
t43 = sin(t37);
t44 = sin(t38);
t45 = sin(t39);
t46 = q0+t38;
t47 = q1+t39;
t50 = t8+t39;
t55 = qdot0.*qdot1.*t6.*t11.*t12.*t23.*5.27076e+8;
t56 = qdot1.*qdot2.*t7.*t11.*t14.*t15.*5.27076e+8;
t35 = cos(t30);
t36 = sin(t30);
t48 = cos(t46);
t49 = cos(t47);
t51 = sin(t46);
t52 = sin(t47);
t53 = cos(t50);
t54 = sin(t50);
t57 = -t56;
if (t24)
    t0 = (t34.*(q0.*t20.*t23.*t25.*-3.1337064e+9-qddot1.*t13.*t14.*t23.*2.611422e+8+qddot0.*t10.*t20.*t23.*7.834266e+8+qddot0.*t13.*t20.*t23.*3.05065167e+8+qddot0.*t14.*t19.*t23.*5.222844e+8-qddot0.*t16.*t17.*t23.*5.222844e+8-qddot0.*t17.*t19.*t21.*2.611422e+8+qddot0.*t18.*t19.*t20.*2.611422e+8+qddot1.*t14.*t19.*t23.*1.96450155e+8-qddot1.*t17.*t19.*t21.*2.611422e+8+qddot1.*t18.*t19.*t20.*2.611422e+8+qddot2.*t18.*t19.*t20.*6.5879055e+7+t4.*t19.*t20.*t27.*5.27076e+8+t3.*t19.*t23.*t26.*5.27076e+8+t5.*t20.*t23.*t25.*2.63538e+9+t12.*t13.*t23.*t26.*5.222844e+8+t13.*t17.*t23.*t25.*5.222844e+8-t11.*t20.*t23.*t25.*6.10130334e+8-t12.*t19.*t23.*t26.*3.9290031e+8+t14.*t19.*t21.*t26.*2.611422e+8-t15.*t19.*t20.*t27.*1.3175811e+8-t19.*t20.*t27.*t42.*5.27076e+8-t19.*t23.*t26.*t41.*5.27076e+8-t20.*t23.*t25.*t43.*2.63538e+9-qddot0.*t13.*t18.*t20.*cos(t50).*2.63538e+8-qddot1.*t13.*t18.*t20.*cos(t50).*1.31769e+8-qddot2.*t13.*t18.*t20.*cos(t50).*1.1979e+6+t11.*t18.*t20.*t25.*cos(t50).*5.27076e+8+t13.*t15.*t20.*t27.*cos(t50).*4.7916e+6-q2.*t13.*t20.*t27.*sin(t50).*7.90614e+8+qddot2.*t13.*t15.*t20.*sin(t50).*2.63538e+8+t13.*t18.*t20.*t25.*sin(t50).*2.63538e+8+t13.*t18.*t20.*t26.*sin(t50).*1.31769e+8+t13.*t18.*t20.*t27.*sin(t50).*1.089e+4-q1.*qddot1.*t3.*t19.*t23.*1.31769e+8-q2.*qddot2.*t4.*t19.*t20.*1.31769e+8-q0.*qddot0.*t5.*t20.*t23.*5.27076e+8+q1.*qddot1.*t19.*t23.*t41.*1.31769e+8+q2.*qddot2.*t19.*t20.*t42.*1.31769e+8+q0.*qddot0.*t20.*t23.*t43.*5.27076e+8+qdot0.*qdot1.*t11.*t14.*t23.*5.222844e+8+qdot0.*qdot1.*t14.*t16.*t23.*5.222844e+8-qdot0.*qdot1.*t12.*t19.*t23.*1.0445688e+9+qdot0.*qdot1.*t14.*t19.*t21.*2.611422e+8-qdot0.*qdot2.*t15.*t19.*t20.*5.222844e+8+qdot0.*qdot2.*t17.*t18.*t19.*2.611422e+8-qdot1.*qdot2.*t15.*t19.*t20.*5.222844e+8+qdot1.*qdot2.*t17.*t18.*t19.*2.611422e+8-q1.*t3.*t16.*t23.*t26.*7.90614e+8-q1.*t6.*t13.*t23.*t26.*7.90614e+8-q2.*t4.*t17.*t19.*t27.*7.90614e+8+q0.*t2.*t20.*t23.*t25.*5.079096e+8+q1.*t6.*t19.*t23.*t26.*5.27076e+8+q2.*t7.*t19.*t20.*t27.*5.27076e+8+q1.*t13.*t23.*t26.*t31.*7.90614e+8-q2.*t13.*t20.*t27.*t32.*7.90614e+8-q2.*t16.*t20.*t27.*t29.*7.90614e+8+q2.*t17.*t19.*t27.*t29.*7.90614e+8+q2.*t13.*t20.*t27.*t36.*7.90614e+8+q1.*t13.*t23.*t26.*t44.*7.90614e+8+q1.*t16.*t23.*t26.*t41.*7.90614e+8+q2.*t17.*t19.*t27.*t42.*7.90614e+8+q0.*t20.*t23.*t25.*t40.*1.43748e+7-q1.*t19.*t23.*t26.*t44.*4.7916e+6-q2.*t19.*t20.*t27.*t45.*4.7916e+6-q1.*t13.*t23.*t26.*t51.*7.90614e+8+q2.*t13.*t20.*t27.*t52.*7.90614e+8+q2.*t16.*t20.*t27.*t49.*7.90614e+8-q2.*t17.*t19.*t27.*t49.*7.90614e+8-qddot0.*t3.*t13.*t14.*t23.*2.63538e+8+qddot1.*t2.*t13.*t14.*t23.*2.611422e+8-qddot1.*t3.*t13.*t14.*t23.*1.1979e+6-qddot0.*t2.*t10.*t20.*t23.*2.587464e+8+qddot1.*t3.*t12.*t16.*t23.*2.63538e+8+qddot1.*t6.*t12.*t13.*t23.*2.63538e+8-qddot1.*t4.*t14.*t18.*t19.*1.31769e+8-qddot2.*t3.*t13.*t18.*t20.*1.305711e+8+qddot2.*t4.*t15.*t17.*t19.*2.63538e+8+qddot0.*t5.*t13.*t17.*t23.*5.222844e+8-qddot0.*t3.*t14.*t19.*t23.*2.611422e+8+qddot0.*t3.*t16.*t17.*t23.*2.611422e+8-qddot0.*t5.*t11.*t20.*t23.*5.222844e+8+qddot0.*t6.*t13.*t17.*t23.*2.611422e+8+qddot0.*t6.*t14.*t16.*t23.*2.63538e+8-qddot0.*t3.*t16.*t20.*t21.*2.611422e+8+qddot0.*t3.*t17.*t19.*t21.*2.611422e+8-qddot0.*t6.*t12.*t19.*t23.*2.63538e+8-qddot0.*t6.*t13.*t20.*t21.*2.611422e+8-qddot1.*t3.*t14.*t19.*t23.*1.305711e+8+qddot1.*t3.*t16.*t17.*t23.*1.305711e+8+qddot1.*t6.*t13.*t17.*t23.*1.305711e+8+qddot1.*t6.*t14.*t16.*t23.*1.1979e+6-qddot0.*t7.*t15.*t19.*t20.*2.63538e+8+qddot0.*t7.*t17.*t18.*t19.*2.63538e+8-qddot1.*t3.*t16.*t20.*t21.*1.305711e+8+qddot1.*t3.*t17.*t19.*t21.*1.305711e+8-qddot1.*t6.*t12.*t19.*t23.*1.31769e+8-qddot1.*t6.*t13.*t20.*t21.*1.305711e+8+qddot1.*t6.*t14.*t19.*t21.*1.305711e+8-qddot1.*t7.*t15.*t19.*t20.*2.63538e+8+qddot1.*t7.*t17.*t18.*t19.*2.63538e+8+qddot2.*t6.*t16.*t18.*t20.*1.305711e+8-qddot2.*t6.*t17.*t18.*t19.*1.305711e+8-qddot2.*t7.*t15.*t19.*t20.*1.31769e+8+qddot2.*t7.*t17.*t18.*t19.*1.31769e+8+qddot0.*t13.*t14.*t23.*t28.*2.63538e+8+qddot1.*t13.*t14.*t23.*t28.*1.1979e+6-qddot0.*t13.*t18.*t20.*t29.*2.63538e+8-qddot1.*t12.*t13.*t23.*t31.*2.63538e+8-qddot1.*t13.*t18.*t20.*t29.*1.31769e+8+qddot1.*t14.*t18.*t19.*t29.*1.31769e+8+qddot2.*t13.*t18.*t20.*t28.*1.305711e+8+qddot2.*t13.*t15.*t20.*t32.*2.63538e+8-qddot2.*t13.*t18.*t20.*t29.*1.31769e+8+qddot2.*t15.*t16.*t20.*t29.*2.63538e+8-qddot2.*t15.*t17.*t19.*t29.*2.63538e+8-qddot0.*t13.*t17.*t23.*t31.*2.611422e+8+qddot0.*t13.*t20.*t21.*t31.*2.611422e+8-qddot1.*t13.*t17.*t23.*t31.*1.305711e+8+qddot0.*t13.*t18.*t20.*t35.*2.63538e+8+qddot0.*t16.*t18.*t20.*t32.*2.63538e+8-qddot0.*t17.*t18.*t19.*t32.*2.63538e+8+qddot1.*t13.*t20.*t21.*t31.*1.305711e+8-qddot2.*t13.*t15.*t20.*t36.*2.63538e+8+qddot1.*t13.*t18.*t20.*t35.*1.31769e+8+qddot1.*t16.*t18.*t20.*t32.*1.31769e+8-qddot1.*t17.*t18.*t19.*t32.*1.31769e+8+qddot2.*t13.*t18.*t20.*t35.*1.31769e+8+qddot2.*t16.*t18.*t20.*t32.*1.31769e+8-qddot2.*t17.*t18.*t19.*t32.*1.31769e+8+qddot0.*t13.*t14.*t23.*t41.*2.63538e+8+qddot1.*t13.*t14.*t23.*t41.*1.1979e+6-qddot0.*t10.*t20.*t23.*t40.*2.3958e+6-qddot1.*t12.*t13.*t23.*t44.*2.63538e+8-qddot1.*t12.*t16.*t23.*t41.*2.63538e+8+qddot1.*t14.*t18.*t19.*t42.*1.31769e+8-qddot2.*t15.*t17.*t19.*t42.*2.63538e+8-qddot0.*t14.*t16.*t23.*t44.*2.63538e+8+qddot0.*t12.*t19.*t23.*t44.*2.63538e+8-qddot0.*t13.*t14.*t23.*t48.*2.63538e+8-qddot1.*t14.*t16.*t23.*t44.*1.1979e+6+qddot0.*t15.*t19.*t20.*t45.*2.63538e+8-qddot0.*t17.*t18.*t19.*t45.*2.63538e+8+qddot1.*t12.*t19.*t23.*t44.*1.1979e+6-qddot1.*t13.*t14.*t23.*t48.*1.1979e+6+qddot0.*t13.*t18.*t20.*t49.*2.63538e+8+qddot1.*t12.*t13.*t23.*t51.*2.63538e+8+qddot1.*t15.*t19.*t20.*t45.*2.63538e+8-qddot1.*t17.*t18.*t19.*t45.*2.63538e+8+qddot1.*t13.*t18.*t20.*t49.*1.31769e+8-qddot1.*t14.*t18.*t19.*t49.*1.31769e+8+qddot2.*t15.*t19.*t20.*t45.*1.1979e+6-qddot2.*t17.*t18.*t19.*t45.*1.1979e+6-qddot2.*t13.*t15.*t20.*t52.*2.63538e+8+qddot2.*t13.*t18.*t20.*t49.*1.1979e+6-qddot2.*t15.*t16.*t20.*t49.*2.63538e+8+qddot2.*t15.*t17.*t19.*t49.*2.63538e+8-qddot0.*t16.*t18.*t20.*t52.*2.63538e+8+qddot0.*t17.*t18.*t19.*t52.*2.63538e+8-qddot1.*t16.*t18.*t20.*t52.*1.31769e+8+qddot1.*t17.*t18.*t19.*t52.*1.31769e+8-qddot2.*t16.*t18.*t20.*t52.*1.1979e+6+qddot2.*t17.*t18.*t19.*t52.*1.1979e+6-t2.*t12.*t13.*t23.*t26.*5.222844e+8+t3.*t11.*t14.*t23.*t25.*5.27076e+8+t3.*t12.*t13.*t23.*t26.*2.659338e+8+t3.*t13.*t15.*t20.*t27.*2.611422e+8+t4.*t12.*t18.*t19.*t26.*2.63538e+8+t2.*t13.*t17.*t23.*t25.*5.222844e+8-t2.*t11.*t20.*t23.*t25.*5.222844e+8-t3.*t13.*t17.*t23.*t25.*2.611422e+8-t5.*t11.*t17.*t23.*t25.*1.0445688e+9-t6.*t13.*t14.*t23.*t25.*2.63538e+8+t3.*t13.*t17.*t23.*t26.*1.305711e+8+t3.*t13.*t20.*t21.*t25.*2.611422e+8-t3.*t14.*t16.*t23.*t26.*1.293732e+8-t6.*t11.*t17.*t23.*t25.*5.222844e+8-t6.*t13.*t14.*t23.*t26.*1.293732e+8+t3.*t12.*t19.*t23.*t26.*1.293732e+8-t3.*t13.*t20.*t21.*t26.*1.305711e+8+t5.*t10.*t20.*t23.*t25.*1.8255996e+9+t6.*t11.*t20.*t21.*t25.*5.222844e+8-t6.*t12.*t16.*t23.*t26.*2.659338e+8-t6.*t12.*t19.*t21.*t26.*2.611422e+8-t6.*t15.*t16.*t20.*t27.*2.611422e+8+t6.*t15.*t17.*t19.*t27.*2.611422e+8-t7.*t14.*t18.*t19.*t26.*2.63538e+8-t4.*t15.*t19.*t20.*t27.*1.31769e+8+t4.*t17.*t18.*t19.*t27.*1.31769e+8-t7.*t15.*t17.*t19.*t27.*5.27076e+8+t6.*t14.*t19.*t23.*t26.*1.305711e+8-t6.*t16.*t17.*t23.*t26.*1.305711e+8+t6.*t16.*t20.*t21.*t26.*1.305711e+8-t6.*t17.*t19.*t21.*t26.*1.305711e+8-t11.*t14.*t23.*t25.*t28.*5.27076e+8-t12.*t13.*t23.*t26.*t28.*2.659338e+8+t11.*t18.*t20.*t25.*t29.*5.27076e+8-t13.*t15.*t20.*t27.*t28.*2.611422e+8-t12.*t18.*t19.*t26.*t29.*2.63538e+8+t13.*t15.*t20.*t27.*t29.*5.27076e+8-t13.*t14.*t23.*t25.*t31.*2.63538e+8-t13.*t17.*t23.*t25.*t28.*2.611422e+8+t11.*t17.*t23.*t25.*t31.*5.222844e+8+t13.*t14.*t23.*t26.*t31.*1.293732e+8-t13.*t17.*t23.*t26.*t28.*1.305711e+8+t13.*t20.*t21.*t25.*t28.*2.611422e+8-t11.*t20.*t21.*t25.*t31.*5.222844e+8-t13.*t18.*t20.*t25.*t32.*2.63538e+8+t13.*t20.*t21.*t26.*t28.*1.305711e+8-t11.*t18.*t20.*t25.*t35.*5.27076e+8+t13.*t18.*t20.*t26.*t32.*1.31769e+8+t16.*t18.*t20.*t26.*t29.*1.31769e+8-t17.*t18.*t19.*t26.*t29.*1.31769e+8-t13.*t15.*t20.*t27.*t35.*5.27076e+8+t13.*t18.*t20.*t27.*t32.*1.31769e+8-t15.*t16.*t20.*t27.*t32.*5.27076e+8+t15.*t17.*t19.*t27.*t32.*5.27076e+8+t16.*t18.*t20.*t27.*t29.*1.31769e+8-t17.*t18.*t19.*t27.*t29.*1.31769e+8-t13.*t18.*t20.*t25.*t36.*2.63538e+8-t13.*t18.*t20.*t26.*t36.*1.31769e+8-t11.*t14.*t23.*t25.*t41.*5.27076e+8-t13.*t18.*t20.*t27.*t36.*1.31769e+8-t12.*t13.*t23.*t26.*t41.*4.7916e+6-t12.*t18.*t19.*t26.*t42.*2.63538e+8+t13.*t14.*t23.*t25.*t44.*2.63538e+8-t13.*t14.*t23.*t26.*t44.*1.089e+4-t14.*t16.*t23.*t26.*t41.*1.089e+4+t10.*t20.*t23.*t25.*t43.*2.178e+4+t11.*t14.*t23.*t25.*t48.*5.27076e+8+t12.*t16.*t23.*t26.*t44.*4.7916e+6+t12.*t19.*t23.*t26.*t41.*1.089e+4+t12.*t13.*t23.*t26.*t48.*4.7916e+6+t14.*t18.*t19.*t26.*t45.*2.63538e+8-t11.*t18.*t20.*t25.*t49.*5.27076e+8+t15.*t17.*t19.*t27.*t45.*4.7916e+6+t15.*t19.*t20.*t27.*t42.*1.089e+4-t17.*t18.*t19.*t27.*t42.*1.089e+4+t12.*t18.*t19.*t26.*t49.*2.63538e+8-t13.*t15.*t20.*t27.*t49.*4.7916e+6+t13.*t14.*t23.*t25.*t51.*2.63538e+8+t13.*t14.*t23.*t26.*t51.*1.089e+4+t13.*t18.*t20.*t25.*t52.*2.63538e+8-t13.*t18.*t20.*t26.*t52.*1.31769e+8-t16.*t18.*t20.*t26.*t49.*1.31769e+8+t17.*t18.*t19.*t26.*t49.*1.31769e+8-t13.*t18.*t20.*t27.*t52.*1.089e+4+t15.*t16.*t20.*t27.*t52.*4.7916e+6-t15.*t17.*t19.*t27.*t52.*4.7916e+6-t16.*t18.*t20.*t27.*t49.*1.089e+4+t17.*t18.*t19.*t27.*t49.*1.089e+4+q1.*qdot0.*qdot1.*t6.*t19.*t23.*7.90614e+8+q2.*qdot0.*qdot2.*t7.*t19.*t20.*7.90614e+8+q2.*qdot1.*qdot2.*t7.*t19.*t20.*7.90614e+8-q1.*qdot0.*qdot1.*t19.*t23.*t44.*7.90614e+8-q2.*qdot0.*qdot2.*t19.*t20.*t45.*7.90614e+8-q2.*qdot1.*qdot2.*t19.*t20.*t45.*7.90614e+8-qdot0.*qdot1.*t2.*t11.*t14.*t23.*5.222844e+8+qdot0.*qdot1.*t3.*t11.*t14.*t23.*2.3958e+6+qdot0.*qdot1.*t3.*t12.*t13.*t23.*2.63538e+8-qdot0.*qdot1.*t6.*t11.*t12.*t23.*5.27076e+8+qdot0.*qdot2.*t3.*t11.*t18.*t20.*2.611422e+8-qdot0.*qdot1.*t5.*t13.*t14.*t23.*7.834266e+8+qdot0.*qdot1.*t3.*t13.*t17.*t23.*1.305711e+8+qdot0.*qdot1.*t3.*t14.*t16.*t23.*2.3958e+6+qdot0.*qdot1.*t6.*t13.*t14.*t23.*1.1979e+6+qdot0.*qdot1.*t3.*t12.*t19.*t23.*2.587464e+8-qdot0.*qdot1.*t3.*t13.*t20.*t21.*1.305711e+8-qdot0.*qdot1.*t3.*t14.*t19.*t21.*2.611422e+8-qdot0.*qdot1.*t6.*t11.*t17.*t23.*2.611422e+8-qdot0.*qdot1.*t6.*t12.*t16.*t23.*5.27076e+8+qdot0.*qdot1.*t6.*t11.*t20.*t21.*2.611422e+8-qdot0.*qdot1.*t7.*t14.*t18.*t19.*2.63538e+8+qdot0.*qdot2.*t3.*t16.*t18.*t20.*2.611422e+8-qdot0.*qdot2.*t3.*t17.*t18.*t19.*2.611422e+8+qdot0.*qdot2.*t6.*t13.*t18.*t20.*1.305711e+8-qdot0.*qdot2.*t4.*t15.*t19.*t20.*2.63538e+8+qdot0.*qdot2.*t4.*t17.*t18.*t19.*2.63538e+8-qdot0.*qdot2.*t7.*t15.*t17.*t19.*5.27076e+8+qdot1.*qdot2.*t3.*t16.*t18.*t20.*2.611422e+8-qdot1.*qdot2.*t3.*t17.*t18.*t19.*2.611422e+8+qdot1.*qdot2.*t6.*t13.*t18.*t20.*2.611422e+8-qdot1.*qdot2.*t4.*t15.*t19.*t20.*2.63538e+8+qdot1.*qdot2.*t4.*t17.*t18.*t19.*2.63538e+8-qdot1.*qdot2.*t7.*t15.*t17.*t19.*5.27076e+8+qdot0.*qdot1.*t6.*t14.*t19.*t23.*2.611422e+8-qdot0.*qdot1.*t6.*t16.*t17.*t23.*2.611422e+8+qdot0.*qdot1.*t6.*t16.*t20.*t21.*2.611422e+8-qdot0.*qdot1.*t6.*t17.*t19.*t21.*2.611422e+8-qdot0.*qdot1.*t11.*t14.*t23.*t28.*2.3958e+6-qdot0.*qdot1.*t12.*t13.*t23.*t28.*7.90614e+8+qdot0.*qdot1.*t11.*t12.*t23.*t31.*5.27076e+8+qdot0.*qdot1.*t11.*t18.*t20.*t29.*2.63538e+8-qdot0.*qdot2.*t11.*t18.*t20.*t28.*2.611422e+8+qdot0.*qdot2.*t13.*t15.*t20.*t29.*2.63538e+8-qdot0.*qdot2.*t11.*t15.*t20.*t32.*5.27076e+8+qdot0.*qdot2.*t11.*t18.*t20.*t29.*2.63538e+8+qdot1.*qdot2.*t13.*t15.*t20.*t29.*5.27076e+8-qdot0.*qdot1.*t13.*t14.*t23.*t31.*3.5937e+6-qdot0.*qdot1.*t13.*t17.*t23.*t28.*3.917133e+8+qdot0.*qdot1.*t11.*t17.*t23.*t31.*2.611422e+8+qdot0.*qdot1.*t13.*t20.*t21.*t28.*3.917133e+8-qdot0.*qdot1.*t11.*t20.*t21.*t31.*2.611422e+8+qdot0.*qdot1.*t13.*t18.*t20.*t32.*1.31769e+8+qdot0.*qdot1.*t14.*t18.*t19.*t32.*2.63538e+8+qdot0.*qdot1.*t16.*t18.*t20.*t29.*2.63538e+8-qdot0.*qdot1.*t17.*t18.*t19.*t29.*2.63538e+8+qdot0.*qdot2.*t11.*t15.*t20.*t36.*5.27076e+8-qdot0.*qdot2.*t13.*t18.*t20.*t31.*3.917133e+8-qdot0.*qdot1.*t11.*t18.*t20.*t35.*2.63538e+8-qdot0.*qdot2.*t13.*t15.*t20.*t35.*7.90614e+8+qdot0.*qdot2.*t13.*t18.*t20.*t32.*1.31769e+8-qdot0.*qdot2.*t15.*t16.*t20.*t32.*5.27076e+8+qdot0.*qdot2.*t15.*t17.*t19.*t32.*5.27076e+8+qdot0.*qdot2.*t16.*t18.*t20.*t29.*2.63538e+8-qdot0.*qdot2.*t17.*t18.*t19.*t29.*2.63538e+8-qdot1.*qdot2.*t13.*t18.*t20.*t31.*2.611422e+8-qdot0.*qdot2.*t11.*t18.*t20.*t35.*2.63538e+8-qdot1.*qdot2.*t13.*t15.*t20.*t35.*5.27076e+8+qdot1.*qdot2.*t13.*t18.*t20.*t32.*2.63538e+8-qdot1.*qdot2.*t15.*t16.*t20.*t32.*5.27076e+8+qdot1.*qdot2.*t15.*t17.*t19.*t32.*5.27076e+8+qdot1.*qdot2.*t16.*t18.*t20.*t29.*2.63538e+8-qdot1.*qdot2.*t17.*t18.*t19.*t29.*2.63538e+8-qdot0.*qdot1.*t13.*t18.*t20.*t36.*3.95307e+8-qdot0.*qdot2.*t13.*t18.*t20.*t36.*3.95307e+8-qdot0.*qdot1.*t11.*t14.*t23.*t41.*2.3958e+6-qdot0.*qdot1.*t12.*t13.*t23.*t41.*2.63538e+8-qdot1.*qdot2.*t13.*t18.*t20.*t36.*2.63538e+8+qdot0.*qdot1.*t11.*t12.*t23.*t44.*5.27076e+8-qdot0.*qdot1.*t13.*t14.*t23.*t44.*1.1979e+6-qdot0.*qdot1.*t14.*t16.*t23.*t41.*2.3958e+6+qdot0.*qdot1.*t12.*t16.*t23.*t44.*5.27076e+8+qdot0.*qdot1.*t12.*t19.*t23.*t41.*2.3958e+6+qdot0.*qdot1.*t11.*t14.*t23.*t48.*2.3958e+6+qdot0.*qdot1.*t12.*t13.*t23.*t48.*7.90614e+8+qdot0.*qdot1.*t14.*t18.*t19.*t45.*2.63538e+8-qdot0.*qdot1.*t11.*t12.*t23.*t51.*5.27076e+8+qdot0.*qdot2.*t15.*t17.*t19.*t45.*5.27076e+8+qdot0.*qdot2.*t15.*t19.*t20.*t42.*2.3958e+6-qdot0.*qdot2.*t17.*t18.*t19.*t42.*2.3958e+6-qdot0.*qdot1.*t11.*t18.*t20.*t49.*2.63538e+8-qdot0.*qdot2.*t13.*t15.*t20.*t49.*2.63538e+8+qdot1.*qdot2.*t15.*t17.*t19.*t45.*5.27076e+8+qdot1.*qdot2.*t15.*t19.*t20.*t42.*2.3958e+6-qdot1.*qdot2.*t17.*t18.*t19.*t42.*2.3958e+6+qdot0.*qdot2.*t11.*t15.*t20.*t52.*5.27076e+8-qdot0.*qdot2.*t11.*t18.*t20.*t49.*2.3958e+6-qdot1.*qdot2.*t13.*t15.*t20.*t49.*5.27076e+8+qdot0.*qdot1.*t13.*t14.*t23.*t51.*3.5937e+6-qdot0.*qdot1.*t13.*t18.*t20.*t52.*1.31769e+8-qdot0.*qdot1.*t14.*t18.*t19.*t52.*2.63538e+8-qdot0.*qdot1.*t16.*t18.*t20.*t49.*2.63538e+8+qdot0.*qdot1.*t17.*t18.*t19.*t49.*2.63538e+8-qdot0.*qdot2.*t13.*t18.*t20.*t52.*1.1979e+6+qdot0.*qdot2.*t15.*t16.*t20.*t52.*5.27076e+8-qdot0.*qdot2.*t15.*t17.*t19.*t52.*5.27076e+8-qdot0.*qdot2.*t16.*t18.*t20.*t49.*2.3958e+6+qdot0.*qdot2.*t17.*t18.*t19.*t49.*2.3958e+6-qdot1.*qdot2.*t13.*t18.*t20.*t52.*2.3958e+6+qdot1.*qdot2.*t15.*t16.*t20.*t52.*5.27076e+8-qdot1.*qdot2.*t15.*t17.*t19.*t52.*5.27076e+8-qdot1.*qdot2.*t16.*t18.*t20.*t49.*2.3958e+6+qdot1.*qdot2.*t17.*t18.*t19.*t49.*2.3958e+6+qdot0.*qdot1.*t11.*t18.*t20.*cos(t50).*2.63538e+8+qdot0.*qdot2.*t13.*t15.*t20.*cos(t50).*7.90614e+8+qdot0.*qdot2.*t11.*t18.*t20.*cos(t50).*2.3958e+6+qdot1.*qdot2.*t13.*t15.*t20.*cos(t50).*5.27076e+8-qdot0.*qdot2.*t11.*t15.*t20.*sin(t50).*5.27076e+8+qdot0.*qdot1.*t13.*t18.*t20.*sin(t50).*3.95307e+8+qdot0.*qdot2.*t13.*t18.*t20.*sin(t50).*3.5937e+6+qdot1.*qdot2.*t13.*t18.*t20.*sin(t50).*2.3958e+6))./(t19.*t20.*1.0e+11);
else
    t0 = NaN;
end
if (t24)
    t1 = (t33.*t34.*(t55+t17.*t23.*t25.*5.222844e+8-q0.*qddot0.*t17.*t23.*2.611422e+8-q1.*t11.*t23.*t26.*2.0891376e+9+qddot1.*t11.*t12.*t23.*5.222844e+8-qddot2.*t11.*t17.*t18.*1.305711e+8+qddot0.*t11.*t17.*t23.*1.96450155e+8+qddot0.*t11.*t18.*t22.*2.611422e+8-qddot0.*t11.*t20.*t21.*2.611422e+8+qddot1.*t11.*t17.*t23.*1.74494067e+8+qddot1.*t11.*t18.*t22.*2.611422e+8-qddot1.*t11.*t20.*t21.*2.611422e+8+qddot2.*t11.*t18.*t22.*6.5879055e+7+t4.*t11.*t22.*t27.*5.27076e+8+t6.*t11.*t23.*t26.*2.63538e+9-t2.*t17.*t23.*t25.*5.222844e+8+t3.*t17.*t23.*t25.*2.3958e+6-t6.*t14.*t23.*t25.*5.27076e+8+t11.*t15.*t17.*t27.*2.611422e+8-t6.*t20.*t23.*t25.*2.611422e+8+t6.*t21.*t22.*t25.*2.611422e+8-t11.*t14.*t23.*t26.*3.48988134e+8-t11.*t15.*t22.*t27.*1.3175811e+8+t11.*t17.*t21.*t26.*2.611422e+8+t14.*t23.*t25.*t31.*5.27076e+8-t17.*t23.*t25.*t28.*2.3958e+6+t18.*t22.*t25.*t29.*2.63538e+8+t20.*t23.*t25.*t31.*2.611422e+8-t21.*t22.*t25.*t31.*2.611422e+8-t18.*t22.*t25.*t35.*2.63538e+8-t11.*t22.*t27.*t42.*5.27076e+8-t11.*t23.*t26.*t44.*2.63538e+9+t14.*t23.*t25.*t44.*5.27076e+8-t17.*t23.*t25.*t41.*2.3958e+6-t14.*t23.*t25.*t51.*5.27076e+8+t17.*t23.*t25.*t48.*2.3958e+6-t18.*t22.*t25.*t49.*2.63538e+8+t18.*t22.*t25.*cos(t50).*2.63538e+8-q0.*qddot0.*t18.*t22.*cos(t50).*1.31769e+8+q0.*t18.*t22.*t25.*sin(t50).*1.31769e+8+q0.*qdot0.*qdot1.*t14.*t23.*5.222844e+8-q2.*qddot2.*t4.*t11.*t22.*1.31769e+8+q0.*qddot0.*t2.*t17.*t23.*2.611422e+8-q1.*qddot1.*t6.*t11.*t23.*5.27076e+8-q0.*qddot0.*t3.*t17.*t23.*1.1979e+6+q0.*qddot0.*t6.*t14.*t23.*2.63538e+8+q0.*qddot0.*t6.*t20.*t23.*1.305711e+8-q0.*qddot0.*t6.*t21.*t22.*1.305711e+8-q0.*qddot0.*t14.*t23.*t31.*2.63538e+8+q0.*qddot0.*t17.*t23.*t28.*1.1979e+6-q0.*qddot0.*t18.*t22.*t29.*1.31769e+8-q0.*qddot0.*t20.*t23.*t31.*1.305711e+8+q0.*qddot0.*t21.*t22.*t31.*1.305711e+8+q0.*qddot0.*t18.*t22.*t35.*1.31769e+8+q2.*qddot2.*t11.*t22.*t42.*1.31769e+8+q1.*qddot1.*t11.*t23.*t44.*5.27076e+8-q0.*qddot0.*t14.*t23.*t44.*2.63538e+8+q0.*qddot0.*t17.*t23.*t41.*1.1979e+6+q0.*qddot0.*t14.*t23.*t51.*2.63538e+8-q0.*qddot0.*t17.*t23.*t48.*1.1979e+6+q0.*qddot0.*t18.*t22.*t49.*1.31769e+8+qdot1.*qdot2.*t11.*t14.*t18.*2.611422e+8-qdot0.*qdot1.*t11.*t14.*t23.*3.9290031e+8+qdot0.*qdot1.*t11.*t17.*t21.*2.611422e+8-qdot0.*qdot2.*t11.*t15.*t22.*5.222844e+8+qdot0.*qdot2.*t11.*t18.*t20.*2.611422e+8-qdot1.*qdot2.*t11.*t15.*t22.*5.222844e+8+qdot1.*qdot2.*t11.*t18.*t20.*2.611422e+8-q1.*t3.*t11.*t23.*t26.*5.366592e+8-q2.*t4.*t11.*t20.*t27.*7.90614e+8-q2.*t7.*t11.*t17.*t27.*7.90614e+8-q0.*t3.*t14.*t23.*t25.*2.63538e+8+q2.*t7.*t11.*t22.*t27.*5.27076e+8-q0.*t5.*t17.*t23.*t25.*2.611422e+8-q0.*t3.*t20.*t23.*t25.*1.305711e+8+q0.*t3.*t21.*t22.*t25.*1.305711e+8-q0.*t6.*t17.*t23.*t25.*1.1979e+6+q2.*t11.*t17.*t27.*t32.*7.90614e+8-q0.*t14.*t23.*t25.*t28.*2.63538e+8-q0.*t17.*t23.*t25.*t31.*1.1979e+6-q0.*t20.*t23.*t25.*t28.*1.305711e+8+q0.*t21.*t22.*t25.*t28.*1.305711e+8-q0.*t18.*t22.*t25.*t32.*1.31769e+8-q0.*t18.*t22.*t25.*t36.*1.31769e+8+q1.*t11.*t23.*t26.*t41.*1.43748e+7+q2.*t11.*t17.*t27.*t45.*7.90614e+8+q2.*t11.*t20.*t27.*t42.*7.90614e+8+q0.*t14.*t23.*t25.*t41.*2.63538e+8-q2.*t11.*t22.*t27.*t45.*4.7916e+6+q0.*t17.*t23.*t25.*t44.*1.1979e+6-q2.*t11.*t17.*t27.*t52.*7.90614e+8+q0.*t14.*t23.*t25.*t48.*2.63538e+8+q0.*t17.*t23.*t25.*t51.*1.1979e+6+q0.*t18.*t22.*t25.*t52.*1.31769e+8-qddot0.*t3.*t11.*t12.*t23.*1.31769e+8+qddot0.*t3.*t10.*t14.*t23.*2.63538e+8-qddot0.*t4.*t11.*t17.*t18.*1.31769e+8+qddot1.*t3.*t11.*t12.*t23.*2.3958e+6-qddot1.*t4.*t11.*t17.*t18.*2.63538e+8+qddot2.*t3.*t11.*t17.*t18.*1.305711e+8+qddot2.*t4.*t11.*t15.*t20.*2.63538e+8-qddot2.*t4.*t11.*t17.*t18.*1.31769e+8+qddot2.*t7.*t11.*t15.*t17.*2.63538e+8-qddot0.*t3.*t11.*t17.*t23.*1.305711e+8-qddot0.*t6.*t11.*t14.*t23.*1.31769e+8+qddot0.*t3.*t11.*t20.*t21.*1.305711e+8+qddot0.*t6.*t11.*t17.*t21.*1.305711e+8-qddot0.*t7.*t11.*t15.*t22.*2.63538e+8-qddot1.*t6.*t11.*t14.*t23.*2.611422e+8+qddot0.*t3.*t10.*t20.*t23.*1.305711e+8-qddot0.*t3.*t10.*t21.*t22.*1.305711e+8+qddot0.*t6.*t10.*t17.*t23.*1.1979e+6+qddot0.*t7.*t11.*t18.*t20.*2.63538e+8+qddot1.*t6.*t11.*t17.*t21.*2.611422e+8-qddot1.*t7.*t11.*t15.*t22.*2.63538e+8+qddot1.*t7.*t11.*t18.*t20.*2.63538e+8-qddot2.*t7.*t11.*t15.*t22.*1.31769e+8+qddot2.*t7.*t11.*t18.*t20.*1.31769e+8+qddot0.*t11.*t17.*t18.*t29.*1.31769e+8+qddot1.*t11.*t17.*t18.*t29.*2.63538e+8-qddot2.*t11.*t15.*t17.*t32.*2.63538e+8+qddot2.*t11.*t17.*t18.*t29.*1.31769e+8-qddot0.*t11.*t18.*t20.*t32.*1.31769e+8+qddot0.*t10.*t18.*t22.*t32.*1.31769e+8+qddot0.*t11.*t12.*t23.*t41.*1.31769e+8-qddot0.*t10.*t14.*t23.*t41.*2.63538e+8+qddot0.*t11.*t17.*t18.*t42.*1.31769e+8-qddot1.*t11.*t12.*t23.*t41.*2.3958e+6+qddot1.*t11.*t17.*t18.*t42.*2.63538e+8-qddot2.*t11.*t15.*t17.*t45.*2.63538e+8-qddot2.*t11.*t15.*t20.*t42.*2.63538e+8+qddot2.*t11.*t17.*t18.*t42.*1.1979e+6+qddot0.*t11.*t14.*t23.*t44.*1.1979e+6+qddot0.*t11.*t15.*t22.*t45.*2.63538e+8-qddot0.*t10.*t17.*t23.*t44.*1.1979e+6-qddot0.*t11.*t18.*t20.*t45.*2.63538e+8+qddot1.*t11.*t15.*t22.*t45.*2.63538e+8-qddot0.*t11.*t17.*t18.*t49.*1.31769e+8-qddot1.*t11.*t18.*t20.*t45.*2.63538e+8+qddot2.*t11.*t15.*t22.*t45.*1.1979e+6-qddot1.*t11.*t17.*t18.*t49.*2.63538e+8-qddot2.*t11.*t18.*t20.*t45.*1.1979e+6+qddot2.*t11.*t15.*t17.*t52.*2.63538e+8-qddot2.*t11.*t17.*t18.*t49.*1.1979e+6+qddot0.*t11.*t18.*t20.*t52.*1.31769e+8-qddot0.*t10.*t18.*t22.*t52.*1.31769e+8-t3.*t11.*t15.*t17.*t27.*2.611422e+8+t4.*t11.*t14.*t18.*t26.*5.27076e+8+t4.*t11.*t15.*t17.*t27.*5.27076e+8-t3.*t11.*t14.*t23.*t26.*2.611422e+8+t3.*t11.*t17.*t21.*t26.*2.611422e+8+t6.*t11.*t12.*t23.*t26.*7.810308e+8-t6.*t11.*t14.*t21.*t26.*5.222844e+8-t4.*t11.*t15.*t22.*t27.*1.31769e+8-t7.*t11.*t17.*t18.*t26.*2.63538e+8+t4.*t11.*t18.*t20.*t27.*1.31769e+8-t7.*t11.*t15.*t20.*t27.*5.27076e+8+t7.*t11.*t17.*t18.*t27.*1.31769e+8-t11.*t14.*t18.*t26.*t29.*5.27076e+8-t11.*t15.*t17.*t27.*t29.*5.27076e+8-t11.*t17.*t18.*t26.*t32.*2.63538e+8-t11.*t17.*t18.*t27.*t32.*1.31769e+8-t11.*t14.*t18.*t26.*t42.*5.27076e+8-t11.*t15.*t17.*t27.*t42.*4.7916e+6+t11.*t12.*t23.*t26.*t44.*2.178e+4+t11.*t15.*t22.*t27.*t42.*1.089e+4+t11.*t17.*t18.*t26.*t45.*2.63538e+8+t11.*t14.*t18.*t26.*t49.*5.27076e+8+t11.*t15.*t20.*t27.*t45.*4.7916e+6-t11.*t17.*t18.*t27.*t45.*1.089e+4-t11.*t18.*t20.*t27.*t42.*1.089e+4+t11.*t15.*t17.*t27.*t49.*4.7916e+6+t11.*t17.*t18.*t26.*t52.*2.63538e+8+t11.*t17.*t18.*t27.*t52.*1.089e+4+q1.*qdot0.*qdot1.*t3.*t11.*t23.*5.27076e+8-q0.*qdot0.*qdot1.*t2.*t14.*t23.*5.222844e+8+q0.*qdot0.*qdot1.*t3.*t14.*t23.*2.659338e+8-q0.*qdot0.*qdot1.*t6.*t12.*t23.*7.90614e+8+q2.*qdot0.*qdot2.*t7.*t11.*t22.*7.90614e+8+q2.*qdot1.*qdot2.*t7.*t11.*t22.*7.90614e+8+q0.*qdot0.*qdot1.*t3.*t20.*t23.*1.305711e+8-q0.*qdot0.*qdot1.*t3.*t21.*t22.*1.305711e+8-q0.*qdot0.*qdot1.*t6.*t17.*t23.*1.293732e+8+q0.*qdot0.*qdot2.*t6.*t18.*t22.*1.305711e+8-q0.*qdot0.*qdot1.*t14.*t23.*t28.*2.659338e+8+q0.*qdot0.*qdot1.*t12.*t23.*t31.*7.90614e+8+q0.*qdot0.*qdot2.*t15.*t22.*t29.*2.63538e+8+q0.*qdot0.*qdot1.*t17.*t23.*t31.*1.293732e+8-q0.*qdot0.*qdot1.*t20.*t23.*t28.*1.305711e+8+q0.*qdot0.*qdot1.*t21.*t22.*t28.*1.305711e+8+q0.*qdot0.*qdot1.*t18.*t22.*t32.*1.31769e+8-q0.*qdot0.*qdot2.*t18.*t22.*t31.*1.305711e+8-q0.*qdot0.*qdot2.*t15.*t22.*t35.*2.63538e+8+q0.*qdot0.*qdot2.*t18.*t22.*t32.*1.31769e+8-q0.*qdot0.*qdot1.*t18.*t22.*t36.*1.31769e+8-q1.*qdot0.*qdot1.*t11.*t23.*t41.*5.27076e+8-q0.*qdot0.*qdot2.*t18.*t22.*t36.*1.31769e+8-q0.*qdot0.*qdot1.*t14.*t23.*t41.*4.7916e+6+q0.*qdot0.*qdot1.*t12.*t23.*t44.*7.90614e+8-q2.*qdot0.*qdot2.*t11.*t22.*t45.*7.90614e+8-q2.*qdot1.*qdot2.*t11.*t22.*t45.*7.90614e+8-q0.*qdot0.*qdot1.*t17.*t23.*t44.*1.089e+4+q0.*qdot0.*qdot1.*t14.*t23.*t48.*4.7916e+6-q0.*qdot0.*qdot1.*t12.*t23.*t51.*7.90614e+8-q0.*qdot0.*qdot2.*t15.*t22.*t49.*2.63538e+8+q0.*qdot0.*qdot1.*t17.*t23.*t51.*1.089e+4-q0.*qdot0.*qdot1.*t18.*t22.*t52.*1.31769e+8-q0.*qdot0.*qdot2.*t18.*t22.*t52.*1.1979e+6+qdot0.*qdot1.*t4.*t11.*t14.*t18.*2.63538e+8-qdot0.*qdot1.*t3.*t10.*t12.*t23.*7.90614e+8+qdot0.*qdot2.*t4.*t11.*t15.*t17.*2.63538e+8-qdot1.*qdot2.*t3.*t11.*t14.*t18.*2.611422e+8+qdot1.*qdot2.*t4.*t11.*t14.*t18.*2.63538e+8+qdot1.*qdot2.*t4.*t11.*t15.*t17.*2.63538e+8-qdot1.*qdot2.*t7.*t11.*t14.*t15.*5.27076e+8+qdot0.*qdot1.*t3.*t11.*t14.*t23.*1.293732e+8-qdot0.*qdot1.*t6.*t11.*t14.*t21.*2.611422e+8-qdot0.*qdot1.*t3.*t10.*t17.*t23.*1.293732e+8-qdot0.*qdot1.*t6.*t10.*t14.*t23.*2.659338e+8-qdot0.*qdot1.*t7.*t11.*t17.*t18.*2.63538e+8-qdot0.*qdot2.*t3.*t11.*t18.*t20.*1.305711e+8-qdot0.*qdot2.*t4.*t11.*t15.*t22.*2.63538e+8-qdot0.*qdot2.*t6.*t11.*t17.*t18.*1.305711e+8+qdot0.*qdot2.*t3.*t10.*t18.*t22.*1.305711e+8+qdot0.*qdot2.*t4.*t11.*t18.*t20.*2.63538e+8-qdot0.*qdot2.*t7.*t11.*t15.*t20.*5.27076e+8+qdot0.*qdot2.*t7.*t11.*t17.*t18.*1.31769e+8-qdot1.*qdot2.*t4.*t11.*t15.*t22.*2.63538e+8-qdot1.*qdot2.*t6.*t11.*t17.*t18.*3.917133e+8+qdot1.*qdot2.*t4.*t11.*t18.*t20.*2.63538e+8-qdot1.*qdot2.*t7.*t11.*t15.*t20.*5.27076e+8+qdot1.*qdot2.*t7.*t11.*t17.*t18.*1.31769e+8+qdot0.*qdot1.*t6.*t11.*t17.*t23.*1.305711e+8-qdot0.*qdot1.*t6.*t11.*t20.*t21.*1.305711e+8-qdot0.*qdot1.*t6.*t10.*t20.*t23.*1.305711e+8+qdot0.*qdot1.*t6.*t10.*t21.*t22.*1.305711e+8-qdot0.*qdot1.*t11.*t14.*t18.*t29.*2.63538e+8-qdot0.*qdot2.*t11.*t15.*t17.*t29.*2.63538e+8+qdot1.*qdot2.*t11.*t14.*t15.*t32.*5.27076e+8-qdot1.*qdot2.*t11.*t14.*t18.*t29.*2.63538e+8-qdot1.*qdot2.*t11.*t15.*t17.*t29.*7.90614e+8-qdot0.*qdot1.*t11.*t18.*t20.*t29.*1.31769e+8+qdot0.*qdot1.*t10.*t18.*t22.*t29.*1.31769e+8+qdot0.*qdot2.*t11.*t15.*t20.*t32.*2.63538e+8-qdot0.*qdot2.*t11.*t17.*t18.*t32.*1.31769e+8-qdot0.*qdot2.*t11.*t18.*t20.*t29.*1.31769e+8-qdot0.*qdot2.*t10.*t15.*t22.*t32.*2.63538e+8+qdot0.*qdot2.*t10.*t18.*t22.*t29.*1.31769e+8-qdot1.*qdot2.*t11.*t17.*t18.*t32.*3.95307e+8-qdot0.*qdot1.*t11.*t14.*t18.*t42.*2.63538e+8+qdot0.*qdot1.*t10.*t12.*t23.*t41.*7.90614e+8-qdot0.*qdot2.*t11.*t15.*t17.*t42.*2.63538e+8+qdot1.*qdot2.*t11.*t14.*t15.*t45.*5.27076e+8-qdot1.*qdot2.*t11.*t14.*t18.*t42.*2.3958e+6-qdot1.*qdot2.*t11.*t15.*t17.*t42.*2.63538e+8+qdot0.*qdot1.*t11.*t14.*t23.*t41.*1.089e+4-qdot0.*qdot1.*t11.*t12.*t23.*t44.*4.7916e+6+qdot0.*qdot1.*t10.*t14.*t23.*t44.*4.7916e+6-qdot0.*qdot1.*t10.*t17.*t23.*t41.*1.089e+4+qdot0.*qdot1.*t11.*t17.*t18.*t45.*2.63538e+8+qdot0.*qdot2.*t11.*t15.*t22.*t42.*2.3958e+6+qdot0.*qdot1.*t11.*t14.*t18.*t49.*2.63538e+8+qdot0.*qdot2.*t11.*t15.*t20.*t45.*5.27076e+8-qdot0.*qdot2.*t11.*t17.*t18.*t45.*1.1979e+6-qdot0.*qdot2.*t11.*t18.*t20.*t42.*2.3958e+6+qdot1.*qdot2.*t11.*t15.*t22.*t42.*2.3958e+6+qdot0.*qdot2.*t11.*t15.*t17.*t49.*2.63538e+8+qdot1.*qdot2.*t11.*t15.*t20.*t45.*5.27076e+8-qdot1.*qdot2.*t11.*t17.*t18.*t45.*1.1979e+6-qdot1.*qdot2.*t11.*t18.*t20.*t42.*2.3958e+6-qdot1.*qdot2.*t11.*t14.*t15.*t52.*5.27076e+8+qdot1.*qdot2.*t11.*t14.*t18.*t49.*2.3958e+6+qdot1.*qdot2.*t11.*t15.*t17.*t49.*7.90614e+8+qdot0.*qdot1.*t11.*t18.*t20.*t49.*1.31769e+8-qdot0.*qdot1.*t10.*t18.*t22.*t49.*1.31769e+8-qdot0.*qdot2.*t11.*t15.*t20.*t52.*2.63538e+8+qdot0.*qdot2.*t11.*t17.*t18.*t52.*1.1979e+6+qdot0.*qdot2.*t11.*t18.*t20.*t49.*1.1979e+6+qdot0.*qdot2.*t10.*t15.*t22.*t52.*2.63538e+8-qdot0.*qdot2.*t10.*t18.*t22.*t49.*1.1979e+6+qdot1.*qdot2.*t11.*t17.*t18.*t52.*3.5937e+6+q0.*qdot0.*qdot2.*t15.*t22.*cos(t50).*2.63538e+8+q0.*qdot0.*qdot1.*t18.*t22.*sin(t50).*1.31769e+8+q0.*qdot0.*qdot2.*t18.*t22.*sin(t50).*1.1979e+6))./(t22.*1.0e+11);
else
    t1 = NaN;
end
if (t24)
    t8 = (1.0./t15.^3.*t33.*(t11.*t21.*t26.*-2.611422e+8+q1.*qddot1.*t11.*t21.*1.305711e+8+q2.*t11.*t14.*t27.*1.0445688e+9-qddot2.*t11.*t14.*t15.*2.611422e+8-qddot0.*t11.*t14.*t21.*6.5879055e+7-qddot1.*t11.*t14.*t21.*6.5879055e+7-qddot2.*t11.*t14.*t21.*4.3922967e+7-t7.*t11.*t14.*t27.*2.63538e+9+t3.*t11.*t21.*t26.*2.611422e+8-t4.*t11.*t21.*t26.*2.63538e+8+t7.*t11.*t18.*t26.*5.27076e+8-t3.*t14.*t21.*t25.*2.611422e+8+t11.*t14.*t18.*t27.*8.7845934e+7-t11.*t18.*t26.*t32.*5.27076e+8+t11.*t21.*t26.*t29.*2.63538e+8+t14.*t21.*t25.*t28.*2.611422e+8+t14.*t18.*t25.*t32.*5.27076e+8-t14.*t21.*t25.*t29.*2.63538e+8-t14.*t18.*t25.*t36.*5.27076e+8+t14.*t21.*t25.*t35.*2.63538e+8+t11.*t14.*t27.*t45.*2.63538e+9-t11.*t18.*t26.*t45.*5.27076e+8+t11.*t21.*t26.*t42.*2.3958e+6+t11.*t18.*t26.*t52.*5.27076e+8-t11.*t21.*t26.*t49.*2.3958e+6-t14.*t18.*t25.*t52.*5.27076e+8+t14.*t21.*t25.*t49.*2.3958e+6-t14.*t21.*t25.*cos(t50).*2.3958e+6+t14.*t18.*t25.*sin(t50).*5.27076e+8+q0.*qddot0.*t14.*t21.*cos(t50).*1.1979e+6-q0.*t14.*t18.*t25.*cos(t50).*2.63538e+8-q0.*qddot0.*t14.*t18.*sin(t50).*2.63538e+8-q0.*t14.*t21.*t25.*sin(t50).*1.1979e+6-q1.*qdot1.*qdot2.*t11.*t18.*2.611422e+8+q2.*qddot2.*t7.*t11.*t14.*5.27076e+8-q1.*qddot1.*t3.*t11.*t21.*1.305711e+8+q0.*qddot0.*t3.*t14.*t21.*1.305711e+8+q1.*qddot1.*t4.*t11.*t21.*1.31769e+8-q1.*qddot1.*t7.*t11.*t18.*2.63538e+8-q0.*qddot0.*t14.*t21.*t28.*1.305711e+8+q1.*qddot1.*t11.*t18.*t32.*2.63538e+8-q1.*qddot1.*t11.*t21.*t29.*1.31769e+8-q0.*qddot0.*t14.*t18.*t32.*2.63538e+8+q0.*qddot0.*t14.*t21.*t29.*1.31769e+8+q0.*qddot0.*t14.*t18.*t36.*2.63538e+8-q0.*qddot0.*t14.*t21.*t35.*1.31769e+8-q2.*qddot2.*t11.*t14.*t45.*5.27076e+8+q1.*qddot1.*t11.*t18.*t45.*2.63538e+8-q1.*qddot1.*t11.*t21.*t42.*1.1979e+6-q1.*qddot1.*t11.*t18.*t52.*2.63538e+8+q1.*qddot1.*t11.*t21.*t49.*1.1979e+6+q0.*qddot0.*t14.*t18.*t52.*2.63538e+8-q0.*qddot0.*t14.*t21.*t49.*1.1979e+6+qdot0.*qdot2.*t11.*t14.*t18.*1.3175811e+8+qdot1.*qdot2.*t11.*t14.*t18.*1.3175811e+8+q2.*t4.*t11.*t14.*t27.*1.581228e+9+q1.*t4.*t11.*t18.*t26.*2.63538e+8+q1.*t6.*t11.*t21.*t26.*1.305711e+8+q0.*t6.*t14.*t21.*t25.*1.305711e+8+q1.*t7.*t11.*t21.*t26.*1.31769e+8+q1.*t11.*t18.*t26.*t29.*2.63538e+8+q0.*t14.*t18.*t25.*t29.*2.63538e+8+q0.*t14.*t21.*t25.*t31.*1.305711e+8+q1.*t11.*t21.*t26.*t32.*1.31769e+8+q0.*t14.*t18.*t25.*t35.*2.63538e+8+q0.*t14.*t21.*t25.*t32.*1.31769e+8+q0.*t14.*t21.*t25.*t36.*1.31769e+8-q2.*t11.*t14.*t27.*t42.*1.43748e+7-q1.*t11.*t18.*t26.*t42.*2.63538e+8-q1.*t11.*t21.*t26.*t45.*1.1979e+6-q1.*t11.*t18.*t26.*t49.*2.63538e+8-q0.*t14.*t18.*t25.*t49.*2.63538e+8-q1.*t11.*t21.*t26.*t52.*1.1979e+6-q0.*t14.*t21.*t25.*t52.*1.1979e+6+qddot0.*t4.*t11.*t14.*t15.*1.31769e+8-qddot0.*t4.*t11.*t12.*t18.*2.63538e+8+qddot1.*t4.*t11.*t14.*t15.*1.31769e+8-qddot1.*t4.*t11.*t12.*t18.*2.63538e+8-qddot2.*t4.*t11.*t14.*t15.*2.63538e+8+qddot0.*t6.*t11.*t12.*t21.*1.305711e+8+qddot0.*t7.*t11.*t14.*t18.*1.31769e+8-qddot0.*t6.*t10.*t14.*t21.*1.305711e+8-qddot0.*t7.*t11.*t12.*t21.*1.31769e+8+qddot1.*t7.*t11.*t14.*t18.*1.31769e+8-qddot1.*t7.*t11.*t12.*t21.*1.31769e+8+qddot0.*t11.*t12.*t18.*t29.*2.63538e+8-qddot0.*t10.*t14.*t18.*t29.*2.63538e+8+qddot0.*t11.*t12.*t21.*t32.*1.31769e+8-qddot0.*t10.*t14.*t21.*t32.*1.31769e+8-qddot0.*t11.*t14.*t15.*t42.*1.31769e+8+qddot0.*t11.*t12.*t18.*t42.*2.63538e+8-qddot1.*t11.*t14.*t15.*t42.*1.31769e+8+qddot1.*t11.*t12.*t18.*t42.*2.63538e+8+qddot2.*t11.*t14.*t15.*t42.*2.3958e+6-qddot0.*t11.*t14.*t18.*t45.*1.1979e+6+qddot0.*t11.*t12.*t21.*t45.*1.1979e+6-qddot1.*t11.*t14.*t18.*t45.*1.1979e+6-qddot0.*t11.*t12.*t18.*t49.*2.63538e+8+qddot1.*t11.*t12.*t21.*t45.*1.1979e+6+qddot0.*t10.*t14.*t18.*t49.*2.63538e+8-qddot0.*t11.*t12.*t21.*t52.*1.1979e+6+qddot0.*t10.*t14.*t21.*t52.*1.1979e+6+t7.*t11.*t14.*t15.*t27.*2.63538e+8-t11.*t14.*t15.*t27.*t45.*2.178e+4-q2.*qdot0.*qdot2.*t4.*t11.*t14.*5.27076e+8-q2.*qdot1.*qdot2.*t4.*t11.*t14.*5.27076e+8+q1.*qdot0.*qdot1.*t4.*t11.*t18.*2.63538e+8+q1.*qdot1.*qdot2.*t3.*t11.*t18.*2.611422e+8-q0.*qdot0.*qdot2.*t3.*t14.*t18.*2.611422e+8-q1.*qdot1.*qdot2.*t4.*t11.*t18.*5.27076e+8+q1.*qdot1.*qdot2.*t7.*t11.*t15.*7.90614e+8-q1.*qdot0.*qdot1.*t6.*t11.*t21.*1.305711e+8+q1.*qdot0.*qdot1.*t7.*t11.*t21.*1.31769e+8-q0.*qdot0.*qdot1.*t6.*t14.*t21.*1.305711e+8-q1.*qdot1.*qdot2.*t7.*t11.*t21.*1.31769e+8-q1.*qdot0.*qdot1.*t11.*t18.*t29.*2.63538e+8-q0.*qdot0.*qdot1.*t14.*t18.*t29.*2.63538e+8+q0.*qdot0.*qdot2.*t14.*t18.*t28.*2.611422e+8-q1.*qdot1.*qdot2.*t11.*t15.*t32.*7.90614e+8+q1.*qdot1.*qdot2.*t11.*t18.*t29.*5.27076e+8+q0.*qdot0.*qdot2.*t14.*t15.*t32.*7.90614e+8-q0.*qdot0.*qdot2.*t14.*t18.*t29.*5.27076e+8-q1.*qdot0.*qdot1.*t11.*t21.*t32.*1.31769e+8+q0.*qdot0.*qdot1.*t14.*t21.*t31.*1.305711e+8-q0.*qdot0.*qdot2.*t14.*t15.*t36.*7.90614e+8+q0.*qdot0.*qdot1.*t14.*t18.*t35.*2.63538e+8-q0.*qdot0.*qdot1.*t14.*t21.*t32.*1.31769e+8+q1.*qdot1.*qdot2.*t11.*t21.*t32.*1.31769e+8+q0.*qdot0.*qdot2.*t14.*t18.*t35.*5.27076e+8-q0.*qdot0.*qdot2.*t14.*t21.*t32.*1.31769e+8+q2.*qdot0.*qdot2.*t11.*t14.*t42.*5.27076e+8+q0.*qdot0.*qdot1.*t14.*t21.*t36.*1.31769e+8+q2.*qdot1.*qdot2.*t11.*t14.*t42.*5.27076e+8+q0.*qdot0.*qdot2.*t14.*t21.*t36.*1.31769e+8-q1.*qdot0.*qdot1.*t11.*t18.*t42.*2.63538e+8-q1.*qdot1.*qdot2.*t11.*t15.*t45.*7.90614e+8+q1.*qdot1.*qdot2.*t11.*t18.*t42.*4.7916e+6-q1.*qdot0.*qdot1.*t11.*t21.*t45.*1.1979e+6+q1.*qdot0.*qdot1.*t11.*t18.*t49.*2.63538e+8+q1.*qdot1.*qdot2.*t11.*t21.*t45.*1.089e+4+q0.*qdot0.*qdot1.*t14.*t18.*t49.*2.63538e+8+q1.*qdot1.*qdot2.*t11.*t15.*t52.*7.90614e+8-q1.*qdot1.*qdot2.*t11.*t18.*t49.*4.7916e+6-q0.*qdot0.*qdot2.*t14.*t15.*t52.*7.90614e+8+q0.*qdot0.*qdot2.*t14.*t18.*t49.*4.7916e+6+q1.*qdot0.*qdot1.*t11.*t21.*t52.*1.1979e+6+q0.*qdot0.*qdot1.*t14.*t21.*t52.*1.1979e+6-q1.*qdot1.*qdot2.*t11.*t21.*t52.*1.089e+4+q0.*qdot0.*qdot2.*t14.*t21.*t52.*1.089e+4+qdot0.*qdot2.*t4.*t11.*t12.*t15.*7.90614e+8+qdot1.*qdot2.*t4.*t11.*t12.*t15.*7.90614e+8+qdot0.*qdot1.*t3.*t11.*t12.*t21.*1.305711e+8-qdot0.*qdot1.*t3.*t10.*t14.*t21.*1.305711e+8+qdot0.*qdot2.*t4.*t11.*t14.*t18.*1.31769e+8-qdot0.*qdot2.*t6.*t11.*t12.*t18.*2.611422e+8-qdot0.*qdot2.*t7.*t11.*t14.*t15.*5.27076e+8-qdot0.*qdot2.*t4.*t11.*t12.*t21.*1.31769e+8+qdot0.*qdot2.*t6.*t10.*t14.*t18.*2.611422e+8+qdot0.*qdot2.*t7.*t11.*t12.*t18.*5.27076e+8+qdot1.*qdot2.*t4.*t11.*t14.*t18.*1.31769e+8-qdot1.*qdot2.*t7.*t11.*t14.*t15.*5.27076e+8-qdot1.*qdot2.*t4.*t11.*t12.*t21.*1.31769e+8+qdot1.*qdot2.*t7.*t11.*t12.*t18.*5.27076e+8-qdot0.*qdot2.*t11.*t12.*t15.*t29.*7.90614e+8+qdot0.*qdot2.*t10.*t14.*t15.*t29.*7.90614e+8-qdot0.*qdot1.*t11.*t12.*t18.*t32.*2.63538e+8+qdot0.*qdot1.*t11.*t12.*t21.*t29.*1.31769e+8+qdot0.*qdot1.*t10.*t14.*t18.*t32.*2.63538e+8-qdot0.*qdot1.*t10.*t14.*t21.*t29.*1.31769e+8-qdot0.*qdot2.*t11.*t12.*t18.*t32.*5.27076e+8+qdot0.*qdot2.*t11.*t12.*t21.*t29.*1.31769e+8+qdot0.*qdot2.*t10.*t14.*t18.*t32.*5.27076e+8-qdot0.*qdot2.*t10.*t14.*t21.*t29.*1.31769e+8-qdot0.*qdot2.*t11.*t12.*t15.*t42.*7.90614e+8-qdot1.*qdot2.*t11.*t12.*t15.*t42.*7.90614e+8+qdot0.*qdot2.*t11.*t14.*t15.*t45.*4.7916e+6-qdot0.*qdot2.*t11.*t14.*t18.*t42.*1.089e+4-qdot0.*qdot2.*t11.*t12.*t18.*t45.*4.7916e+6+qdot0.*qdot2.*t11.*t12.*t21.*t42.*1.089e+4+qdot1.*qdot2.*t11.*t14.*t15.*t45.*4.7916e+6-qdot1.*qdot2.*t11.*t14.*t18.*t42.*1.089e+4+qdot0.*qdot2.*t11.*t12.*t15.*t49.*7.90614e+8-qdot1.*qdot2.*t11.*t12.*t18.*t45.*4.7916e+6+qdot1.*qdot2.*t11.*t12.*t21.*t42.*1.089e+4-qdot0.*qdot2.*t10.*t14.*t15.*t49.*7.90614e+8+qdot0.*qdot1.*t11.*t12.*t18.*t52.*2.63538e+8-qdot0.*qdot1.*t11.*t12.*t21.*t49.*1.1979e+6-qdot0.*qdot1.*t10.*t14.*t18.*t52.*2.63538e+8+qdot0.*qdot1.*t10.*t14.*t21.*t49.*1.1979e+6+qdot0.*qdot2.*t11.*t12.*t18.*t52.*4.7916e+6-qdot0.*qdot2.*t11.*t12.*t21.*t49.*1.089e+4-qdot0.*qdot2.*t10.*t14.*t18.*t52.*4.7916e+6+qdot0.*qdot2.*t10.*t14.*t21.*t49.*1.089e+4-q0.*qdot0.*qdot1.*t14.*t18.*cos(t50).*2.63538e+8-q0.*qdot0.*qdot2.*t14.*t18.*cos(t50).*4.7916e+6+q0.*qdot0.*qdot2.*t14.*t15.*sin(t50).*7.90614e+8-q0.*qdot0.*qdot1.*t14.*t21.*sin(t50).*1.1979e+6-q0.*qdot0.*qdot2.*t14.*t21.*sin(t50).*1.089e+4).*(-1.0e-11))./t14;
else
    t8 = NaN;
end
dH_spc_dqdot_dot = [t0;t1;t8];
