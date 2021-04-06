function C = C_fun(in1,in2,alpha,in4,in5)
%C_FUN
%    C = C_FUN(IN1,IN2,ALPHA,IN4,IN5)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    06-Apr-2021 15:14:19

l0 = in4(1,:);
l1 = in4(2,:);
l2 = in4(3,:);
m0 = in5(1,:);
m1 = in5(2,:);
m2 = in5(3,:);
q0 = in1(1,:);
q1 = in1(2,:);
q2 = in1(3,:);
qdot0 = in2(1,:);
qdot1 = in2(2,:);
qdot2 = in2(3,:);
et1 = m2.*((q2.*cos(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2).*abs(l2).^2.*1.0./abs(q2).^2)./l2+(q2.^2.*abs(l2).^2.*1.0./abs(q2).^4.*(sin(conj(alpha)+conj(q0)+conj(q1))-sin(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2)))./l2);
et2 = -(qdot0.*((l1.*(sin(alpha+q0+q1)-sin(alpha+q0)))./q1+l0.*1.0./q0.^2.*cos(alpha+q0).*2.0+(l0.*sin(alpha+q0))./q0-l0.*1.0./q0.^3.*(sin(alpha+q0)-sin(alpha)).*2.0-(l2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)))./q2)+qdot1.*((l1.*sin(alpha+q0+q1))./q1+l1.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0))-(l2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)))./q2)+l2.*1.0./q2.^2.*qdot2.*(-cos(alpha+q0+q1)+cos(alpha+q0+q1+q2)+q2.*sin(alpha+q0+q1+q2)));
et3 = m2.*((q2.*sin(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2).*abs(l2).^2.*1.0./abs(q2).^2)./l2-(q2.^2.*abs(l2).^2.*1.0./abs(q2).^4.*(cos(conj(alpha)+conj(q0)+conj(q1))-cos(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2)))./l2);
et4 = -(qdot0.*(-(l0.*cos(alpha+q0))./q0+l0.*1.0./q0.^2.*sin(alpha+q0).*2.0+l0.*1.0./q0.^3.*(cos(alpha+q0)-cos(alpha)).*2.0+(l2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)))./q2-(l1.*(cos(alpha+q0+q1)-cos(alpha+q0)))./q1)+qdot1.*(l1.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0))-(l1.*cos(alpha+q0+q1))./q1+(l2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)))./q2)-l2.*1.0./q2.^2.*qdot2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)+q2.*cos(alpha+q0+q1+q2)));
et5 = m2.*(qdot0.*((l1.*(sin(alpha+q0+q1)-sin(alpha+q0)))./q1+l0.*1.0./q0.^2.*cos(alpha+q0).*2.0+(l0.*sin(alpha+q0))./q0-l0.*1.0./q0.^3.*(sin(alpha+q0)-sin(alpha)).*2.0-(l2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)))./q2)+qdot1.*((l1.*sin(alpha+q0+q1))./q1+l1.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0))-(l2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)))./q2)+l2.*1.0./q2.^2.*qdot2.*(-cos(alpha+q0+q1)+cos(alpha+q0+q1+q2)+q2.*sin(alpha+q0+q1+q2)));
et6 = -((q2.*abs(l2).^2.*1.0./abs(q2).^2.*(cos(conj(alpha)+conj(q0)+abs(q1).^2./q1+abs(q2).^2./q2)-cos((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l2+(q1.*cos((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1).*abs(l1).^2.*1.0./abs(q1).^2)./l1+(q1.^2.*abs(l1).^2.*1.0./abs(q1).^4.*(sin(conj(alpha)+conj(q0))-sin((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l1);
et7 = m2.*((q2.*abs(l2).^2.*1.0./abs(q2).^2.*(sin(conj(alpha)+conj(q0)+abs(q1).^2./q1+abs(q2).^2./q2)-sin((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l2+(q1.*sin((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1).*abs(l1).^2.*1.0./abs(q1).^2)./l1-(q1.^2.*abs(l1).^2.*1.0./abs(q1).^4.*(cos(conj(alpha)+conj(q0))-cos((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l1);
et8 = -(qdot0.*(-(l0.*cos(alpha+q0))./q0+l0.*1.0./q0.^2.*sin(alpha+q0).*2.0+l0.*1.0./q0.^3.*(cos(alpha+q0)-cos(alpha)).*2.0+(l2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)))./q2-(l1.*(cos(alpha+q0+q1)-cos(alpha+q0)))./q1)+qdot1.*(l1.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0))-(l1.*cos(alpha+q0+q1))./q1+(l2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)))./q2)-l2.*1.0./q2.^2.*qdot2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)+q2.*cos(alpha+q0+q1+q2)));
et9 = et5.*et6+m1.*(qdot0.*((l0.*cos(alpha+q0))./q0-l0.*1.0./q0.^2.*sin(alpha+q0).*2.0-l0.*1.0./q0.^3.*(cos(alpha+q0)-cos(alpha)).*2.0+(l1.*(cos(alpha+q0+q1)-cos(alpha+q0)))./q1)+l1.*1.0./q1.^2.*qdot1.*(-sin(alpha+q0+q1)+sin(alpha+q0)+q1.*cos(alpha+q0+q1))).*((q1.*sin((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1).*abs(l1).^2.*1.0./abs(q1).^2)./l1-(q1.^2.*abs(l1).^2.*1.0./abs(q1).^4.*(cos(conj(alpha)+conj(q0))-cos((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l1);
et10 = -m1.*(qdot0.*((l1.*(sin(alpha+q0+q1)-sin(alpha+q0)))./q1+l0.*1.0./q0.^2.*cos(alpha+q0).*2.0+(l0.*sin(alpha+q0))./q0-l0.*1.0./q0.^3.*(sin(alpha+q0)-sin(alpha)).*2.0)+l1.*1.0./q1.^2.*qdot1.*(cos(alpha+q0+q1)-cos(alpha+q0)+q1.*sin(alpha+q0+q1))).*((q1.*cos((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1).*abs(l1).^2.*1.0./abs(q1).^2)./l1+(q1.^2.*abs(l1).^2.*1.0./abs(q1).^4.*(sin(conj(alpha)+conj(q0))-sin((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l1)+et7.*et8;
et11 = m1.*(qdot0.*((l0.*cos(alpha+q0))./q0-l0.*1.0./q0.^2.*sin(alpha+q0).*2.0-l0.*1.0./q0.^3.*(cos(alpha+q0)-cos(alpha)).*2.0+(l1.*(cos(alpha+q0+q1)-cos(alpha+q0)))./q1)+l1.*1.0./q1.^2.*qdot1.*(-sin(alpha+q0+q1)+sin(alpha+q0)+q1.*cos(alpha+q0+q1)));
et12 = -((q1.*abs(l1).^2.*1.0./abs(q1).^2.*(sin((abs(q0).^2+q0.*conj(alpha))./q0)-sin((q0.*abs(q1).^2+q1.*abs(q0).^2+q0.*q1.*conj(alpha))./(q0.*q1))))./l1-(q0.*sin((abs(q0).^2+q0.*conj(alpha))./q0).*abs(l0).^2.*1.0./abs(q0).^2)./l0+(q0.^2.*abs(l0).^2.*1.0./abs(q0).^4.*(cos(conj(alpha))-cos((abs(q0).^2+q0.*conj(alpha))./q0)))./l0);
et13 = m1.*(qdot0.*((l1.*(sin(alpha+q0+q1)-sin(alpha+q0)))./q1+l0.*1.0./q0.^2.*cos(alpha+q0).*2.0+(l0.*sin(alpha+q0))./q0-l0.*1.0./q0.^3.*(sin(alpha+q0)-sin(alpha)).*2.0)+l1.*1.0./q1.^2.*qdot1.*(cos(alpha+q0+q1)-cos(alpha+q0)+q1.*sin(alpha+q0+q1)));
et14 = -((q0.^2.*abs(l0).^2.*1.0./abs(q0).^4.*(sin(conj(alpha))-sin((abs(q0).^2+q0.*conj(alpha))./q0)))./l0-(q1.*abs(l1).^2.*1.0./abs(q1).^2.*(cos((abs(q0).^2+q0.*conj(alpha))./q0)-cos((q0.*abs(q1).^2+q1.*abs(q0).^2+q0.*q1.*conj(alpha))./(q0.*q1))))./l1+(q0.*cos((abs(q0).^2+q0.*conj(alpha))./q0).*abs(l0).^2.*1.0./abs(q0).^2)./l0);
et15 = (q2.*abs(l2).^2.*1.0./abs(q2).^2.*(sin(conj(alpha)+abs(q0).^2./q0+abs(q1).^2./q1+abs(q2).^2./q2)-sin((q0.*abs(q1).^2+q1.*abs(q0).^2+q0.*q1.*conj(alpha))./(q0.*q1))))./l2-(q1.*abs(l1).^2.*1.0./abs(q1).^2.*(sin((abs(q0).^2+q0.*conj(alpha))./q0)-sin((q0.*abs(q1).^2+q1.*abs(q0).^2+q0.*q1.*conj(alpha))./(q0.*q1))))./l1+(q0.*sin((abs(q0).^2+q0.*conj(alpha))./q0).*abs(l0).^2.*1.0./abs(q0).^2)./l0;
et16 = -(q0.^2.*abs(l0).^2.*1.0./abs(q0).^4.*(cos(conj(alpha))-cos((abs(q0).^2+q0.*conj(alpha))./q0)))./l0;
et17 = (q0.^2.*abs(l0).^2.*1.0./abs(q0).^4.*(sin(conj(alpha))-sin((abs(q0).^2+q0.*conj(alpha))./q0)))./l0+(q2.*abs(l2).^2.*1.0./abs(q2).^2.*(cos(conj(alpha)+abs(q0).^2./q0+abs(q1).^2./q1+abs(q2).^2./q2)-cos((q0.*abs(q1).^2+q1.*abs(q0).^2+q0.*q1.*conj(alpha))./(q0.*q1))))./l2-(q1.*abs(l1).^2.*1.0./abs(q1).^2.*(cos((abs(q0).^2+q0.*conj(alpha))./q0)-cos((q0.*abs(q1).^2+q1.*abs(q0).^2+q0.*q1.*conj(alpha))./(q0.*q1))))./l1;
et18 = (q0.*cos((abs(q0).^2+q0.*conj(alpha))./q0).*abs(l0).^2.*1.0./abs(q0).^2)./l0;
et19 = et11.*et12+et13.*et14-m2.*(qdot0.*(-(l0.*cos(alpha+q0))./q0+l0.*1.0./q0.^2.*sin(alpha+q0).*2.0+l0.*1.0./q0.^3.*(cos(alpha+q0)-cos(alpha)).*2.0+(l2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)))./q2-(l1.*(cos(alpha+q0+q1)-cos(alpha+q0)))./q1)+qdot1.*(l1.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0))-(l1.*cos(alpha+q0+q1))./q1+(l2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)))./q2)-l2.*1.0./q2.^2.*qdot2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)+q2.*cos(alpha+q0+q1+q2))).*(et15+et16);
et20 = -m2.*(et17+et18).*(qdot0.*((l1.*(sin(alpha+q0+q1)-sin(alpha+q0)))./q1+l0.*1.0./q0.^2.*cos(alpha+q0).*2.0+(l0.*sin(alpha+q0))./q0-l0.*1.0./q0.^3.*(sin(alpha+q0)-sin(alpha)).*2.0-(l2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)))./q2)+qdot1.*((l1.*sin(alpha+q0+q1))./q1+l1.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0))-(l2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)))./q2)+l2.*1.0./q2.^2.*qdot2.*(-cos(alpha+q0+q1)+cos(alpha+q0+q1+q2)+q2.*sin(alpha+q0+q1+q2)));
et21 = -l0.*m0.*1.0./q0.^3.*qdot0.*((q0.*sin((abs(q0).^2+q0.*conj(alpha))./q0).*abs(l0).^2.*1.0./abs(q0).^2)./l0-(q0.^2.*abs(l0).^2.*1.0./abs(q0).^4.*(cos(conj(alpha))-cos((abs(q0).^2+q0.*conj(alpha))./q0)))./l0).*(cos(alpha+q0).*2.0-cos(alpha).*2.0+q0.*sin(alpha+q0).*2.0-q0.^2.*cos(alpha+q0));
et22 = -l0.*m0.*1.0./q0.^3.*qdot0.*((q0.^2.*abs(l0).^2.*1.0./abs(q0).^4.*(sin(conj(alpha))-sin((abs(q0).^2+q0.*conj(alpha))./q0)))./l0+(q0.*cos((abs(q0).^2+q0.*conj(alpha))./q0).*abs(l0).^2.*1.0./abs(q0).^2)./l0).*(sin(alpha+q0).*-2.0+sin(alpha).*2.0+q0.*cos(alpha+q0).*2.0+q0.^2.*sin(alpha+q0));
et23 = m2.*((q2.*cos(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2).*abs(l2).^2.*1.0./abs(q2).^2)./l2+(q2.^2.*abs(l2).^2.*1.0./abs(q2).^4.*(sin(conj(alpha)+conj(q0)+conj(q1))-sin(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2)))./l2);
et24 = -(-qdot1.*(l1.*1.0./q1.^3.*(sin(alpha+q0+q1)-sin(alpha+q0)).*2.0-l1.*1.0./q1.^2.*cos(alpha+q0+q1).*2.0-(l1.*sin(alpha+q0+q1))./q1+(l2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)))./q2)+qdot0.*((l1.*sin(alpha+q0+q1))./q1+l1.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0))-(l2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)))./q2)+l2.*1.0./q2.^2.*qdot2.*(-cos(alpha+q0+q1)+cos(alpha+q0+q1+q2)+q2.*sin(alpha+q0+q1+q2)));
et25 = m2.*((q2.*sin(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2).*abs(l2).^2.*1.0./abs(q2).^2)./l2-(q2.^2.*abs(l2).^2.*1.0./abs(q2).^4.*(cos(conj(alpha)+conj(q0)+conj(q1))-cos(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2)))./l2);
et26 = -(qdot1.*(-(l1.*cos(alpha+q0+q1))./q1+l1.*1.0./q1.^2.*sin(alpha+q0+q1).*2.0+(l2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)))./q2+l1.*1.0./q1.^3.*(cos(alpha+q0+q1)-cos(alpha+q0)).*2.0)+qdot0.*(l1.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0))-(l1.*cos(alpha+q0+q1))./q1+(l2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)))./q2)-l2.*1.0./q2.^2.*qdot2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)+q2.*cos(alpha+q0+q1+q2)));
et27 = m2.*(-qdot1.*(l1.*1.0./q1.^3.*(sin(alpha+q0+q1)-sin(alpha+q0)).*2.0-l1.*1.0./q1.^2.*cos(alpha+q0+q1).*2.0-(l1.*sin(alpha+q0+q1))./q1+(l2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)))./q2)+qdot0.*((l1.*sin(alpha+q0+q1))./q1+l1.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0))-(l2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)))./q2)+l2.*1.0./q2.^2.*qdot2.*(-cos(alpha+q0+q1)+cos(alpha+q0+q1+q2)+q2.*sin(alpha+q0+q1+q2)));
et28 = -((q2.*abs(l2).^2.*1.0./abs(q2).^2.*(cos(conj(alpha)+conj(q0)+abs(q1).^2./q1+abs(q2).^2./q2)-cos((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l2+(q1.*cos((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1).*abs(l1).^2.*1.0./abs(q1).^2)./l1+(q1.^2.*abs(l1).^2.*1.0./abs(q1).^4.*(sin(conj(alpha)+conj(q0))-sin((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l1);
et29 = m2.*(qdot1.*(-(l1.*cos(alpha+q0+q1))./q1+l1.*1.0./q1.^2.*sin(alpha+q0+q1).*2.0+(l2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)))./q2+l1.*1.0./q1.^3.*(cos(alpha+q0+q1)-cos(alpha+q0)).*2.0)+qdot0.*(l1.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0))-(l1.*cos(alpha+q0+q1))./q1+(l2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)))./q2)-l2.*1.0./q2.^2.*qdot2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)+q2.*cos(alpha+q0+q1+q2)));
et30 = -((q2.*abs(l2).^2.*1.0./abs(q2).^2.*(sin(conj(alpha)+conj(q0)+abs(q1).^2./q1+abs(q2).^2./q2)-sin((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l2+(q1.*sin((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1).*abs(l1).^2.*1.0./abs(q1).^2)./l1-(q1.^2.*abs(l1).^2.*1.0./abs(q1).^4.*(cos(conj(alpha)+conj(q0))-cos((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l1);
et31 = m1.*((q1.*sin((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1).*abs(l1).^2.*1.0./abs(q1).^2)./l1-(q1.^2.*abs(l1).^2.*1.0./abs(q1).^4.*(cos(conj(alpha)+conj(q0))-cos((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l1).*(l1.*1.0./q1.^2.*qdot0.*(-sin(alpha+q0+q1)+sin(alpha+q0)+q1.*cos(alpha+q0+q1))-l1.*1.0./q1.^3.*qdot1.*(cos(alpha+q0+q1).*2.0-cos(alpha+q0).*2.0-q1.^2.*cos(alpha+q0+q1)+q1.*sin(alpha+q0+q1).*2.0))+et27.*et28;
et32 = -m1.*((q1.*cos((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1).*abs(l1).^2.*1.0./abs(q1).^2)./l1+(q1.^2.*abs(l1).^2.*1.0./abs(q1).^4.*(sin(conj(alpha)+conj(q0))-sin((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l1).*(l1.*1.0./q1.^3.*qdot1.*(sin(alpha+q0+q1).*-2.0+sin(alpha+q0).*2.0+q1.^2.*sin(alpha+q0+q1)+q1.*cos(alpha+q0+q1).*2.0)+l1.*1.0./q1.^2.*qdot0.*(cos(alpha+q0+q1)-cos(alpha+q0)+q1.*sin(alpha+q0+q1)))+et29.*et30;
et33 = m2.*(qdot1.*(-(l1.*cos(alpha+q0+q1))./q1+l1.*1.0./q1.^2.*sin(alpha+q0+q1).*2.0+(l2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)))./q2+l1.*1.0./q1.^3.*(cos(alpha+q0+q1)-cos(alpha+q0)).*2.0)+qdot0.*(l1.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0))-(l1.*cos(alpha+q0+q1))./q1+(l2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)))./q2)-l2.*1.0./q2.^2.*qdot2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)+q2.*cos(alpha+q0+q1+q2)));
et34 = conj(l0).*1.0./conj(q0).^2.*(cos(conj(alpha))-cos(conj(alpha)+conj(q0)))-(sin(conj(alpha)+conj(q0)).*conj(l0))./conj(q0)-(q2.*abs(l2).^2.*1.0./abs(q2).^2.*(sin(conj(alpha)+conj(q0)+abs(q1).^2./q1+abs(q2).^2./q2)-sin((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l2+(q1.*abs(l1).^2.*1.0./abs(q1).^2.*(sin(conj(alpha)+conj(q0))-sin((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l1;
et35 = m2.*(-qdot1.*(l1.*1.0./q1.^3.*(sin(alpha+q0+q1)-sin(alpha+q0)).*2.0-l1.*1.0./q1.^2.*cos(alpha+q0+q1).*2.0-(l1.*sin(alpha+q0+q1))./q1+(l2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)))./q2)+qdot0.*((l1.*sin(alpha+q0+q1))./q1+l1.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0))-(l2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)))./q2)+l2.*1.0./q2.^2.*qdot2.*(-cos(alpha+q0+q1)+cos(alpha+q0+q1+q2)+q2.*sin(alpha+q0+q1+q2)));
et36 = -((cos(conj(alpha)+conj(q0)).*conj(l0))./conj(q0)+conj(l0).*1.0./conj(q0).^2.*(sin(conj(alpha))-sin(conj(alpha)+conj(q0)))+(q2.*abs(l2).^2.*1.0./abs(q2).^2.*(cos(conj(alpha)+conj(q0)+abs(q1).^2./q1+abs(q2).^2./q2)-cos((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l2-(q1.*abs(l1).^2.*1.0./abs(q1).^2.*(cos(conj(alpha)+conj(q0))-cos((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l1);
et37 = et33.*et34+et35.*et36-m1.*(l1.*1.0./q1.^3.*qdot1.*(sin(alpha+q0+q1).*-2.0+sin(alpha+q0).*2.0+q1.^2.*sin(alpha+q0+q1)+q1.*cos(alpha+q0+q1).*2.0)+l1.*1.0./q1.^2.*qdot0.*(cos(alpha+q0+q1)-cos(alpha+q0)+q1.*sin(alpha+q0+q1))).*((cos(conj(alpha)+conj(q0)).*conj(l0))./conj(q0)+conj(l0).*1.0./conj(q0).^2.*(sin(conj(alpha))-sin(conj(alpha)+conj(q0)))-(q1.*abs(l1).^2.*1.0./abs(q1).^2.*(cos(conj(alpha)+conj(q0))-cos((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l1);
et38 = -m1.*(l1.*1.0./q1.^2.*qdot0.*(-sin(alpha+q0+q1)+sin(alpha+q0)+q1.*cos(alpha+q0+q1))-l1.*1.0./q1.^3.*qdot1.*(cos(alpha+q0+q1).*2.0-cos(alpha+q0).*2.0-q1.^2.*cos(alpha+q0+q1)+q1.*sin(alpha+q0+q1).*2.0)).*(conj(l0).*1.0./conj(q0).^2.*(cos(conj(alpha))-cos(conj(alpha)+conj(q0)))-(sin(conj(alpha)+conj(q0)).*conj(l0))./conj(q0)+(q1.*abs(l1).^2.*1.0./abs(q1).^2.*(sin(conj(alpha)+conj(q0))-sin((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l1);
et39 = m2.*((q2.*sin(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2).*abs(l2).^2.*1.0./abs(q2).^2)./l2-(q2.^2.*abs(l2).^2.*1.0./abs(q2).^4.*(cos(conj(alpha)+conj(q0)+conj(q1))-cos(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2)))./l2).*(l2.*1.0./q2.^3.*qdot2.*(cos(alpha+q0+q1).*2.0-cos(alpha+q0+q1+q2).*2.0-q2.*sin(alpha+q0+q1+q2).*2.0+q2.^2.*cos(alpha+q0+q1+q2))+l2.*1.0./q2.^2.*qdot0.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)+q2.*cos(alpha+q0+q1+q2))+l2.*1.0./q2.^2.*qdot1.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)+q2.*cos(alpha+q0+q1+q2)));
et40 = -m2.*((q2.*cos(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2).*abs(l2).^2.*1.0./abs(q2).^2)./l2+(q2.^2.*abs(l2).^2.*1.0./abs(q2).^4.*(sin(conj(alpha)+conj(q0)+conj(q1))-sin(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2)))./l2).*(l2.*1.0./q2.^2.*qdot0.*(-cos(alpha+q0+q1)+cos(alpha+q0+q1+q2)+q2.*sin(alpha+q0+q1+q2))+l2.*1.0./q2.^2.*qdot1.*(-cos(alpha+q0+q1)+cos(alpha+q0+q1+q2)+q2.*sin(alpha+q0+q1+q2))+l2.*1.0./q2.^3.*qdot2.*(sin(alpha+q0+q1).*2.0-sin(alpha+q0+q1+q2).*2.0+q2.*cos(alpha+q0+q1+q2).*2.0+q2.^2.*sin(alpha+q0+q1+q2)));
et41 = m2.*(-(cos(conj(alpha)+conj(q0)+conj(q1)).*conj(l1))./conj(q1)+conj(l1).*1.0./conj(q1).^2.*(sin(conj(alpha)+conj(q0)+conj(q1))-sin(conj(alpha)+conj(q0)))+(q2.*abs(l2).^2.*1.0./abs(q2).^2.*(cos(conj(alpha)+conj(q0)+conj(q1))-cos(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2)))./l2).*(l2.*1.0./q2.^2.*qdot0.*(-cos(alpha+q0+q1)+cos(alpha+q0+q1+q2)+q2.*sin(alpha+q0+q1+q2))+l2.*1.0./q2.^2.*qdot1.*(-cos(alpha+q0+q1)+cos(alpha+q0+q1+q2)+q2.*sin(alpha+q0+q1+q2))+l2.*1.0./q2.^3.*qdot2.*(sin(alpha+q0+q1).*2.0-sin(alpha+q0+q1+q2).*2.0+q2.*cos(alpha+q0+q1+q2).*2.0+q2.^2.*sin(alpha+q0+q1+q2)));
et42 = m2.*((sin(conj(alpha)+conj(q0)+conj(q1)).*conj(l1))./conj(q1)+conj(l1).*1.0./conj(q1).^2.*(cos(conj(alpha)+conj(q0)+conj(q1))-cos(conj(alpha)+conj(q0)))-(q2.*abs(l2).^2.*1.0./abs(q2).^2.*(sin(conj(alpha)+conj(q0)+conj(q1))-sin(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2)))./l2).*(l2.*1.0./q2.^3.*qdot2.*(cos(alpha+q0+q1).*2.0-cos(alpha+q0+q1+q2).*2.0-q2.*sin(alpha+q0+q1+q2).*2.0+q2.^2.*cos(alpha+q0+q1+q2))+l2.*1.0./q2.^2.*qdot0.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)+q2.*cos(alpha+q0+q1+q2))+l2.*1.0./q2.^2.*qdot1.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)+q2.*cos(alpha+q0+q1+q2)));
et43 = m2.*(l2.*1.0./q2.^2.*qdot0.*(-cos(alpha+q0+q1)+cos(alpha+q0+q1+q2)+q2.*sin(alpha+q0+q1+q2))+l2.*1.0./q2.^2.*qdot1.*(-cos(alpha+q0+q1)+cos(alpha+q0+q1+q2)+q2.*sin(alpha+q0+q1+q2))+l2.*1.0./q2.^3.*qdot2.*(sin(alpha+q0+q1).*2.0-sin(alpha+q0+q1+q2).*2.0+q2.*cos(alpha+q0+q1+q2).*2.0+q2.^2.*sin(alpha+q0+q1+q2)));
et44 = -((conj(l1).*(cos(conj(alpha)+conj(q0)+conj(q1))-cos(conj(alpha)+conj(q0))))./conj(q1)+(cos(conj(alpha)+conj(q0)).*conj(l0))./conj(q0)+conj(l0).*1.0./conj(q0).^2.*(sin(conj(alpha))-sin(conj(alpha)+conj(q0)))-(q2.*abs(l2).^2.*1.0./abs(q2).^2.*(cos(conj(alpha)+conj(q0)+conj(q1))-cos(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2)))./l2);
et45 = m2.*(l2.*1.0./q2.^3.*qdot2.*(cos(alpha+q0+q1).*2.0-cos(alpha+q0+q1+q2).*2.0-q2.*sin(alpha+q0+q1+q2).*2.0+q2.^2.*cos(alpha+q0+q1+q2))+l2.*1.0./q2.^2.*qdot0.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)+q2.*cos(alpha+q0+q1+q2))+l2.*1.0./q2.^2.*qdot1.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)+q2.*cos(alpha+q0+q1+q2)));
et46 = -(conj(l0).*1.0./conj(q0).^2.*(cos(conj(alpha))-cos(conj(alpha)+conj(q0)))-(conj(l1).*(sin(conj(alpha)+conj(q0)+conj(q1))-sin(conj(alpha)+conj(q0))))./conj(q1)-(sin(conj(alpha)+conj(q0)).*conj(l0))./conj(q0)+(q2.*abs(l2).^2.*1.0./abs(q2).^2.*(sin(conj(alpha)+conj(q0)+conj(q1))-sin(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2)))./l2);
C = reshape([et19+et20+et21+et22,et9+et10,et1.*et2+et3.*et4,et37+et38,et31+et32,et23.*et24+et25.*et26,et43.*et44+et45.*et46,et41+et42,et39+et40],[3,3]);
