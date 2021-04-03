function B = B_fun(in1,alpha,in3,in4)
%B_FUN
%    B = B_FUN(IN1,ALPHA,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    03-Apr-2021 18:48:14

l0 = in3(1,:);
l1 = in3(2,:);
l2 = in3(3,:);
m0 = in4(1,:);
m1 = in4(2,:);
m2 = in4(3,:);
q0 = in1(1,:);
q1 = in1(2,:);
q2 = in1(3,:);
et1 = m2.*((q2.*sin(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2).*abs(l2).^2.*1.0./abs(q2).^2)./l2-(q2.^2.*abs(l2).^2.*1.0./abs(q2).^4.*(cos(conj(alpha)+conj(q0)+conj(q1))-cos(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2)))./l2).*((l1.*(sin(alpha+q0+q1)-sin(alpha+q0)))./q1+(l0.*sin(alpha+q0))./q0+l0.*1.0./q0.^2.*(cos(alpha+q0)-cos(alpha))-(l2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)))./q2);
et2 = m2.*((q2.*cos(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2).*abs(l2).^2.*1.0./abs(q2).^2)./l2+(q2.^2.*abs(l2).^2.*1.0./abs(q2).^4.*(sin(conj(alpha)+conj(q0)+conj(q1))-sin(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2)))./l2).*((l0.*cos(alpha+q0))./q0-l0.*1.0./q0.^2.*(sin(alpha+q0)-sin(alpha))-(l2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)))./q2+(l1.*(cos(alpha+q0+q1)-cos(alpha+q0)))./q1);
et3 = m2.*((q2.*abs(l2).^2.*1.0./abs(q2).^2.*(sin(conj(alpha)+conj(q0)+abs(q1).^2./q1+abs(q2).^2./q2)-sin((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l2+(q1.*sin((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1).*abs(l1).^2.*1.0./abs(q1).^2)./l1-(q1.^2.*abs(l1).^2.*1.0./abs(q1).^4.*(cos(conj(alpha)+conj(q0))-cos((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l1);
et4 = (l1.*(sin(alpha+q0+q1)-sin(alpha+q0)))./q1+(l0.*sin(alpha+q0))./q0+l0.*1.0./q0.^2.*(cos(alpha+q0)-cos(alpha))-(l2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)))./q2;
et5 = m2.*((q2.*abs(l2).^2.*1.0./abs(q2).^2.*(cos(conj(alpha)+conj(q0)+abs(q1).^2./q1+abs(q2).^2./q2)-cos((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l2+(q1.*cos((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1).*abs(l1).^2.*1.0./abs(q1).^2)./l1+(q1.^2.*abs(l1).^2.*1.0./abs(q1).^4.*(sin(conj(alpha)+conj(q0))-sin((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l1);
et6 = (l0.*cos(alpha+q0))./q0-l0.*1.0./q0.^2.*(sin(alpha+q0)-sin(alpha))-(l2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)))./q2+(l1.*(cos(alpha+q0+q1)-cos(alpha+q0)))./q1;
et7 = et3.*et4+m1.*((q1.*sin((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1).*abs(l1).^2.*1.0./abs(q1).^2)./l1-(q1.^2.*abs(l1).^2.*1.0./abs(q1).^4.*(cos(conj(alpha)+conj(q0))-cos((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l1).*((l1.*(sin(alpha+q0+q1)-sin(alpha+q0)))./q1+(l0.*sin(alpha+q0))./q0+l0.*1.0./q0.^2.*(cos(alpha+q0)-cos(alpha)));
et8 = m1.*((q1.*cos((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1).*abs(l1).^2.*1.0./abs(q1).^2)./l1+(q1.^2.*abs(l1).^2.*1.0./abs(q1).^4.*(sin(conj(alpha)+conj(q0))-sin((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l1).*((l0.*cos(alpha+q0))./q0-l0.*1.0./q0.^2.*(sin(alpha+q0)-sin(alpha))+(l1.*(cos(alpha+q0+q1)-cos(alpha+q0)))./q1)+et5.*et6;
et9 = (q2.*abs(l2).^2.*1.0./abs(q2).^2.*(sin(conj(alpha)+abs(q0).^2./q0+abs(q1).^2./q1+abs(q2).^2./q2)-sin((q0.*abs(q1).^2+q1.*abs(q0).^2+q0.*q1.*conj(alpha))./(q0.*q1))))./l2-(q1.*abs(l1).^2.*1.0./abs(q1).^2.*(sin((abs(q0).^2+q0.*conj(alpha))./q0)-sin((q0.*abs(q1).^2+q1.*abs(q0).^2+q0.*q1.*conj(alpha))./(q0.*q1))))./l1+(q0.*sin((abs(q0).^2+q0.*conj(alpha))./q0).*abs(l0).^2.*1.0./abs(q0).^2)./l0;
et10 = -(q0.^2.*abs(l0).^2.*1.0./abs(q0).^4.*(cos(conj(alpha))-cos((abs(q0).^2+q0.*conj(alpha))./q0)))./l0;
et11 = (q0.^2.*abs(l0).^2.*1.0./abs(q0).^4.*(sin(conj(alpha))-sin((abs(q0).^2+q0.*conj(alpha))./q0)))./l0+(q2.*abs(l2).^2.*1.0./abs(q2).^2.*(cos(conj(alpha)+abs(q0).^2./q0+abs(q1).^2./q1+abs(q2).^2./q2)-cos((q0.*abs(q1).^2+q1.*abs(q0).^2+q0.*q1.*conj(alpha))./(q0.*q1))))./l2-(q1.*abs(l1).^2.*1.0./abs(q1).^2.*(cos((abs(q0).^2+q0.*conj(alpha))./q0)-cos((q0.*abs(q1).^2+q1.*abs(q0).^2+q0.*q1.*conj(alpha))./(q0.*q1))))./l1;
et12 = (q0.*cos((abs(q0).^2+q0.*conj(alpha))./q0).*abs(l0).^2.*1.0./abs(q0).^2)./l0;
et13 = m2.*((l1.*(sin(alpha+q0+q1)-sin(alpha+q0)))./q1+(l0.*sin(alpha+q0))./q0+l0.*1.0./q0.^2.*(cos(alpha+q0)-cos(alpha))-(l2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)))./q2).*(et9+et10);
et14 = m1.*((l0.*cos(alpha+q0))./q0-l0.*1.0./q0.^2.*(sin(alpha+q0)-sin(alpha))+(l1.*(cos(alpha+q0+q1)-cos(alpha+q0)))./q1).*((q0.^2.*abs(l0).^2.*1.0./abs(q0).^4.*(sin(conj(alpha))-sin((abs(q0).^2+q0.*conj(alpha))./q0)))./l0-(q1.*abs(l1).^2.*1.0./abs(q1).^2.*(cos((abs(q0).^2+q0.*conj(alpha))./q0)-cos((q0.*abs(q1).^2+q1.*abs(q0).^2+q0.*q1.*conj(alpha))./(q0.*q1))))./l1+(q0.*cos((abs(q0).^2+q0.*conj(alpha))./q0).*abs(l0).^2.*1.0./abs(q0).^2)./l0);
et15 = m2.*((l0.*cos(alpha+q0))./q0-l0.*1.0./q0.^2.*(sin(alpha+q0)-sin(alpha))-(l2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)))./q2+(l1.*(cos(alpha+q0+q1)-cos(alpha+q0)))./q1).*(et11+et12);
et16 = -m1.*((l1.*(sin(alpha+q0+q1)-sin(alpha+q0)))./q1+(l0.*sin(alpha+q0))./q0+l0.*1.0./q0.^2.*(cos(alpha+q0)-cos(alpha))).*((q1.*abs(l1).^2.*1.0./abs(q1).^2.*(sin((abs(q0).^2+q0.*conj(alpha))./q0)-sin((q0.*abs(q1).^2+q1.*abs(q0).^2+q0.*q1.*conj(alpha))./(q0.*q1))))./l1-(q0.*sin((abs(q0).^2+q0.*conj(alpha))./q0).*abs(l0).^2.*1.0./abs(q0).^2)./l0+(q0.^2.*abs(l0).^2.*1.0./abs(q0).^4.*(cos(conj(alpha))-cos((abs(q0).^2+q0.*conj(alpha))./q0)))./l0);
et17 = l0.*m0.*1.0./q0.^2.*((q0.*sin((abs(q0).^2+q0.*conj(alpha))./q0).*abs(l0).^2.*1.0./abs(q0).^2)./l0-(q0.^2.*abs(l0).^2.*1.0./abs(q0).^4.*(cos(conj(alpha))-cos((abs(q0).^2+q0.*conj(alpha))./q0)))./l0).*(cos(alpha+q0)-cos(alpha)+q0.*sin(alpha+q0))+l0.*m0.*1.0./q0.^2.*((q0.^2.*abs(l0).^2.*1.0./abs(q0).^4.*(sin(conj(alpha))-sin((abs(q0).^2+q0.*conj(alpha))./q0)))./l0+(q0.*cos((abs(q0).^2+q0.*conj(alpha))./q0).*abs(l0).^2.*1.0./abs(q0).^2)./l0).*(-sin(alpha+q0)+sin(alpha)+q0.*cos(alpha+q0));
et18 = -m2.*((q2.*cos(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2).*abs(l2).^2.*1.0./abs(q2).^2)./l2+(q2.^2.*abs(l2).^2.*1.0./abs(q2).^4.*(sin(conj(alpha)+conj(q0)+conj(q1))-sin(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2)))./l2).*(l1.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0))-(l1.*cos(alpha+q0+q1))./q1+(l2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)))./q2);
et19 = m2.*((q2.*sin(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2).*abs(l2).^2.*1.0./abs(q2).^2)./l2-(q2.^2.*abs(l2).^2.*1.0./abs(q2).^4.*(cos(conj(alpha)+conj(q0)+conj(q1))-cos(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2)))./l2).*((l1.*sin(alpha+q0+q1))./q1+l1.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0))-(l2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)))./q2);
et20 = -m2.*((q2.*abs(l2).^2.*1.0./abs(q2).^2.*(cos(conj(alpha)+conj(q0)+abs(q1).^2./q1+abs(q2).^2./q2)-cos((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l2+(q1.*cos((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1).*abs(l1).^2.*1.0./abs(q1).^2)./l1+(q1.^2.*abs(l1).^2.*1.0./abs(q1).^4.*(sin(conj(alpha)+conj(q0))-sin((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l1).*(l1.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0))-(l1.*cos(alpha+q0+q1))./q1+(l2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)))./q2);
et21 = m2.*((l1.*sin(alpha+q0+q1))./q1+l1.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0))-(l2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)))./q2).*((q2.*abs(l2).^2.*1.0./abs(q2).^2.*(sin(conj(alpha)+conj(q0)+abs(q1).^2./q1+abs(q2).^2./q2)-sin((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l2+(q1.*sin((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1).*abs(l1).^2.*1.0./abs(q1).^2)./l1-(q1.^2.*abs(l1).^2.*1.0./abs(q1).^4.*(cos(conj(alpha)+conj(q0))-cos((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l1);
et22 = l1.*m1.*1.0./q1.^2.*((q1.*cos((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1).*abs(l1).^2.*1.0./abs(q1).^2)./l1+(q1.^2.*abs(l1).^2.*1.0./abs(q1).^4.*(sin(conj(alpha)+conj(q0))-sin((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l1).*(-sin(alpha+q0+q1)+sin(alpha+q0)+q1.*cos(alpha+q0+q1));
et23 = l1.*m1.*1.0./q1.^2.*((q1.*sin((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1).*abs(l1).^2.*1.0./abs(q1).^2)./l1-(q1.^2.*abs(l1).^2.*1.0./abs(q1).^4.*(cos(conj(alpha)+conj(q0))-cos((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l1).*(cos(alpha+q0+q1)-cos(alpha+q0)+q1.*sin(alpha+q0+q1));
et24 = m2.*(l1.*1.0./q1.^2.*(sin(alpha+q0+q1)-sin(alpha+q0))-(l1.*cos(alpha+q0+q1))./q1+(l2.*(cos(alpha+q0+q1)-cos(alpha+q0+q1+q2)))./q2).*((cos(conj(alpha)+conj(q0)).*conj(l0))./conj(q0)+conj(l0).*1.0./conj(q0).^2.*(sin(conj(alpha))-sin(conj(alpha)+conj(q0)))+(q2.*abs(l2).^2.*1.0./abs(q2).^2.*(cos(conj(alpha)+conj(q0)+abs(q1).^2./q1+abs(q2).^2./q2)-cos((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l2-(q1.*abs(l1).^2.*1.0./abs(q1).^2.*(cos(conj(alpha)+conj(q0))-cos((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l1);
et25 = -1.0;
et26 = m2.*((l1.*sin(alpha+q0+q1))./q1+l1.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0))-(l2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)))./q2);
et27 = -(conj(l0).*1.0./conj(q0).^2.*(cos(conj(alpha))-cos(conj(alpha)+conj(q0)))-(sin(conj(alpha)+conj(q0)).*conj(l0))./conj(q0)-(q2.*abs(l2).^2.*1.0./abs(q2).^2.*(sin(conj(alpha)+conj(q0)+abs(q1).^2./q1+abs(q2).^2./q2)-sin((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l2+(q1.*abs(l1).^2.*1.0./abs(q1).^2.*(sin(conj(alpha)+conj(q0))-sin((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l1);
et28 = et24.*et25+et26.*et27-l1.*m1.*1.0./q1.^2.*(cos(alpha+q0+q1)-cos(alpha+q0)+q1.*sin(alpha+q0+q1)).*(conj(l0).*1.0./conj(q0).^2.*(cos(conj(alpha))-cos(conj(alpha)+conj(q0)))-(sin(conj(alpha)+conj(q0)).*conj(l0))./conj(q0)+(q1.*abs(l1).^2.*1.0./abs(q1).^2.*(sin(conj(alpha)+conj(q0))-sin((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l1);
et29 = l1.*m1.*1.0./q1.^2.*(-sin(alpha+q0+q1)+sin(alpha+q0)+q1.*cos(alpha+q0+q1)).*((cos(conj(alpha)+conj(q0)).*conj(l0))./conj(q0)+conj(l0).*1.0./conj(q0).^2.*(sin(conj(alpha))-sin(conj(alpha)+conj(q0)))-(q1.*abs(l1).^2.*1.0./abs(q1).^2.*(cos(conj(alpha)+conj(q0))-cos((abs(q1).^2+q1.*conj(alpha)+q1.*conj(q0))./q1)))./l1);
et30 = l2.*m2.*1.0./q2.^2.*((q2.*sin(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2).*abs(l2).^2.*1.0./abs(q2).^2)./l2-(q2.^2.*abs(l2).^2.*1.0./abs(q2).^4.*(cos(conj(alpha)+conj(q0)+conj(q1))-cos(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2)))./l2).*(-cos(alpha+q0+q1)+cos(alpha+q0+q1+q2)+q2.*sin(alpha+q0+q1+q2));
et31 = l2.*m2.*1.0./q2.^2.*((q2.*cos(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2).*abs(l2).^2.*1.0./abs(q2).^2)./l2+(q2.^2.*abs(l2).^2.*1.0./abs(q2).^4.*(sin(conj(alpha)+conj(q0)+conj(q1))-sin(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2)))./l2).*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)+q2.*cos(alpha+q0+q1+q2));
et32 = -l2.*m2.*1.0./q2.^2.*(-(cos(conj(alpha)+conj(q0)+conj(q1)).*conj(l1))./conj(q1)+conj(l1).*1.0./conj(q1).^2.*(sin(conj(alpha)+conj(q0)+conj(q1))-sin(conj(alpha)+conj(q0)))+(q2.*abs(l2).^2.*1.0./abs(q2).^2.*(cos(conj(alpha)+conj(q0)+conj(q1))-cos(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2)))./l2).*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)+q2.*cos(alpha+q0+q1+q2));
et33 = l2.*m2.*1.0./q2.^2.*((sin(conj(alpha)+conj(q0)+conj(q1)).*conj(l1))./conj(q1)+conj(l1).*1.0./conj(q1).^2.*(cos(conj(alpha)+conj(q0)+conj(q1))-cos(conj(alpha)+conj(q0)))-(q2.*abs(l2).^2.*1.0./abs(q2).^2.*(sin(conj(alpha)+conj(q0)+conj(q1))-sin(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2)))./l2).*(-cos(alpha+q0+q1)+cos(alpha+q0+q1+q2)+q2.*sin(alpha+q0+q1+q2));
et34 = l2.*m2.*1.0./q2.^2.*(sin(alpha+q0+q1)-sin(alpha+q0+q1+q2)+q2.*cos(alpha+q0+q1+q2)).*((conj(l1).*(cos(conj(alpha)+conj(q0)+conj(q1))-cos(conj(alpha)+conj(q0))))./conj(q1)+(cos(conj(alpha)+conj(q0)).*conj(l0))./conj(q0)+conj(l0).*1.0./conj(q0).^2.*(sin(conj(alpha))-sin(conj(alpha)+conj(q0)))-(q2.*abs(l2).^2.*1.0./abs(q2).^2.*(cos(conj(alpha)+conj(q0)+conj(q1))-cos(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2)))./l2);
et35 = -l2.*m2.*1.0./q2.^2.*(-cos(alpha+q0+q1)+cos(alpha+q0+q1+q2)+q2.*sin(alpha+q0+q1+q2)).*(conj(l0).*1.0./conj(q0).^2.*(cos(conj(alpha))-cos(conj(alpha)+conj(q0)))-(conj(l1).*(sin(conj(alpha)+conj(q0)+conj(q1))-sin(conj(alpha)+conj(q0))))./conj(q1)-(sin(conj(alpha)+conj(q0)).*conj(l0))./conj(q0)+(q2.*abs(l2).^2.*1.0./abs(q2).^2.*(sin(conj(alpha)+conj(q0)+conj(q1))-sin(conj(alpha)+conj(q0)+conj(q1)+abs(q2).^2./q2)))./l2);
B = reshape([et13+et14+et15+et16+et17,et7+et8,et1+et2,et28+et29,et20+et21+et22+et23,et18+et19,et34+et35,et32+et33,et30+et31],[3,3]);
