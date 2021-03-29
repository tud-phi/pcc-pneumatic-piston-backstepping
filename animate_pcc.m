alpha = out.alpha.Data;

l = out.l.Data;
l0 = l(:, 1);
l1 = l(:, 2);
l2 = l(:, 3);

q = out.q.Data;
q0 = q(:, 1);
q1 = q(:, 2);
q2 = q(:, 3);

kappa0 = q0/l0;
kappa1 = q1/l1;
kappa2 = q2/l2;
kappa = [kappa0, kappa1, kappa2];

s0 = (0:l0/100:l0)';
s1 = (0:l1/100:l1)';
s2 = (0:l2/100:l2)';
s = zeros(size(s0, 1)+size(s1, 1)+size(s2, 1), 3);
s(1:size(s0, 1), 1) = s0;
s(1:size(s0, 1), 2) = 0;
s(1:size(s0, 1), 3) = 0;
s(1+size(s0, 1):size(s0, 1)+size(s1, 1), 1) = l0;
s(1+size(s0, 1):size(s0, 1)+size(s1, 1), 2) = s1;
s(1+size(s0, 1):size(s0, 1)+size(s1, 1), 3) = 0;
s(1+size(s0, 1)+size(s1, 1):size(s0, 1)+size(s1, 1)+size(s2, 1), 1) = l0;
s(1+size(s0, 1)+size(s1, 1):size(s0, 1)+size(s1, 1)+size(s2, 1), 2) = l1;
s(1+size(s0, 1)+size(s1, 1):size(s0, 1)+size(s1, 1)+size(s2, 1), 3) = s2;

s_m = [l0, 0, 0;
       l0, l1, 0;
       l0, l1, l2];

fh = figure;
fh.Visible = 'off';
frame_range = 1:50:size(kappa, 1);
clear M;
M(size(frame_range, 2)) = struct('cdata',[],'colormap',[]);
for it=1:length(frame_range)
    idx = frame_range(it);
    idx
    kappa_pcc_t = repmat(kappa(idx, :), size(s, 1), 1);
    x_pcc_t = forward_kinematics(alpha, s, kappa_pcc_t);
    plot(x_pcc_t(:, 1), x_pcc_t(:, 2))
    hold on;
    kappa_m_t = repmat(kappa(idx, :), size(s_m, 1), 1);
    x_m_t = forward_kinematics(alpha, s_m, kappa_m_t);
    plot(x_m_t(:, 1), x_m_t(:, 2), 'r*')
    xlim([-(l0+l1+l2), (l0+l1+l2)]);
    ylim([-(l0+l1+l2), (l0+l1+l2)]);
    hold off;
    drawnow;
    M(it) = getframe;
end
fh.Visible = 'on';
movie(M);

