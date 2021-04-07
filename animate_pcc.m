% This script uses the outputs of the simulink model saved in the out
% variable of the workspace

% frames per second
fps = 10;
% how many times to repeat the movie
repeat = 1;

% time information
time = out.tout;
% delta between time steps
delta_t = time(2:end) - time(1:end-1);
% maximal step size
max_delta_t = max(delta_t);

% we adjust fps accordingly
fps = min([fps, 1/max_delta_t])

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

%% gathering of frames
fh = figure;
fh.Visible = 'off';
sim_range = 1:1:size(kappa, 1);
num_frames = ceil(time(end)*fps);
clear M;
M(num_frames) = struct('cdata',[],'colormap',[]);
t_last_frame = -Inf;
frame_idx = 0;
f = waitbar(0, 'Creating movie...');
for sim_idx=1:length(sim_range)
    t = time(sim_idx);
    if (t - t_last_frame) >= 1/fps
        frame_idx = frame_idx + 1;
        waitbar(frame_idx/num_frames, f);
        kappa_pcc_t = repmat(kappa(sim_idx, :), size(s, 1), 1);
        x_pcc_t = forward_kinematics(alpha, s, kappa_pcc_t);
        plot(x_pcc_t(:, 1), x_pcc_t(:, 2))
        hold on;
        kappa_m_t = repmat(kappa(sim_idx, :), size(s_m, 1), 1);
        x_m_t = forward_kinematics(alpha, s_m, kappa_m_t);
        plot(x_m_t(:, 1), x_m_t(:, 2), 'r*')
        xlim([-(l0+l1+l2), (l0+l1+l2)]);
        ylim([-(l0+l1+l2), (l0+l1+l2)]);
        hold off;
        drawnow;
        M(frame_idx) = getframe;
        t_last_frame = t;
    end
end
close(f);

%% Display of movie
fh.Visible = 'on';
movie(M, [repeat], fps);

