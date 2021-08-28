%% Run startup
startup

%% Parameters
% This script uses the outputs of the simulink model saved in the out
% variable of the workspace

% frames per second
fps = 20;
% how many times to repeat the movie
repeat = 1;

%% Animation
% time information
time = out.tout;
% delta between time steps
delta_t = time(2:end) - time(1:end-1);
% maximal step size
max_delta_t = max(delta_t);

% we adjust fps accordingly
fps = min([fps, 1/max_delta_t])

alpha = out.alpha.Data;
out_l = out.l.Data;

q = out.q.Data;
% introduce tolerance for numerical stability
q = set_min_abs_val(q, 0.005);
q0 = q(:, 1);
q1 = q(:, 2);
q2 = q(:, 3);

kappa0 = q0/out_l(1);
kappa1 = q1/out_l(2);
kappa2 = q2/out_l(3);
kappa = [kappa0, kappa1, kappa2];

s0 = (0:out_l(1)/100:out_l(1))';
s1 = (0:out_l(2)/100:out_l(2))';
s2 = (0:out_l(3)/100:out_l(3))';
s = zeros(size(s0, 1)+size(s1, 1)+size(s2, 1), 3);
s(1:size(s0, 1), 1) = s0;
s(1:size(s0, 1), 2) = 0;
s(1:size(s0, 1), 3) = 0;
s(1+size(s0, 1):size(s0, 1)+size(s1, 1), 1) = out_l(1);
s(1+size(s0, 1):size(s0, 1)+size(s1, 1), 2) = s1;
s(1+size(s0, 1):size(s0, 1)+size(s1, 1), 3) = 0;
s(1+size(s0, 1)+size(s1, 1):size(s0, 1)+size(s1, 1)+size(s2, 1), 1) = out_l(1);
s(1+size(s0, 1)+size(s1, 1):size(s0, 1)+size(s1, 1)+size(s2, 1), 2) = out_l(2);
s(1+size(s0, 1)+size(s1, 1):size(s0, 1)+size(s1, 1)+size(s2, 1), 3) = s2;

s_m = [out_l(1), 0, 0;
       out_l(1), out_l(2), 0;
       out_l(1), out_l(2), out_l(3)];

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
        xlim([-(sum(l)), (sum(l))]);
        ylim([-(sum(l)), (sum(l))]);
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

