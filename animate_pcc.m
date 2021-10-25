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

%% gathering of frames
fh = figure;
fh.Position(3:4) = [640 640];
set(gcf,'color','w');
grid on
box on
axis equal
% fh.Visible = 'off';

sim_range = 1:1:size(out.tout, 1);
num_frames = ceil(time(end)*fps);
clear M;
M_movie(num_frames) = struct('cdata',[],'colormap',[]);
M_video(num_frames) = struct('cdata',[],'colormap',[]);
t_last_frame = -Inf;
frame_idx = 0;
f = waitbar(0, 'Creating movie...');

% plot robot positions
for sim_idx=1:length(sim_range)
    t = time(sim_idx);
    if (t - t_last_frame) >= 1/fps
        frame_idx = frame_idx + 1;
        waitbar(frame_idx/num_frames, f);

        for i=1:length(out_l)
            s = gen_s_cartesian_evolution(out.l.Data, i);
            kappa_pcc_steady_t = repmat(set_min_abs_val(out.q.Data(sim_idx, :), 0.001) ./ out.l.Data, size(s, 1), 1);
            x_pcc_steady_t = forward_kinematics(out.alpha.Data, s, kappa_pcc_steady_t);
            plot(x_pcc_steady_t(:, 1)*100, x_pcc_steady_t(:, 2)*100, LineWidth=3)
            hold on;
        end
        set(gca, 'ColorOrderIndex', 1)

        % plot cartesian evolution of tip of segments
        for i=1:length(out_l)
            xy_data = out.('x'+string(i-1)).Data;
            plot(xy_data(1:sim_idx, 1)*100, xy_data(1:sim_idx, 2)*100, ':', LineWidth=1.5);
        end
        set(gca, 'ColorOrderIndex', 1)

        xlim([-(sum(l)*105), (sum(l)*105)]);
        ylim([-(sum(l)*105), (sum(l)*105)]);
        xlabel('$x$ [cm]', Interpreter='latex');
        ylabel('$y$ [cm]', Interpreter='latex');
        
        hold off;
        drawnow;
        M_movie(frame_idx) = getframe;
        M_video(frame_idx) = getframe(fh);
        t_last_frame = t;
    end
end
close(f);

%% Save movie
v = VideoWriter('animate_pcc.mp4', 'MPEG-4');
v.FrameRate = fps;
disp(v.Height)
disp(v.Width)
open(v);
writeVideo(v, M_video);
close(v);

%% Display of movie
clf(fh, 'reset');
fh.Visible = 'on';
movie(M_movie, [repeat], fps);
