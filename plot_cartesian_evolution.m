%% Run startup
startup

%% Plot time series
out_sim = load('data/out_backstepping.mat').out;

f = figure;
grid on
box on
% f.Position(3:4) = [560 420];
set(gcf,'color','w');
hold on

% plot cartesian evolution of tip of segments
xlabel('$x$ [cm]', Interpreter='latex');
ylabel('$y$ [cm]', Interpreter='latex');
for i=1:n_b
    xy_data = out_sim.('x'+string(i-1)).Data;
    plot(xy_data(:, 1)*100, xy_data(:, 2)*100, ':');
end
% lgd = legend('segment 1','segment 2', 'segment 3', Interpreter='latex');
% lgd.FontSize = 8;
set(gca, 'ColorOrderIndex', 1)

% plot reference robot configurations
qref_ts = out_sim.q_ref;
ts_step_size = floor(length(qref_ts.Time) / 100);
q_prior = 0;
for time_idx = 1:ts_step_size:length(qref_ts.Time)
    if q_prior ~= qref_ts.Data(time_idx, :)
        disp('New reference at time '+string(qref_ts.Time(time_idx))+': ');
        disp(qref_ts.Data(time_idx, :));
        
        for i=1:n_b
            s = gen_s_cartesian_evolution(out_sim.l.Data, i);
            kappa_pcc_t = repmat(set_min_abs_val(qref_ts.Data(time_idx, :), 0.005) ./ out_sim.l.Data, size(s, 1), 1);
            x_pcc_t = forward_kinematics(out_sim.alpha.Data, s, kappa_pcc_t);
            plot(x_pcc_t(:, 1)*100, x_pcc_t(:, 2)*100)
        end
        set(gca, 'ColorOrderIndex', 1)
        
        q_prior = qref_ts.Data(time_idx, :);
    end
end

hold off