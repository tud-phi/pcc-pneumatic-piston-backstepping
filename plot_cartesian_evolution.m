%% Run startup
startup

%% Plot time series
out_sim = load('data/out_backstepping.mat').out;

f = figure('Name', 'Cartesian evolution');
grid on
box on
f.Position(3:4) = [380 420];
set(gcf,'color','w');
axis equal
hold on

% plot cartesian evolution of tip of segments
xlabel('$x$ [cm]', Interpreter='latex');
ylabel('$y$ [cm]', Interpreter='latex');
for i=1:n_b
    xy_data = out_sim.('x'+string(i-1)).Data;
    plot(xy_data(:, 1)*100, xy_data(:, 2)*100, ':', LineWidth=1.5);
end
% lgd = legend('segment 1','segment 2', 'segment 3', Interpreter='latex');
% lgd.FontSize = 8;
set(gca, 'ColorOrderIndex', 1)

% plot reference robot configurations
qref_ts = out_sim.q_ref;
ts_step_size = floor(length(qref_ts.Time) / 5000);
q_prior = 0;
for time_idx = 1:ts_step_size:length(qref_ts.Time)
    if q_prior ~= qref_ts.Data(time_idx, :)
        disp('New reference at time '+string(qref_ts.Time(time_idx))+': ');
        disp(qref_ts.Data(time_idx, :));
        
        for i=1:n_b
            s = gen_s_cartesian_evolution(out_sim.l.Data, i);
            kappa_pcc_ref_t = repmat(set_min_abs_val(qref_ts.Data(time_idx, :), 0.001) ./ out_sim.l.Data, size(s, 1), 1);
            x_pcc_ref_t = forward_kinematics(out_sim.alpha.Data, s, kappa_pcc_ref_t);
            plot(x_pcc_ref_t(:, 1)*100, x_pcc_ref_t(:, 2)*100, 'k', LineWidth=6)
        end
        
        if time_idx > 1
            for i=1:n_b
                s = gen_s_cartesian_evolution(out_sim.l.Data, i);
                kappa_pcc_steady_t = repmat(set_min_abs_val(out_sim.q.Data(time_idx-1, :), 0.001) ./ out_sim.l.Data, size(s, 1), 1);
                x_pcc_steady_t = forward_kinematics(out_sim.alpha.Data, s, kappa_pcc_steady_t);
                plot(x_pcc_steady_t(:, 1)*100, x_pcc_steady_t(:, 2)*100, LineWidth=2.5)
            end
            set(gca, 'ColorOrderIndex', 1)
        end
        
        q_prior = qref_ts.Data(time_idx, :);
    end
end
for i=1:n_b
    s = gen_s_cartesian_evolution(out_sim.l.Data, i);
    kappa_pcc_steady_t = repmat(set_min_abs_val(out_sim.q.Data(end, :), 0.001) ./ out_sim.l.Data, size(s, 1), 1);
    x_pcc_steady_t = forward_kinematics(out_sim.alpha.Data, s, kappa_pcc_steady_t);
    plot(x_pcc_steady_t(:, 1)*100, x_pcc_steady_t(:, 2)*100, LineWidth=2.5)
end
set(gca, 'ColorOrderIndex', 1)

xlim([-2 +35]);
ylim([-32 +15]);
hold off