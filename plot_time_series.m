%% Run startup
startup

%% Plot time
% standard actuator dynamics configuration (m_p = 0.19kg)
out_backstepping = load('data/out_backstepping_v3_d_p-10000.mat').out;
out_pid = load('data/out_pid_d_p-10000.mat').out;
% increase piston mass m_p to 0.5kg
% out_backstepping = load('data/out_backstepping_v3_m_p-0.5_d_p-10000.mat').out;
% out_pid = load('data/out_pid_m_p-0.5_d_p-10000_untuned.mat').out;

f = figure('Name', 'Configuration time series');
grid on
box on
f.Position(3:4) = [560 315];
set(gcf,'color','w');
hold on
% title('Configuration $q$', Interpreter='latex');
xlabel('Time [s]', Interpreter='latex');
ylabel('$q$ [rad]', Interpreter='latex');
% ylim([-2.0071 +0.7854]);
ylim([-2.0071 +1.8]);
plot(out_backstepping.q_ref, LineWidth=1.75);
set(gca, 'ColorOrderIndex', 1)
plot(out_backstepping.q, '--', LineWidth=1.75);
set(gca, 'ColorOrderIndex', 1)
plot(out_pid.q, ':', LineWidth=1.75);
lgd = legend('$\bar{q}_1$','$\bar{q}_2$', '$\bar{q}_3$', ...
             '$q_{\mathrm{Back},1}$', '$q_{\mathrm{Back},2}$', '$q_{\mathrm{Back},3}$', ...
             '$q_{\mathrm{PID},1}$', '$q_{\mathrm{PID},2}$', '$q_{\mathrm{PID},3}$', ...
             Interpreter='latex');
lgd.FontSize = 11;
hold off


f = figure('Name', 'Piston position time series');
f.Position(3:4) = [560 315];
grid on
box on
set(gcf,'color','w');
hold on
% title('Piston position $\mu_\mathrm{p}$', Interpreter='latex');
xlabel('Time [s]', Interpreter='latex');
ylabel('$\mu_\mathrm{p}$ [mm]', Interpreter='latex');
plot(out_backstepping.mu_p*1000, '--', LineWidth=1.75);
set(gca, 'ColorOrderIndex', 1)
plot(out_pid.mu_p*1000, ':', LineWidth=1.75);
lgd = legend('$\mu_{\mathrm{p},1,\mathrm{L}}$', ...
             '$\mu_{\mathrm{p},1,\mathrm{R}}$', ...
             '$\mu_{\mathrm{p},2,\mathrm{L}}$', ...
             '$\mu_{\mathrm{p},2,\mathrm{R}}$', ...
             '$\mu_{\mathrm{p},3,\mathrm{L}}$', ...
             '$\mu_{\mathrm{p},3,\mathrm{R}}$', ...
             Interpreter='latex');
lgd.FontSize = 11;
hold off

f = figure('Name', 'Actuation force time series');
f.Position(3:4) = [560 315];
grid on
box on
set(gcf,'color','w');
hold on
% title('Actuation force $f_\mathrm{p}$', Interpreter='latex');
xlabel('Time [s]', Interpreter='latex');
ylabel('$f_\mathrm{p}$ [N]', Interpreter='latex');
ylim([-350 0]);
plot(out_backstepping.f_p, '--', LineWidth=1.75);
set(gca, 'ColorOrderIndex', 1)
plot(out_pid.f_p, ':', LineWidth=1.75);
lgd = legend('$f_{\mathrm{p},1,\mathrm{L}}$', ...
             '$f_{\mathrm{p},1,\mathrm{R}}$', ...
             '$f_{\mathrm{p},2,\mathrm{L}}$', ...
             '$f_{\mathrm{p},2,\mathrm{R}}$', ...
             '$f_{\mathrm{p},3,\mathrm{L}}$', ...
             '$f_{\mathrm{p},3,\mathrm{R}}$', ...
             Interpreter='latex');
lgd.FontSize = 11;
hold off

f = figure('Name', 'Pressure time series');
f.Position(3:4) = [560 315];
grid on
box on
set(gcf,'color','w');
hold on
% title('Actuation force $f_\mathrm{p}$', Interpreter='latex');
xlabel('Time [s]', Interpreter='latex');
ylabel('$p$ [bar]', Interpreter='latex');
% ylim([1 5]);
plot(out_backstepping.p ./ 10^5, '--', LineWidth=1.75);
set(gca, 'ColorOrderIndex', 1)
plot(out_pid.p ./ 10^5, ':', LineWidth=1.75);
lgd = legend('$p_{1,\mathrm{L}}$', ...
             '$p_{1,\mathrm{R}}$', ...
             '$p_{2,\mathrm{L}}$', ...
             '$p_{2,\mathrm{R}}$', ...
             '$p_{3,\mathrm{L}}$', ...
             '$p_{3,\mathrm{R}}$', ...
             Interpreter='latex');
lgd.FontSize = 11;
hold off