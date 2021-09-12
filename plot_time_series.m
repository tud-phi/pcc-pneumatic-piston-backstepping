%% Run startup
startup

%% Plot time series
out_backstepping = load('data/out_backstepping_v1.mat').out;
out_pid = load('data/out_pid.mat').out;

f = figure('Name', 'Configuration time series');
grid on
box on
% f.Position(3:4) = [560 2*420];
set(gcf,'color','w');
hold on
% title('Configuration $q$', Interpreter='latex');
xlabel('Time [s]', Interpreter='latex');
ylabel('$q$ [rad]', Interpreter='latex');
ylim([-115 +45]);
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