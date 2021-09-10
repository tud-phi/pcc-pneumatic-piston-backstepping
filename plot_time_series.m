%% Run startup
startup

%% Plot time series
out_backstepping = load('data/out_backstepping.mat').out;
out_pid = load('data/out_pid.mat').out;

f = figure('Name', 'Configuration time series');
grid on
box on
% f.Position(3:4) = [560 2*420];
set(gcf,'color','w');
hold on
% title('Configuration $q$', Interpreter='latex');
xlabel('Time [s]', Interpreter='latex');
ylabel('$q$ [$^{\circ}$]', Interpreter='latex');
ylim([-120 +45]);
plot(out_backstepping.q_ref/pi*180, LineWidth=2);
set(gca, 'ColorOrderIndex', 1)
plot(out_backstepping.q/pi*180, '--', LineWidth=2);
set(gca, 'ColorOrderIndex', 1)
plot(out_pid.q/pi*180, ':', LineWidth=2);
lgd = legend('$q_1$','$q_2$', '$q_3$', Interpreter='latex');
lgd.FontSize = 8;
hold off


f = figure('Name', 'Piston position time series');
grid on
box on
set(gcf,'color','w');
hold on
% title('Piston position $\mu_\mathrm{p}$', Interpreter='latex');
xlabel('Time [s]', Interpreter='latex');
ylabel('$\mu_\mathrm{p}$ [mm]', Interpreter='latex');
plot(out_backstepping.mu_p*1000, '--', LineWidth=2);
set(gca, 'ColorOrderIndex', 1)
plot(out_pid.mu_p*1000, ':', LineWidth=2);
lgd = legend('$\mu_{\mathrm{p},1,\mathrm{L}}$', ...
             '$\mu_{\mathrm{p},1,\mathrm{R}}$', ...
             '$\mu_{\mathrm{p},2,\mathrm{L}}$', ...
             '$\mu_{\mathrm{p},2,\mathrm{R}}$', ...
             '$\mu_{\mathrm{p},3,\mathrm{L}}$', ...
             '$\mu_{\mathrm{p},3,\mathrm{R}}$', ...
             Interpreter='latex');
lgd.FontSize = 8;
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
plot(out_backstepping.f_p, '--', LineWidth=1);
set(gca, 'ColorOrderIndex', 1)
plot(out_pid.f_p, ':', LineWidth=1);
lgd = legend('$f_{\mathrm{p},1,\mathrm{L}}$', ...
             '$f_{\mathrm{p},1,\mathrm{R}}$', ...
             '$f_{\mathrm{p},2,\mathrm{L}}$', ...
             '$f_{\mathrm{p},2,\mathrm{R}}$', ...
             '$f_{\mathrm{p},3,\mathrm{L}}$', ...
             '$f_{\mathrm{p},3,\mathrm{R}}$', ...
             Interpreter='latex');
lgd.FontSize = 8;
hold off