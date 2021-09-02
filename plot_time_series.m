out_backstepping = load('data/out_backstepping.mat').out;
out_pid = load('data/out_backstepping_k1-250_k2-100.mat').out;

f = figure;
f.Position(3:4) = [560 2*420];
set(gcf,'color','w');
num_plots = 3;

q_subplot = subplot(num_plots, 1, 1);
hold on
title('Configuration $q$', Interpreter='latex');
xlabel('Time [s]', Interpreter='latex');
ylabel('$q$ [$^{\circ}$]', Interpreter='latex');
ylim([-45 +45]);
plot(out_backstepping.q/pi*180, '--');
set(gca, 'ColorOrderIndex', 1)
plot(out_pid.q/pi*180, ':');
lgd = legend('$q_1$','$q_2$', '$q_3$', Interpreter='latex');
lgd.FontSize = 8;
hold off


mu_p_subplot = subplot(num_plots, 1, 2);
hold on
title('Piston position $\mu_\mathrm{p}$', Interpreter='latex');
xlabel('Time [s]', Interpreter='latex');
ylabel('$\mu_\mathrm{p}$ [mm]', Interpreter='latex');
plot(out_backstepping.mu_p*1000, '--');
set(gca, 'ColorOrderIndex', 1)
plot(out_pid.mu_p*1000, ':');
lgd = legend('$\mu_{\mathrm{p},1,\mathrm{L}}$', ...
             '$\mu_{\mathrm{p},1,\mathrm{R}}$', ...
             '$\mu_{\mathrm{p},2,\mathrm{L}}$', ...
             '$\mu_{\mathrm{p},2,\mathrm{R}}$', ...
             '$\mu_{\mathrm{p},3,\mathrm{L}}$', ...
             '$\mu_{\mathrm{p},3,\mathrm{R}}$', ...
             Interpreter='latex');
lgd.FontSize = 8;
hold off

f_p_subplot = subplot(num_plots, 1, 3);
hold on
title('Actuation force $f_\mathrm{p}$', Interpreter='latex');
xlabel('Time [s]', Interpreter='latex');
ylabel('$f_\mathrm{p}$ [N]', Interpreter='latex');
ylim([-500 +500]);
plot(out_backstepping.f_p, '--');
set(gca, 'ColorOrderIndex', 1)
plot(out_pid.f_p, ':');
lgd = legend('$f_{\mathrm{p},1,\mathrm{L}}$', ...
             '$f_{\mathrm{p},1,\mathrm{R}}$', ...
             '$f_{\mathrm{p},2,\mathrm{L}}$', ...
             '$f_{\mathrm{p},2,\mathrm{R}}$', ...
             '$f_{\mathrm{p},3,\mathrm{L}}$', ...
             '$f_{\mathrm{p},3,\mathrm{R}}$', ...
             Interpreter='latex');
lgd.FontSize = 8;
hold off