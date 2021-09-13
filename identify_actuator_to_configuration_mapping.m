%% Run startup
startup

%% Identify actuator to configuration mapping
% open_system('pcc_actuator_to_configuration_mapping_identification');
% load_system('pcc_actuator_to_configuration_mapping_identification')

% deactivate all simulink warnings
w=warning('off','all');

max_s_step_gain = 0.005;
num_steps = 50;
s_array = zeros(num_steps, n_b);
q_array = zeros(num_steps, n_b);
a_array = zeros(num_steps, n_b);
idx = 0;
for s_step_magnitude = -max_s_step_gain:2*max_s_step_gain/num_steps:max_s_step_gain
    % skip configurations close to zero because of numerical issues
    if abs(s_step_magnitude/max_s_step_gain) < 0.05
       continue 
    end
    
    idx = idx + 1;
    
    s_step_magnitude
    s = s_step_magnitude*[1;1;1];
    s_step_gain = s;
    s_array(idx, :) = s;
    
    out = sim('pcc_actuator_to_configuration_mapping_identification');
    q_steady_state = out.q.Data(end, :);
    q_array(idx, :) = q_steady_state';
    
    % a = k / s * q
    a_idx = out.k.Data(end, :) ./ s' .* q_steady_state;
    a_array(idx, :) = a_idx;
end

figure;
grid on
box on
set(gcf,'color','w');

hold on
for i=1:n_b
    plot(s_array(1:idx, i)*1000, q_array(1:idx, i), '-o')
end

xlabel('piston step [mm]');
ylabel('configuration [rad]');
legend('q1', 'q2', 'q3');
hold off

a = mean(a_array(1:idx, :), 1)