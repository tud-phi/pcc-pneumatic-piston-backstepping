function x = forward_kinematics(alpha, l, kappa)
    % Theta_0: 1x1 of initial offset angle [rad]
    % k: nx3 curvatures of each of 3 segments
    % l: nx3 lengths of each of 3 segments
    
    theta0 = alpha + kappa(:, 1).*l(:, 1);
    theta1 = theta0 + kappa(:, 2).*l(:, 2);
    theta2 = theta1 + kappa(:, 3).*l(:, 3);
    x0 = [(sin(theta0)-sin(alpha))./kappa(:,1), -(cos(theta0)-cos(alpha))./kappa(:,1)];
    x1 = x0 + [(sin(theta1)-sin(theta0))./kappa(:,2), -(cos(theta1)-cos(theta0))./kappa(:,2)];
    x2 = x1 + [(sin(theta2)-sin(theta1))./kappa(:,3), -(cos(theta2)-cos(theta1))./kappa(:,3)];
    x = x2;
end

