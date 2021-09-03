function [s] = gen_s_cartesian_evolution(l, i)
    % l: 3x1 vector containing the length of each segment
    % i: segment index for which we want to generate the s matrix
    n_b = length(l);
    s_steps = (0:l(i)/100:l(i))';
    s = zeros(size(s_steps, 1), 3);
    for i_hat = 1:n_b
        if i_hat == i
            s(1:size(s_steps, 1), i_hat) = s_steps;
            break
        else
            s(1:size(s_steps, 1), i_hat) = l(i_hat);
        end
    end
end

