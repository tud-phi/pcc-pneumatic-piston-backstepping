function q_hat = set_min_abs_val(q, tol)
    q_hat = q;
    
    pos_selector = 0 <= q_hat & q_hat < tol;
    q_hat(pos_selector) = tol;
    neg_selector = -tol <= q_hat & q_hat < 0;
    q_hat(neg_selector) = -tol;
    
%     for i=1:length(q)
%         if 0 <= q(i) && q(i) < tol
%             q_hat(i) = tol;
%         elseif q(i) < 0 && -tol < q(i)
%             q_hat(i) = -tol;
%         end
%     end
end

