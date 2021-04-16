function tau_split = distribute_tau(tau)
    tau_split = zeros(2*length(tau),1);
    for i=1:length(tau)
       tau_split(2*i-1) = tau(i)/2;
       tau_split(2*i) = -tau(i)/2;
    end
end

