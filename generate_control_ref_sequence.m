%% Run startup
startup

%% Compute reference sequence
sim_duration = 90; % [s]
ts_rate = 10; % sampling rate of timeseries

timevals = 0:1/ts_rate:sim_duration;

% matrix with n_b rows (because we have n_b segments) and ts_rate*sim_duration rows
datavals = zeros(length(timevals), n_b);

start_idx = 1;

% equilibrium position
period_duration = 30;
stop_idx = start_idx + ts_rate*period_duration;
datavals(start_idx:stop_idx, :) = repmat(q_eq', ts_rate*period_duration+1, 1);
start_idx = stop_idx;

% zero position
period_duration = 30;
stop_idx = start_idx + ts_rate*period_duration;
datavals(start_idx:stop_idx, :) = -5*repmat([1, 1, 1], ts_rate*period_duration+1, 1)/180*pi;
start_idx = stop_idx;

% wave position
period_duration = 30;
stop_idx = start_idx + ts_rate*period_duration;
datavals(start_idx:stop_idx, :) = repmat([30, -30, 15], ts_rate*period_duration+1, 1)/180*pi;
start_idx = stop_idx;

qref_ts = timeseries(datavals,timevals);
save('data/in_qref_ts', 'qref_ts', '-v7.3');