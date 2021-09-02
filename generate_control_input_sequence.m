%% Run startup
startup

%% Compute input sequence
sim_duration = 90; % [s]
ts_rate = 10; % sampling rate of timeseries

timevals = 0:1/ts_rate:sim_duration;

% matrix with n_b rows (because we have n_b segments) and ts_rate*sim_duration rows
datavals = 20/180*pi*ones(length(timevals), n_b);

qref_ts = timeseries(datavals,timevals);
save('data/in_qref_ts', 'qref_ts', '-v7.3');