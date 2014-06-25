record_length = 500;
num_values = 5;
imu_data = zeros(record_length, num_values);

figure(1);

clf;

x1 = line();
set(x1, 'Color', [1 0 0]);

x2 = line();
set(x2, 'Color', [0 1 0]);

x3 = line();
set(x3, 'Color', [0 0 1]);

x4 = line();
set(x4, 'Color', [1 1 0]);

x5 = line();
set(x4, 'Color', [0 1 1]);

% x6 = line();
% set(x4, 'Color', [1 0 1]);

% ylim([-20 20]);

while 1
    message = robot.writeRaw([2 0 0 0]);
    
    imu_rx = typecast(uint8(message.data), 'single');       % convert to singles (floats)
    imu_rx = reshape(imu_rx, num_values, length(imu_rx)/num_values)';         % reshape to 3 x n
    
    imu_data = cat(1, imu_data, imu_rx);                    % add to plot FIFO
    imu_data = imu_data(end-record_length+1:end, :);          % trim FIFO to fixed length
    
    
    set(x1, 'XData', 1:record_length, 'ydata', imu_data(:,1))
    set(x2, 'XData', 1:record_length, 'ydata', imu_data(:,2))
    set(x3, 'XData', 1:record_length, 'ydata', imu_data(:,3))
    set(x4, 'XData', 1:record_length, 'ydata', imu_data(:,4))
    set(x5, 'XData', 1:record_length, 'ydata', imu_data(:,5))
%     set(x6, 'XData', 1:record_length, 'ydata', imu_data(:,6))
    
    drawnow;
    
%     pause(0.1);
end