% Load logged data from the VN-100 IMU.
load('imu_data.mat', 'angular_velocity', 'linear_acceleration', 'magnetic_field');
Fs = 40;
t0 = 1/Fs;

% Convert magnetic field data from tesla to gauss
magnetic_field_gauss = magnetic_field * 1e4; % 1 tesla = 10^4 gauss

% Prepare data for IMU Sensor
omega = rad2deg(angular_velocity); % Gyroscope data
sensorData = {linear_acceleration, magnetic_field_gauss, omega}; % {accelerometer, magnetometer, gyroscope}
sensorNames = {'Accelerometer', 'Magnetometer', 'Gyroscope'};
sensorUnits = {'(m/s^2)', '(gauss)', '(deg/s)'}; % Updated magnetometer unit to gauss

% Function to calculate Allan variance
function [tau, adev] = calculate_allan_variance(data, Fs)
    t0 = 1/Fs;
    L = length(data);
    
    % Calculate the Allan variance
    maxNumM = 100; 
    maxM = 2.^floor(log2(L/2)); 
    m = logspace(log10(1), log10(maxM), maxNumM).'; 
    m = ceil(m); 
    m = unique(m); 

    tau = m * t0; 
    avar = zeros(numel(m), 1); 
    for i = 1:numel(m)
        mi = m(i);
        if (1 + 2*mi <= L) && (1 + mi <= L - mi) && (1 <= L - 2*mi)
            avar(i) = sum((data(1 + 2*mi:L) - 2*data(1 + mi:L - mi) + data(1:L - 2*mi)).^2);
        else
            avar(i) = NaN; 
        end
    end
    
    avar = avar ./ (2 * tau.^2 .* (L - 2 * m)); 
    adev = sqrt(avar);
end

% Loop through each sensor
for s = 1:length(sensorData)
    data = sensorData{s}; % Get the current sensor data
    numAxes = size(data, 2); % Number of axes (x, y, z)
    
    figure;
    for axis = 1:numAxes
        subplot(numAxes, 1, axis); % Create a subplot for each axis
        % Calculate Allan deviation for the current axis
        [tau, adev] = calculate_allan_variance(data(:, axis), Fs);
        
        % Find the angle random walk coefficient (N)
        slope = -0.5;
        logtau = log10(tau);
        logadev = log10(adev);
        dlogadev = diff(logadev) ./ diff(logtau);
        [~, i] = min(abs(dlogadev - slope));
        b = logadev(i) - slope*logtau(i);
        logN = slope*log(1) + b;
        N = 10^logN;
        
        % Find the rate random walk coefficient (K)
        slope = 0.5;
        [~, i] = min(abs(dlogadev - slope));
        b = logadev(i) - slope*logtau(i);
        logK = slope*log10(3) + b;
        K = 10^logK;
        
        % Find the bias instability coefficient (B)
        slope = 0;
        [~, i] = min(abs(dlogadev - slope));
        b = logadev(i) - slope*logtau(i);
        scfB = sqrt(2*log(2)/pi);
        logB = b - log10(scfB);
        B = 10^logB;

        % Calculate lines for N, K, B
        tauN = 1; lineN = N ./ sqrt(tau);
        tauK = 3; lineK = K .* sqrt(tau/3);
        tauB = tau(i); lineB = B * scfB * ones(size(tau));
        
        % Plot Allan deviation and lines
        loglog(tau, adev, 'DisplayName', '\sigma', 'LineWidth', 1.5); hold on;
        loglog(tau, lineN, '--', 'DisplayName', '\sigma_N', 'LineWidth', 1.5); 
        loglog(tau, lineK, '--', 'DisplayName', '\sigma_K', 'LineWidth', 1.5); 
        loglog(tau, lineB, '--', 'DisplayName', '\sigma_B', 'LineWidth', 1.5); 
        
        % Add point markers and text annotation at key points (N, K, B)
        loglog(tauN, N, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');
        text(tauN, N, sprintf('N=%.4f', N), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
        
        loglog(tauK, K, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');
        text(tauK, K, sprintf('K=%.4f', K), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
        
        loglog(tauB, B, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');
        text(tauB, B, sprintf('B=%.4f', B), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');

        hold off;
        
        title([sensorNames{s} ' Allan Deviation Axis ' num2str(axis)]);
        xlabel('\tau (s)');
        ylabel(['\sigma(\tau) ' sensorUnits{s}]);
        legend('show', 'Location', 'best');
        grid on;
        axis equal;
    end
end
