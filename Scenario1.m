% Vehicle and controller parameters 

v_ego = 15; % m/s (Initial speed) 

v_lead = 15; % m/s (Initial speed) 

x_ego = 0;   % Initial position 

x_lead = 20; % Initial position  

 

desired_distance = 17.5; % m (Target distance) 

critical_distance = 10;  % m (minimum distance) 

upper_distance_limit = 25; % m (Target upper boundary) 

 

max_force = 8000;      % N (Max acceleration/braking force) 

mass_ego = 1500;       % kg 

 

% Simulation setup 

time_step = 0.1;  

simulation_time = 20;  

time_vector = 0:time_step:simulation_time; 

 

% PID state 

integral_error = 0; 

previous_error = 0; 

 

% Data Logging 

v_ego_log = zeros(size(time_vector)); % Ego Velocity 

v_lead_log = zeros(size(time_vector)); % Lead Velocity 

distance_log = zeros(size(time_vector)); % Inter-Vehicle Distance 

 

for i = 1:length(time_vector) 

    t = time_vector(i); 

 

    % Random fluctuation in lead vehicle's acceleration 

    a_lead = randn() * 0.3 +randn(); % random noise with deviation 0f 0.3 + random num 

    v_lead = max(0, v_lead + a_lead * time_step); % Update lead speed, ensure non-negative 

    x_lead = x_lead + v_lead * time_step;         % Update lead position 

 

    % Compute distance between vehicles 

    distance = max(0, x_lead - x_ego); % Ensure non-negative distance 

 

    % PID error relative to the desired distance 

    error = distance - desired_distance; 

 

    % --- Control Logic --- % 

    if distance < critical_distance 

        % Emergency Override: Apply full braking if below critical distance 

        control_force = -max_force; 

        integral_error = 0; % Reset integral term 

    else 

        % Adaptive PID: Adjust gains based on current distance relative to boundaries 

        if distance < 14 % Getting close to the lower bound (10m < d < 14m) 

            Kp = 800;  Ki = 30; Kd = 250; % Aggressive gain to increase distance (brake harder) 

        elseif distance > 22 % Getting close to the upper bound (d > 22m) 

            Kp = 600;  Ki = 10; Kd = 150; % Gain to decrease distance (accelerate) 

        else % Near the desired distance (14m <= d <= 22m) 

            Kp = 500;  Ki = 20; Kd = 100; % Moderate gain 

        end 

 

        % PID controller calculation 

        integral_error = integral_error + error * time_step; 

        derivative_error = (error - previous_error) / time_step; 

        control_force = Kp * error + Ki * integral_error + Kd * derivative_error; 

        % Saturate control force to vehicle limits 

        control_force = max(-max_force, min(control_force, max_force)); 

    end 

 

    % Apply control force to ego vehicle dynamics 

    a_ego = control_force / mass_ego; 

    v_ego = max(0, v_ego + a_ego * time_step); % Update ego speed, ensure non-negative 

    x_ego = x_ego + v_ego * time_step;         % Update ego position 

 

    % Log data for plotting 

    v_ego_log(i) = v_ego; 

    v_lead_log(i) = v_lead; 

    distance_log(i) = distance; 

 

    % Save error for next iteration's derivative calculation 

    previous_error = error; 

 

   % Print simulation state 

    if mod(i-1, round(1/time_step)) == 0  % Print every 1 second 

        fprintf('Time: %6.2f s | Ego Speed: %6.2f m/s | Lead Speed: %6.2f m/s | Distance: %6.2f m\n', ... 

            t, v_ego, v_lead, distance); 

    end 

 

end 

 

% Plot results 

figure; 

 

% Speed Plot 

subplot(2,1,1); 

plot(time_vector, v_ego_log, 'b', 'LineWidth', 1.5); hold on; 

plot(time_vector, v_lead_log, 'r--', 'LineWidth', 1.5); 

xlabel('Time (s)'); ylabel('Speed (m/s)'); 

title('Vehicle Speeds'); legend('Ego Vehicle', 'Lead Vehicle', 'Location', 'best'); grid on; 

 

% Distance Plot 

subplot(2,1,2); 

plot(time_vector, distance_log, 'k', 'LineWidth', 1.5); hold on; 

% Plot boundaries and desired distance 

yline(critical_distance, 'm:', 'LineWidth', 1.5, 'Label', 'Critical Distance (10m)'); 

yline(upper_distance_limit, 'c:', 'LineWidth', 1.5, 'Label', 'Upper Limit (25m)'); 

yline(desired_distance, 'g--', 'LineWidth', 1, 'Label', 'Desired Distance (17.5m)'); 

xlabel('Time (s)'); ylabel('Distance (m)'); 

title('Inter-Vehicle Distance'); 

ylim([0, max(max(distance_log), upper_distance_limit + 5)]); % Adjust y-axis limit 

legend('Actual Distance', 'Location', 'best'); grid on; 

 

hold off; 