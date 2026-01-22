% State-Space Model Simulation 

% Vehicle and controller parameters 

v_ego_init = 15; % m/s (Initial speed) 

v_lead_init = 15; % m/s (Initial speed) 

x_ego_init = 0;   % Initial position 

x_lead_init = 20; % Initial position 

desired_distance = 17.5; % m (Target distance) 

critical_distance = 10;  % m (minimum distance) 

upper_distance_limit = 25; % m (Target upper boundary) 

max_force = 8000;      % N (Max acceleration/braking force) 

mass_ego = 1500;       % kg 

 

% Simulation setup 

time_step = 0.1; 

simulation_time = 20; 

time_vector = 0:time_step:simulation_time; 

num_steps = length(time_vector); 

 

% State vector initialization 

x = zeros(5, num_steps); 

x(:, 1) = [x_ego_init; v_ego_init; x_lead_init; v_lead_init; 0]; % Initial state 

 

% Data Logging  

distance_log = zeros(1, num_steps); 

v_ego_log = zeros(1, num_steps); 

v_lead_log = zeros(1, num_steps); 

a_ego_log = zeros(1, num_steps);  

 

% Assign initial logged values 

v_ego_log(1) = x(2, 1); 

v_lead_log(1) = x(4, 1); 

distance_log(1) = x(3, 1) - x(1, 1); % Initial distance 

 

% --- Simulation Loop --- 

for i = 1:num_steps - 1 

    % Extract current state 

    current_x = x(:, i); 

    current_x_ego = current_x(1); 

    current_v_ego = current_x(2); 

    current_x_lead = current_x(3); 

    current_v_lead = current_x(4); 

    current_integral_error = current_x(5); 

 

    % --- Input (Lead vehicle dynamics) --- 

    % Random fluctuation in lead vehicle's acceleration  

    a_lead = randn() * 0.3 + randn(); % random noise to create Speed Deviations 

 

    % --- Calculate Control (Ego vehicle acceleration, a_ego) --- 

    % These calculations are based on the current state variables 

 

    % Compute distance and relative velocity 

    distance = current_x_lead - current_x_ego; 

    % Ensure non-negative distance for control logic 

    distance_for_control = max(0, distance); 

 

    relative_velocity = current_v_lead - current_v_ego; 

 

    % PID error relative to the desired distance 

    error = distance_for_control - desired_distance; 

 

    % Adaptive PID: Adjust gains based on current distance 

    if distance_for_control < critical_distance 

        % Emergency Override: Apply full braking if below critical distance 

        control_force = -max_force; 

        Kp = 0; Ki = 0; Kd = 0;  

    else 

        % Adaptive Gain Values 

        if distance_for_control < 14 % Getting close to the lower bound (10m < d < 14m) 

            Kp = 800;  Ki = 30; Kd = 250; % Aggressive gain to increase distance (brake harder) 

        elseif distance_for_control > 22 % Getting close to the upper bound (d > 22m) 

            Kp = 600;  Ki = 10; Kd = 200; % Gain to decrease distance (accelerate) 

        else % Near the desired distance (14m <= d <= 22m) 

            Kp = 250;  Ki = 20; Kd = 180; % Moderate gain (Cruising) 

        end 

 

        % PID controller calculation 

        % Note: Derivative term uses current relative velocity, which is d_dot 

        intended_control_force = Kp * error + Ki * current_integral_error + Kd * relative_velocity; 

 

        % Change control force to be within vehicle limits 

        control_force = max(-max_force, min(intended_control_force, max_force)); 

    end 

 

    % Calculate ego acceleration from control force 

    a_ego = control_force / mass_ego; 

 

    % --- Calculate State Derivatives (dx/dt) --- 

    % dx/dt = f(x, u) 

    dxdt = zeros(5, 1); 

    dxdt(1) = current_v_ego;                                % dx_ego/dt = v_ego 

    dxdt(2) = a_ego;                                        % dv_ego/dt = a_ego 

    dxdt(3) = current_v_lead;                               % dx_lead/dt = v_lead 

    dxdt(4) = a_lead;                                       % dv_lead/dt = a_lead (input) 

    dxdt(5) = distance - desired_distance;                  % d(integral_error)/dt = error (x_lead - x_ego - d_des) 

 

    % --- State Update (Euler Integration) --- 

    % x(k+1) = x(k) + dx/dt(k) * time_step 

    next_x = current_x + dxdt * time_step; 

 

    % --- Apply Non-negative Constraints --- 

    % These are applied after the state update 

    next_x(2) = max(0, next_x(2)); % Ensure v_ego is non-negative 

    next_x(4) = max(0, next_x(4)); % Ensure v_lead is non-negative 

   

    % Store next state 

    x(:, i+1) = next_x; 

 

    % --- Log Data --- 

    v_ego_log(i+1) = x(2, i+1); 

    v_lead_log(i+1) = x(4, i+1); 

    distance_log(i+1) = x(3, i+1) - x(1, i+1); 

    a_ego_log(i+1) = a_ego; % Log the calculated ego acceleration 

 

    % --- Print Simulation State --- 

    if mod(i, round(1/time_step)) == 0 % Print every 1 second 

         fprintf('Time: %6.2f s | Ego Speed: %6.2f m/s | Lead Speed: %6.2f m/s | Distance: %6.2f m | Ego Accel: %6.2f m/s^2\n', ... 

             time_vector(i+1), v_ego_log(i+1), v_lead_log(i+1), distance_log(i+1), a_ego_log(i+1)); 

    end 

end 

 

% --- Plot Results --- 

figure; 

 

% Speed Plot 

subplot(2,1,1); 

plot(time_vector, v_ego_log, 'b', 'LineWidth', 1.5); hold on; 

plot(time_vector, v_lead_log, 'r', 'LineWidth', 1.5); 

xlabel('Time (s)'); ylabel('Speed (m/s)'); 

title('Vehicle Velocity'); 

legend('Ego Vehicle', 'Lead Vehicle', 'Location', 'best'); grid on; 

 

% Distance Plot 

subplot(2,1,2); 

plot(time_vector, distance_log, 'k', 'LineWidth', 1.5); hold on; 

% Plot boundaries and desired distance 

yline(critical_distance, 'm:', 'LineWidth', 1.5, 'Label', 'Critical Distance (10m)'); 

yline(upper_distance_limit, 'c:', 'LineWidth', 1.5, 'Label', 'Upper Limit (25m)'); 

yline(desired_distance, 'g--', 'LineWidth', 1, 'Label', 'Desired Distance (17.5m)'); 

xlabel('Time (s)'); ylabel('Distance (m)'); 

title('Inter-Vehicle Distance'); 

ylim([min(0, min(distance_log)), max(max(distance_log), upper_distance_limit + 5)]); % Adjust y-axis limit 

legend('Actual Distance', 'Location', 'best'); grid on; 

hold off; 

 