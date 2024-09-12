%% Initialize gains for a fixed-wing aircraft
Kp_pitch = 30; Kd_pitch = 5;  % Pitch control gains
Kp_roll = 30; Kd_roll = 5;    % Roll control gains
Kp_yaw = 40; Kd_yaw = 12;     % Yaw control gains
Kp_altitude = 0.08; Kd_altitude = 0.09;  % Altitude control gains
Kp_velocity = 0.08; Kd_velocity = 0.09;  % Velocity control gains

%% Aircraft Parameters
Ixx = 0.1;  % Moment of inertia around X axis
Iyy = 0.1;  % Moment of inertia around Y axis
Izz = 0.2;  % Moment of inertia around Z axis
m = 5;      % Mass of the aircraft in Kg
g = 9.81;   % Gravitational acceleration

%% Initial Values
Xinit = 0; Yinit = 0; Zinit = 0;  % Initial position
Phiinit = 0; Thetainit = 0; Psiinit = 0;  % Initial angles (roll, pitch, yaw)

%% Simulink Block Inputs
% PIDctrl: [pitch_control, roll_control, yaw_control, throttle]
% e: [error_pitch, error_roll, error_yaw, error_altitude, error_velocity]
% edot: [error_dot_pitch, error_dot_roll, error_dot_yaw, error_dot_altitude, error_dot_velocity]

function y = FixedWingController(PIDctrl, e, edot)
    % Unpack inputs
    pitch_control = PIDctrl(1);
    roll_control = PIDctrl(2);
    yaw_control = PIDctrl(3);
    throttle = PIDctrl(4);
    
    error_pitch = e(1);
    error_roll = e(2);
    error_yaw = e(3);
    error_altitude = e(4);
    error_velocity = e(5);
    
    error_dot_pitch = edot(1);
    error_dot_roll = edot(2);
    error_dot_yaw = edot(3);
    error_dot_altitude = edot(4);
    error_dot_velocity = edot(5);
    
    % Compute control efforts
    pitch_effort = Kp_pitch * error_pitch + Kd_pitch * error_dot_pitch + pitch_control;
    roll_effort = Kp_roll * error_roll + Kd_roll * error_dot_roll + roll_control;
    yaw_effort = Kp_yaw * error_yaw + Kd_yaw * error_dot_yaw + yaw_control;
    altitude_effort = Kp_altitude * error_altitude + Kd_altitude * error_dot_altitude;
    velocity_effort = Kp_velocity * error_velocity + Kd_velocity * error_dot_velocity;
    
    % Adjust for dynamic limits
    pitch_effort = max(min(pitch_effort, pi/2), -pi/2);
    roll_effort = max(min(roll_effort, pi/2), -pi/2);
    yaw_effort = max(min(yaw_effort, pi/2), -pi/2);
    
    % Combine efforts into the control output
    y = [pitch_effort; roll_effort; yaw_effort; throttle + altitude_effort + velocity_effort];
end

%% Dynamics block for the fixed-wing aircraft
% Simulink block will use the calculated forces and moments to update the aircraft state

function xdot = AircraftDynamics(t, x, u)
    % x = [x_position; x_velocity; y_position; y_velocity; z_position; z_velocity;
    %      phi; phi_dot; theta; theta_dot; psi; psi_dot];
    
    % u = [Thrust; tau_phi; tau_theta; tau_psi]
    Thrust = u(1);
    tau_phi = u(2);
    tau_theta = u(3);
    tau_psi = u(4);
    
    % Dynamics equations for fixed-wing aircraft
    xdot = zeros(12, 1);
    xdot(1) = x(2);  % Xdot
    xdot(2) = (sin(x(11))*sin(x(7)) + cos(x(11))*sin(x(9))*cos(x(7))) * Thrust / m;  % Xdotdot
    xdot(3) = x(4);  % Ydot
    xdot(4) = (-cos(x(11))*sin(x(7)) + sin(x(11))*sin(x(9))*cos(x(7))) * Thrust / m;  % Ydotdot
    xdot(5) = x(6);  % Zdot
    xdot(6) = -g + (cos(x(9))*cos(x(7))) * Thrust / m;  % Zdotdot
    xdot(7) = x(8);  % Phidot
    xdot(8) = ((Iyy - Izz) / Ixx) * x(10) * x(12) + tau_phi / Ixx;  % Phidotdot
    xdot(9) = x(10);  % Thetadot
    xdot(10) = ((Izz - Ixx) / Iyy) * x(8) * x(12) + tau_theta / Iyy;  % Thetadotdot
    xdot(11) = x(12);  % Psidot
    xdot(12) = ((Ixx - Iyy) / Izz) * x(8) * x(10) + tau_psi / Izz;  % Psidotdot
end