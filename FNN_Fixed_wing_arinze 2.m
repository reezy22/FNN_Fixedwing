%% Clear
clc
clear all
close all

%% Initialize state

kend = 100000;
x(1:kend,1:12) = 0; 
t=0.002;0.001;

%% Initalize gains
% Adjusted gains for a fixed-wing UAV

% Pitch Control (typically slower response due to aerodynamic surfaces)
Kpp = 10;  % Reduced proportional gain
Kdp = 2;   % Reduced derivative gain

% Throttle Control (can remain similar, might depend on specific engine dynamics)
Kpt = 25;  % Slightly reduced proportional gain
Kdt = 4;   % Slightly reduced derivative gain

% Side Slip Control (might need more tuning based on airframe)
Kpps = 20; % Reduced proportional gain
Kdps = 3;  % Reduced derivative gain

% Altitude Control (more stable in fixed-wing, but depends on climb rate)
Kpz = 30;  % Reduced proportional gain
Kdz = 10;  % Reduced derivative gain

% X-Axis Control (related to forward motion control, adjusted for stable flight)
Kpx = 0.05; % Reduced proportional gain
Kdx = 0.06; % Reduced derivative gain

% Y-Axis Control (lateral stability, often more stable in fixed-wing)
Kpy = 0.05; % Reduced proportional gain
Kdy = 0.06; % Reduced derivative gain


%% Initalize Constants
% Parameters for N9258 C-182 Fixed-Wing UAV

Ixx = 0.18;       % Moment of inertia around X-axis (kg*m^2)
Iyy = 0.26;       % Moment of inertia around Y-axis (kg*m^2)
Izz = 0.35;       % Moment of inertia around Z-axis (kg*m^2)
Jtp = 0;          % Total rotational moment of inertia around the propeller axis (typically negligible for fixed-wing)
b = 0;            % Thrust factor (not applicable as defined for quadcopter)
d = 0;            % Drag factor (not applicable as defined for quadcopter)
l = 1.5;          % Distance to the center of mass from control surfaces (e.g., from ailerons to center of mass)
m = 1.2;          % Mass of the N9258 C-182 in Kg (example value, adjust based on actual aircraft)
g = 9.81;         % Gravitational acceleration (m/s^2)

%% %% Initial values

Xinit = 0;            % Initial X position (m)
Yinit = 0;            % Initial Y position (m)
Zinit = 0;            % Initial Z position (m)
Phiinit = 0;          % Initial roll angle (rad)
Thetainit = 0;        % Initial pitch angle (rad)
Psiinit = 0;          % Initial yaw angle (rad)

%% Desired Trajectories

% Time vector
time_vector = t * (1:kend);

% Desired X and Y positions (circular path)
radius = 2;  % Radius of the circular path
Xd = radius * cos(time_vector);  % X position (m)
Yd = radius * sin(time_vector);  % Y position (m)

% Desired Z position (constant altitude)
Zd = 10 * ones(1, kend);  % Altitude (m)

% Desired Yaw angle (keeping it constant for simplicity)
Psid = 0 * ones(1, kend);  % Yaw angle (rad)


%% High-Level Fuzzy NN parameters

FNN_HIGH = 1;  % Flag to indicate the use of high-level FNN

% Centers of Gaussian membership functions
c_high = permute(repmat([-10 0 10; -10 0 10], [1 1 3]), [3 1 2]);

% Standard deviations of Gaussian membership functions
sigma1_high = permute(repmat([6 6 6; 6 6 6], [1 1 3]), [3 1 2]);
sigma2_high = permute(repmat([4 4 4; 4 4 4], [1 1 3]), [3 1 2]);

% Initial weights for the fuzzy rules, small random values
a_high = (rand(3, 9, 2) - 0.5) / 100;
b_high = (rand(3, 9) - 0.5) / 100;

% Learning rate or adaptation parameter
q = 0.5;

% Additional parameters for the fuzzy neural network
alpha_high = repmat(3, [3, 1]);  % Possibly learning rate or scaling factor
sigma_high = repmat(0.9, [3, 1]);  % Spread or standard deviation
nu_high = repmat(0.1, [3, 1]);  % Adaptation or convergence parameter


%% Low-Level Fuzzy NN parameters for Fixed-Wing UAV

FNN_LOW = 1;  % Flag to indicate the use of low-level FNN

% Centers of Gaussian membership functions
% Reflecting typical ranges for roll, pitch, and control inputs
c_low = permute(repmat([-pi/6 0 pi/6; -0.5 0 0.5], [1 1 3]), [3 1 2]);

% Standard deviations of Gaussian membership functions
% sigma1_low = permute(repmat([0.5 0.5 0.5; 0.3 0.3 0.3], [1 1 3]), [3 1 2]);%proposed values 
% sigma2_low = permute(repmat([0.3 0.3 0.3; 0.2 0.2 0.2], [1 1 3]), [3 1 2]);%proposed values 
sigma1_low = permute(repmat([0.9 0.9 0.9; 0.6 0.6 0.6], [1 1 3]), [3 1 2]);
sigma2_low = permute(repmat([0.6 0.6 0.6; 0.4 0.4 0.4], [1 1 3]), [3 1 2]);

% Initial weights for the fuzzy rules, small random values
a_low = (rand(3, 9, 2) - 0.5) / 200;  % Smaller random initialization
b_low = (rand(3, 9) - 0.5) / 200;    % Smaller random initialization

% Learning rate or adaptation parameter
alpha_low = repmat(0.3, [3, 1]);  % Learning rate or scaling factor
sigma_low = repmat(0.1, [3, 1]);  % Spread or standard deviation
nu_low = repmat(0.1, [3, 1]);  % Adaptation or convergence parameter

%% Control loop for Fixed-Wing UAV

% Initial state vector for position, velocity, and orientation
x(1,1) = Xinit;
x(1,2) = 0;  % Initial X velocity
x(1,3) = Yinit;
x(1,4) = 0;  % Initial Y velocity
x(1,5) = Zinit;
x(1,6) = 0;  % Initial Z velocity
x(1,7) = Phiinit;
x(1,8) = 0;  % Initial roll rate
x(1,9) = Thetainit;
x(1,10) = 0;  % Initial pitch rate
x(1,11) = Psiinit;
x(1,12) = 0;  % Initial yaw rate

% Initialize control and FNN parameters
Y_high = [0 0 0];  % High-level FNN output
Y_low = [0 0 0];   % Low-level FNN output
Attituded = [0 0 0];  % Desired attitude angles
Alpha_high = [alpha_high'];
A_high = [reshape(a_high, [1,54])];
B_high = [reshape(b_high, [1,27])];
Alpha_low = [alpha_low'];

% Control loop execution
% for k = 1:kend-1
% %% Compute position errors
%     ex = cos(x(k,11)) * (Xd(k) - x(k,1)) - sin(x(k,11)) * (Yd(k) - x(k,3));
%     edx = cos(x(k,11)) * (- x(k,2)) - sin(x(k,11)) * (- x(k,4));
% 
%     ey = -sin(x(k,11)) * (Xd(k) - x(k,1)) - cos(x(k,11)) * (Yd(k) - x(k,3));
%     edy = -sin(x(k,11)) * (- x(k,2)) - cos(x(k,11)) * (- x(k,4));
% 
%     ez = Zd(k) - x(k,5);
%     edz = - x(k,6);
% 
%     error_high = [ex edx; ey edy; ez edz];
%     error_high(:, 1) = max(min(error_high(:, 1), 10), -10);
%     error_high(:, 2) = max(min(error_high(:, 2), 10), -10);
% 
%     %% Position controller
% 
%     Thetad = Kpx * ex + Kdx * edx; % Desired pitch angle
%     Phid = Kpy * ey + Kdy * edy; % Desired roll angle
%     Thrust = m*(g + Kpz * ez + Kdz * edz)/(cos(x(k,9))*cos(x(k,7)));   % Total thrust along the z-axis
% 
%     %% High-Level: Fuzification
% 
%     mfU = exp(-(repmat(error_high, [1 1 3]) - c_high).^2./(2*sigma1_high.^2));
%     mfL = exp(-(repmat(error_high, [1 1 3]) - c_high).^2./(2*sigma2_high.^2));
% 
%     %% High-Level: Firing levels
% 
%     wU1 = mfU(:,1,:) .* permute(mfU(:,2,:), [1 3 2]);
%     wU = wU1(:,:);
% 
%     wL1 = mfL(:,1,:) .* permute(mfL(:,2,:), [1 3 2]);
%     wL = wL1(:,:);
% 
%     sumW = sum(wU,2);
%     wUtilde(:,:) = wU(:,:) ./ sumW(:);
% 
%     sumW = sum(wL,2);
%     wLtilde(:,:) = wL(:,:) ./ sumW(:);
% 
%     %% High-Level: Compute the output
% 
%     for i = 1:3
%         f = permute(a_high(i,:,:), [2 3 1]) * error_high(i,:)'; + b_high(i,:)';
%         y(i) = q * wUtilde(i,:) * f + (1 - q) * wLtilde(i,:) * f;
%     end
% 
%     if (FNN_HIGH == 0)
%         y = [0 0 0];
%     end
% 
%     Y_high = [Y_high; y];
% 
%     %% High-Level: Update the parameters
% 
%     F1 = q * wLtilde + (1 - q) * wUtilde;
%     F = F1 ./ F1 .* F1;
%     a_high = a_high - permute(repmat(error_high, [1 1 9]), [1 3 2]) .* repmat(F .* alpha_high .* sign([Thetad; Phid; Thrust]), [1 1 2]) * t;
%     b_high = b_high - F .* alpha_high .* sign([Thetad; Phid; Thrust]) * t;
%     alpha_high = alpha_high + (4 * sigma_high .* abs([Thetad; Phid; Thrust]) - nu_high .* sigma_high .* alpha_high) * t;
% 
%     %% Attitude references
% 
%     Thetad = Thetad - y(1);
%     Phid = Phid - y(2);
%     Thrust = Thrust - y(3);
% 
%     Thetad = max([min([Thetad pi/2]) -pi/2]); % Limit pitch angle
%     Phid = max([min([Phid pi/2]) -pi/2]); % Limit roll angle
% 
%     Attituded = [Attituded; Thetad Phid Thrust]; % Update desired attitudes
% 
%     %% Compute attitude errors
% 
%     ephi = Phid - x(k,7);
%     edphi = - x(k,8);
% 
%     etheta = Thetad - x(k,9);
%     edtheta = - x(k,10);
% 
%     epsi = Psid(k) - x(k,11);
%     edpsi = - x(k,12);
% 
%     error_low = [ephi edphi; etheta edtheta; epsi edpsi];
%     error_low(:, 1) = max(min(error_low(:, 1), pi/2), -pi/2);
%     error_low(:, 2) = max(min(error_low(:, 2), 1), -1);
% 
%     %% Attitude controller
% 
%     tau_phi = Kpp * ephi + Kdp * edphi; % Roll control input
%     tau_theta = Kpt * etheta + Kdt * edtheta; % Pitch control input
%     tau_psi = Kpps * epsi + Kdps * edpsi; % Yaw control input
% 
%     %% Low-Level: Fuzification
% 
%     mfU = exp(-(repmat(error_low, [1 1 3]) - c_low).^2./(2*sigma1_low.^2));
%     mfL = exp(-(repmat(error_low, [1 1 3]) - c_low).^2./(2*sigma2_low.^2));
% 
%     %% Low-Level: Firing levels
% 
%     wU1 = mfU(:,1,:) .* permute(mfU(:,2,:), [1 3 2]);
%     wU = wU1(:,:);
% 
%     wL1 = mfL(:,1,:) .* permute(mfL(:,2,:), [1 3 2]);
%     wL = wL1(:,:);
% 
%     sumW = sum(wU,2);
%     wUtilde(:,:) = wU(:,:) ./ sumW(:);
% 
%     sumW = sum(wL,2);
%     wLtilde(:,:) = wL(:,:) ./ sumW(:);
% 
%     %% Low-Level: Compute the output
% 
%     for i = 1:3
%         f = permute(a_low(i,:,:), [2 3 1]) * error_low(i,:)'; + b_low(i,:)';
%         y(i) = q * wUtilde(i,:) * f + (1 - q) * wLtilde(i,:) * f;
%     end
% 
%     if (FNN_LOW == 0)
%         y = [0 0 0];
%     end
% 
%     Y_low = [Y_low; y];
% 
%     %% Low-Level: Update the parameters
% 
%     F1 = q * wLtilde + (1 - q) * wUtilde;
%     F = F1 ./ F1 .* F1;
%     a_low = a_low - permute(repmat(error_low, [1 1 9]), [1 3 2]) .* repmat(F .* alpha_low .* sign([tau_phi; tau_theta; tau_psi]), [1 1 2]) * t;
%     b_low = b_low - F .* alpha_low .* sign([tau_phi; tau_theta; tau_psi]) * t;
%     alpha_low = alpha_low + (4 * sigma_low .* abs([tau_phi; tau_theta; tau_psi]) - nu_low .* sigma_low .* alpha_low) * t;
% 
%     %% Control inputs
% 
%     tau_phi = tau_phi - y(1);
%     tau_theta = tau_theta - y(2);
%     tau_psi = tau_psi - y(3);
% 
%     % Control inputs for fixed-wing UAV
%     Aileron = tau_phi;   % Aileron control for roll
%     Elevator = tau_theta; % Elevator control for pitch
%     Rudder = tau_psi;    % Rudder control for yaw
%     Throttle = Thrust;   % Throttle control for thrust
% 
%     U = [Throttle Aileron Elevator Rudder];
% 
%     %% System dynamics for fixed-wing UAV
%     %% System dynamics for Fixed-Wing UAV
% 
%     % Parameters
%     rho = 1.225; % Air density (kg/m^3)
%     S = 30; % Wing area (m^2)
%     b = 15; % Wing span (m)
%     c = 2; % Mean aerodynamic chord (m)
%     V = sqrt(x(k,2)^2 + x(k,4)^2); % Airspeed (m/s)
% 
%     % Aerodynamic coefficients
%     Cl_alpha = 2 * pi; % Lift coefficient slope (rad^-1)
%     Cd_0 = 0.02; % Zero-lift drag coefficient
%     Cd_alpha = 0.1; % Drag coefficient slope (rad^-1)
%     Cl_0 = 0.2; % Zero-lift coefficient
% 
%     % Compute aerodynamic forces and moments
%     q = 0.5 * rho * V^2; % Dynamic pressure
% 
%     % Lift and Drag forces
%     L = q * S * (Cl_0 + Cl_alpha * x(k,9)); % Lift force
%     D = q * S * (Cd_0 + Cd_alpha * x(k,9)); % Drag force
% 
%     % Moments
%     Cl_delta = 0.1; % Change in lift coefficient per degree of control surface deflection
%     Cm_0 = 0.0; % Zero-lift pitching moment coefficient
%     Cm_delta = -0.1; % Pitching moment coefficient per degree of control surface deflection
% 
%     M = q * S * c * (Cm_0 + Cm_delta * U(3)); % Pitching moment
% 
%     % System dynamics
%     xdot(1) = x(k,2); % Xdot (Velocity in x-direction)
%     xdot(2) = (L * sin(x(k,7)) - D * cos(x(k,7))) / m; % Xddot (Acceleration in x-direction)
% 
%     xdot(3) = x(k,4); % Ydot (Velocity in y-direction)
%     xdot(4) = (L * cos(x(k,7)) - D * sin(x(k,7))) / m; % Yddot (Acceleration in y-direction)
% 
%     xdot(5) = x(k,6); % Zdot (Vertical velocity)
%     xdot(6) = -g + (L * sin(x(k,9)) * cos(x(k,7)) - D * cos(x(k,9)) * sin(x(k,7))) / m; % Zddot (Acceleration in z-direction)
% 
%     xdot(7) = x(k,8); % p (Roll rate)
%     xdot(8) = ((Iyy - Izz) * x(k,10) * x(k,12) - (Jtp / Ixx) * x(k,10) + U(1) / Ixx); % p-dot (Roll acceleration)
% 
%     xdot(9) = x(k,10); % q (Pitch rate)
%     xdot(10) = ((Izz - Ixx) * x(k,8) * x(k,12) + (Jtp / Iyy) * x(k,8) + (U(2) - M) / Iyy); % q-dot (Pitch acceleration)
% 
%     xdot(11) = x(k,12); % r (Yaw rate)
%     xdot(12) = ((Ixx - Iyy) * x(k,8) * x(k,10) + U(3) / Izz); % r-dot (Yaw acceleration)
% 
%     % Update state
%     x(k + 1,:) = x(k,:) + t * xdot;
% 
%     %sim ()
% end

for k=1:kend-1
    
    %% Compute position errors
    
    ex = cos(x(k,11)) * (Xd(k) - x(k,1)) - sin(x(k,11)) * (Yd(k) - x(k,3));
    edx = cos(x(k,11)) * (- x(k,2)) - sin(x(k,11)) * (- x(k,4));
    
    ey = -sin(x(k,11)) * (Xd(k) - x(k,1)) - cos(x(k,11)) * (Yd(k) - x(k,3));
    edy = -sin(x(k,11)) * (- x(k,2)) - cos(x(k,11)) * (- x(k,4));
    
    ez = Zd(k) - x(k,5);
    edz = - x(k,6);
    
    error_high = [ex edx; ey edy; ez edz];
    error_high(:, 1) = max(min(error_high(:, 1), 10), -10);
    error_high(:, 2) = max(min(error_high(:, 2), 10), -10);
    
    %% Position controller
    
    Thetad = Kpx * ex + Kdx * edx; % 
    Phid = Kpy * ey + Kdy * edy; % 
    Thrust = m*(g + Kpz * ez + Kdz * edz)/(cos(x(k,9))*cos(x(k,7)));   % Total Thrust on the body along z-axis
       
    %% High-Level: Fuzification
    
    mfU = exp(-(repmat(error_high, [1 1 3]) - c_high).^2./(2*sigma1_high.^2));
    mfL = exp(-(repmat(error_high, [1 1 3]) - c_high).^2./(2*sigma2_high.^2));
    
    %% High-Level: Firing levels
    
    wU1 = mfU(:,1,:) .* permute(mfU(:,2,:), [1 3 2]);
    wU = wU1(:,:);
    
    wL1 = mfL(:,1,:) .* permute(mfL(:,2,:), [1 3 2]);
    wL = wL1(:,:);
    
    sumW = sum(wU,2);
    wUtilde(:,:) = wU(:,:) ./ sumW(:);
    
    sumW = sum(wL,2);
    wLtilde(:,:) = wL(:,:) ./ sumW(:);
    
    %% High-Level: Compute the output
    
    for i = 1:3
        f = permute(a_high(i,:,:), [2 3 1]) * error_high(i,:)'; + b_high(i,:)';
        y(i) = q * wUtilde(i,:) * f + (1 - q) * wLtilde(i,:) * f;
    end
    
    if(FNN_HIGH == 0)
        y = [0 0 0];
    end
    
    Y_high = [Y_high; y];
    
    %% High-Level: Update the parameters
    
    F1 = q * wLtilde + (1 - q) * wUtilde;
    F = F1 ./ F1 .* F1;
    a_high = a_high - permute(repmat(error_high, [1 1 9]), [1 3 2]) .* repmat(F .* alpha_high .* sign([Thetad; Phid; Thrust]), [1 1 2]) * t;
    b_high = b_high - F .* alpha_high .* sign([Thetad; Phid; Thrust]) * t;
    alpha_high = alpha_high + (4 * sigma_high .* abs([Thetad; Phid; Thrust]) - nu_high .* sigma_high .* alpha_high) * t;
    
%     A_high = [A_high; reshape(a_high, [1,54])];
%     B_high = [B_high; reshape(b_high, [1,27])];
%     Alpha_high = [Alpha_high; alpha_high'];
    
    %% Attitude references
     
    Thetad = Thetad - y(1);
    Phid = Phid - y(2);
    Thrust = Thrust - y(3);
    
    Thetad = max([min([Thetad pi/2]) -pi/2]); %
    Phid = max([min([Phid pi/2]) -pi/2]); %
    
    Attituded = [Attituded; Thetad Phid Thrust];%burayi da anlamadim.
    
    %% Compute attitude errors
    
    ephi = Phid - x(k,7);
    edphi = - x(k,8);
    
    etheta = Thetad - x(k,9);
    edtheta = - x(k,10);
    
    epsi = Psid(k) - x(k,11);
    edpsi = - x(k,12);
    
    error_low = [ephi edphi; etheta edtheta; epsi edpsi];
    error_low(:, 1) = max(min(error_low(:, 1), pi/2), -pi/2);
    error_low(:, 2) = max(min(error_low(:, 2), 1), -1);
    
    %% Attitude controller
    
    tau_phi = Kpp * ephi + Kdp * edphi; % Roll input
    tau_theta = Kpt * etheta + Kdt * edtheta; % Pitch input
    tau_psi = Kpps * epsi + Kdps * edpsi; % Yawing moment
    
    %% Low-Level: Fuzification
    
    mfU = exp(-(repmat(error_low, [1 1 3]) - c_low).^2./(2*sigma1_low.^2));
    mfL = exp(-(repmat(error_low, [1 1 3]) - c_low).^2./(2*sigma2_low.^2));
    
    %% Low-Level: Firing levels
    
    wU1 = mfU(:,1,:) .* permute(mfU(:,2,:), [1 3 2]);
    wU = wU1(:,:);
    
    wL1 = mfL(:,1,:) .* permute(mfL(:,2,:), [1 3 2]);
    wL = wL1(:,:);
    
    sumW = sum(wU,2);
    wUtilde(:,:) = wU(:,:) ./ sumW(:);
    
    sumW = sum(wL,2);
    wLtilde(:,:) = wL(:,:) ./ sumW(:);
    
    %% Low-Level: Compute the output
    
    for i = 1:3
        f = permute(a_low(i,:,:), [2 3 1]) * error_low(i,:)'; + b_low(i,:)';
        y(i) = q * wUtilde(i,:) * f + (1 - q) * wLtilde(i,:) * f;
    end
    
    if(FNN_LOW == 0)
        y = [0 0 0];
    end
    
    Y_low = [Y_low; y];
    
    %% Low-Level: Update the parameters
    
    F1 = q * wLtilde + (1 - q) * wUtilde;
    F = F1 ./ F1 .* F1;
    a_low = a_low - permute(repmat(error_low, [1 1 9]), [1 3 2]) .* repmat(F .* alpha_low .* sign([tau_phi; tau_theta; tau_psi]), [1 1 2]) * t;
    b_low = b_low - F .* alpha_low .* sign([tau_phi; tau_theta; tau_psi]) * t;
    alpha_low = alpha_low + (4 * sigma_low .* abs([tau_phi; tau_theta; tau_psi]) - nu_low .* sigma_low .* alpha_low) * t;
    
%     Alpha_low = [Alpha_low; alpha_low'];
    
    %% Control inputs
    
    tau_phi = tau_phi - y(1);
    tau_theta = tau_theta - y(2);
    tau_psi = tau_psi - y(3);
    
    U = [Thrust tau_phi tau_theta tau_psi];
    
    %% System dynamics
    %%should be different 
    
    xdot(1) = x(k,2); % Xdot
    xdot(2) = (sin(x(k,11))*sin(x(k,7)) + cos(x(k,11))*sin(x(k,9))*cos(x(k,7)))*(U(1)/m);    % Xdotdot
    xdot(3) = x(k,4); % Ydot
    xdot(4) = (-cos(x(k,11))*sin(x(k,7)) + sin(x(k,11))*sin(x(k,9))*cos(x(k,7)))*(U(1)/m);	% Ydotdot
    xdot(5) = x(k,6); % Zdot
    xdot(6) = - g + (cos(x(k,9))*cos(x(k,7)))*(U(1)/m);    % Zdotdot
    xdot(7) = x(k,8); % phydot
    xdot(8) = ((Iyy - Izz)/Ixx)*x(k,10)*x(k,12) - (Jtp/Ixx)*x(k,10) + (U(2)/Ixx); % pdot = phydotdot
    xdot(9) = x(k,10);    % thetadot
    xdot(10) = ((Izz - Ixx)/Iyy)*x(k,8)*x(k,12) + (Jtp/Iyy)*x(k,8) + (U(3)/Iyy);	% qdot = thetadotdot
    xdot(11) = x(k,12);   % thetadot
    xdot(12) = ((Ixx - Iyy)/Izz)*x(k,8)*x(k,10) + (U(4)/Izz);	% rdot = psidotdot
    
    x(k + 1,:) = x(k,:) + t * xdot;
    
    %sim ()
end

%% Plot x tracking
figure('Name', 'x Tracking', 'NumberTitle', 'off');
plot(0:(kend-1), Xd, 'b', 'LineWidth', 2); % Desired X trajectory
hold on;
plot(0:(kend-1), x(:, 1), 'r--', 'LineWidth', 2); % Actual X trajectory
xlabel('Time (s)');
ylabel('X Position (m)');
title('X Position Tracking');
legend('Desired', 'Actual');
grid on;

%% Plot y tracking
figure('Name', 'y Tracking', 'NumberTitle', 'off');
plot(0:(kend-1), Yd, 'b', 'LineWidth', 2); % Desired Y trajectory
hold on;
plot(0:(kend-1), x(:, 3), 'r--', 'LineWidth', 2); % Actual Y trajectory
xlabel('Time (s)');
ylabel('Y Position (m)');
title('Y Position Tracking');
legend('Desired', 'Actual');
grid on;

%% Plot z tracking
figure('Name', 'z Tracking', 'NumberTitle', 'off');
plot(0:(kend-1), Zd, 'b', 'LineWidth', 2); % Desired Z trajectory
hold on;
plot(0:(kend-1), x(:, 5), 'r--', 'LineWidth', 2); % Actual Z trajectory
xlabel('Time (s)');
ylabel('Z Position (m)');
title('Z Position Tracking');
legend('Desired', 'Actual');
grid on;

%% 3D Tracking Plot
figure('Name', '3D Tracking', 'NumberTitle', 'off');
plot3(Xd, Yd, Zd, 'b', 'LineWidth', 2); % Desired trajectory
hold on;
plot3(x(:, 1), x(:, 3), x(:, 5), 'r--', 'LineWidth', 2); % Actual trajectory
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('3D Position Tracking');
legend('Desired', 'Actual');
grid on;

%% Roll Tracking
figure('Name', 'Roll Tracking', 'NumberTitle', 'off');
plot(0:(kend-1), Attituded(:, 1) / pi * 180, 'b', 'LineWidth', 2); % Desired Roll angle
hold on;
plot(0:(kend-1), x(:, 7) / pi * 180, 'r--', 'LineWidth', 2); % Actual Roll angle
xlabel('Time (s)');
ylabel('Roll Angle (degrees)');
title('Roll Angle Tracking');
legend('Desired', 'Actual');
grid on;

%% Pitch Tracking
figure('Name', 'Pitch Tracking', 'NumberTitle', 'off');
plot(0:(kend-1), Attituded(:, 2) / pi * 180, 'b', 'LineWidth', 2); % Desired Pitch angle
hold on;
plot(0:(kend-1), x(:, 9) / pi * 180, 'r--', 'LineWidth', 2); % Actual Pitch angle
xlabel('Time (s)');
ylabel('Pitch Angle (degrees)');
title('Pitch Angle Tracking');
legend('Desired', 'Actual');
grid on;

%% Yaw Tracking
figure('Name', 'Yaw Tracking', 'NumberTitle', 'off');
plot(0:(kend-1), Psid / pi * 180, 'b', 'LineWidth', 2); % Desired Yaw angle
hold on;
plot(0:(kend-1), x(:, 11) / pi * 180, 'r--', 'LineWidth', 2); % Actual Yaw angle
xlabel('Time (s)');
ylabel('Yaw Angle (degrees)');
title('Yaw Angle Tracking');
legend('Desired', 'Actual');
grid on;

%% Calculate and display performance metrics
error = sqrt((Xd - x(:,1)').^2 + (Yd - x(:,3)').^2 + (Zd - x(:,5)').^2);
mae = mean(error); % Mean Absolute Error
mse = mean(error.^2); % Mean Squared Error
rmse = sqrt(mse); % Root Mean Squared Error

fprintf('Performance Metrics:\n');
fprintf('Mean Absolute Error (MAE): %.4f\n', mae);
fprintf('Mean Squared Error (MSE): %.4f\n', mse);
fprintf('Root Mean Squared Error (RMSE): %.4f\n', rmse);

