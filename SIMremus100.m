function SIMremus100(scenario)
% Dependencies:rk
%   remus100.m            - Dynamics of the Remus 100 AUV
%   integralSMCheading.m  - Integral sliding mode control for heading
%   refModel.m            - Reference model for autopilot systems
%   ALOS.m                - ALOS guidance algorithm for path following
%   LOSobserver.m         - Observer for LOS guidance 
%   lowPassFilowlter.m       - Low-pass filter
%   q2euler.m, ssa.m      - Utilities for quaternion to Euler conversion 
%                           and angle wrapping
%   displayVehicleData    - Display the vehicle data and an image of the
%                           vehicle
% Simulink Models:
%   demoAUVdepthHeadingControl.slx : Depth and heading control.
%
% References:
%   E. M. Coates and T. I. Fossen (2025). Aspherical Amplitude–Phase Formulation 
%       for 3-D Adaptive Line-of-Sight (ALOS) Guidance with USGES Stability
%       Guarantees, Automatica, Submitted. 
%   T. I. Fossen & P. Aguiar (2024). A Uniform Semiglobal Exponential  
%      Stable Adaptive Line-of-Sight (ALOS) Guidance Law for 3-D Path 
%      Following. Automatica, 163, 111556. 
%      https://doi.org/10.1016/j.automatica.2024.111556

% Author: Thor I. Fossen
% Date: 2021-06-28

if nargin < 1
    scenario = 2;
end

fprintf('------------------- Scenario: %.0f -------------------\n',scenario)

%% SIMULATOR CONFIGURATION ================================================================================
h = 0.05;                           % Sampling time (s)
T_final = 680;	                    % Final simulation time (s)

ControlFlag = 3;
KinematicsFlag = 1;

% Define waypoints for 3D path following
wpt.pos.x = [0 -10 -50 0 100 100 200 350];
wpt.pos.y = [0 100 300 475 650 900 1100 1300];
wpt.pos.z = [0 5 50 50 25 25 5 5];

wpt_ori = wpt;
% wpt.pos.x = [0 200 -100 0 200];
% wpt.pos.y = [0 400 600 900 1300];
% wpt.pos.z = [0 10 100 100 50];

% Obstacle
obs.pos = {[-25;200;30], [100;800;0]};
obs.r = {30,40};

% Initialize position and orientation
xn = 0; yn = 0; zn = 0;             % Initial North-East-Down positions (m)
phi = 0; theta = 0;                 % Initial Euler angles (radians)
psi = atan2(wpt.pos.y(2) - wpt.pos.y(1), ...
    wpt.pos.x(2) - wpt.pos.x(1));   % Yaw angle towards next waypoint
U = 1;                              % Initial speed (m/s)

% Initial control and state setup
theta_d = 0; q_d = 0;               % Initial pitch references
psi_d = psi; r_d = 0; a_d = 0;      % Initial yaw references
xf_z_d = zn;                        % Initial low-pass filter state

% State vector initialization
if KinematicsFlag == 1  % Euler angles
    x = [U; zeros(5,1); xn; yn; zn; phi; theta; psi];
else % Unit quaternions
    quat = euler2q(phi, theta, psi);
    x = [U; zeros(5,1); xn; yn; zn; quat];
end
etadot = [0 0 0];

% Initialize propeller dynamics
n_max = 1525;                  % Maximum propeller speed (RPM)
n_rate = 0.1;                  % Rate limit (RPM per update)
n = 1000;                      % Initial propeller speed (RPM)
n_d = 1300;                    % Desired propeller speed (RPM)

% Time vector initialization
t = 0:h:T_final;                % Time vector from 0 to T_final          
nTimeSteps = length(t);         % Number of time steps

%% CONTROL SYSTEM CONFIGURATION
delta_max = deg2rad(20);       % Maximum rudder and stern angle (rad)

% Setup for depth and heading control
z_max = 100;                   % Maximum depth (m)
psi_step = deg2rad(-60);       % Step change in heading angle (rad)
z_step = 30;                   % Step change in depth, max 100 m

% Integral states for autopilots
z_int = 0;                     % Integral state for depth control
theta_int = 0;                 % Integral state for pitch control
psi_int = 0;                   % Integral state for yaw control

% Depth controller (suceessive-loop closure)
z_d = zn;                      % Initial depth target (m)
wn_d_z = 0.02;                 % Natural frequency for depth control
Kp_z = 0.1;                    % Proportional gain for depth
T_z = 100;                     % Time constant for integral action in depth control
Kp_theta = 5.0;                % Proportional gain for pitch control
Kd_theta = 2.0;                % Derivative gain for pitch control
Ki_theta = 0.3;                % Integral gain for pitch control

% Heading control parameters (using Nomoto model)
K_yaw = 5 / 20;                % Gain, max rate of turn over max. rudder angle
T_yaw = 1;                     % Time constant for yaw dynamics
zeta_d_psi = 1.0;              % Desired damping ratio for yaw control
wn_d_psi = 0.1;                % Natural frequency for yaw control
r_max = deg2rad(5.0);          % Maximum allowable rate of turn (rad/s)

% Heading autopilot (Equation 16.479 in Fossen 2021)
% sigma = r-r_d + 2*lambda*ssa(psi-psi_d) + lambda^2 * integral(ssa(psi-psi_d))
% delta = (T_yaw*r_r_dot + r_r - K_d*sigma - K_sigma*(sigma/phi_b)) / K_yaw
lambda = 0.1;
phi_b = 0.1;                   % Boundary layer thickness

if ControlFlag == 1 % PID control parameters
    K_d = 0.5;                 % Derivative gain for PID controller
    K_sigma = 0;               % Gain for SMC (inactive when using PID)
else                        
    % SMC control parameters
    K_d = 0;                   % Derivative gain inactive in SMC mode
    K_sigma = 0.05;            % Sliding mode control gain
end

% Initial slip angle
alpha_c_hat = 0;
beta_c_hat = 0;

% initial observer state
zetahat = [1.1 1.5 1.2];
ehat = [5 2 5];
Uvhat = 0;

% Additional parameter for straigh-line path following
R_switch = 5;               % Radius of switching circle
K_f = 0.5;                  % LOS observer gain
tau = 0;
switching = false;

% For Obstacle Avoidance
dim = 3;
opt_func = [];
boundary = [-1000,1000; -1000,1000; -1000,1000];
nsols = 30;
b = 0.5;
a = 2.0;
a_step = 2.0/20;
maximize = false;
woa = WOA_PathPlanning(dim, opt_func, boundary, nsols, b, a, a_step, maximize);
current_alt_wp = [0 0 0];
to_alt_wp = false;

%% INPUT RANGE CHECK ======================================================================================
rangeCheck(n_d, 0, n_max); 
rangeCheck(z_step, 0, z_max);
rangeCheck(U, 0, 5);

%% MAIN LOOP ===============================================================================================
simData = zeros(nTimeSteps, length(x) + 10); % Preallocate table for simulation data
alosData = zeros(nTimeSteps, 7); % Preallocate table for ALOS guidance data
distData = zeros(nTimeSteps, 3);
observerData = zeros(nTimeSteps, 8);
obsAvoidData = zeros(nTimeSteps,2);

for i = 1:nTimeSteps

    % Measurements 
    q = x(5);                  % Pitch rate (rad/s)
    r = x(6);                  % Yaw rate (rad/s)
    xn = x(7);                 % North position (m)
    yn = x(8);                 % East position (m)
    zn = x(9);                 % Down position (m), depth

    % Kinematic representation
    switch KinematicsFlag
        case 1
            theta = x(11); psi = x(12); % Euler angles
        otherwise
            [~,theta,psi] = q2euler(x(10:13)); % Quaternion to Euler angles
    end

    % Control systems 
    if ControlFlag
        % Heading autopilot using the tail rudder (integral SMC)
        % delta_r = integralSMCheading(psi, r, psi_d, r_d, a_d, ...
        %     K_d, K_sigma, 1, phi_b, K_yaw, T_yaw, h);
        delta_r = -15.0 * ssa( psi - psi_d )...
            - 1.0 * r - 0.1 * psi_int;
        delta_r = sat(delta_r, delta_max);

        % Depth autopilot using the stern planes (PID)
        delta_s = -Kp_theta * ssa( theta - theta_d )...
            - Kd_theta * q - Ki_theta * theta_int;
        delta_s = sat(delta_s, delta_max);

        % ALOS guidance lawk
        [alpha_c_act, beta_c_act] = ActualSlipAngle(x);
        % [Uvhat, Uvact] = calculate_uhat(etadot,x(1:6));
        
        if scenario == 1
            [psi_ref, theta_ref, e, alpha_c_hat, beta_c_hat, target_wp, ~, ~, switching, idx_wp] = ALOS3D(xn,yn,zn, ...
            h, R_switch, wpt, alpha_c_hat, beta_c_hat, t(i));
            [Uvhat,Uvact] = calculate_uhat(etadot,x(1:6),beta_c_hat);
        elseif scenario == 2
            if switching && tau > 60
                tau = 0;
                [psi_ref, theta_ref, e, alpha_c_hat, beta_c_hat, target_wp,~,~, switching, idx_wp] = ALOS3D(xn,yn,zn, ...
                h, R_switch, wpt, alpha_c_hat, beta_c_hat,t(i));
                [~, ~, ehat, zetahat] = observer(e,pii,x,ehat,zetahat,Uvhat,h);
                [Uvhat,Uvact] = calculate_uhat(etadot,x(1:6),beta_c_hat);
            elseif ~switching && tau < 60
                tau = tau + 1;
                [psi_ref, theta_ref, e, alpha_c_hat, beta_c_hat, target_wp,pi_h, pi_v, switching, idx_wp] = ALOS3D(xn,yn,zn, ...
                h, R_switch, wpt, alpha_c_hat, beta_c_hat, t(i));
                pii = [pi_h,pi_v];
                [~, ~, ehat, zetahat] = observer(e,pii,x,ehat,zetahat,Uvhat,h);
                [Uvhat,Uvact] = calculate_uhat(etadot,x(1:6),beta_c_hat);
            else
                [psi_ref, theta_ref, e, ~, ~, target_wp, pi_h, pi_v, switching, idx_wp] = ALOS3D(xn,yn,zn, ...
                h, R_switch, wpt, alpha_c_hat, beta_c_hat, t(i));
                pii = [pi_h,pi_v];
                [alpha_c_hat, beta_c_hat, ehat, zetahat] = observer(e,pii,x,ehat,zetahat,Uvhat,h);
                [Uvhat,Uvact] = calculate_uhat(etadot,x(1:6),beta_c_hat);
                tau = tau + 1;
            end
        else
            error('Scenario not valid')
        end

        % if switching
        %     fprintf('Switching at t:%.2f with pos: (%.2f,%.2f,%.2f)',t(i),xn,yn,zn)
        % end

        x_e = e(1); y_e = e(2); z_e = e(3);

        % ALOS observer
        [theta_d, q_d] = LOSobserver(theta_d, q_d, theta_ref, h, K_f);
        [psi_d, r_d] = LOSobserver(psi_d, r_d, psi_ref, h, K_f);
        r_d = sat(r_d, r_max);
        
    end

    % Ocean current dynamics
    [Vc,alphaVc,betaVc,wc] = disturbance_function(t(i),h);
    % Vc = 0; alphaVc = 0; betaVc = 0; wc = 0;

    % Sonar check for obstacle 
    [threat_dist,detected_obs,r_obs,ang_obs,intersect_obs,toward_obs,idx_obs] = sonar3D(x,etadot,target_wp,obs);
    ang_obs = rad2deg(ang_obs);
    if ~isempty(detected_obs) && intersect_obs
        fprintf('Obs detected in range %.0f with psi: %.1f°, theta: %.1f° from pos. [%.0f %.0f %.0f] with idx: %.0f\n', ...
            r_obs, ang_obs(1), ang_obs(2), x(7), x(8), x(9),idx_wp);
        radius_obs = obs.r{idx_obs};
        alt_wp = woa.repath_planning(x(7:9),target_wp,detected_obs,radius_obs);
        if idx_wp >= 9
            fprintf('Pos obs: [%.0f %.0f %.0f] \n',detected_obs(1), detected_obs(2), detected_obs(3))
            fprintf('Current wp [%.0f %.0f %.0f] --  alt wp [%.0f %.0f %.0f] -- target: [%.0f %.0f %.0f] \n',...
                current_alt_wp(1),current_alt_wp(2),current_alt_wp(3),alt_wp(1),alt_wp(2),alt_wp(3),...
                target_wp(1),target_wp(2),target_wp(3))
        end
        if ~isequal(round(current_alt_wp,1),round(alt_wp,1))
            % fprintf('\n New WP Generated because obs at [%.0f %.0f %.0f] at k:%.0f \n',detected_obs(1),detected_obs(2),detected_obs(3),idx_wp)
            % wpt = add_alternative_waypoint(wpt,alt_wp,idx_wp);
            fprintf('New WP at: [%.1f %.1f %.1f]',alt_wp(1),alt_wp(2),alt_wp(3))
            switching = true;
            to_alt_wp = true;
        end
        current_alt_wp = alt_wp;
    end

    if switching && to_alt_wp
        to_alt_wp = false;
    end

    % Propeller speed (RPM)
    if n < n_d
        n = n + n_rate;
    elseif n > n_d
        n = n - n_rate;
    end
    n = sat(n, n_max);

    % Control input vector
    ui = [delta_r, -delta_s, n]';            

    % Store simulation data in a table
    simData(i,:) = [z_d theta_d psi_d r_d Vc betaVc wc ui' x'];

    % Store ALOS data in table
    alosData(i,:) = [x_e y_e z_e alpha_c_hat beta_c_hat alpha_c_act beta_c_act];

    % Disturbance data
    distData(i,:) = [Vc alphaVc betaVc];

    % Observer data
    observerData(i,:) = [ehat(1) ehat(2) ehat(3) zetahat(1) zetahat(2) zetahat(3) Uvhat Uvact];

    % Obstacle Avoidance Data
    obsAvoidData(i,:) = [threat_dist(1) threat_dist(2)];

    if (KinematicsFlag == 1)
        % Euler angles x = [ u v w p q r x y z phi theta psi ]'
        x = rk4(@remus100, h, x, ui, Vc, betaVc, alphaVc, wc);  % RK4 method x(k+1)
        etadot = EulerAngleDot(x); 
    else
        % Unit quaternions x = [ u v w p q r x y z eta eps1 eps2 eps3 ]'  
        xdot = remus100(x, ui, Vc, betaVc, wc);
        quat = x(10:13);                            % Unit quaternion
        x(1:6) = x(1:6) + h * xdot(1:6);            % Forward Euler
        x(7:9) = x(7:9) + h * Rquat(quat) * x(1:3); % Backward Euler
        quat = expm(Tquat(x(4:6)) * h) * quat;  % Exact quat. discretization
        x(10:13) = quat / norm(quat);           % Normalization
    end

    % Euler's integration method (k+1)
    z_int = z_int + h * ( zn - z_d );
    theta_int = theta_int + h * ssa( theta - theta_d );
    psi_int = psi_int + h * ssa( psi - psi_d );

end

filename = strcat('data/scenario_', num2str(scenario), '.mat');
filename1 =  strcat('data/obs_avoid_', num2str(scenario), '.mat');
save(filename, 'simData', 'alosData','distData','observerData', 't','-v7');
save('data/obstacles.mat', 'obs', '-v7');
save('data/waypoints.mat', 'wpt', 'wpt_ori', '-v7');
save(filename1,'obsAvoidData', '-v7');

%% PLOTS ===================================================================================================
scrSz = get(0, 'ScreenSize'); % Returns [left bottom width height]
legendLocation = 'best'; legendSize = 12;
if isoctave; legendLocation = 'northeast'; end

% simData = [z_d theta_d psi_d r_d Vc betaVc wc ui' x']
z_d     = simData(:,1);
theta_d = simData(:,2);
psi_d   = simData(:,3);
r_d     = simData(:,4); %#ok<*NASGU>
Vc      = simData(:,5);
betaVc  = simData(:,6);
wc      = simData(:,7);
ui      = simData(:,8:10);
nu      = simData(:,11:16);
eta     = simData(:,17:22);

u = nu(:,1); v = nu(:,2); w = nu(:,3);
p = nu(:,4); q = nu(:,5); r = nu(:,6);

% alosData = [x_e y_e z_e alpha_c_hat beta_c_hat alpha_c_act beta_c_act]
x_e = alosData(:,1);
y_e = alosData(:,2);
z_e = alosData(:,3);

alpha_c_hat = alosData(:,4);
beta_c_hat = alosData(:,5);

% Ocean current velocities 
uc = Vc .* cos(betaVc);
vc = Vc .* sin(betaVc);

% Crab angles, AOA and SSA
U = sqrt(u.^2+v.^2+w.^2); % Speed
gamma = asin( (u.*sin(theta)-(v.*sin(phi)+w.*cos(phi)).*cos(theta)) ./ U );
alpha_c = theta - gamma; % Horizontal crab angle
beta_c = atan2(v.*cos(phi)-w.*sin(phi), ...
    u.*cos(theta)+(v.*sin(phi)+w.*cos(phi)).*sin(theta)); % Vertical crab angle
alpha = asin( (w-wc) ./ (u-uc) ); % AOA
beta  = atan2( (v-vc), (u-uc) ); % SSA

%% Generalized velocity
%PlotState();

%% Heave position and Euler angles
%PlotEta();

%% Control signals
%PlotControlSignal();

%% Ocean currents and speed
%PlotCurrent();

%% Crab angles, SSA and AOA
if ControlFlag == 3
    %PlotSlipAngle()
end

%% 2-D position plots with waypoints
if ControlFlag == 3
    %Plot2DPos()
end

%% 3-D position plot with waypoints
if ControlFlag == 3
    %Plot3DPos()
end

%PlotTrackError()

% Display the vehicle data and an image of the vehicle
vehicleData = {...
    'Length', '1.6 m',...
    'Diameter', '19 cm',...
    'Mass', '31.9 kg',...
    'Max speed', '2.5 m/s',...
    'Max propeller speed', '1525 RPM'};
% displayVehicleData('Remus100 AUV', vehicleData, 'remus100.jpg', 9);

end % SIMremus100