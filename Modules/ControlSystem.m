% Quadcopter PID Control Tuning and Simulation
% Based on Andrew Gibiansky's report

clc; clear; close all;

% Define global variables
global g m Ixx Iyy Izz 

% Physical Constants
g = 9.81;  % Gravity (m/s^2)
m = 0.5;   % Mass of the quadcopter (kg)
L = 0.25;  % Distance from center to rotor (m)
Ixx = 5e-3; % Moment of inertia around x-axis (kg*m^2)
Iyy = 5e-3; % Moment of inertia around y-axis (kg*m^2)
Izz = 1e-2; % Moment of inertia around z-axis (kg*m^2)
kF = 3e-6;  % Thrust coefficient
kM = 1e-7;  % Moment coefficient

% Define Transfer Functions for PID Tuning
s = tf('s');
G_altitude = 1/(m*s^2); % Altitude control plant
G_attitude = 1/(Ixx*s^2); % Attitude control plant (example for roll)

% Automatic PID Tuning
C_altitude = pidtune(G_altitude, 'PID');
C_attitude = pidtune(G_attitude, 'PID');

disp('Tuned Altitude PID Controller:');
display(C_altitude);
disp('Tuned Attitude PID Controller:');
display(C_attitude);

% Initial State (x, y, z, phi, theta, psi, vx, vy, vz, p, q, r)
x0 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];

% Time Span
tspan = [0 10];

% Solve Dynamics
[t, state] = ode45(@quadcopter_dynamics, tspan, x0);

% Plot Results
figure;
subplot(3,1,1); plot(t, state(:,1:3)); title('Position (m)'); legend('x','y','z');
subplot(3,1,2); plot(t, state(:,4:6)); title('Angles (rad)'); legend('\phi','\theta','\psi');
subplot(3,1,3); plot(t, state(:,7:9)); title('Velocity (m/s)'); legend('vx','vy','vz');

disp('Simulation Complete');

% Dynamics Function
function dx = quadcopter_dynamics(~, x)
    global g m Ixx Iyy Izz
    dx = zeros(12,1);
    
    % Extract states
    phi = x(4); theta = x(5); psi = x(6);
    p = x(10); q = x(11); r = x(12);
    
    % Rotation matrix
    R = [cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi);
         cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta),  cos(phi)*cos(psi), sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi);
         -cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta)];
    
    % Force and Torque Inputs (Simple Hover Case)
    U1 = m * g;  % Total thrust
    U2 = 0; U3 = 0; U4 = 0; % No external torques
    
    % Translational Motion
    acc = R * [0; 0; U1] / m - [0; 0; g];
    dx(1:3) = x(7:9);
    dx(7:9) = acc;
    
    % Rotational Motion
    dx(4:6) = [p; q; r];
    dx(10) = (U2 / Ixx);
    dx(11) = (U3 / Iyy);
    dx(12) = (U4 / Izz);
end
