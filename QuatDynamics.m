% This script solves the dynamic behavior of a simple rigid body
% spacecraft. In particular, this script incorporates a simple simulation
% of a rigid body spacecraft described using euler symmetric parameters
% (quaternions). This script also includes a simple control system used to
% apply control moments to the bus in order to produce the desired angular
% velocity and orientation.

clear;
clc;

%% Setup

% Simulation Parameters:
global DT
SIM_LENGTH = 15; % length of simulation (in seconds)
DT = 0.001; % timestep length (in seconds)

% Initial Conditions
% though the simulation is done in quaternions, it is simpler for users to
% interface with Euler angles. As such, initial and desired states are
% inputted using Euler Angles and then converted into quaternions for use
% in the simulation.
initial_EulerAngles = [1.5; 2.2; 4.5]; % radians
initial_AngularVelocity = 2*[1.1; 1.2; -1.33]; % radians/second

desired_EulerAngles = [0; 0; 0];
desired_AngularVelocity = [0; 0; 0];

% Spacecraft Properties
I = [340.87, 104.95, 127.92 ;
     104.95, 390.52, 124.63;
     127.92, 124.63, 379.42] / (1000); % Moment of Inertia (kg*m^2)
I_inv = pinv(I);


% Controller Properties
global MC_saturate MC_history
MC_saturate = 0.2; % the saturation value for the control actuator
MC_history = [];


% if PID:
K_EA = 0.01*[1; 1; 1];
K_om = 0.2*[1; 1; 1];
global q_error_int q_error_prev w_error_int w_error_prev
q_error_int = 0;
q_error_prev = 0;
w_error_int = 0;
w_error_prev = 0;

% if MPC:
% variables here


%% Convert to Quaternions
q0 = C_toQuat(EA_toC(initial_EulerAngles)) / norm(C_toQuat(EA_toC(initial_EulerAngles)));
w0 = initial_AngularVelocity;

global Kq Kw q_des w_des
q_des = C_toQuat(EA_toC(desired_EulerAngles)) / norm(C_toQuat(EA_toC(desired_EulerAngles)));
w_des = desired_AngularVelocity;

% PID gains
Kq = C_toQuat(EA_toC(K_EA)) / norm(C_toQuat(EA_toC(K_EA)));
Kw = K_om;

%% Simulation

%global q0 w0 q_des w_des I

iters = SIM_LENGTH/DT;
%iters = 60;

q_sim = zeros(4,iters);
w_sim = zeros(3,iters);

q_sim(:,1) = q0;
w_sim(:,1) = w0;

M_ext = [0; 0; 0];

for i = 2:iters
    
    M_c = controller(i, q_sim, w_sim);
    
    q_dot = 0.5*quat_mult(q_sim(:,i-1), [0; w_sim(:,i-1)]);
    w_dot = I_inv*((M_ext+M_c)-(wcross(w_sim(:,i-1))*I*w_sim(:,i-1)));
    
    w_sim(:,i) = w_sim(:,i-1) + DT.*w_dot;
    q_sim(:,i) = q_sim(:,i-1) + DT.*q_dot;
    
    q_sim(:,i) = q_sim(:,i) / norm(q_sim(:,i));
end


%% Plotting

time = 0:DT:SIM_LENGTH-DT;

figure(1)
plot(time, w_sim(1,:))
hold on
plot(time, w_sim(2,:))
plot(time, w_sim(3,:))
xlabel('Time')
ylabel('Angular Velocity (rad/s)')
title('')
legend('w_x','w_y','w_z')

figure(2)
plot(time, q_sim(1,:))
hold on
plot(time, q_sim(2,:))
plot(time, q_sim(3,:))
plot(time, q_sim(4,:))
xlabel('Time')
ylabel('Euler Symmetric Parameter')
title('')
legend('q0','q1','q2','q3')

% figure(3)
% plot(time, MC_history(1,:))
% hold on
% plot(time, MC_history(2,:))
% plot(time, MC_history(3,:))
% xlabel('Time')
% ylabel('Controller History (N-m)')
% title('')
% legend('Mx','My','Mz')


%% Useful Functions

function C = EA_toC(EA)
x = EA(1);
y = EA(2);
z = EA(3);

C = [cos(y).*cos(z), -cos(y).*sin(z), sin(y);
     sin(x).*sin(y).*cos(z)+cos(x).*sin(z), -sin(x).*sin(y).*sin(z)+cos(x).*cos(z), -sin(x).*cos(y);
     -cos(x).*sin(y).*cos(z)+sin(x).*sin(z), cos(x).*sin(y).*sin(z)+sin(x).*cos(z), cos(x).*cos(y)];
end


function q = C_toQuat(C)
q4 = 0.5*sqrt(1 + C(1,1) + C(2,2) + C(3,3));
q1 = 0.5*sqrt(1 + C(1,1) - C(2,2) - C(3,3));
q2 = 0.5*sqrt(1 - C(1,1) + C(2,2) - C(3,3));
q3 = 0.5*sqrt(1 - C(1,1) - C(2,2) + C(3,3));

if q4 > q1 && q4 > q2 && q4 > q3
    q4 = 0.5*sqrt(1 + C(1,1) + C(2,2) + C(3,3));
    q1 = (C(3,2) - C(2,3)) / (4*q4);
    q2 = (C(1,3) - C(3,1)) / (4*q4);
    q3 = (C(2,1) - C(1,2)) / (4*q4);
elseif q3 > q1 && q3 > q2 && q3 > q4
    q3 = 0.5*sqrt(1 - C(1,1) - C(2,2) + C(3,3));
    q4 = (C(2,1) - C(1,2)) / (4*q3);
    q1 = (C(1,3) + C(3,1)) / (4*q3);
    q2 = (C(2,3) + C(3,2)) / (4*q3);
elseif q2 > q1 && q2 > q3 && q2 > q4
    q2 = 0.5*sqrt(1 - C(1,1) + C(2,2) - C(3,3));
    q3 = (C(2,3) + C(3,2)) / (4*q2);
    q4 = (C(1,3) - C(3,1)) / (4*q2);
    q1 = (C(1,2) + C(2,1)) / (4*q2);
else
    q1 = 0.5*sqrt(1 + C(1,1) - C(2,2) - C(3,3));
    q2 = (C(1,2) + C(2,1)) / (4*q1);
    q3 = (C(1,3) + C(3,1)) / (4*q1);
    q4 = (C(3,2) - C(2,3)) / (4*q1);
end
q0 = q4;
q = [q0; q1; q2; q3];

end

function wx = wcross(w)
    wx = [0, -w(3), w(2);
          w(3), 0, -w(1);
          -w(2), w(1), 0];
end

function mult_result = quat_mult(p, q)
    p0 = p(1);
    p1 = p(2);
    p2 = p(3);
    p3 = p(4);
    
    P = [p0 -p1 -p2 -p3;
         p1  p0 -p3  p2;
         p2  p3  p0 -p1;
         p3 -p2  p1  p0];
    mult_result = P*q;
    
end

function conjugate = quat_conj(q)
    conjugate = [q(1) -q(2) -q(3) -q(4)].';
end


%% Controller

function M_c = controller(i, q, w)
    global DT Kq Kw q_des w_des MC_saturate q_error_int q_error_prev w_error_int w_error_prev
    global MC_history
    is_PID = true;
    is_MPC = false;

    if is_PID == 1
        % proporitional
        w_err = w(:,i-1) - w_des;
        q_err = quat_mult(q_des, quat_conj(q(:,i-1)));
        if q_err(1) < 0
            q_err = [-q_err(2) -q_err(3) -q_err(4)].';
        else
            q_err = [q_err(2) q_err(3) q_err(4)].';
        end
        M_c_proportional = -Kq(1)*q_err - Kw(1)*w_err;
        
        % integral
        q_error_int = q_error_int + q_err*DT;
        w_error_int = w_error_int + w_err*DT;
        
        M_c_integral = -Kq(2)*q_error_int - Kw(2)*w_error_int;
        
        % derivative
        q_error_der = (q_err - q_error_prev)/DT;
        w_error_der = (w_err - w_error_prev)/DT;
        
        M_c_derivative = -Kq(3)*q_error_der - Kw(3)*w_error_der;
        
        
        % totals
        M_c = M_c_proportional + M_c_integral + M_c_derivative;
        if norm(M_c) > MC_saturate
            M_c = MC_saturate*(M_c / norm(M_c));
        end
        
    elseif is_MPC == 1
        M_c = [0; 0; 0];
    else
        M_c = [0; 0; 0];
    end
    
    MC_history(:,i) = M_c;
    
end
