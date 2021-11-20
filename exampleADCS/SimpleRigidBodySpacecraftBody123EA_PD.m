% This script solves simulates the dynamic behavior of a simple rigid body
% spacecraft in which control moments are applied to the bus in an effort
% produced the desired angular velocity and orientation. This particular 
% model use Body123 Euler Angles to parameterize the orientation.)
									  
% The problem is solve the full nonlinear equations by integrating them in
% time numerical using "ode45"
									  
clear global variable %clear the global variables so that you are sure of 
          % what your are starting with
clear all;close all;clc % clear memory; close all windows; clear the command window

global K_EA K_om EA_desired om_desired I M_Csaturate MCoft % declare what 
   % quantities are global so that they can be easily used by the the dydt script

								
%Initalize state and provide key quantities by running initialization
%script
%initialize_attitudeEA
initialize_attitudeEA


%Simulation duration input in "initializ_attitude.m"
%Tfinal = 10.0 % [s]	Duration of simulation

									  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Numerical Solution 
tspan = [0,Tfinal] %specify the time interval over which the simulation is
                   % run
Y0 = cat(1,om,EA) % Specify intial value for first order state variables                   


%fname = 'PlanarRealisticRocketDyDt' % specify name of .m file containing the 
fname = @SimpleRigidBodySpacecraftBody123EA_PD_DyDt % specify name of .m file containing the 
          % script which determines the state variable time derivatives
        
         

% Run ODE45 to integrate these equations (fname) and return the result
%[t,Y,TE,YE,IE]= ode45(fname,tspan,Y0, odeset('Events','on'));
[t,Y]= ode45(fname,tspan,Y0);
 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Some good quantities to plot (for visualizing behavior)
EA1=Y(:,4);
EA2=Y(:,5);
EA3=Y(:,6);

C11_List = cos(EA2).*cos(EA3);
% C12=-cos(EA2).*sin(EA3);
% C13=sin(EA2);
% C21=sin(EA1).*sin(EA2).*cos(EA3)+cos(EA1).*sin(EA3);
C22_List = -sin(EA1).*sin(EA2).*sin(EA3)+cos(EA1).*cos(EA3);
%C23=-sin(EA1).*cos(EA2);
%C31=-cos(EA1).*sin(EA2).*cos(EA3)+sin(EA1).*sin(EA3);
%C32=cos(EA1).*sin(EA2).*sin(EA3)+sin(EA1).*cos(EA3);
C33_List = cos(EA1).*cos(EA2);
%DCM_actual=[C11 C12 C13; C21 C22 C23; C31 C32 C33];   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Construct the DCM associated with a BODY123 Transformation
% relating the desired orientation of the S/c b-basis frame to the 
% Newtonian n-basis frame 

% Cij is the i, j-th element of the DCM which relates directions b_j
% (j=1,2,3) fixed in the s/c to directions n_i (i=1,2,3) fixed in Newtonian
% Reference frame N.   B^C^N <=> Cij = b_i . n_j

EAd1=EA_desired(1);
EAd2=EA_desired(2);
EAd3=EA_desired(3);
C11d=cos(EAd2)*cos(EAd3);
C12d=-cos(EAd2)*sin(EAd3);
C13d=sin(EAd2);
C21d=sin(EAd1)*sin(EAd2)*cos(EAd3)+cos(EAd1)*sin(EAd3);
C22d=-sin(EAd1)*sin(EAd2)*sin(EAd3)+cos(EAd1)*cos(EAd3);
C23d=-sin(EAd1)*cos(EAd2);
C31d=-cos(EAd1)*sin(EAd2)*cos(EAd3)+sin(EAd1)*sin(EAd3);
C32d=cos(EAd1)*sin(EAd2)*sin(EAd3)+sin(EAd1)*cos(EAd3);
C33d=cos(EAd1)*cos(EAd2);
DCM_desired=[C11d C12d C13d; C21d C22d C23d; C31d C32d C33d];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Construct the DCM which characterizes the error between the current (actual)
% orientation and the desired orientation. 
%DCM_error = DCM_desired'*DCM_actual %D_C_A

phi1_List = acos(C11_List); 
phi2_List = acos(C22_List);
phi3_List = acos(C33_List);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


						 




plot(t,Y(:,1), 'b', t, Y(:,2), 'r', t, Y(:,3),'g' )
title ('Spacecraft Angular Velocity vs. Time')
xlabel('Time [s]')
ylabel('Angular Velocity [rad/s]')
legend('omega_1','omega_2', 'omega_3')


figure(2)
plot(t,Y(:,4), 'b', t, Y(:,5), 'r', t, Y(:,6),'g' )
title ('Spacecraft Body123 Euler Angles vs. Time')
xlabel('Time [s]')
ylabel('Euler Angle [rad]')
legend('theta_1','theta_2', 'theta_3')

figure(3)
plot(t,phi1_List, 'b', t, phi2_List, 'r', t, phi3_List,'g' )
title ('Spacecraft b_1, b_2, b_3 Off-Pointing Angles vs. Time')
xlabel('Time [s]')
ylabel('Off-Pointing Angle [rad]')
legend('phi_1','phi_2', 'phi_3')
  