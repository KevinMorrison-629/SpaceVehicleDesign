%%initialize attitude example
%bong wie, chapter 6, page 361, equations 6.48 -- chapter 5, equations 5.67

Tfinal=16
% Specify S/C Inertia properties
I1 = 1; %kg*m^2
I2 = 2; %kg*m^2
I3 = 3; %kg*m^2
I = [I1;I2;I3]

% specify desired Gains
K_EA = 1.0*[1.0; 2.0; 3.0]
% K_EA = [1.0;2.0; 3.0]
%K_om = 1.0*[1.0; 1.5; 2.0] % Original
K_om = 1.0*[1.0; 1.5; 2.0]

K_q=K_EA

% Specify S/C initial state
om = 20.0*[1.1; 1.2; -1.33];% rad/s
%EA = 2.1*[1.0; 2.0; -3.0]; %rad
EA = 1.0*[1.0; 2.0; -0*3.0]; %rad

%Specify desired state (simple time independent set-point)
om_desired =[0.0; 0.0; 0.0]
EA_desired = [0.0; 0.0; 0.0]

% Specify control moment saturation value
M_Csaturate = 20