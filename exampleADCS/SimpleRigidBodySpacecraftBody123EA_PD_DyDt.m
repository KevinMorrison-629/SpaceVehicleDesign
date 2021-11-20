function dydt =SimpleRigidBodySpacraftDyDt(t, y)
% Program determine the first order time derivatives of variable vector
%   y = [y1,y2] for the current values of y and time t. Returns dydt(t) to ODE45

global K_EA K_om EA_desired om_desired I M_Csaturate MCoft % declare what 
   % quantities are global so that they can be easily used by the the dydt script

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Update state variables

om=[y(1);y(2);y(3)];
EA = [y(4);y(5);y(6)];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Construct the DCM associated with a SPACE123 Transformation
% describing the actual "A" (current) orientation of the S/c

% Cij is the i, j-th element of the DCM which relates directions n_j
% (j=1,2,3)fixed in Newtonian Reference frame N, to directions b_i (i=1,2,3)
%  fixed in the s/c bus B.   N^C^B <=> Cij = b_i . n_j

EA1=EA(1);
EA2=EA(2);
EA3=EA(3);

C11=cos(EA2).*cos(EA3);
C12=-cos(EA2).*sin(EA3);
C13=sin(EA2);
C21=sin(EA1).*sin(EA2).*cos(EA3)+cos(EA1).*sin(EA3);
C22=-sin(EA1).*sin(EA2).*sin(EA3)+cos(EA1).*cos(EA3);
C23=-sin(EA1).*cos(EA2);
C31=-cos(EA1).*sin(EA2).*cos(EA3)+sin(EA1).*sin(EA3);
C32=cos(EA1).*sin(EA2).*sin(EA3)+sin(EA1).*cos(EA3);
C33=cos(EA1).*cos(EA2);
DCM_actual=[C11 C12 C13; C21 C22 C23; C31 C32 C33];% N^C^A
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Construct the DCM associated with a Body123 Transformation
% describing the desired "D" orientation of the S/c to N

% Cij is the i, j-th element of the DCM which relates directions n_j
% (j=1,2,3) fixed in the s/c to directions b_i (i=1,2,3) fixed in Newtonian
% Reference frame N.   N^C^B <=> Cij = n_i . b_j

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
DCM_error = DCM_desired'*DCM_actual; %D_C_A

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Determine the associated BODY123 Orientation Angles from the associated
% DCM D_C_A. (from Actual Orientation to Desired orientation)

EAerror1=atan2(-DCM_error(2,3),DCM_error(3,3)); % Orientation angle associated
             % with rotation #1 ( which is about direction n_1)
%if norm(DCM_error(1,3))>1,
%    DCM_error(1,3)=sign(DCM_error(1,3)); 
%end
 
 DCM11 = DCM_error(1,1);
 if abs(DCM11)< 1.0e-8
     DCM11 = 1.0e-8;
 end %endif
 
 EAerror3=atan2(-DCM_error(1,2),DCM_error(1,1)); % Orientation angle associated
            % with rotation #3 ( which is about direction n-3)
 % rotation #2 ( which is about direction n-2)
 EAerror2=atan2(DCM_error(1,3)*cos(EAerror3),DCM11);  % Orientation angle associated with
            % rotation #2 ( which is about direction n-2)
            
EA_error=[EAerror1; EAerror2; EAerror3];
 %EAerror2Delta = EAerror2 -EAerror2A;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% construct Control Moment based on Error in Angular Velocity (om_error) and 
% Euler Angles (EA_error)

% determine erroe in Angular Velocity of B in N
om_error = om - om_desired; 

M_C = -K_EA .* EA_error - K_om.*om_error;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Equations associated with dynamics and control of simple Rigid body S/C
% Simplifed model assumes that the principal directions are b1,b2,b3, which
% are body fixed basis vectors used to define most matrices in this script

% Definitions:
% om = [om1;om2;om3] <-> om1*b1 + om2*b2 + om3*b3 is the angular velocity
       % of the Spacecraft body B in N
% omdot = [om1dot; om2dot; om3dot]  is the angular acceleration of the
       % Spacecraft body B in the Newtonian reference frame N
       
% M_E = External moments (due to solar pressure, gravity gradient, etc.  
       % applied to the space craft    
% M_C = Control moments (Thrusters, magnetic torquers [but not reactions 
       % wheels applied to the spacecraft body B (carrier)
% M_Csaturate = saturation value of M_C       

% Spacecraft Euler's Equations
I1 = I(1);
I2 = I(2);
I3 = I(3);

% If control moment is too large, set to saturation value
for i1=1:3
    if   M_C(i1)>M_Csaturate
         M_C(i1)= M_Csaturate; 
    end %endif  
end % endfor    


% Total Apparent external moment acting on Spacecraft Bus
M_E= [0;0;0];% M_E is zero until defined otherwise

M = M_E + M_C;

om1=om(1);
om2=om(2);
om3=om(3);
omdot1=(M(1)+(I2-I3)*om2*om3)/I1;
omdot2=(M(2)+(I3-I1)*om3*om1)/I2;
omdot3=(M(3)+(I1-I2)*om1*om2)/I3;
omdot=[omdot1; omdot2; omdot3];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define Kinematic Differential Equaions of Motion Associate with Body123
% transformation
EA1=EA(1);
EA2=EA(2);
EA3=EA(3);

EAdot1=(1/cos(EA2))*(om1*cos(EA3)-om2*sin(EA3));
EAdot2= om1*sin(EA3) + om2*cos(EA3);
EAdot3=(1/cos(EA2))*(-om1*cos(EA3)+om2*sin(EA3))*sin(EA2)+om3;
EAdot=[EAdot1; EAdot2; EAdot3];

% specify state derivative values
dydt= cat(1,omdot,EAdot);
MCoft = cat(1,MCoft,M_C');


%EOF SimpleRigidBodySpacecraftEA_DyDt.m


