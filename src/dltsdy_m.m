function [ torq ] = dltsdy_m( ang,ang_d,ang_dd, x_dd )
% /***********************************************************************
% * Name:       Delta simplified dynamics
% * Type:       Function
% * Author:     Yujiong Liu
% * Date:       2015.1.14
% * 
% * Input:      [IN]    ang     Angles of the joints
% *             [IN]    ang_dd  Second derivative of the joint angles
% *             [IN]    x_dd    Second derivative of the travelling plate
% *                             position
% * Output:     [OUT]   torq    Torque of the actuators
% * Function:   Compute the dynamic torque of the actuators by virtual
% *             work principle. This is the simplified model.
% ***********************************************************************/

% PARAMETER
% /**********************************************************************/
%GEOMETRY
L_A=270/1000;                %length of the drive link
C_R=33;                      %reductor coefficient

%INERTIA
G=9.8;                  %gravity acceleration
I_R=0.22;                   %inertia of the reductor
I_M=(I_R+0.26)*10^(-4)*C_R^2;  %inertia of the motor and reductor(convert to output axis)
M_A=0.8806;             %mass of the drive link
M_E=0.21;%0.064;              %mass of elbow
M_F=0.490;                %mass of the passive link
M_T=0.710;                  %mass of the travelling plate and payload
F_V=diag([3.25,1.16,1.32]);                 %viscous friction coefficients
F_C=diag([2.13,1.74,3.17]);                 %Coulomb friction
C_A=diag([0.08,0.19,0.1]);                 %acceleration compensation
% COMPUTATION
% /**********************************************************************/
i_aci=I_M+L_A^2*(M_A/3+M_E+M_F*2/3);
i_ac=diag([i_aci,i_aci,i_aci]);
g_ac=(M_A/2+M_E+M_F/2)*L_A*G*cos(ang);
m_tc=M_T+M_F;
jc=dltjc(ang);
g_tcg=jc'*(M_T+3*M_F/2)*[0;0;G];
torq=i_ac*ang_dd-g_ac+m_tc*jc'*x_dd-g_tcg+F_V*ang_d+F_C*sign(ang_d)+C_A*ang_dd;
end

