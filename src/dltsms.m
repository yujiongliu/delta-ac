function [ mass ] = dltsms( ang )
% /***********************************************************************
% * Name:       Delta simplified mass matrix
% * Type:       Function
% * Author:     Yujiong Liu
% * Date:       2015.1.10
% * 
% * Input:      [IN]    ang     Angles of the joints
% * Output:     [OUT]   mass    Mass matrix
% * Function:   Compute the mass matrix. This is the simplified model.
% ***********************************************************************/

% PARAMETER
% /**********************************************************************/
%GEOMETRY
L_A=270/1000;                %length of the drive link
C_R=33;                      %reductor coefficient

%INERTIA
I_R=0.22;                   %inertia of the reductor
I_M=(I_R+0.26)*10^(-4)*C_R^2;  %inertia of the motor and reductor(convert to output axis)
M_A=0.8806;             %mass of the drive link
M_E=0.21;%0.064;              %mass of elbow
M_F=0.490;                %mass of the passive link
M_T=0.710;                  %mass of the travelling plate and payload
% COMPUTATION
% /**********************************************************************/
i_aci=I_M+L_A^2*(M_A/3+M_E+M_F*2/3);
jc=dltjc(ang);
mass=diag([i_aci,i_aci,i_aci])+(M_T+M_F)*jc'*jc;
end

