function [ jac ] = dltjc( ang )
% /***********************************************************************
% * Name:       Delta jacobian matrix of the travelling plate
% * Type:       Function
% * Author:     Yujiong Liu
% * Date:       2015.1.10
% * 
% * Input:      [IN]    ang     Angles of the joints
% * Output:     [OUT]   jac     Jacobian matrix
% * Function:   Compute the jacobian matrix of the travelling plate by
% *             analytical method
% ***********************************************************************/

% PARAMETER
% /**********************************************************************/
R_B=200/1000;                %radius of the base
L_A=270/1000;                %length of the drive link
R_T=45/1000;                %radius of the travelling plate

% COMPUTATION
% /**********************************************************************/
rot_1=[1,0,0;0,1,0;0,0,1];                          %first frame
rot_2=[-0.5,-sqrt(3)/2,0;sqrt(3)/2,-0.5,0;0,0,1];   %second frame
rot_3=[-0.5,sqrt(3)/2,0;-sqrt(3)/2,-0.5,0;0,0,1];   %third frame
r_r=R_B-R_T;                                        %reduced radius
ps_t=dltgfk(ang);                                   %find the position of
                                                    %travelling plate
s_1=ps_t-rot_1*[r_r+L_A*cos(ang(1));0;L_A*sin(ang(1))];
s_2=ps_t-rot_2*[r_r+L_A*cos(ang(2));0;L_A*sin(ang(2))];
s_3=ps_t-rot_3*[r_r+L_A*cos(ang(3));0;L_A*sin(ang(3))];

c_1=L_A*rot_1*[-sin(ang(1));0;cos(ang(1))];
c_2=L_A*rot_2*[-sin(ang(2));0;cos(ang(2))];
c_3=L_A*rot_3*[-sin(ang(3));0;cos(ang(3))];

jac=[s_1';s_2';s_3']^(-1)*diag([s_1'*c_1,s_2'*c_2,s_3'*c_3]);
end

