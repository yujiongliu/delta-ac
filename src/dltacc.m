function [ acc ] = dltacc( ang, ang_d, ang_dd )
% /***********************************************************************
% * Name:       Delta acceleration of the travelling plate
% * Type:       Function
% * Author:     Yujiong Liu
% * Date:       2015.1.14/corrected in 6.9
% * 
% * Input:      [IN]    ang     Angles of the joints
% *             [IN]    ang_d   First derivative of the joint angles
% *             [IN]    ang_dd  Second derivative of the joint angles
% * Output:     [OUT]   acc     Acceleration of travelling plate
% * Function:   Compute the acceleration of the travelling plate
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
ps_t_d=dltjc(ang)*[ang_d(1);ang_d(2);ang_d(3)];

s_1=ps_t-rot_1*[r_r+L_A*cos(ang(1));0;L_A*sin(ang(1))];
s_2=ps_t-rot_2*[r_r+L_A*cos(ang(2));0;L_A*sin(ang(2))];
s_3=ps_t-rot_3*[r_r+L_A*cos(ang(3));0;L_A*sin(ang(3))];

c_1=L_A*rot_1*[-sin(ang(1));0;cos(ang(1))];
c_2=L_A*rot_2*[-sin(ang(2));0;cos(ang(2))];
c_3=L_A*rot_3*[-sin(ang(3));0;cos(ang(3))];

s_1_d=[ps_t_d(1);ps_t_d(2);ps_t_d(3)]-c_1*ang_d(1);
s_2_d=[ps_t_d(1);ps_t_d(2);ps_t_d(3)]-c_2*ang_d(2);
s_3_d=[ps_t_d(1);ps_t_d(2);ps_t_d(3)]-c_3*ang_d(3);

c_1_d=L_A*rot_1*[-cos(ang(1));0;-sin(ang(1))]*ang_d(1);
c_2_d=L_A*rot_2*[-cos(ang(2));0;-sin(ang(2))]*ang_d(2);
c_3_d=L_A*rot_3*[-cos(ang(3));0;-sin(ang(3))]*ang_d(3);

item=[s_1'*c_1*ang_dd(1);s_2'*c_2*ang_dd(2);s_3'*c_3*ang_dd(3)]+[s_1'*c_1_d*ang_d(1);s_2'*c_2_d*ang_d(2);s_3'*c_3_d*ang_d(3)]-[s_1_d'*s_1_d;s_2_d'*s_2_d;s_3_d'*s_3_d];
acc=[s_1';s_2';s_3']^(-1)*item;
end

