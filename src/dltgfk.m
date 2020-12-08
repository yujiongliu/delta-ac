function [ ps_t ] = dltgfk( ang )
% /***********************************************************************
% * Name:       Delta geometric forward kinematics
% * Type:       Function
% * Author:     Yujiong Liu
% * Date:       2014.4.20
% * 
% * Input:      [IN]    ang     Angles of the joints
% * Output:     [OUT]   ps_t    Position of the travelling plate
% * Function:   Compute the forward kinematics by geometric method
% ***********************************************************************/

% PARAMETER
% /**********************************************************************/
R_B=200/1000;                %radius of the base
L_A=270/1000;                %length of the arm
L_F=800/1000;               %length of the forearm
R_T=45/1000;                %radius of the tracelling plate

% COMPUTATION
% /**********************************************************************/
rot_1=[1,0,0;0,1,0;0,0,1];                          %first frame
rot_2=[-0.5,-sqrt(3)/2,0;sqrt(3)/2,-0.5,0;0,0,1];   %second frame
rot_3=[-0.5,sqrt(3)/2,0;-sqrt(3)/2,-0.5,0;0,0,1];   %third frame
r_r=R_B-R_T;                                        %reduced radius

b_1=rot_1*[r_r+L_A*cos(ang(1));0;L_A*sin(ang(1))];
b_2=rot_2*[r_r+L_A*cos(ang(2));0;L_A*sin(ang(2))];
b_3=rot_3*[r_r+L_A*cos(ang(3));0;L_A*sin(ang(3))];  %positions of B_i

b_1b_2=b_2-b_1;
b_1b_3=b_3-b_1;
nv=cross(b_1b_2,b_1b_3);
env=nv/norm(nv);
co=[env';b_1b_2';b_1b_3'];
aug=[0;(norm(b_1b_2)^2)/2;(norm(b_1b_3)^2)/2];
b_1c_p=co\aug;
if L_F>norm(b_1c_p)
c_pcl=sqrt(L_F^2-norm(b_1c_p)^2);
c_pc=c_pcl*env;
ps_t=b_1+b_1c_p+c_pc;
else warning=1;ps_t=0;
end
end

