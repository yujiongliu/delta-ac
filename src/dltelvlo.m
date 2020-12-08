function [ elvlo ] = dltelvlo( ang, ang_d )
% /***********************************************************************
% * Name:       Delta velocity of the elbow
% * Type:       Function
% * Author:     Yujiong Liu
% * Date:       2015.1.14
% * 
% * Input:      [IN]    ang     Angles of the joints
% *             [IN]    ang_d   First derivative of the joint angles
% * Output:     [OUT]   elvlo   Velocity of the elbow
% * Function:   Compute the velocity of the elbow
% ***********************************************************************/

% PARAMETER
% /**********************************************************************/
L_A=270/1000;

% COMPUTATION
% /**********************************************************************/
rot_1=[1,0,0;0,1,0;0,0,1];                          %first frame
rot_2=[-0.5,-sqrt(3)/2,0;sqrt(3)/2,-0.5,0;0,0,1];   %second frame
rot_3=[-0.5,sqrt(3)/2,0;-sqrt(3)/2,-0.5,0;0,0,1];   %third frame
elvlo(:,1)=L_A*rot_1*[-sin(ang(1));0;cos(ang(1))]*ang_d(1);
elvlo(:,2)=L_A*rot_2*[-sin(ang(2));0;cos(ang(2))]*ang_d(2);
elvlo(:,3)=L_A*rot_3*[-sin(ang(3));0;cos(ang(3))]*ang_d(3);
end

