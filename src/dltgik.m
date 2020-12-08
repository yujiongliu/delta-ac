function [ out ] = dltgik( in )
% /***********************************************************************
% * Name:       Delta geometric inverse kinematics
% * Type:       Function
% * Author:     Yujiong Liu
% * Date:       2014.4.22
% * 
% * Input:      [IN]    ps_t    Position of the travelling plate
% * Output:     [OUT]   ang     Angles of the joints
% * Function:   Compute the inverse kinematics by geometric method
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

c_1=rot_1*[R_T;0;0]+[in(1);in(2);in(3)];
c_2=rot_2*[R_T;0;0]+[in(1);in(2);in(3)];
c_3=rot_3*[R_T;0;0]+[in(1);in(2);in(3)];

a_1=rot_1*[R_B;0;0];
a_2=rot_2*[R_B;0;0];
a_3=rot_3*[R_B;0;0];

nv_1=cross(a_1',[0,0,1]);
env_1=nv_1/norm(nv_1);
lamda_1=-(env_1*c_1)/(env_1*env_1');
cp_1=lamda_1*env_1'+c_1;
acp_1=cp_1-a_1;
dis_acp_1=norm(acp_1);
%no need rotational matrix for the first one
ang_cao_1=acos(acp_1(1)/dis_acp_1);
ang_cab_1=acos((L_A^2+dis_acp_1^2+lamda_1^2-L_F^2)/(2*L_A*dis_acp_1));
if ~isreal(ang_cab_1)
    disp('Error: Uncorrect AngleCAB 1, Please Check Input!!!');
    out_1=0;%error occurs
    return
end
out_1=ang_cao_1-ang_cab_1;

nv_2=cross(a_2',[0,0,1]);
env_2=nv_2/norm(nv_2);
lamda_2=-(env_2*c_2)/(env_2*env_2');
cp_2=lamda_2*env_2'+c_2;
acp_2=cp_2-a_2;
dis_acp_2=norm(acp_2);
%inverse rotational matrix for the second one
rot_2_i=[-0.5,sqrt(3)/2,0;-sqrt(3)/2,-0.5,0;0,0,1];   %inverse rotation
acp_2_r=rot_2_i*acp_2;
ang_cao_2=acos(acp_2_r(1)/dis_acp_2);
ang_cab_2=acos((L_A^2+dis_acp_2^2+lamda_2^2-L_F^2)/(2*L_A*dis_acp_2));
if ~isreal(ang_cab_2)
    disp('Error: Uncorrect AngleCAB 2, Please Check Input!!!');
    out_2=0;           %error occurs
    return
end
out_2=ang_cao_2-ang_cab_2;

nv_3=cross(a_3',[0,0,1]);
env_3=nv_3/norm(nv_3);
lamda_3=-(env_3*c_3)/(env_3*env_3');
cp_3=lamda_3*env_3'+c_3;
acp_3=cp_3-a_3;
dis_acp_3=norm(acp_3);
%inverse rotational matrix for the second one
rot_3_i=[-0.5,-sqrt(3)/2,0;sqrt(3)/2,-0.5,0;0,0,1];   %inverse rotation
acp_3_r=rot_3_i*acp_3;
ang_cao_3=acos(acp_3_r(1)/dis_acp_3);
ang_cab_3=acos((L_A^2+dis_acp_3^2+lamda_3^2-L_F^2)/(2*L_A*dis_acp_3));
if ~isreal(ang_cab_3)
    disp('Error: Uncorrect AngleCAB 3, Please Check Input!!!');
    out_3=0;              %error occurs
    return
end
out_3=ang_cao_3-ang_cab_3;
out=[out_1;out_2;out_3];
end

