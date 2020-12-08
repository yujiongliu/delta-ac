function [ ang_jo ] = abs2jo( abs )
% /***********************************************************************
% * Name:       Absolute value to angle
% * Type:       Function
% * Author:     Yujiong Liu
% * Date:       2015.6.5
% * 
% * Input:      [IN]    abs     Absolute value[position_1,circle_1;
% *                                            position_1,circle_1;
% *                                            position_1,circle_1;] 
% *
% * Output:     [OUT]   ang     Converted joint angle[joint#1,joint#2,joint#3]
% * Function:   Convert the absolute value of encoder to the absolute joint
% *             angle.
% ***********************************************************************/

%***********************REFERENCE****************************************
%LIMIT&ABSOLUTE NUMBER
%Axis#      Limit   Absolute Position
%1          -45     14circles   +1784
%           100     0circles    +98713

%2          -45     13circles   +120485
%           100     0circles    +87782

%3          -45     13circles   +77696
%           100     0circles    +44275

%************************COMPUTATION**************************************
%Axis#1
ang_jo(1,1)=(131072*abs(1,2)+abs(1,1)-98713)*pi*29/36/(14*131072+1784-98713)-5*pi/9;
%Axis#2
ang_jo(2,1)=(131072*abs(2,2)+abs(2,1)-87782)*pi*29/36/(13*131072+120485-87782)-5*pi/9;
%Axis#3
ang_jo(3,1)=(131072*abs(3,2)+abs(3,1)-44275)*pi*29/36/(13*131072+77696-44275)-5*pi/9;
end

