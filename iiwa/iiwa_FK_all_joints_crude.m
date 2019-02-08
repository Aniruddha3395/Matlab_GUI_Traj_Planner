function T = iiwa_FK_all_joints_crude(targetJoints, base_T)
% analytical
    wT1 = [
         [1     0     0     0];
         [0     1     0     0];
         [0     0     1     0];
         [0     0     0     1];   
    ];
    wT2 = [
        [1.0000         0         0         0];
        [     0    1.0000         0         0];
        [     0         0    1.0000    0.1500];
        [     0         0         0    1.0000];      
    ];
    wT3 = [
       [-1.0000    0.0000   -0.0000         0];
       [-0.0000   -0.0000    1.0000         0];
       [      0    1.0000    0.0000    0.3400];
       [      0         0         0    1.0000];     
    ];
    wT4 = [
        [1.0000   -0.0000    0.0000    0.0000];
        [0.0000    1.0000    0.0000   -0.0000];
       [-0.0000         0    1.0000    0.5500];
       [      0         0         0    1.0000];    
    ];
    wT5 = [
        [1.0000    0.0000    0.0000    0.0000];
        [0.0000    0.0000   -1.0000   -0.0000];
       [-0.0000    1.0000    0.0000    0.7400];
       [      0         0         0    1.0000];    
    ];
    wT6 = [
       [-1.0000    0.0000    0.0000    0.0000];
       [-0.0000   -1.0000    0.0000    0.0000];
        [0.0000    0.0000    1.0000    0.9500];
        [     0         0         0    1.0000];    
    ];
    wT7 = [
        [-1.0000    0.0000   -0.0000    0.0000];
        [-0.0000    0.0000    1.0000   -0.0607];
        [0.0000    1.0000   -0.0000    1.1400];
        [     0         0         0    1.0000];  
    ];
    wT8 = [
        [1.0000   -0.0000    0.0000    0.0000];
        [0.0000    1.0000    0.0000    0.0000];
       [-0.0000    0.0000    1.0000    1.2210];
        [     0         0         0    1.0000];    
    ];
    wT9 = [
        1.0000   -0.0000    0.0000    0.0000;
        0.0000    1.0000    0.0000    0.0000;
       -0.0000    0.0000    1.0000    1.2660;
             0         0         0    1.0000;    
    ];

%     T12 = inv(wT1) * wT2;
%     T23 = inv(wT2) * wT3;
%     T34 = inv(wT3) * wT4;
%     T45 = inv(wT4) * wT5;
%     T56 = inv(wT5) * wT6;
%     T67 = inv(wT6) * wT7;
%     T78 = inv(wT7) * wT8;
%     T89 = inv(wT8) * wT9;

    T12 = wT1 \ wT2;
    T23 = wT2 \ wT3;
    T34 = wT3 \ wT4;
    T45 = wT4 \ wT5;
    T56 = wT5 \ wT6;
    T67 = wT6 \ wT7;
    T78 = wT7 \ wT8;
    T89 = wT8 \ wT9;    
    
    wT1 = base_T * wT1; 
    wT2 = wT1 * T12 * ROTZ(targetJoints(1)) ;
    wT3 = wT2 * T23 * ROTZ(targetJoints(2));
    wT4 = wT3 * T34 * ROTZ(targetJoints(3));
    wT5 = wT4 * T45 * ROTZ(targetJoints(4));
    wT6 = wT5 * T56 * ROTZ(targetJoints(5));
    wT7 = wT6 * T67 * ROTZ(targetJoints(6));
    wT8 = wT7 * T78 * ROTZ(targetJoints(7));            
    wT9 = wT8 * T89;
    
    T = [wT1;wT2;wT3;wT4;wT5;wT6;wT7;wT8;wT9];
    
%     T = {wT1,wT2,wT3,wT4,wT5,wT6,wT7,wT8,wT9};            

    % using matlab toolbox    

%             T = get_iiwa_FK(targetJoints);
    % returns cell array T of size 9
    % T{1} through T{8} represents the transformation for 
    % Link 1 ~ Link 8
    % T{9} represents transformation for end-effector
    % T{9} should be applied to tool
end

function [ rot_mat ] = ROTZ( theta )
%ROTZ -
%   Calculate 3*3 rotation matrix along z axis with theta radian
    rot_mat = [cos(theta) -sin(theta) 0 0; sin(theta) cos(theta) 0 0;...
        0 0 1 0; 0 0 0 1];
end