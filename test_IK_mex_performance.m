clc;
clear;
close all;

%test single IK for 1000 points

sample_count = 1000;
success_count = 0;
tic;
for i=1:sample_count
    
    input_joint_config = zeros(7,1);
    input_joint_config(1) = (-168*pi/180) + (2*168*pi*rand(1,1)/180);
    input_joint_config(2) = (-118*pi/180) + (2*118*pi*rand(1,1)/180);
    input_joint_config(3) = (-168*pi/180) + (2*168*pi*rand(1,1)/180);
    input_joint_config(4) = (-118*pi/180) + (2*118*pi*rand(1,1)/180);
    input_joint_config(5) = (-168*pi/180) + (2*168*pi*rand(1,1)/180);
    input_joint_config(6) = (-118*pi/180) + (2*118*pi*rand(1,1)/180);
    input_joint_config(7) = (-173*pi/180) + (2*173*pi*rand(1,1)/180);
    T = get_iiwa_FK_mex(input_joint_config,eye(4));
    [bx,by,bz] = euler_to_bxbybz(rotm2eul(T(1:3,1:3)));
    xyzbxbybz = [T(1,4),T(2,4),T(3,4),bx,by,bz];
    
    % xyzbxbybz =  [0.3351 ,  -0.0878 ,   0.1346  , -0.3743,    0.8588 ,   0.3499, 0.8296   , 0.1414 ,   0.5402  ,  0.4186  ,  0.4829  , -0.7692];
    joint_config =zeros(7,1);
    
    [joint_config,status] = ascent_IK_mex(joint_config,xyzbxbybz,eye(4));
    if status==1
%         fprintf('%d = success\n',i);
        success_count = success_count +1;
    else
%         fprintf('%d = fail\n',i);
    end
    
end
toc;
success_rate = success_count*100/sample_count;
fprintf('\nsuccess rate is %f percent\n', success_rate);