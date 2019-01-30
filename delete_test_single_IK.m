clc;
clear;
close all;

%test single IK for 1000 points

sample_count = 1000;
success_count = 0;
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
    
    % joint_config = -pi/2 + pi*rand(7,1);
    
%     options = optimoptions('fmincon','Algorithm', 'interior-point');
%     options.MaxIterations = 100000;
%     options.MaxFunctionEvaluations = 1e80;
%     options.OptimalityTolerance = 1e-80;
%     options.StepTolerance = 1e-100;
%     options.ConstraintTolerance = 1e-100;
%     % options.Display = 'iter';
%     options.Display = 'none';
%     options.SpecifyObjectiveGradient = false;
%     options.ObjectiveLimit = 1e-100;
    
    options = optimoptions('fmincon','Algorithm', 'interior-point');
    options.MaxIterations = 200;
    options.MaxFunctionEvaluations = 1e4;
    options.OptimalityTolerance = 1e-6;
    options.StepTolerance = 1e-6;
    options.ConstraintTolerance = 1e-6;
    % options.Display = 'iter';
    options.Display = 'none';
    options.SpecifyObjectiveGradient = false;
    options.ObjectiveLimit = 1e-6;
    
    tolerances(1) = 0.003;
    tolerances(2) = 0.0524;
    tolerances(3) = 0.0524;
    tolerances(4) = 1;
    
    theta_th(1) = -168*pi/180; %true 170
    theta_th(2) = -118*pi/180; %true 120
    theta_th(3) = -168*pi/180; %true 170
    theta_th(4) = -118*pi/180; %true 120
    theta_th(5) = -168*pi/180; %true 170
    theta_th(6) = -118*pi/180; %true 120
    theta_th(7) = -173*pi/180; %true 175
    theta_lb = theta_th';
    theta_ub = -theta_th;
    
    [joint_config,status] = ascent_IK( joint_config,xyzbxbybz,...
        tolerances,options, theta_lb, theta_ub );
    if status==1
        fprintf('%d = success\n',i);
        success_count = success_count +1;
    else
        fprintf('%d = fail\n',i);
    end
    
end

success_rate = success_count*100/sample_count;
fprintf('\nsuccess rate is %f percent\n', success_rate);

%try 1 : success rate 94.2%
%     options = optimoptions('fmincon','Algorithm', 'interior-point');
%     options.MaxIterations = 100000;
%     options.MaxFunctionEvaluations = 1e80;
%     options.OptimalityTolerance = 1e-80;
%     options.StepTolerance = 1e-100;
%     options.ConstraintTolerance = 1e-100;
%     % options.Display = 'iter';
%     options.Display = 'none';
%     options.SpecifyObjectiveGradient = false;
%     options.ObjectiveLimit = 1e-100;
%     input_joint_config(1) = (-170*pi/180) + (2*170*pi*rand(1,1)/180);
%     input_joint_config(2) = (-120*pi/180) + (2*120*pi*rand(1,1)/180);
%     input_joint_config(3) = (-170*pi/180) + (2*170*pi*rand(1,1)/180);
%     input_joint_config(4) = (-120*pi/180) + (2*120*pi*rand(1,1)/180);
%     input_joint_config(5) = (-170*pi/180) + (2*170*pi*rand(1,1)/180);
%     input_joint_config(6) = (-120*pi/180) + (2*120*pi*rand(1,1)/180);
%     input_joint_config(7) = (-175*pi/180) + (2*175*pi*rand(1,1)/180);


%try 2 : success rate 93.1%
%     options = optimoptions('fmincon','Algorithm', 'interior-point');
%     options.MaxIterations = 200;
%     options.MaxFunctionEvaluations = 1e4;
%     options.OptimalityTolerance = 1e-6;
%     options.StepTolerance = 1e-6;
%     options.ConstraintTolerance = 1e-6;
%     % options.Display = 'iter';
%     options.Display = 'none';
%     options.SpecifyObjectiveGradient = false;
%     options.ObjectiveLimit = 1e-6;
%     input_joint_config(1) = (-170*pi/180) + (2*170*pi*rand(1,1)/180);
%     input_joint_config(2) = (-120*pi/180) + (2*120*pi*rand(1,1)/180);
%     input_joint_config(3) = (-170*pi/180) + (2*170*pi*rand(1,1)/180);
%     input_joint_config(4) = (-120*pi/180) + (2*120*pi*rand(1,1)/180);
%     input_joint_config(5) = (-170*pi/180) + (2*170*pi*rand(1,1)/180);
%     input_joint_config(6) = (-120*pi/180) + (2*120*pi*rand(1,1)/180);
%     input_joint_config(7) = (-175*pi/180) + (2*175*pi*rand(1,1)/180);

%try 1 : success rate 93.7%
%     options = optimoptions('fmincon','Algorithm', 'interior-point');
%     options.MaxIterations = 200;
%     options.MaxFunctionEvaluations = 1e4;
%     options.OptimalityTolerance = 1e-6;
%     options.StepTolerance = 1e-6;
%     options.ConstraintTolerance = 1e-6;
%     % options.Display = 'iter';
%     options.Display = 'none';
%     options.SpecifyObjectiveGradient = false;
%     options.ObjectiveLimit = 1e-6;
%     input_joint_config(1) = (-168*pi/180) + (2*168*pi*rand(1,1)/180);
%     input_joint_config(2) = (-118*pi/180) + (2*118*pi*rand(1,1)/180);
%     input_joint_config(3) = (-168*pi/180) + (2*168*pi*rand(1,1)/180);
%     input_joint_config(4) = (-118*pi/180) + (2*118*pi*rand(1,1)/180);
%     input_joint_config(5) = (-168*pi/180) + (2*168*pi*rand(1,1)/180);
%     input_joint_config(6) = (-118*pi/180) + (2*118*pi*rand(1,1)/180);
%     input_joint_config(7) = (-173*pi/180) + (2*173*pi*rand(1,1)/180);
%     T = get_iiwa_FK_mex(input_joint_config,eye(4));