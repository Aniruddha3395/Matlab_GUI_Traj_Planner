%% OPTIMIZER

theta_th(1) = -168*pi/180; %true 170
theta_th(2) = -118*pi/180; %true 120
theta_th(3) = -168*pi/180; %true 170
theta_th(4) = -118*pi/180; %true 120
theta_th(5) = -168*pi/180; %true 170
theta_th(6) = -118*pi/180; %true 120
theta_th(7) = -173*pi/180; %true 175
theta_lb = theta_th';
theta_ub = -theta_th;
bounds_lim = 10;

options = optimoptions('fmincon','Algorithm', 'interior-point');
options.MaxIterations = 2000;
options.MaxFunctionEvaluations = 1e10;
options.OptimalityTolerance = 1e-18;
options.StepTolerance = 1e-10;
options.ConstraintTolerance = 1e-200;
% options.Display = 'iter';
options.Display = 'none';
options.SpecifyObjectiveGradient = false;
options.ObjectiveLimit = 1e-20;

no_samples = 1;
success_cases = 0;
avg_time = 0;
overall_strt = tic;
s_time = [];
f_time = [];
updated_X = [];
reach_points = [];
not_strt_pt = false;
for poses = 1:no_samples
    x = [0.775, -0.182, 0.0467,  0.8212,  0, 0 ,0.57066]';
    target_pts = xyz_bxbybz;
    reach = 0;
    joint_angles = [];
    failed_idx = [];
    for i = 1:size(grp_idx,1)
        joint_config = zeros(7,1);   %start joint config
        for j = grp_idx(i,1):grp_idx(i,2)
            [joint_config,status] = ascent_IK( joint_config,target_pts(j,:),...
                tolerances,options, theta_lb, theta_ub );
            if status
                if ~ismember(j, grp_idx(:,1))
                    sanity_inf_norm_check = norm(joint_angles(end,:)' - joint_config, inf);
                end
                fprintf('point %d : success\n',j);
                reach = reach + 1;
            else
                failed_idx = [failed_idx; j ];
                fprintf('point %d : failure\n',j);
            end
            joint_angles = [joint_angles; joint_config'];
            theta_lb = joint_config-bounds_lim*pi/180';
            theta_ub = joint_config+bounds_lim*pi/180';
            for theta_count = 1:size(theta_lb,1)
                if theta_lb(theta_count)<theta_th(theta_count)
                    theta_lb(theta_count) = theta_th(theta_count);
                end
                if theta_ub(theta_count)>-theta_th(theta_count)
                    theta_ub(theta_count) = -theta_th(theta_count);
                end
            end   
            
        end
    end
    
    if no_pts-reach<lim_on_failure
        fprintf('\n Solution verified \n');
        success_cases = success_cases + 1;
    else
        fprintf('\n Solution does not exist \n');
    end
    updated_X = [updated_X, [x;reach]];
end
