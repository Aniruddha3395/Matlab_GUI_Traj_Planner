%% OPTIMIZER

global lim_on_failure;
global robot1;
global use_cpp_IK_solver;

theta_th(1) = -168*pi/180; %true angle : 170
theta_th(2) = -118*pi/180; %true angle : 120
theta_th(3) = -168*pi/180; %true angle : 170
theta_th(4) = -118*pi/180; %true angle : 120
theta_th(5) = -168*pi/180; %true angle : 170
theta_th(6) = -118*pi/180; %true angle : 120
theta_th(7) = -173*pi/180; %true angle : 175
bounds_lim = 10;
compute_traj_attempt_lim = 5;

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

% joint_config = zeros(7,1);   %start joint config
traj_successful = false;
joint_config = [-0.1321;0.1415;0.0895;-1.5916;-0.0033;1.4041;-0.0312];

for new_seed_strt = 1:compute_traj_attempt_lim
    theta_lb = theta_th';
    theta_ub = -theta_th';
    reach = 0;
    joint_angles = [];
    failed_idx = [];
    for  target_idx= 1:size(xyz_bxbybz,1)
        %compute IK for each point
        if ~use_cpp_IK_solver
            [joint_config,status] = ascent_IK( joint_config,xyz_bxbybz(target_idx,:),...
                tolerances,options, theta_lb, theta_ub );
        else
            if strcmp(robot1.rob_type,'iiwa7')
                [joint_config,status] = ascent_IK_mex(joint_config,xyz_bxbybz(target_idx,:),robot1.robot_ree_T_tee,7,theta_lb,theta_ub);
            elseif strcmp(robot1.rob_type,'iiwa14')
                [joint_config,status] = ascent_IK_mex(joint_config,xyz_bxbybz(target_idx,:),robot1.robot_ree_T_tee,14,theta_lb,theta_ub);
            end
        end
        
        if status==1
%                         fprintf('%d : success\n',target_idx);
            reach = reach + 1;
        else
%                         fprintf('%d : failure\n',target_idx);
            failed_idx = [failed_idx;target_idx];
            if size(failed_idx,1)>lim_on_failure
                break;
            end
        end
        joint_angles = [joint_angles; joint_config'];
        theta_lb = joint_config-bounds_lim*pi/180';
        theta_ub = joint_config+bounds_lim*pi/180';
        for theta_count = 1:size(theta_lb,1)
            % lower bound check
            if theta_lb(theta_count)<theta_th(theta_count)
                theta_lb(theta_count) = theta_th(theta_count);
            end
            % upper bound check
            if theta_ub(theta_count)>-theta_th(theta_count)
                theta_ub(theta_count) = -theta_th(theta_count);
            end
        end
    end
    
    if size(xyz_bxbybz,1)-reach<lim_on_failure
        %         fprintf('\n Solution verified \n');
        traj_successful = true;
        break;
    else
        %         fprintf('\n Solution does not exist...trying again \n');
        joint_config = -pi/2 + pi*rand(7,1);
    end
end
