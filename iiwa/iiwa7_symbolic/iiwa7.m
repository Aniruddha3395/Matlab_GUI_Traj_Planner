% FK computation optimized by analytical solution
% IK computaiton improved by removing unneccessary instances


classdef iiwa7
    properties
        a % DH
        d % DH
        alpha % DH
        base_T % base of robot
        ROBOT_LINKs  % stick figure definition of links by point coordinates on the link
        ROBOT_LINK_DISTs % length of links
        ROBOT_TOOL
        ROBOT_TOOL_FMM
        ROBOT_TOOL_LINK_DISTs;
        DOF % number of DOF/joints for the robot
        Links % number of links for the robot
        Tjj % array of x y z A B C transformation information between each parent and child link
        DH_available % boolean flag to decide if FK/IK will be computed through DH parameters
        JOINT_BOUNDS;
        MANIPULATOR_MAX_JOINT_SPEED;
        DISCRETE_JOINT_BOUNDS;
        end_effector_T; % transformation matrix that contains pose of e_e
        matlab_robot;
        robotConfig;
        ik;
        wT1;wT2;wT3;wT4;wT5;wT6;wT7;wT8;wT9;
        T12;T23;T34;T45;T56;T67;T78;T89;
        in_tool;
        robot_ree_T_tee;
        robot_spheres;
        tool_spheres;
    end
    
    methods 
        % Initialization method for a robot
        function self = iiwa7(in_base_T,in_TOOL) % base_T = 4x4 homogenous transformation matrix for manipulator base
            self.in_tool = in_TOOL;
            self.robot_spheres = {};
            self.tool_spheres = [];
            
            self.matlab_robot = importrobot('iiwa7.urdf');
            self.robotConfig = self.getRobotConfigFromJointAngles([0 0 0 0 0 0 0]);
            self.ik = robotics.InverseKinematics('RigidBodyTree',self.matlab_robot);
            self.wT1 = [
                 [1     0     0     0];
                 [0     1     0     0];
                 [0     0     1     0];
                 [0     0     0     1];   
            ];
            self.wT2 = [
                [1.0000         0         0         0];
                [     0    1.0000         0         0];
                [     0         0    1.0000    0.1500];
                [     0         0         0    1.0000];      
            ];
            self.wT3 = [
               [-1.0000    0.0000   -0.0000         0];
               [-0.0000   -0.0000    1.0000         0];
               [      0    1.0000    0.0000    0.3400];
               [      0         0         0    1.0000];     
            ];
            self.wT4 = [
                [1.0000   -0.0000    0.0000    0.0000];
                [0.0000    1.0000    0.0000   -0.0000];
               [-0.0000         0    1.0000    0.5500];
               [      0         0         0    1.0000];    
            ];
            self.wT5 = [
                [1.0000    0.0000    0.0000    0.0000];
                [0.0000    0.0000   -1.0000   -0.0000];
               [-0.0000    1.0000    0.0000    0.7400];
               [      0         0         0    1.0000];    
            ];
            self.wT6 = [
               [-1.0000    0.0000    0.0000    0.0000];
               [-0.0000   -1.0000    0.0000    0.0000];
                [0.0000    0.0000    1.0000    0.9500];
                [     0         0         0    1.0000];    
            ];
            self.wT7 = [
                [-1.0000    0.0000   -0.0000    0.0000];
                [-0.0000    0.0000    1.0000   -0.0607];
                [0.0000    1.0000   -0.0000    1.1400];
                [     0         0         0    1.0000];  
            ];
            self.wT8 = [
                [1.0000   -0.0000    0.0000    0.0000];
                [0.0000    1.0000    0.0000    0.0000];
               [-0.0000    0.0000    1.0000    1.2210];
                [     0         0         0    1.0000];    
            ];
            self.wT9 = [
                1.0000   -0.0000    0.0000    0.0000;
                0.0000    1.0000    0.0000    0.0000;
               -0.0000    0.0000    1.0000    1.2660;
                     0         0         0    1.0000;    
            ];
            self.T12 = inv(self.wT1) * self.wT2;
            self.T23 = inv(self.wT2) * self.wT3;
            self.T34 = inv(self.wT3) * self.wT4;
            self.T45 = inv(self.wT4) * self.wT5;
            self.T56 = inv(self.wT5) * self.wT6;
            self.T67 = inv(self.wT6) * self.wT7;
            self.T78 = inv(self.wT7) * self.wT8;
            self.T89 = inv(self.wT8) * self.wT9;
            
% %             <origin xyz="0 0 0.15" rpy="0 0 0"/>
%             self.T12 = T_from_xyzrpy(0, 0, 0.15, 0, 0, 0);
%             
% %             <origin xyz="0 0 0.19" rpy="${PI / 2}   0 ${PI}"/>
%             self.T23 = T_from_xyzrpy(0, 0, 0.19, pi/2, 0, pi);
%             
% %             <origin xyz="0 0.21 0" rpy="${PI / 2} 0 ${PI}"/>
%             self.T34 = T_from_xyzrpy(0, 0.21, 0, pi/2, 0, pi);
%             
% %             <origin xyz="0 0 0.19" rpy="${PI / 2} 0 0"/>
%             self.T45 = T_from_xyzrpy(0, 0, 0.19, pi/2, 0, 0);
%             
% %             <origin xyz="0 0.21 0" rpy="${-PI / 2} ${PI} 0"/>
%             self.T56 = T_from_xyzrpy(0, 0.21, 0, -pi/2, pi, 0);
%             
% %             <origin xyz="0 0.06070 0.19" rpy="${PI / 2} 0 0"/>
%             self.T67 = T_from_xyzrpy(0, 0.0607, 0.19, pi/2, 0, 0);
%             
% %             <origin xyz="0 0.081 0.06070" rpy="${- PI / 2} ${PI} 0"/>
%             self.T78 = T_from_xyzrpy(0, 0.081, 0.0607, -pi/2, pi, 0);
%             
% %             <origin xyz="0 0 0.045" rpy="0 0 0"/>
%             self.T89 = T_from_xyzrpy(0, 0, 0.045, 0, 0, 0);            
            
            
            
            
            self.base_T = in_base_T;
            self.DOF = 7;
            self.Links = 8;
            
            % [stick figure definition]
            self.ROBOT_LINKs = {};
%             self.ROBOT_LINKs{end+1} = [0, 0, 0, 1; 0, 0, 0.15985, 1]'./1; % Link1
%             self.ROBOT_LINKs{end+1} = [0, 0, -0.01, 1; 0, 0, 0.25968, 1]'./1;
%             self.ROBOT_LINKs{end+1} = [ 0, -0.0688, 0, 1; 0, 0.19277, 0, 1]'./1;
%             self.ROBOT_LINKs{end+1} = [ 0, 0, -0.01, 1; 0, 0, 0.2846, 1]'./1;
%             self.ROBOT_LINKs{end+1} = [ 0, -0.06878, 0, 1; 0, 0.19277, 0, 1]'./1;
%             self.ROBOT_LINKs{end+1} = [ 0, 0, -0.01, 1; 0, 0, 0.2962, 1]'./1;
%             self.ROBOT_LINKs{end+1} = [ 0, -0.11449278, 0, 1; 0, 0.09740544, 0, 1]'./1;
%             self.ROBOT_LINKs{end+1} = [ 0, 0, 0, 1; 0, 0, 0.04960116, 1]'./1;

            self.ROBOT_LINKs{end+1} = [ 0, 0, 0, 1; 0, 0, 0.15, 1]'./1; % Link1
            self.ROBOT_LINKs{end+1} = [ 0, 0, 0, 1; 0, 0, 0.16, 1]'./1;
            self.ROBOT_LINKs{end+1} = [ 0, 0, 0, 1; 0, 0.17, 0, 1]'./1;
            self.ROBOT_LINKs{end+1} = [ 0, 0, 0, 1; 0, 0, 0.22, 1]'./1;
            self.ROBOT_LINKs{end+1} = [ 0, 0, 0, 1; 0, 0.17, 0, 1]'./1;
            self.ROBOT_LINKs{end+1} = [ 0, 0, 0, 1; 0, 0, 0.25, 1]'./1;
            self.ROBOT_LINKs{end+1} = [ 0, 0, 0, 1; 0, 0.09, 0, 1]'./1;
            self.ROBOT_LINKs{end+1} = [ 0, 0, 0, 1; 0, 0, 0.05, 1]'./1;
            
            % [link lengths]
            self.ROBOT_LINK_DISTs = {};            
            for jid = 1:self.Links
                robot_pts = self.ROBOT_LINKs{jid};
                curr_dists = [];
                if size(robot_pts,2) > 1
                    for rpid = 1:size(robot_pts,2)-1
                        dist = compute_distance(robot_pts(1:3,rpid)', robot_pts(1:3, rpid+1)');
                        curr_dists = [curr_dists; dist];
                    end
                    self.ROBOT_LINK_DISTs{end+1} = curr_dists;
                else
                    disp('ERROR')
                end
            end
            
            
            if (size(self.in_tool,2)<1)
                self.ROBOT_TOOL = {};
                self.ROBOT_TOOL{end+1} = [ 0, 0, 0, 1000;  0, 0, 153, 1000]'./1000;
                self.ROBOT_TOOL{end+1} = [ 0, 0, 170, 1000;  -95.25, 0, 170, 1000]'./1000;

                self.ROBOT_TOOL_FMM = {};
                self.ROBOT_TOOL_FMM{end+1} = [ 0, 0, 0, 1000;  0, 0, 153, 1000]'./1000;
                self.ROBOT_TOOL_FMM{end+1} = [ 0, 0, 170, 1000;  -85.25, 0, 170, 1000]'./1000;
                
                ree_R_tee = [[0; 0; 1] , [0; 1; 0], [-1;0;0]]; % toolz = world -x, toolx = world -y, tooly = world z
                self.robot_ree_T_tee = [ree_R_tee, [-0.09525; 0; 0.170]; [0 0 0 1]];                
            else
                self.ROBOT_TOOL = self.in_tool{1};
                self.ROBOT_TOOL_FMM = self.in_tool{2};
                self.robot_ree_T_tee = self.in_tool{3};
            end
            
            % [tool link lengths]
            self.ROBOT_TOOL_LINK_DISTs = {};       
            for jid = 1:size(self.ROBOT_TOOL,2)
                tool_pts = self.ROBOT_TOOL{jid};
                curr_dists = [];
                if size(tool_pts,2) > 1
                    for rpid = 1:size(tool_pts,2)-1
                        dist = compute_distance(tool_pts(1:3,rpid)', tool_pts(1:3, rpid+1)');
                        curr_dists = [curr_dists; dist];
                    end
                    self.ROBOT_TOOL_LINK_DISTs{end+1} = curr_dists;
                else
                    disp('ERROR')
                end
            end
            
            % link to link transformations. x y z A B C. [A B C = x y z euler angles]
            self.Tjj = [
                [0   0.0    0.15   0    0  0]; % T01
                [0   0.0    0.19   pi/2 0  pi];
                [0   0.21   0      pi/2 0  pi];
                [0   0.0    0.19   pi/2 0  0 ];
                [0   0.21   0      pi/2 0  pi];
                [0   0.0607 0.19   pi/2 0  0];
                [0   0.081  0.0607 pi/2 0  pi];
                [0   0.0    0.045  0    0  0] % T7E
                ];
            
            self.JOINT_BOUNDS = [  -170.0/180.0 * pi, 170.0/180.0 * pi; % J1
                                   -120.0/180.0 * pi, 120.0/180.0 * pi; % J2
                                   -170.0/180.0 * pi, 170.0/180.0 * pi; % J3
                                   -120.0/180.0 * pi, 120.0/180.0 * pi; % J4
                                   -170.0/180.0 * pi, 170.0/180.0 * pi; % J5
                                   -120.0/180.0 * pi, 120.0/180.0 * pi; % J6
                                   -175.0/180.0 * pi, 175.0/180.0 * pi; % J7
                                   ];
%             self.JOINT_BOUNDS = [  -35.0/180.0 * pi, 10.0/180.0 * pi; % J1
%                                    5.0/180.0 * pi, 30.0/180.0 * pi; % J2
%                                    -2.0/180.0 * pi, 2.0/180.0 * pi; % J3
%                                    -100.0/180.0 * pi, -30.0/180.0 * pi; % J4
%                                    -10.0/180.0 * pi, 45.0/180.0 * pi; % J5
%                                    70.0/180.0 * pi, 120.0/180.0 * pi; % J6
%                                    50.0/180.0 * pi, 90.0/180.0 * pi; % J7
%                                    ]; 
                               
           self.MANIPULATOR_MAX_JOINT_SPEED = 50 * pi / 180; % rad / sec
           self.DISCRETE_JOINT_BOUNDS = [];

        end
        
        function [ rot_mat ] = ROTZ( self, theta )
        %ROTZ -
        %   Calculate 3*3 rotation matrix along z axis with theta radian
            rot_mat = [cos(theta) -sin(theta) 0 0; sin(theta) cos(theta) 0 0;...
                0 0 1 0; 0 0 0 1];
        end
        
        % returns list of transformation matrices for pose of each link
        function T = fwd_kin_all_joints(self, targetJoints)
            % analytical

            wT1 = self.base_T * self.wT1; 
            wT2 = wT1 * self.T12 * self.ROTZ(targetJoints(1)) ;
            wT3 = wT2 * self.T23 * self.ROTZ(targetJoints(2));
            wT4 = wT3 * self.T34 * self.ROTZ(targetJoints(3));
            wT5 = wT4 * self.T45 * self.ROTZ(targetJoints(4));
            wT6 = wT5 * self.T56 * self.ROTZ(targetJoints(5));
            wT7 = wT6 * self.T67 * self.ROTZ(targetJoints(6));
            wT8 = wT7 * self.T78 * self.ROTZ(targetJoints(7));            
            wT9 = wT8 * self.T89;
            T = {wT1,wT2,wT3,wT4,wT5,wT6,wT7,wT8,wT9};            
            
            % using matlab toolbox    
            
%             T = self.get_iiwa_FK(targetJoints);
            % returns cell array T of size 9
            % T{1} through T{8} represents the transformation for 
            % Link 1 ~ Link 8
            % T{9} represents transformation for end-effector
            % T{9} should be applied to tool
        end
        
%         function J = get_jacobian(self, targetJoints,varargin)
%             matlab_jac = '';
%             use_tool = '';
%             use_point_on_curve = '';
%             
%             if(size(varargin,2)>0)
%                 matlab_jac = varargin{1};
%             end
%             if(size(varargin,2)>1)
%                use_tool = varargin{2};
%             end            
%             if(size(varargin,2)>2)
%                use_point_on_curve = varargin{3};
%             end            
%             
%             if(strcmp(matlab_jac,'MRT')) % calculate jacobian using MATLAB robotics toolbox
%                 for i = 1:size(targetJoints,2)
%                     self.robotConfig(i).JointPosition = targetJoints(i);
%                 end
%                 J = geometricJacobian(self.matlab_robot,self.robotConfig,'iiwa_link_ee');                    
%                 return;
%             end            
%             
%             
%             all_transf_mat = self.fwd_kin_all_joints(targetJoints);
% %             T = {};
% %             T{end+1} = eye(4);
% %             for i = 1:size(all_transf_mat,2)
% %                 T{end+1} = all_transf_mat{i};
% %             end
%             T = all_transf_mat;
%             T_ee = T{end};
%             if(strcmp(use_tool,'tool'))
%                 T_ee = T_ee * self.robot_ree_T_tee;
%             end
%             if(strcmp(use_point_on_curve,'pc'))
%                 T_ee(1:3,4) = varargin{4};
%             end            
%             J = zeros(6,self.DOF);
%             o_n = T_ee(1:3,4);
%             for i =1:self.DOF
%                 vec_z = [0;0;1];
%                 Ti = T{i+1};
%                 Ri = Ti(1:3,1:3);
%                 
%                 z_im1 = Ri * vec_z;
%                 o_im1 = Ti(1:3,4);
%                 dir_vec = o_n-o_im1;
%                 
%                 z_im1 = z_im1 / norm(z_im1);
%                 dir_vec = dir_vec / norm(dir_vec);
%                 
%                 J(:,i) = [cross(z_im1,(o_n-o_im1)); z_im1];
%             end
% 
%             pose = pose_from_T(T_ee);
% 
%             phi = pose(4); theta = pose(5); psi = pose(6);
%             T_gamma = [
%                 0   -sin(phi) cos(phi)*sin(theta); 
%                 0    cos(phi) sin(phi)*sin(theta);
%                 1       0     cos(theta)
%                 ];
%             TA = [eye(3), zeros(3,3); zeros(3,3), T_gamma];
%             JG = J;
%             JA = inv(TA + 0.0001) * JG;
%             J = JA;
%             
%         end
        
        function J = get_jacobian(self, targetJoints)
%             J = [dx/dt;dy/dt;dz/dt;drz/dt;dry/dt;drx/dt;];
            J = iiwa_analytical_jacobian(targetJoints);
        end
        
        function J = get_jacobian_with_base(self, targetJoints,brzyx)
%             J = [dx/dt;dy/dt;dz/dt;drz/dt;dry/dt;drx/dt;];
            J = with_base_iiwa_analytical_jacobian(brzyx,targetJoints);
        end        
        
        function T = mrt_fwd_kin_all_joints(self, targetJoints)
            
            % using matlab toolbox    
            T = self.get_iiwa_FK(targetJoints);

            % returns cell array T of size 9
            % T{1} through T{8} represents the transformation for 
            % Link 1 ~ Link 8
            % T{9} represents transformation for end-effector
            % T{9} should be applied to tool
        end        
        
        
        % returns IK solution closest to nominal_angles for given T
        function joint_angles_ik = inv_kin(self, T, nominal_angles)
            joint_angles_ik = self.get_iiwa_IK(T, nominal_angles);  
            
%             % using tracIK
%             pose = [T(1:3,4)',T(1,1:3),T(2,1:3),T(3,1:3)];
%             ik_attempts = 0;
%             while ik_attempts < 10
%                 joint_angles_ik = iiwa7_IK(pose, nominal_angles);
%                 if joint_angles_ik(1) > -999
%                     break;
%                 else
%                     joint_angles_ik = [];
%                 end
%                 ik_attempts = ik_attempts + 1;
%             end
            
            
        end         
        
        
        function robotConfig = getRobotConfigFromJointAngles(self, joint_angles)
            robotConfig = self.matlab_robot.randomConfiguration;
            for i = 1:size(joint_angles,2)
                robotConfig(i).JointPosition = joint_angles(i);
            end    
        end

        function T = get_iiwa_FK(self, joint_angles)
%             robotConfig = self.getRobotConfigFromJointAngles(joint_angles);
            for i = 1:7
                self.robotConfig(i).JointPosition = joint_angles(i);
            end
            T = {};
            for i = 1:self.matlab_robot.NumBodies
                tform = getTransform(self.matlab_robot,self.robotConfig,self.matlab_robot.Bodies{i}.Name,self.matlab_robot.BaseName);
                
                tform = self.base_T * tform;
                
                T{end+1} = tform;
        %         disp(strcat("transform from base to ", robot.Bodies{i}.Name))
        %         disp(tform)
            end    
        end

        function T = get_iiwa_FK_between_two_links(self, joint_angles, leaf, root)
%             robotConfig = getRobotConfigFromJointAngles(self.matlab_robot, joint_angles);
            for i = 1:7
                self.robotConfig(i).JointPosition = joint_angles(i);
            end            
            T = getTransform(self.matlab_robot,self.robotConfig,leaf,root);
        end

        function joint_angles = get_iiwa_IK(self, T, initialguess)
%             ik = robotics.InverseKinematics('RigidBodyTree',self.matlab_robot);
            % Weight for pose tolerances, specified as a 6-element vector. 
            % The first three elements correspond to the weights on the error in orientation for the desired pose. 
            % The last three elements correspond to the weights on the error in xyz position for the desired pose.
            weights = [0.25 0.25 0.25 1 1 1];
%             initialguess = self.getRobotConfigFromJointAngles(initialguess);
            for i = 1:7
                self.robotConfig(i).JointPosition = initialguess(i);
            end                        
            [configSoln,solnInfo] = self.ik('iiwa_link_ee',T,weights,self.robotConfig);
            joint_angles = [0 0 0 0 0 0 0];
            for i = 1:size(configSoln,2)
                joint_angles(i) = configSoln(i).JointPosition;
            end
%             show(self.matlab_robot,configSoln);    
        end        

        
    end
    
end