clear all; close all;

profile on;

N = 1000000;
theta = -pi + rand(N,7) * 2 * pi;
% cpp_time = 0;
% matlab_time = 0;

for i = 1:N
    get_iiwa_FK_mex( theta(i,:), eye(4) );
%     get_iiwa_FK_all_joints_mex( theta(i,:), eye(4) );
end

for i = 1:N
%     iiwa_FK_symbolic_WE(theta(i,:), eye(4));
%     iiwa_FK_all_joints_symbolic(theta(i,:), eye(4));
    
    get_iiwa_FK_all_joints_mex( theta(i,:), eye(4) );
    
end

% err = 0;
% for i = 1:N
% %     T1 = iiwa_FK_symbolic_WE(theta(i,:), eye(4));
% %     T2 = get_iiwa_FK_mex( theta(i,:), eye(4) );
% 
% %     T1 = iiwa_FK_all_joints_symbolic(theta(i,:), eye(4));
% %     T2 = get_iiwa_FK_all_joints_mex( theta(i,:), eye(4) );
% 
%     T1 = get_iiwa_FK_mex( theta(i,:), eye(4) );
%     T2 = get_iiwa_FK_all_joints_mex( theta(i,:), eye(4) );
%     T2 = T2(33:36,:);
%     
%     err = err + sum(sum(T1-T2));
% end
% disp(err)

profile viewer;

% disp('cpp time per call')
% cpp_time/N
% disp('matlab time per call')
% matlab_time/N
% disp('factor: matlab/cpp')
% matlab_time / cpp_time