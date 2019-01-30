clear all; close all; clc;

data = dlmread('multi_IK_2.csv');
joints = data(:,end-6:end);
S = data(:,7);
% for i = 1:size(joints,1)
%     disp('s');
%     disp(joints(i,1));
%     get_iiwa_GeoJac(joints(i,2:end))
% end


% [2 4 5]
% [4 6 7]
% [6 8 9]

nominal_idx = 1 +1;
drum = [];
for nominal_idx = 1:8
    th1 = joints(nominal_idx,:);
    T1 = get_iiwa_FK_all_joints_mex(th1, eye(4));
    J1 = get_iiwa_GeoJac(T1);
    for i = 1:size(joints,1)
        th2a = joints(i,:);    
        T2a = get_iiwa_FK_all_joints_mex(th2a, eye(4));
        J2a = get_iiwa_GeoJac(T2a);

        dx = data(i,1:3)-data(nominal_idx,1:3);
        dxp = J1 * (th2a-th1)';

        disp('############################')
        disp('s, corr, inf_norm, ||dx||, dot')
        disp([S(i),corr2(J1,J2a),norm(th1-th2a,inf)/pi*180, norm(data(i,1:3)-data(nominal_idx,1:3)),dot(dx,dxp(4:6,1)')])
        drum = [drum; [S(i),corr2(J1,J2a),norm(th1-th2a,inf)/pi*180, norm(data(i,1:3)-data(nominal_idx,1:3)),dot(dx,dxp(4:6,1)')]];
    %     disp(dx)
    %     disp(dxp(4:6,1)')
    %     disp(dot(dx,dxp(4:6,1)'))
    end
end













% th1 = joints(25,:);
% th2a = joints(26,:);
% th2b = joints(27,:);
% 
% T1 = get_iiwa_FK_all_joints_mex(th1, eye(4));
% T2a = get_iiwa_FK_all_joints_mex(th2a, eye(4));
% T2b = get_iiwa_FK_all_joints_mex(th2b, eye(4));
% 
% 
% J1 = get_iiwa_GeoJac(T1);
% J2a = get_iiwa_GeoJac(T2a);
% J2b = get_iiwa_GeoJac(T2b);
% 
% % J1   = J1(1:3,:);
% % J2a = J2a(1:3,:);
% % J2b = J2b(1:3,:);
% 
% J1 * J2a'
% 
% J1 * J2b'
% 
% J1
% J2a
% J2b
% 
% corr2(J1,J2a)
% corr2(J1,J2b)