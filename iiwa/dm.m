% clear all; close all;
% 
% joints = -pi + rand(1,7)*2*pi;
% % base_T = eye(4);
% % base_T = robot1.base_T;
% base_T = [
%     0.0000   -1.0000         0         0;
%     1.0000    0.0000         0         0;
%          0         0    1.0000         0;
%          0         0         0    1.0000 ];
% 
% % T = iiwa_FK_crude(joints, base_T);
% % T1 = T{end};
% tic;
% T1 = iiwa_FK_symbolic_W1(joints, base_T);
% T2 = iiwa_FK_symbolic_W2(joints, base_T);
% T3 = iiwa_FK_symbolic_W3(joints, base_T);
% T4 = iiwa_FK_symbolic_W4(joints, base_T);
% T5 = iiwa_FK_symbolic_W5(joints, base_T);
% T6 = iiwa_FK_symbolic_W6(joints, base_T);
% T7 = iiwa_FK_symbolic_W7(joints, base_T);
% T8 = iiwa_FK_symbolic_W8(joints, base_T);
% T9 = iiwa_FK_symbolic_WE(joints, base_T);
% 
% 
% % T1 = T1(33:36,:);
% toc;

% tic;
% % all_transf_mat = robot1.fwd_kin_all_joints(joints);
% % T2 = all_transf_mat{end}; % eff transf  
% 
% TX = iiwa_FK_all_joints_symbolic(joints, base_T);
% toc;
% % sum(sum(T1-T2));
% sum([sum(sum(T1-TX(1:4,:)));
% sum(sum(T2-TX(5:8,:)));
% sum(sum(T3-TX(9:12,:)));
% sum(sum(T4-TX(13:16,:)));
% sum(sum(T5-TX(17:20,:)));
% sum(sum(T6-TX(21:24,:)));
% sum(sum(T7-TX(25:28,:)));
% sum(sum(T8-TX(29:32,:)));
% sum(sum(T9-TX(33:36,:)))])



% theta = sym('th',[1 7]);
% base_T = sym('BT',[4 4]);
% T = iiwa_FK_crude(theta, base_T);
% for i = 1:9
%     T{i} = simplify(T{i});
% end
% matlabFunction(T{1},'File','iiwa_FK_symbolic_W1');
% matlabFunction(T{2},'File','iiwa_FK_symbolic_W2');
% matlabFunction(T{3},'File','iiwa_FK_symbolic_W3');
% matlabFunction(T{4},'File','iiwa_FK_symbolic_W4');
% matlabFunction(T{5},'File','iiwa_FK_symbolic_W5');
% matlabFunction(T{6},'File','iiwa_FK_symbolic_W6');
% matlabFunction(T{7},'File','iiwa_FK_symbolic_W7');
% matlabFunction(T{8},'File','iiwa_FK_symbolic_W8');
% matlabFunction(T{end},'File','iiwa_FK_symbolic_WE');
% ccode(T{end},'File','iiwa_FK_symbolic_WE.c');

% 
% T = iiwa_FK_all_joints_crude(theta, base_T);
% T = simplify(T);
% matlabFunction(T ,'File','iiwa_FK_all_joints_symbolic');
% ccode(T,'File','iiwa_FK_all_joints_symbolic.c');




% theta = sym('th',[1 7]);
% base_T = sym('BT',[4 4]);
% T = iiwa_FK_pairs(theta, base_T);
% % for i = 1:8
% %     T{i} = simplify(T{i});
% % end
% matlabFunction(T{1},'File','iiwa_FK_symbolic_T12');
% matlabFunction(T{2},'File','iiwa_FK_symbolic_T23');
% matlabFunction(T{3},'File','iiwa_FK_symbolic_T34');
% matlabFunction(T{4},'File','iiwa_FK_symbolic_T45');
% matlabFunction(T{5},'File','iiwa_FK_symbolic_T56');
% matlabFunction(T{6},'File','iiwa_FK_symbolic_T67');
% matlabFunction(T{7},'File','iiwa_FK_symbolic_T78');
% matlabFunction(T{8},'File','iiwa_FK_symbolic_T89');