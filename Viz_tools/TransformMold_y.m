function TransformMold_y(hObject, ~, ~)

global mold_v_transformed;
global mold_f;
global mold_v;
global mold_n;
global mold_n_transformed;
global p_mold;
global joint_angles;
global joint_angles_group;
global xyz_bxbybz_groups;
global group_idx;
global plot_traj_arr;
global idx;
global plot_failed_traj_arr;
global rob_T_part;
global msgbx;

rob_T_part(2,4) = get(hObject,'Value')*1000;

[mold_v_transformed] = apply_transformation(mold_v,rob_T_part);
rob_T_part_n = rob_T_part;
rob_T_part_n(1:3,4) = 0;
[mold_n_transformed] = apply_transformation(mold_n,rob_T_part_n);
delete(p_mold);
p_mold = patch('Faces',mold_f,'Vertices',mold_v_transformed,'FaceVertexCData',...
    [0.8,0.8,0.8],'FaceColor',[0.3,0.3,0.3],'EdgeColor','none');
joint_angles = [];
joint_angles_group = [];
xyz_bxbybz_groups = [];
group_idx = [];
idx = 1;
delete(plot_traj_arr);
delete(plot_failed_traj_arr);
msgbx.String = strcat('MOLD MOVED...SELECT START POINT');

end