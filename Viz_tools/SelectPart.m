function SelectPart(hObject, ~, ~)

global store_file_str;
global mold_v_transformed;
global mold_f;
global mold_n_transformed;
global p_mold;
global joint_angles;
global joint_angles_group;
global xyz_bxbybz_groups;
global group_idx;
global plot_traj_arr;
global idx;
global plot_failed_traj_arr;

file_idx =  get(hObject,'Value');
mold_base = strcat('CAD_stl/',store_file_str{file_idx});
[mold_v, mold_f, mold_n, ~] = stlRead(mold_base);
[mold_v_transformed,mold_n_transformed] = robot_to_part(mold_v,mold_n,[],[]);
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

end