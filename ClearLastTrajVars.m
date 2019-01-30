function ClearLastTrajVars(~,~)

global joint_angles joint_angles_group xyz_bxbybz_groups group_idx;
global plot_traj_arr idx;

if size(group_idx,1)>=2
    joint_angles = joint_angles_group(group_idx(end-1,1):group_idx(end-1,2),:);
end
if ~isempty(joint_angles_group)
    joint_angles_group(group_idx(end,1):group_idx(end,2),:) = [];
else
    fprintf('\nNo trajectory available\n');
end
if ~isempty(xyz_bxbybz_groups)
    xyz_bxbybz_groups(group_idx(end,1):group_idx(end,2),:) = [];
end
if ~isempty(group_idx)
    idx = idx - (group_idx(end,2)-group_idx(end,1))-1;
    group_idx(end,:) = [];
end
if ~isempty(plot_traj_arr)
    delete(plot_traj_arr(end));
    plot_traj_arr(end) = [];
end



end