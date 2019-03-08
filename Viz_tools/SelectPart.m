function SelectPart(hObject, ~)

global store_file_str;
global mold_v_transformed;
global mold_f;
global mold_n_transformed;
global p_mold;
global mold_v;
global mold_n;
global rob_T_part;
global original_rob_T_part;
global move_part_x;
global move_part_y;
global move_part_z;
global move_part_Rz;
global move_part_Ry;
global move_part_Rx;
global prev_traj_status;

file_idx =  get(hObject,'Value');
mold_base = strcat('CAD_stl/Molds/',store_file_str{file_idx});
[mold_v, mold_f, mold_n, ~] = stlRead(mold_base);
move_part_x.Value = original_rob_T_part(1,4)/1000;
move_part_y.Value = original_rob_T_part(2,4)/1000;
move_part_z.Value = original_rob_T_part(3,4)/1000;
eul_an = rotm2eul(original_rob_T_part(1:3,1:3));
move_part_Rz.Value = eul_an(1);
move_part_Ry.Value = eul_an(2);
move_part_Rx.Value = eul_an(3);
rob_T_part = original_rob_T_part;
[mold_v_transformed,mold_n_transformed] = robot_to_part(mold_v,mold_n,[],[]);
delete(p_mold);
p_mold = patch('Faces',mold_f,'Vertices',mold_v_transformed,'FaceVertexCData',...
    [0.8,0.8,0.8],'FaceColor',[0.3,0.3,0.3],'EdgeColor','none');
ClearAllVars();
prev_traj_status = 'MOLD CHANGED...';

end