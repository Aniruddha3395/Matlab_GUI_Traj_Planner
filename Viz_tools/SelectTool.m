function SelectTool(hObject,~)

global store_file_tool_str;
global h_tool;
global p_tool;
global FK_T;
global show_tool;
global tool1;
global robot1;
global prev_traj_status;

file_idx =  get(hObject,'Value');
if ~strcmp(store_file_tool_str{file_idx},'NO_TOOL')
    ee_tool = strcat('CAD_stl/Tools/',store_file_tool_str{file_idx});
    [tool_v, tool_f, ~, ~] = stlRead(ee_tool);
    delete(p_tool);
    p_tool = patch('Faces',tool_f,'Vertices',tool_v,'FaceVertexCData',...
    [0.8,0.8,0.8],'FaceColor',[0.1,0.1,0.1],'EdgeColor','none','FaceAlpha',1);
    set(p_tool,'parent', h_tool);
    set(h_tool, 'matrix', FK_T(33:36,:));
    show_tool = true;
    robot1.robot_ree_T_tee = tool1(store_file_tool_str{file_idx});
else
    delete(p_tool);
    p_tool = patch();
    set(p_tool,'parent', h_tool);
    set(h_tool, 'matrix', FK_T(33:36,:));
    robot1.robot_ree_T_tee = tool1('NO_TOOL');
end

ClearAllVars();
prev_traj_status = 'TOOL CHANGED...';

end