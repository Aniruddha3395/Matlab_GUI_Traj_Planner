function LastTrajViz(~,~)

global h0 h1 h2 h3 h4 h5 h6 h7 h_tool;
global take_video show_tool;
global joint_angles;
global home_pos;
global kill_sig;
global cl_last_data;
global cl_data;
global cl_failed_traj;
global move_rob_all;
global msgbx;
global prev_traj_status;
global select_part;
global select_tool;
global move_part_x;
global move_part_y;
global move_part_z;
global move_part_Rz;
global move_part_Ry;
global move_part_Rx;
global repl_path_l;
global repl_path_r;
global robot1;
global select_robot;

if take_video
    vid = VideoWriter('video1.mp4');
    open(vid);
end

hold on;
temp_msgstring = msgbx.String;
if ~isempty(joint_angles)
    msgbx.String = 'ROBOT MOTION OVER LAST SUCCESSFUL TRAJECTORY...';
    cl_last_data.Enable = 'off';
    cl_data.Enable = 'off';
    cl_failed_traj.Enable = 'off';
    move_rob_all.Enable = 'off';
    select_robot.Enable = 'off';
    select_part.Enable = 'off';
    select_tool.Enable = 'off';
    move_part_x.Enable = 'off';
    move_part_y.Enable = 'off';
    move_part_z.Enable = 'off';
    move_part_Rz.Enable = 'off';
    move_part_Ry.Enable = 'off';
    move_part_Rx.Enable = 'off';
    repl_path_l.Enable = 'off';
    repl_path_r.Enable = 'off';
    
    for i= 1:size(joint_angles,1)
        if kill_sig
            kill_sig=false;
            msgbx.String = temp_msgstring;
            break;
        end
        if strcmp(robot1.rob_type,'iiwa7') 
            FK_T = get_iiwa7_FK_all_joints_mex(joint_angles(i,:),eye(4));
        elseif strcmp(robot1.rob_type,'iiwa14') 
            FK_T = get_iiwa14_FK_all_joints_mex(joint_angles(i,:),eye(4));
        end
        FK_T(1:3,4) = FK_T(1:3,4).*1000;
        FK_T(5:7,4) = FK_T(5:7,4).*1000;
        FK_T(9:11,4) = FK_T(9:11,4).*1000;
        FK_T(13:15,4) = FK_T(13:15,4).*1000;
        FK_T(17:19,4) = FK_T(17:19,4).*1000;
        FK_T(21:23,4) = FK_T(21:23,4).*1000;
        FK_T(25:27,4) = FK_T(25:27,4).*1000;
        FK_T(29:31,4) = FK_T(29:31,4).*1000;
        
        set(h0,'matrix',FK_T(1:4,:));
        set(h1,'matrix',FK_T(5:8,:));
        set(h2,'matrix',FK_T(9:12,:));
        set(h3,'matrix',FK_T(13:16,:));
        set(h4,'matrix',FK_T(17:20,:));
        set(h5,'matrix',FK_T(21:24,:));
        set(h6,'matrix',FK_T(25:28,:));
        set(h7,'matrix',FK_T(29:32,:));
        
        if show_tool
            FK_T(33:35,4) = FK_T(33:35,4).*1000;
            set(h_tool, 'matrix', FK_T(33:36,:));
        end
        
        if take_video
            frame = getframe(gcf);
            writeVideo(vid,frame);
        end
        pause(0.001);
    end
else
    prev_traj_status = 'NO DATA AVAILABLE...';
end

%move back to home position
if strcmp(robot1.rob_type,'iiwa7') 
    FK_T = get_iiwa7_FK_all_joints_mex(home_pos,eye(4));
elseif strcmp(robot1.rob_type,'iiwa14') 
    FK_T = get_iiwa14_FK_all_joints_mex(home_pos,eye(4));
end
FK_T(1:3,4) = FK_T(1:3,4).*1000;
FK_T(5:7,4) = FK_T(5:7,4).*1000;
FK_T(9:11,4) = FK_T(9:11,4).*1000;
FK_T(13:15,4) = FK_T(13:15,4).*1000;
FK_T(17:19,4) = FK_T(17:19,4).*1000;
FK_T(21:23,4) = FK_T(21:23,4).*1000;
FK_T(25:27,4) = FK_T(25:27,4).*1000;
FK_T(29:31,4) = FK_T(29:31,4).*1000;

set(h0,'matrix',FK_T(1:4,:));
set(h1,'matrix',FK_T(5:8,:));
set(h2,'matrix',FK_T(9:12,:));
set(h3,'matrix',FK_T(13:16,:));
set(h4,'matrix',FK_T(17:20,:));
set(h5,'matrix',FK_T(21:24,:));
set(h6,'matrix',FK_T(25:28,:));
set(h7,'matrix',FK_T(29:32,:));

if show_tool
    FK_T(33:35,4) = FK_T(33:35,4).*1000;
    set(h_tool, 'matrix', FK_T(33:36,:));
end

cl_last_data.Enable = 'on';
cl_data.Enable = 'on';
cl_failed_traj.Enable = 'on';
move_rob_all.Enable = 'on';
select_robot.Enable = 'on';
select_part.Enable = 'on';
select_tool.Enable = 'on';
move_part_x.Enable = 'on';
move_part_y.Enable = 'on';
move_part_z.Enable = 'on';
move_part_Rz.Enable = 'on';
move_part_Ry.Enable = 'on';
move_part_Rx.Enable = 'on';   
repl_path_l.Enable = 'on';
repl_path_r.Enable = 'on';    
msgbx.String = temp_msgstring;
end