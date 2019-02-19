function SelectRobot(hObject,~)

global robot1;
global home_pos;
global FK_T;
global h0 h1 h2 h3 h4 h5 h6 h7;
global p0 p1 p2 p3 p4 p5 p6 p7;
global joint_angles;
global joint_angles_group;
global xyz_bxbybz_groups;
global group_idx;
global idx;
global plot_traj_arr;
global plot_failed_traj_arr;
global prev_traj_status;
global h_tool;

if hObject.Value==1
    if ~strcmp(robot1.rob_type,'iiwa7')
        robot1.rob_type='iiwa7';
        load STL_iiwa7_DATA_mm.mat;
        FK_T = get_iiwa7_FK_all_joints_mex(home_pos,eye(4));
        FK_T(1:3,4) = FK_T(1:3,4).*1000;
        FK_T(5:7,4) = FK_T(5:7,4).*1000;
        FK_T(9:11,4) = FK_T(9:11,4).*1000;
        FK_T(13:15,4) = FK_T(13:15,4).*1000;
        FK_T(17:19,4) = FK_T(17:19,4).*1000;
        FK_T(21:23,4) = FK_T(21:23,4).*1000;
        FK_T(25:27,4) = FK_T(25:27,4).*1000;
        FK_T(29:31,4) = FK_T(29:31,4).*1000;
        FK_T(33:35,4) = FK_T(33:35,4).*1000;
        delete(p0);
        delete(p1);
        delete(p2);
        delete(p3);
        delete(p4);
        delete(p5);
        delete(p6);
        delete(p7);
        p0 = patch('Faces',Link0_f,'Vertices',Link0_v,'FaceVertexCData',[0.8,0.8,0.8],...
            'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
        set(p0,'parent', h0);
        p1 = patch('Faces',Link1_f,'Vertices',Link1_v,'FaceVertexCData',[0.8,0.8,0.8],...
            'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
        set(p1,'parent', h1);
        p2 = patch('Faces',Link2_f,'Vertices',Link2_v,'FaceVertexCData',[0.8,0.8,0.8],...
            'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
        set(p2,'parent', h2);
        p3 = patch('Faces',Link3_f,'Vertices',Link3_v,'FaceVertexCData',[0.8,0.8,0.8],...
            'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
        set(p3,'parent', h3);
        p4 = patch('Faces',Link4_f,'Vertices',Link4_v,'FaceVertexCData',[0.8,0.8,0.8],...
            'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
        set(p4,'parent', h4);
        p5 = patch('Faces',Link5_f,'Vertices',Link5_v,'FaceVertexCData',[0.8,0.8,0.8],...
            'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
        set(p5,'parent', h5);
        p6 = patch('Faces',Link6_f,'Vertices',Link6_v,'FaceVertexCData',[0.8,0.8,0.8],...
            'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
        set(p6,'parent', h6);
        p7 = patch('Faces',Link7_f,'Vertices',Link7_v,'FaceVertexCData',[0.8,0.8,0.8],...
            'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
        set(p7,'parent', h7);
        hold on;
        set(h0,'matrix',FK_T(1:4,:));
        set(h1,'matrix',FK_T(5:8,:));
        set(h2,'matrix',FK_T(9:12,:));
        set(h3,'matrix',FK_T(13:16,:));
        set(h4,'matrix',FK_T(17:20,:));
        set(h5,'matrix',FK_T(21:24,:));
        set(h6,'matrix',FK_T(25:28,:));
        set(h7,'matrix',FK_T(29:32,:));
        set(h_tool, 'matrix', FK_T(33:36,:));
        joint_angles = [];
        joint_angles_group = [];
        xyz_bxbybz_groups = [];
        group_idx = [];
        idx = 1;
        delete(plot_traj_arr);
        delete(plot_failed_traj_arr);
        prev_traj_status = 'ROBOT CHANGED...';
    end
else
    if ~strcmp(robot1.rob_type,'iiwa14')
        robot1.rob_type='iiwa14';
        load STL_iiwa14_DATA_mm.mat;
        FK_T = get_iiwa14_FK_all_joints_mex(home_pos,eye(4));
        FK_T(1:3,4) = FK_T(1:3,4).*1000;
        FK_T(5:7,4) = FK_T(5:7,4).*1000;
        FK_T(9:11,4) = FK_T(9:11,4).*1000;
        FK_T(13:15,4) = FK_T(13:15,4).*1000;
        FK_T(17:19,4) = FK_T(17:19,4).*1000;
        FK_T(21:23,4) = FK_T(21:23,4).*1000;
        FK_T(25:27,4) = FK_T(25:27,4).*1000;
        FK_T(29:31,4) = FK_T(29:31,4).*1000;
        FK_T(33:35,4) = FK_T(33:35,4).*1000;
        delete(p0);
        delete(p1);
        delete(p2);
        delete(p3);
        delete(p4);
        delete(p5);
        delete(p6);
        delete(p7);
        p0 = patch('Faces',Link0_f,'Vertices',Link0_v,'FaceVertexCData',[0.8,0.8,0.8],...
            'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
        set(p0,'parent', h0);
        p1 = patch('Faces',Link1_f,'Vertices',Link1_v,'FaceVertexCData',[0.8,0.8,0.8],...
            'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
        set(p1,'parent', h1);
        p2 = patch('Faces',Link2_f,'Vertices',Link2_v,'FaceVertexCData',[0.8,0.8,0.8],...
            'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
        set(p2,'parent', h2);
        p3 = patch('Faces',Link3_f,'Vertices',Link3_v,'FaceVertexCData',[0.8,0.8,0.8],...
            'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
        set(p3,'parent', h3);
        p4 = patch('Faces',Link4_f,'Vertices',Link4_v,'FaceVertexCData',[0.8,0.8,0.8],...
            'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
        set(p4,'parent', h4);
        p5 = patch('Faces',Link5_f,'Vertices',Link5_v,'FaceVertexCData',[0.8,0.8,0.8],...
            'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
        set(p5,'parent', h5);
        p6 = patch('Faces',Link6_f,'Vertices',Link6_v,'FaceVertexCData',[0.8,0.8,0.8],...
            'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
        set(p6,'parent', h6);
        p7 = patch('Faces',Link7_f,'Vertices',Link7_v,'FaceVertexCData',[0.8,0.8,0.8],...
            'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
        set(p7,'parent', h7);
        hold on;
        set(h0,'matrix',FK_T(1:4,:));
        set(h1,'matrix',FK_T(5:8,:));
        set(h2,'matrix',FK_T(9:12,:));
        set(h3,'matrix',FK_T(13:16,:));
        set(h4,'matrix',FK_T(17:20,:));
        set(h5,'matrix',FK_T(21:24,:));
        set(h6,'matrix',FK_T(25:28,:));
        set(h7,'matrix',FK_T(29:32,:));
        set(h_tool, 'matrix', FK_T(33:36,:));
        joint_angles = [];
        joint_angles_group = [];
        xyz_bxbybz_groups = [];
        group_idx = [];
        idx = 1;
        delete(plot_traj_arr);
        delete(plot_failed_traj_arr);
        prev_traj_status = 'ROBOT CHANGED...';
    end
end


end