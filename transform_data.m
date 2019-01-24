% Choose what color the CAD part needs to be displayed.
col_matrix = [0.941176, 0.972549, 1];

% part = 'trial1.stl';
[v, f, n, stltitle] = stlRead(part);
delete(gca);
close all;
[v,n,a,b] = robot_to_part( v,n,[],[] );
figure(1)
hold on;
patch('Faces',f,'Vertices',v,'FaceVertexCData',col_matrix,'FaceColor',col_matrix);
info = csvread('data_files/xyz_bxbybz_wrt_partFrame.csv');

% enable this later
[points,bx,by,bz] = robot_to_part(info(:,1:3),info(:,4:6),info(:,7:9),info(:,10:12));
cba = bxbybz_to_euler(bx,by,bz);
cba = real(cba);
xyz_cba = horzcat(points,cba);

% Plotting Global
scatter3(0,0,0,100,'d','filled','b');
hold on;
quiver3(0,0,0,1,0,0,100,'r');hold on;quiver3(0,0,0,0,1,0,100,'g');hold on;quiver3(0,0,0,0,0,1,100,'b');
hold on;
plot_tcp([1,0,0],[0,1,0],[0,0,1],[0,0,0]);
daspect([1,1,1]);
hold on;
scatter3(points(:,1),points(:,2),points(:,3),200,'*','b'); %Plot the Points
hold on;
plot_tcp(bx,by,bz,points);

% write data to file
csvwrite('data_files/xyz_eul_wrt_robotFrame.csv',xyz_cba);
csvwrite('data_files/xyz_bxbybz_wrt_robotFrame.csv',[points,bx,by,bz]);
