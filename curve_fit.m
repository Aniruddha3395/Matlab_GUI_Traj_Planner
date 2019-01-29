%% Curve Fitting Function
% This Function Fits a curve between two points by lienarly interpolating a
% line and correcting it to lie on the surface.

% It takes the Start and End Point Vector as input arguments

function [points,normals,faces] = curve_fit(strt_pt,end_pt,v,f,n)

% Generating a Line from Start and End Point with given Resolution
global resolution; %Set the distance between two points

lin_curve = end_pt - strt_pt;
lin_curve_mag = norm(lin_curve);
dir_vec = lin_curve / lin_curve_mag; %Unit Vector along that line
num_seg = lin_curve_mag / resolution; %Number of Segments in line
points = zeros(floor(num_seg),size(strt_pt,2));
for i=1:num_seg
    points(i,:) = (strt_pt + dir_vec.*(resolution*i));
end

%% Filtering the faces which lie on surface only
idx_face = zeros(size(n,1),1);
idx_face_counter = 1;
for i=1:size(f)
    if n(i,3)>0       % positive value....equivalent to top surface
        idx_face(idx_face_counter,1)=i;
        idx_face_counter = idx_face_counter + 1;
    end
end
idx_face = idx_face(1:idx_face_counter-1,:);

%% Mapping Points to the 3D Surface

% This section takes a point on the line, shifts the point and triangle
% to XY plane
% Then checks if point lies in the bounds. If it does, it dictates
% equation of plane of triangle in 3D and solves for it finding the new
% coordinates and stores it in matrix.

xyz = [];
normals = [];
faces = [];
for j=1:size(idx_face)
    p1 = v(f(idx_face(j),1),:);
    p2 = v(f(idx_face(j),2),:);
    p3 = v(f(idx_face(j),3),:);
    triangle = [p1;p2;p3;p1];
    % Check if the all grid points lie inside any triangle for face j.
    in = inpolygon(points(:,1),points(:,2),triangle(:,1),triangle(:,2));
    % Find the row numbers of elements that actually lie inside.
    k = find(in);
    
    if isempty(k)
    else
        faces = [faces;f(idx_face(j),:)];
        temp = [points(k,1),points(k,2)];
        normal = n(idx_face(j),:) / norm(n(idx_face(j),:));
        for norm_cnt = 1:size(k)
            normals = [normals; normal];
        end
        % Transfer the point to the plane
        a = ((p2(2)-p1(2))*(p3(3)-p1(3)))-((p3(2)-p1(2))*(p2(3)-p1(3)));
        b = ((p2(3)-p1(3))*(p3(1)-p1(1)))-((p3(3)-p1(3))*(p2(1)-p1(1)));
        c = ((p2(1)-p1(1))*(p3(2)-p1(2)))-((p3(1)-p1(1))*(p2(2)-p1(2)));
        d = -(a*p1(1))-(b*p1(2))-(c*p1(3));
        
        for count = 1:size(k)
            zval = ((-d-(a*temp(count,1))-(b*temp(count,2)))/c);
            xyz = [xyz;[temp(count,1)],[temp(count,2)],[zval]];
        end
    end
end

%% Reorder the Points in xyz according to points-array
xyz_temp = [];
normals_temp = [];
for i=1:size(points)
    for j=1:size(xyz)
        if abs(points(i,1)-xyz(j,1))<0.001 && abs(points(i,2)-xyz(j,2))<0.001
            xyz_temp = [xyz_temp;xyz(j,:)];
            normals_temp = [normals_temp; normals(j,:)];
        end
    end
end
points = xyz_temp;
normals = normals_temp;

end