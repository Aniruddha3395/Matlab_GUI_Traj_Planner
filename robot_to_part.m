%% Compute Transformation of Points and Euler Angles from part to Robot

function [points,bx,by,bz] = robot_to_part(points,bx,by,bz)
global rob_T_part;
xyz_abc = [];

part_points = [571.57,571.50,13.61;
    285.78,571.50,46.74;
    285.78,0,46.74;
    0,285.75,112.05
    ];

robot_points = [227.14,-324.13,24.24;
    521.57,-324.85,57.33;
    519.77,248.20,56.55;
    800.02,-35.91,123.16
    ];

[R,t] = point_pairs_transformation(part_points,robot_points);
T = eye(4);
T(1:3,1:3) = R;
T(1:3,4) = t;
rob_T_part = T;
Rdash = T;
Rdash(1:3,4) = [0;0;0];
for i=1:size(points)
    homogeneous_pt = zeros(1,4);
    homogeneous_pt(1,1:3) = points(i,:);
    homogeneous_pt(1,4) = 1;
    transformed_pt = T*homogeneous_pt(1,:)';
    pt = transformed_pt';
    points(i,:) = pt(1,1:3);
end

if size(bx,1)~=0
    for i=1:size(bx)
        homogeneous_norm = zeros(1,4);
        homogeneous_norm(1,1:3) = bx(i,:);
        homogeneous_norm(1,4) = 1;
        transformed_norm = Rdash*homogeneous_norm(1,:)';
        norm = transformed_norm';
        bx(i,:) = norm(1,1:3);
    end
end

if size(by,1)~=0
    for i=1:size(by)
        homogeneous_norm = zeros(1,4);
        homogeneous_norm(1,1:3) = by(i,:);
        homogeneous_norm(1,4) = 1;
        transformed_norm = Rdash*homogeneous_norm(1,:)';
        norm = transformed_norm';
        by(i,:) = norm(1,1:3);
    end
end

if size(bz,1)~=0
    for i=1:size(bz)
        homogeneous_norm = zeros(1,4);
        homogeneous_norm(1,1:3) = bz(i,:);
        homogeneous_norm(1,4) = 1;
        transformed_norm = Rdash*homogeneous_norm(1,:)';
        norm = transformed_norm';
        bz(i,:) = norm(1,1:3);
    end
end

% This function computes the Rotation and Translation matrix.
    function [R,T] = point_pairs_transformation(A,B)
        centroid_A = mean(A);
        centroid_B = mean(B);
        A = A - centroid_A;
        B = B - centroid_B;
        H = A'*B;
        [U,S,V] = svd(H);
        R = V*[1 0 0;0 1 0;0 0 det(V*U')]*U';
        if det(R)>0
            T = -R*centroid_A' + centroid_B';
        else
            fprintf('Determinant of rotation matrix is negative...')
        end
    end

end