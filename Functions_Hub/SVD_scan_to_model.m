function [Transformation] = SVD_scan_to_model(part_pts,scan_pts)

% part to robot transformation 

centroid_part_pts = mean(part_pts);
centroid_scan_pts = mean(scan_pts);
part_pts = part_pts - centroid_part_pts;
scan_pts = scan_pts - centroid_scan_pts;

CrossCovariance_Mat = part_pts'*scan_pts;
[U,S,V] = svd(CrossCovariance_Mat);         % singular value decomposition
R = V*[1 0 0;0 1 0;0 0 det(V*U')]*U';       % to take care of reflection case due to negative eigen vectors

if det(R)>0
    T = -R*centroid_part_pts' + centroid_scan_pts';
    Transformation = [[R,T];0 0 0 1];
else
    fprintf('Determinant of rotation matrix is negative...')
end






