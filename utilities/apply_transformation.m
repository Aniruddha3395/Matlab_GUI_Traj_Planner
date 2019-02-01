function [transformed_data] = apply_transformation(data,transformation_matrix)
% NOTE: Homogeneous Tranformation Matrix (4x4)

% putting data in [x, y, z, 1]' format
data_with_fourth_row = [data';ones(1,size(data,1))];
transformed_data_with_fourth_row = [transformation_matrix*data_with_fourth_row]';
transformed_data = transformed_data_with_fourth_row(:,1:3);
end