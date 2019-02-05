function [row_wise_dist] = dist(A,B)
% row wise distance between all pairs of A and B...
% i.e. [a1-b1;a2-b2;....an-bn]...where ai and bi are vectors

vec_dist = A-B;
row_wise_dist = vecnorm(vec_dist,2,2);

end