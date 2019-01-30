%% Compute bx, by, bz from Euler Angles

function [bx,by,bz] = euler_to_bxbybz(abc)

% input euler alpha,beta,gamma for ZYX (in radians)
bx =zeros(size(abc,1),3);
by =zeros(size(abc,1),3);
bz =zeros(size(abc,1),3);

for i=1:size(abc,1)
    R = eul2rotm(abc(i,:));
    bx(i,:) = [R(1,1),R(2,1),R(3,1)];
    by(i,:) = [R(1,2),R(2,2),R(3,2)];
    bz(i,:) = [R(1,3),R(2,3),R(3,3)];
end

end