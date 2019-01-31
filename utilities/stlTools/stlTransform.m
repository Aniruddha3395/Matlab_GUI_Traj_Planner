function [v,n] = stlTransform(vi,ni,T)

    v = [vi, ones(size(vi,1),1)];
    v = v';
    v = T * v;
    v = v';
    v(:,4) = [];

    n = ni';
    n = T(1:3,1:3) * n;
    n = n';
    
    
end