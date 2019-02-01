% complete jacobian

theta = sym ('theta', [1,7]);
base_T = sym('base_T',[4,4]);
Tee = iiwa_FK_all_joints_crude(theta, base_T);
Tee = Tee(33:36,1:4);
abc = rotm2eulZYX(Tee(1:3,1:3));
x = Tee(1,4); y = Tee(2,4); z = Tee(3,4);
rz = abc(1); ry = abc(2); rx = abc(3);

Dx = [];Dy = [];Dz = [];
Drx = [];Dry = [];Drz = [];
for i = 1:size(theta,2)
    dx = diff(x,theta(i));
    Dx = [Dx,dx];
    
    dy = diff(y,theta(i));
    Dy = [Dy,dy];
    
    dz = diff(z,theta(i));
    Dz = [Dz,dz];    
    
    drz = diff(rz,theta(i));
    Drz = [Drz,drz];        
    
    dry = diff(ry,theta(i));
    Dry = [Dry,dry];            
    
    drx = diff(rx,theta(i));
    Drx = [Drx,drx];     
end

J = [Dx;Dy;Dz;Drz;Dry;Drx];

tic;
matlabFunction(J,'File','iiwa_jacobian_symbolic_baseTee');
toc;
% tic;
% ccode(J,'File','iiwa_jacobian_symbolic_baseTee.c');
% toc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% jacobian for links
% theta = sym ('theta', [1,7]);
% base_T = sym('base_T',[4,4]);
% Tee = iiwa_FK_all_joints_crude(theta, base_T);
% 
% for link = 2:8
% 
%     T = Tee( (link-1)*4 + 1 : (link-1)*4 + 4, 1:4);
%     abc = rotm2eulZYX(T(1:3,1:3));
%     x = T(1,4); y = T(2,4); z = T(3,4);
%     rz = abc(1); ry = abc(2); rx = abc(3);
%     
%     Dx = [];Dy = [];Dz = [];
%     Drx = [];Dry = [];Drz = [];
%     for i = 1:link-1
%         dx = diff(x,theta(i));
%         Dx = [Dx,dx];
% 
%         dy = diff(y,theta(i));
%         Dy = [Dy,dy];
% 
%         dz = diff(z,theta(i));
%         Dz = [Dz,dz];    
% 
%         drz = diff(rz,theta(i));
%         Drz = [Drz,drz];        
% 
%         dry = diff(ry,theta(i));
%         Dry = [Dry,dry];            
% 
%         drx = diff(rx,theta(i));
%         Drx = [Drx,drx];     
%     end
% 
%     J = [Dx;Dy;Dz;Drz;Dry;Drx];    
%     
%     fname = strcat('iiwa_jacobian_symbolic_link',num2str(link));
%     tic;
%     matlabFunction(J,'File',fname);
%     toc;    
% end






