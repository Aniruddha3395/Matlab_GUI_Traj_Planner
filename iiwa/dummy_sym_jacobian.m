% 
% 
% theta = [0 0 0 0 0 0 0];
% % theta = [-pi+rand(1)*pi -pi+rand(1)*pi -pi+rand(1)*pi -pi+rand(1)*pi -pi+rand(1)*pi -pi+rand(1)*pi -pi+rand(1)*pi];
% tic;
% J1 = iiwa_analytical_jacobian(theta);
% toc;
% 
% J2 = robot2.get_jacobian(theta);
% brzyx = [-pi/2 0 0];
% J3 = with_base_iiwa_analytical_jacobian(brzyx,theta);
% 
% % J3 = robot1.get_jacobian(theta, 'MRT');
% 
% 
% T_ee = robot1.fwd_kin_all_joints(theta);
% T_ee = T_ee{end};
% pose = pose_from_T(T_ee);
% phi = pose(4); theta = pose(5); psi = pose(6);
% T_gamma = [
% 0   -sin(phi) cos(phi)*sin(theta); 
% 0    cos(phi) sin(phi)*sin(theta);
% 1       0     cos(theta)
% ];
% TA = [eye(3), zeros(3,3); zeros(3,3), T_gamma];
% 
% % disp(sum(sum(J1-J3))) 
% % disp(det(TA))
% 
% disp(J1)
% disp(J3)










rz = 90/180.0*pi;
ry = 30/180.0*pi;
rx = 60/180.0*pi;

R1 = rot_ZYX(rz,ry,rx);
R2 = eul2rotm([rz,ry,rx]);
% R3 = eul2rotm([rx,ry,rz]);

disp(rotm2eul(R1)*180/pi);
disp(rotm2eul(R2)*180/pi);
% disp(R3)


tic;
theta = sym ('theta', [1,7]);
base_T = sym('base_T',[4,4]);
syms bx by bz brx bry brz;

% base_t = [bx;by;bz];
% base_R = rot_ZYX(brz,bry,brx);
% base_T = [base_R,base_t; 0 0 0 1];
% Tee = robot1.fwd_kin_all_joints(theta);

Tee = iiwa_FK_all_joints_crude(theta, base_T);
% Tee = Tee{end};
% Tee = base_T*Tee;
% Ree = Tee(1:3,1:3);
% abc = rotm2eul(Ree);
% abc = get_rzryrx_from_T(Tee);
abc = rotm2eulZYX(Tee(1:3,1:3));
x = Tee(1,4); y = Tee(2,4); z = Tee(3,4);
rz = abc(1); ry = abc(2); rx = abc(3);
toc;

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




function rzryrx = get_rzryrx_from_T(T)
    R = T(1:3,1:3);
    
    rz = atan2(R(2,1),R(1,1));
    ry = atan2(-R(3,1),sqrt(R(1,1)^2+R(2,1)^2));
    rx = atan2(R(3,2),R(3,3));
    
    rzryrx = [rz,ry,rx];
end

% function R = rot_ZYX(rz,ry,rx)
% 
%     rotz = [
%         cos(rz) -sin(rz) 0; 
%         sin(rz) cos(rz) 0;
%         0 0 1; 
%         ];
% 
%     roty = [
%         cos(ry) 0 sin(ry); 
%         0 1 0;
%         -sin(ry) 0 cos(ry); 
%         ];   
%     rotx = [
%         1 0 0; 
%         0 cos(rx) -sin(rx);
%         0 sin(rx) cos(rx); 
%         ];    
%     R = rotz * roty * rotx;
% end