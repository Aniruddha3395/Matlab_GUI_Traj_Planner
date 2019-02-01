%% plot joint angles change

function disp_joint_angle_change(joint_angles)

figure;
subplot(4,2,1);
plot(1:size(joint_angles,1),joint_angles(:,1)*180/pi);
hold on;
scatter(1:size(joint_angles,1),joint_angles(:,1)*180/pi,'filled');
title('joint 1');
subplot(4,2,2);
plot(1:size(joint_angles,1),joint_angles(:,2)*180/pi);
hold on;
scatter(1:size(joint_angles,1),joint_angles(:,2)*180/pi,'filled');
title('joint 2');
subplot(4,2,3);
plot(1:size(joint_angles,1),joint_angles(:,3)*180/pi);
hold on;
scatter(1:size(joint_angles,1),joint_angles(:,3)*180/pi,'filled');
title('joint 3');
subplot(4,2,4);
plot(1:size(joint_angles,1),joint_angles(:,4)*180/pi);
hold on;
scatter(1:size(joint_angles,1),joint_angles(:,4)*180/pi,'filled');
title('joint 4');
subplot(4,2,5);
plot(1:size(joint_angles,1),joint_angles(:,5)*180/pi);
hold on;
scatter(1:size(joint_angles,1),joint_angles(:,5)*180/pi,'filled');
title('joint 5');
subplot(4,2,6);
plot(1:size(joint_angles,1),joint_angles(:,6)*180/pi);
hold on;
scatter(1:size(joint_angles,1),joint_angles(:,6)*180/pi,'filled');
title('joint 6');
subplot(4,2,7);
plot(1:size(joint_angles,1),joint_angles(:,7)*180/pi);
hold on;
scatter(1:size(joint_angles,1),joint_angles(:,7)*180/pi,'filled');
title('joint 7');

end