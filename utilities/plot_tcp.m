function plot_tcp(bx,by,bz,points)
    hold on
    % Plot the X Vector
    quiver3(points(:,1),points(:,2),points(:,3),...
       bx(:,1),bx(:,2),bx(:,3),'r','linewidth',2,'AutoScaleFactor',0.8,'MaxHeadSize',0.6);
    hold on
    % Plot the Y Vector
    quiver3(points(:,1),points(:,2),points(:,3),...
       by(:,1),by(:,2),by(:,3),'g','linewidth',2,'AutoScaleFactor',0.8,'MaxHeadSize',0.6);
    hold on
    % Plot the Z vector
    quiver3(points(:,1),points(:,2),points(:,3),...
       bz(:,1),bz(:,2),bz(:,3),'b','linewidth',2,'AutoScaleFactor',0.8,'MaxHeadSize',0.6);
end