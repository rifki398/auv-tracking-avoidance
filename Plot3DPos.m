figure(7);
plot3(eta(:,2),eta(:,1),eta(:,3),'b-')
hold on;
plot3(wpt.pos.y, wpt.pos.x, wpt.pos.z, 'ro', 'MarkerSize', 4);
plot3(wpt.pos.y, wpt.pos.x, wpt.pos.z, 'k--');
hold off
title('North-East-Down plot (m)')
xlabel('East'); ylabel('North'); zlabel('Down');
legend('Actual path','Waypoints','Desired Path','Location',legendLocation),grid
set(gca, 'ZDir', 'reverse');
view(-25, 30);  % view(AZ,EL)
set(findall(gcf,'type','line'),'linewidth',2)
set(findall(gcf,'type','line'),'linewidth',4)
set(gcf,'color','white')