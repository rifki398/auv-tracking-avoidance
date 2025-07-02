figure(2);
if ~isoctave; set(gcf,'Position',[scrSz(3)/3, 1, scrSz(3)/3, scrSz(4)]); end
if ControlFlag == 3; z_d = eta(:,3); end
subplot(411),plot(t,eta(:,3),t,z_d)
xlabel('Time (s)'),title('Heave position (m)'),grid
legend('True','Desired')
subplot(412),plot(t,rad2deg(eta(:,4)))
xlabel('Time (s)'),title('Roll angle (deg)'),grid
subplot(413),plot(t,rad2deg(eta(:,5)),t,rad2deg(theta_d))
xlabel('Time (s)'),title('Pitch angle (deg)'),grid
legend('True','Desired')
subplot(414),plot(t,rad2deg(eta(:,6)),t,rad2deg(psi_d))
xlabel('Time (s)'),title('Yaw angle (deg)'),grid
legend('True','Desired')
set(findall(gcf,'type','line'),'linewidth',2)
set(findall(gcf,'type','text'),'FontSize',14)
set(findall(gcf,'type','legend'),'FontSize',legendSize)