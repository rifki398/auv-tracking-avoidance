figure(5);
if ~isoctave; set(gcf,'Position',[100,scrSz(4)/2,scrSz(3)/3,scrSz(4)]); end
subplot(211)
plot(t,rad2deg(alpha),'g',t,rad2deg(alpha_c),'b',t,rad2deg(alpha_c_hat),'r')
title('Vertical crab angle and AOA (deg)')
xlabel('Time (s)')
grid
legend('\alpha Angle of attack (AOA)','\alpha_c Vertical crab angle','\alpha_c ALOS estimate','Location',legendLocation)
subplot(212)
plot(t,rad2deg(beta),'g',t,rad2deg(beta_c),'b',t,rad2deg(beta_c_hat),'r')
title('Horizontal crab angle and SSA (deg)')
xlabel('Time (s)')
grid
legend('\beta Sideslip angle (SSA)','\beta_c Horizontal crab angle','\beta_c ALOS estimate','Location',legendLocation)
set(findall(gcf,'type','line'),'linewidth',2)
set(findall(gcf,'type','text'),'FontSize',14)
set(findall(gcf,'type','legend'),'FontSize',legendSize)