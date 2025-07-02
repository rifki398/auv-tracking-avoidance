figure(4);
if ~isoctave; set(gcf,'Position',[2*scrSz(3)/3,1,scrSz(3)/3,scrSz(4)/2]);end
subplot(311),plot(t,sqrt(nu(:,1).^2+nu(:,2).^2),t,Vc)
xlabel('Time (s)'),grid
legend('Vehicle horizontal speed (m/s)','Ocean current horizontal speed (m/s)',...
    'Location',legendLocation)
subplot(312),plot(t,nu(:,3),t,wc)
xlabel('Time (s)'),grid
legend('Vehicle heave velocity (m/s)','Ocean current heave velocity (m/s)',...
    'Location',legendLocation)
subplot(313),plot(t,rad2deg(betaVc),'r')
xlabel('Time (s)'),grid
legend('Ocean current direction (deg)','Location',legendLocation)
set(findall(gcf,'type','line'),'linewidth',2)
set(findall(gcf,'type','text'),'FontSize',14)
set(findall(gcf,'type','legend'),'FontSize',legendSize)