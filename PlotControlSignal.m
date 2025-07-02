figure(3);
if ~isoctave; set(gcf,'Position',[2*scrSz(3)/3,scrSz(4)/2,scrSz(3)/3,scrSz(4)/2]);end
subplot(311),plot(t,rad2deg(ui(:,1)))
xlabel('Time (s)'),title('Rudder command \delta_r (deg)'),grid
subplot(312),plot(t,rad2deg(ui(:,2)))
xlabel('Time (s)'),title('Stern-plane command \delta_s (deg)'),grid
subplot(313),plot(t,ui(:,3))
xlabel('Time (s)'),title('Propeller speed command n (rpm)'),grid
set(findall(gcf,'type','line'),'linewidth',2)
set(findall(gcf,'type','text'),'FontSize',14)