figure(1);
if ~isoctave; set(gcf,'Position',[1, 1, scrSz(3)/3, scrSz(4)]); end
subplot(611),plot(t,u)
xlabel('Time (s)'),title('Surge velocity (m/s)'),grid
subplot(612),plot(t,v)
xlabel('Time (s)'),title('Sway velocity (m/s)'),grid
subplot(613),plot(t,w)
xlabel('Time (s)'),title('Heave velocity (m/s)'),grid
subplot(614),plot(t,(180/pi)*p)
xlabel('Time (s)'),title('Roll rate (deg/s)'),grid
subplot(615),plot(t,(180/pi)*q)
xlabel('Time (s)'),title('Pitch rate (deg/s)'),grid
subplot(616),plot(t,(180/pi)*r)
xlabel('Time (s)'),title('Yaw rate (deg/s)'),grid
set(findall(gcf,'type','line'),'linewidth',2)
set(findall(gcf,'type','text'),'FontSize',14)
set(findall(gcf,'type','legend'),'FontSize',legendSize)