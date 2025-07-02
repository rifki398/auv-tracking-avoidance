figure(8)

subplot(311)
plot(t,x_e)
title('Along-track error (m)'),grid
xlabel('Time (s)')
ylabel('x_e(m)')
legend('Actual')


subplot(312)
plot(t,y_e)
title('Cross-track error (m)'),grid
xlabel('Time (s)')
ylabel('y_e(m)')
legend('Actual')

subplot(313)
plot(t,z_e)
title('Vertical-track error (m)'),grid
xlabel('Time (s)')
ylabel('z_e(m)')
legend('Actual')

set(findall(gcf,'type','line'),'linewidth',2)
set(findall(gcf,'type','text'),'FontSize',14)
set(findall(gcf,'type','legend'),'FontSize',legendSize)