close all;
clc;

s1 = load('data/alos_1.mat');       % ALOS By Fossen
s2 = load('data/alos_2.mat');       % Proposed Method
t = load('data/time.mat');

t = t.t;
alos1 = s1.alosData;
alos2 = s2.alosData;

ye2 = alos2(:,1);
ze2 = alos2(:,2);

ye1 = alos1(:,1);
ze1 = alos1(:,2);

subplot(211)
plot(t,ye1,t,ye2)
title('Cross-track error (m)'),grid
xlabel('Time (s)')
ylabel('y_e(m)')
legend('Fossen (2024)','Proposed')

subplot(212)
plot(t,ze1,t,ze2)
title('Vertical-track error (m)'),grid
xlabel('Time (s)')
ylabel('z_e(m)')
legend('Fossen (2024)','Proposed')

set(findall(gcf,'type','line'),'linewidth',2)
set(findall(gcf,'type','text'),'FontSize',14)
set(gcf,'Color','white')