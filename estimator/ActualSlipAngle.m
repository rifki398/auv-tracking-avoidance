function [alpha_c,beta_c] = ActualSlipAngle(x)
u = x(1);
v = x(2);
w = x(3);
phi = x(10);
theta = x(11);

Uv = sqrt(u^2 + w^2);

alpha_c = atan((v*sin(phi) + w*cos(phi))/u);
beta_c = atan((v*cos(phi) - w*sin(phi))/Uv*cos(theta-alpha_c));
