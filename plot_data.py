from pylib.load import *
import matplotlib.pyplot as plt 
import os
os.system("cls||clear")

# 1: Fossen
# 2: Proposed method

n = 2

data = load_data(n)

obs = data[1]['obs']
t = data[1]['t']
wpt = data[1]['wpt']
wpt_ori = data[1]['wpt_ori']
aact = data[1]['alpha_c_act']
bact = data[1]['beta_c_act']
Vc = data[1]['Vc']
aVc = data[1]['alphaVc']
bVc = data[1]['betaVc']

u = data[1]['u']
v = data[1]['v']
w = data[1]['w']
p = data[1]['p']
q = data[1]['q']
r = data[1]['r']

eta1 = data[0]['eta']
eta2 = data[1]['eta']

xe1 = data[0]['x_e']
xe2 = data[1]['x_e']
xehat = data[1]['x_e_hat']

ye1 = data[0]['y_e']
ye2 = data[1]['y_e']
yehat = data[1]['y_e_hat']

ze1 = data[0]['z_e']
ze2 = data[1]['z_e']
zehat = data[1]['z_e_hat']

ac1 = data[0]['alpha_c_hat']
ac2 = data[1]['alpha_c_hat']

bc1 = data[0]['beta_c_hat']
bc2 = data[1]['beta_c_hat']

zeta1 = data[1]['zeta1']
zeta2 = data[1]['zeta2']
zeta3 = data[1]['zeta3']

Uv = data[1]['Uv']
Uvhat = data[1]['Uvhat']

d11 = data[0]['d1']
d21 = data[0]['d2']
d12 = data[1]['d1']
d22 = data[1]['d2']

theta_d1 = data[0]['theta_d']
theta_d2 = data[1]['theta_d']
psi_d1 = data[0]['psi_d']
psi_d2 = data[1]['psi_d']

plot_wpt(obs,wpt,wpt_ori,(eta1,eta2),('Fossen','Proposed Method',))
# plot_cte_vte((xe1,xe2),(ye1,ye2),(ze1,ze2),t,('Fossen','Proposed Method'))
# plot_track_error_est(xe2,ye2,ze2,xehat,yehat,zehat,t)
# plot_slip_angle((ac2,), (bc2,), aact, bact, t, ('Proposed method',))
# plot_wpt_2d(wpt,(eta2,))
# plot_wpt_as_func_t(wpt,(eta1,eta2),t,('Fossen','Proposed Method'))
# plot_current(Vc,aVc,bVc,t)
# plot_state_observer(zeta1,zeta2,zeta3,t)
# plot_Uvhat(Uv,Uvhat,t)
# plot_threat((d11,d12),(d21,d22),t,('Fossen','Proposed Method'))
# plot_angle_ref(t,theta_d2,psi_d2,eta2)
# plot_state(t,u,v,w,p,q,r)
plot_pos_ort(t,eta2)
plt.show()