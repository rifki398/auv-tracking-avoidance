from pylib.load import *
import matplotlib.pyplot as plt 
import os
os.system("cls||clear")

# 1: Fossen
# 2: Proposed method

n = 2

data = load_data(n)

obs = data[0]['obs']
t = data[0]['t']
wpt = data[0]['wpt']
aact = data[1]['alpha_c_act']
bact = data[1]['beta_c_act']
Vc = data[0]['Vc']
aVc = data[0]['alphaVc']
bVc = data[0]['betaVc']

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

plot_wpt(obs,wpt,(eta1,eta2),('Fossen','Proposed Method'))
# plot_cte_vte((xe1,xe2),(ye1,ye2),(ze1,ze2),t,('Fossen','Proposed Method'))
# plot_track_error_est(xe2,ye2,ze2,xehat,yehat,zehat,t)
# plot_slip_angle((ac2,), (bc2,), aact, bact, t, ('Proposed Method',))
# plot_wpt_2d(wpt,(eta1,eta2))
# plot_current(Vc,aVc,bVc,t)
# plot_state_observer(zeta1,zeta2,zeta3,t)

plt.show()