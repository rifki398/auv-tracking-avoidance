import matplotlib.pyplot as plt
import scipy.io
import numpy as np

def load_data(n):
    all_results = []
    for i in range(n):
        # Ganti angka skenario sesuai kebutuhan
        filename1 = f'data/scenario_{i+1}.mat'
        filename2 = 'data/obstacles.mat'
        filename3 = f'data/obs_avoid_{i+1}.mat'

        # Load .mat file
        mat_data1 = scipy.io.loadmat(filename1)
        mat_data2 = scipy.io.loadmat(filename2, squeeze_me=True, struct_as_record=False)
        mat_data3 = scipy.io.loadmat('data/waypoints.mat', squeeze_me=True, struct_as_record=False)
        mat_data4 = scipy.io.loadmat(filename3,squeeze_me=True, struct_as_record=False)

        # Ambil data
        simData = mat_data1['simData']
        alosData = mat_data1['alosData']
        t = mat_data1['t'].squeeze()  # Hilangkan dimensi ekstra jika perlu
        obs = mat_data2['obs']
        wpt = mat_data3['wpt']
        wpt_ori = mat_data3['wpt_ori']
        distData = mat_data1['distData']
        observerData = mat_data1['observerData']
        obsAvoidData = mat_data4['obsAvoidData']

        # Contoh akses data
        print(f"simData shape: {simData.shape}")
        print(f"alosData shape: {alosData.shape}")
        print(f"Wpt-x: {wpt.pos.x.shape} wpt-y: {wpt.pos.y.shape} wpt-z: {wpt.pos.z.shape}")

        # === simData ===
        z_d      = simData[:, 0]
        theta_d  = simData[:, 1]
        psi_d    = simData[:, 2]
        r_d      = simData[:, 3]
        Vc       = simData[:, 4]
        betaVc   = simData[:, 5]
        wc       = simData[:, 6]
        ui       = simData[:, 7:10]     # columns 8–10 (MATLAB) → 7:10 (Python)
        nu       = simData[:, 10:16]    # columns 11–16 → 10:16
        eta      = simData[:, 16:22]    # columns 17–22 → 16:22

        # Bagi nu menjadi komponen individual
        u = nu[:, 0]
        v = nu[:, 1]
        w = nu[:, 2]
        p = nu[:, 3]
        q = nu[:, 4]
        r = nu[:, 5]

        # === alosData ===
        x_e         = alosData[:, 0]
        y_e         = alosData[:, 1]
        z_e         = alosData[:, 2]
        alpha_c_hat = alosData[:, 3]
        beta_c_hat  = alosData[:, 4]
        alpha_c_act = alosData[:, 5]
        beta_c_act  = alosData[:, 6]
        
        # === disturbance data ===
        Vc = distData[:,0]
        alphaVc = distData[:,1]
        betaVc = distData[:,2]

        # === Observer Data ===
        x_e_hat = observerData[:,0]
        y_e_hat = observerData[:,1]
        z_e_hat = observerData[:,2]
        zeta1 = observerData[:,3]
        zeta2 = observerData[:,4]
        zeta3 = observerData[:,5]
        Uvhat = observerData[:,6]
        Uv = observerData[:,7]
        
        # === Obstacle Avoidance ===
        d1 = obsAvoidData[:,0]
        d2 = obsAvoidData[:,1]

        result = {
            'simData': simData,
            'alosData': alosData,
            't': t,
            'u': u, 'v': v, 'w': w,
            'p': p, 'q': q, 'r': r,
            'Vc': Vc, 'alphaVc': alphaVc, 'betaVc': betaVc, 'wc': wc,
            'eta': eta, 'nu': nu, 'ui': ui,
            'x_e': x_e, 'y_e': y_e, 'z_e': z_e,
            'x_e_hat' : x_e_hat, 'y_e_hat' : y_e_hat ,'z_e_hat' : z_e_hat,
            'zeta1' : zeta1, 'zeta2' : zeta2, 'zeta3' : zeta3,
            'Uvhat' : Uvhat, 'Uv' : Uv,
            'alpha_c_hat': alpha_c_hat, 'beta_c_hat': beta_c_hat,
            'alpha_c_act': alpha_c_act, 'beta_c_act': beta_c_act,
            'obs': obs,
            'wpt': wpt, 'wpt_ori' : wpt_ori,
            'd1' : d1, 'd2' : d2,
            'theta_d' : theta_d, 'psi_d' : psi_d
        }
        all_results.append(result)
        
    return all_results

def plot_wpt(obs,wpt,wpt_ori,etas : tuple, labels):
    # obs.pos dan obs.r masing-masing adalah cell array (dianggap list oleh Python)
    obs_pos = np.array(obs.pos) 
    obs_r   = np.array(obs.r) 

    x_path = np.array(wpt.pos.x).astype(float)
    y_path = np.array(wpt.pos.y).astype(float)
    z_path = np.array(wpt.pos.z).astype(float)

    x_path_ori = np.array(wpt_ori.pos.x).astype(float)
    y_path_ori = np.array(wpt_ori.pos.y).astype(float)
    z_path_ori = np.array(wpt_ori.pos.z).astype(float)

    fig1 = plt.figure(figsize=(12,6))

    ax1 = fig1.add_subplot(projection='3d')
    ax1.grid()
    # ax1.plot(x_path, y_path, z_path, '.--', label='New Path')
    ax1.plot(x_path_ori,y_path_ori,z_path_ori,'--',label='Predefined Path')

    for pos,r in zip(obs_pos,obs_r):
        plot_sphere(ax1, pos, r)

    for i,eta in enumerate(etas):
        # State vectors
        lab = labels[i]
        x = eta[:, 0]
        y = eta[:, 1]
        z = eta[:, 2]

        if i == 0:
        #     ax1.plot(x[0],y[0],-z[0],'b*', label='AUV start')
            ax1.plot(x[-1],y[-1],z[-1],'k*', label='AUV')
            
        ax1.plot(x[0],y[0],z[0],'b*')
        ax1.plot(x[-1],y[-1],z[-1],'k*')

        ax1.plot(x,y,z, label=lab)
        
    ax1.plot(x_path[0], y_path[0], z_path[0], 'ko',label='Start')
    ax1.plot(x_path[-1],y_path[-1],z_path[-1],'ro',label='Goal')

    # for i, (xp, yp, zp) in enumerate(vehicle_data.alt_path):
    #     if i == 0 and j == 0:
    #         ax1.plot(xp, yp, -zp, 'g*', label='Alt. Waypoint')
    #     else:
    #         ax1.plot(xp, yp, -zp, 'g*')

    ax1.set_xlabel('North (m)')
    ax1.set_ylabel('East (m)')
    ax1.set_zlabel('Down (m)')
    ax1.set_title('NED Position')
    ax1.legend(loc='upper left', bbox_to_anchor=(-0.1, 1.05), borderaxespad=0.)

    # Gabungkan semua titik yang ingin dihitung untuk limit sumbu
    all_x = np.concatenate([x, x_path, [obs[0] for obs in obs_pos]])
    all_y = np.concatenate([y, y_path, [obs[1] for obs in obs_pos]])
    all_z = np.concatenate([z, z_path, [obs[2] for obs in obs_pos]])

    # Hitung titik tengah dan rentang terluas
    x_mid = (np.max(all_x) + np.min(all_x)) / 2
    y_mid = (np.max(all_y) + np.min(all_y)) / 2
    z_mid = (np.max(all_z) + np.min(all_z)) / 2

    max_range = max(
        np.max(all_x) - np.min(all_x),
        np.max(all_y) - np.min(all_y),
        np.max(all_z) - np.min(all_z)
    ) / 2

    # Atur ulang limit supaya rasio 1:1:1 bisa terlihat bagus
    ax1.set_xlim(x_mid - max_range, x_mid + max_range)
    ax1.set_ylim(y_mid - max_range, y_mid + max_range)
    ax1.set_zlim(z_mid - max_range, z_mid + max_range)

def plot_sphere(ax, center, radius, color='k', alpha=1.0):
    u = np.linspace(0, 2 * np.pi, 10)
    v = np.linspace(0, np.pi, 10)
    x = radius * np.outer(np.cos(u), np.sin(v)) + center[0]
    y = radius * np.outer(np.sin(u), np.sin(v)) + center[1]
    z = radius * np.outer(np.ones(np.size(u)), np.cos(v)) + center[2]
    ax.plot_surface(x, y, z, color=color, alpha=alpha)

def plot_Uvhat(Uv,Uvhat,t):
    plt.figure()
    plt.plot(t,Uv,label='Actual')
    plt.plot(t,Uvhat,label='Estimated')
    plt.xlabel('t (s)')
    plt.ylabel('Uv (m/s)')
    plt.legend()
    plt.grid()
    
def plot_cte_vte(x_es : tuple,y_es : tuple,z_es : tuple, t,labels):
    fig,ax = plt.subplots(3,1,figsize=(10,8))
    for i,(ate,cte,vte) in enumerate(zip(x_es,y_es,z_es)):
        lab = labels[i]

        rmse_cte = np.sqrt(np.mean(cte**2))
        rmse_vte = np.sqrt(np.mean(vte**2))

        # STD (standar deviasi)
        std_cte = np.std(cte)
        std_vte = np.std(vte)

        # Max absolute error
        maxabs_cte = np.max(np.abs(cte))
        maxabs_vte = np.max(np.abs(vte))

        print(f"\n Method by {lab}")
        print(f"RMSE CTE     : {rmse_cte:.2f}")
        print(f"STD  CTE     : {std_cte:.2f}")
        print(f"MaxAbs CTE   : {maxabs_cte:.2f}")

        print(f"RMSE VTE     : {rmse_vte:.2f}")
        print(f"STD  VTE     : {std_vte:.2f}")
        print(f"MaxAbs VTE   : {maxabs_vte:.2f}")

        ax[0].plot(t,ate, label=lab)
        #ax[0].plot(t,ate_hat, label='Estimated')
        ax[1].plot(t,cte, label=lab)
        #ax[1].plot(t,cte_hat, label='Estimated')
        ax[2].plot(t,vte, label=lab)
        #ax[2].plot(t,vte_hat, label='Estimated')

    ax[0].grid()
    ax[1].grid()
    ax[2].grid()

    ax[0].set_ylabel('x_e (m)')
    ax[1].set_ylabel('y_e (m)')
    ax[2].set_ylabel('z_e (m)')
    ax[2].set_xlabel('t (s)')
    ax[0].legend()
    ax[1].legend()
    ax[2].legend()

def plot_track_error_est(xe,ye,ze,xehat,yehat,zehat,t):
    fig,ax = plt.subplots(3,1,figsize=(10,8))

    ax[0].plot(t,xe, label='Actual')
    ax[0].plot(t,xehat, label='Estimated')
    ax[1].plot(t,ye, label='Actual')
    ax[1].plot(t,yehat, label='Estimated')
    ax[2].plot(t,ze, label='Actual')
    ax[2].plot(t,zehat, label='Estimated')

    ax[0].grid()
    ax[1].grid()
    ax[2].grid()

    ax[0].set_ylabel('x_e (m)')
    ax[1].set_ylabel('y_e (m)')
    ax[2].set_ylabel('z_e (m)')
    ax[2].set_xlabel('t (s)')
    ax[0].legend()
    ax[1].legend()
    ax[2].legend()

def plot_slip_angle(alpha_c_hat : tuple, beta_c_hat : tuple, alpha_c_act, beta_c_act, t, labels):
    fig,ax = plt.subplots(2,1,figsize=(10,8))

    for i,(alpha_c,beta_c) in enumerate(zip(alpha_c_hat,beta_c_hat)):
        lab = labels[i]

        #ax[0].plot(t,np.rad2deg(beta_c_hat),'-',label=f'{vehicle_data.vehicle.name} Estimated')
        ax[0].plot(t,np.rad2deg(beta_c),label=lab)

        #ax[1].plot(t,np.rad2deg(alpha_c_hat),'-',label=f'{vehicle_data.vehicle.name} Estimated')
        ax[1].plot(t,np.rad2deg(alpha_c),label=lab)

    ax[0].plot(t,np.rad2deg(beta_c_act),label='Actual')
    ax[1].plot(t,np.rad2deg(alpha_c_act),label='Actual')

    ax[0].grid()
    ax[1].grid()
    
    ax[0].set_ylabel('beta_c (deg)')
    ax[1].set_ylabel('alpha_c (deg)')
    ax[1].set_xlabel('t (s)')
    
    ax[0].legend()
    ax[1].legend()

def plot_wpt_as_func_t(wpt,etas : tuple,t,labels):
    x_path = np.array(wpt.pos.x).astype(float)
    y_path = np.array(wpt.pos.y).astype(float)
    z_path = np.array(wpt.pos.z).astype(float)

    fig1,ax = plt.subplots(3,1,figsize=(8,4))

    # ax[0].plot(t,x_path,'--g')
    # ax[0].plot(t,x_path,'og')

    # ax[1].plot(t,y_path,'--r')
    # ax[1].plot(t,y_path,'or')

    # ax[2].plot(t,z_path,'--b')
    # ax[2].plot(t,z_path,'ob')

    ax[0].grid()
    ax[1].grid()
    ax[2].grid()

    ax[2].set_xlabel('t (s)')
    ax[0].set_ylabel('North (m)')
    ax[1].set_ylabel('East (m)')
    ax[2].set_ylabel('Down (m)')

    for i,(eta,lab) in enumerate(zip(etas,labels)):
        # State vectors
        x = eta[:, 0]
        y = eta[:, 1]
        z = eta[:, 2]

        ax[0].plot(t,x,label=lab)
        ax[1].plot(t,y,label=lab)
        ax[2].plot(t,z,label=lab)
    
    ax[0].legend()
    ax[1].legend()
    ax[2].legend()

def plot_wpt_2d(wpt,etas : tuple):
    # obs.pos dan obs.r masing-masing adalah cell array (dianggap list oleh Python)

    x_path = np.array(wpt.pos.x).astype(float)
    y_path = np.array(wpt.pos.y).astype(float)
    z_path = np.array(wpt.pos.z).astype(float)

    fig1,ax = plt.subplots(2,1,figsize=(8,4))

    ax[0].plot(y_path,x_path,'-g')
    ax[0].plot(y_path,x_path,'og')
    ax[1].plot(y_path,z_path,'-r')
    ax[1].plot(y_path,z_path,'or')

    ax[0].grid()
    ax[1].grid()

    ax[0].set_ylabel('North (m)')
    ax[1].set_ylabel('Down (m)')
    ax[1].set_xlabel('East (m)')

    plt.show(block=False)

def plot_current(Vc,alphaVc,betaVc,t):
    fig,axs = plt.subplots(3,1,figsize=(10,8))
    axs[0].plot(t,Vc)
    axs[1].plot(t,np.rad2deg(alphaVc))
    axs[2].plot(t,np.rad2deg(betaVc))

    axs[0].grid()
    axs[1].grid()
    axs[2].grid()

    axs[0].set_ylabel('Vc (m/s)')
    axs[1].set_ylabel('alphaVc (deg)')
    axs[2].set_ylabel('betaVc (deg)')
    axs[2].set_xlabel('t (s)')

def plot_state_observer(zeta1,zeta2,zeta3,t):
    fig,ax = plt.subplots(3,1,figsize=(10,8))

    ax[0].plot(t,zeta1)
    ax[1].plot(t,zeta2)
    ax[2].plot(t,zeta3)

    ax[0].grid()
    ax[1].grid()
    ax[2].grid()

    ax[0].set_ylabel('zeta1 (m/s)')
    ax[1].set_ylabel('zeta2 (m/s)')
    ax[2].set_ylabel('zeta3 (m/s)')
    ax[2].set_xlabel('t (s)')

def plot_threat(d1list : tuple,d2list : tuple,t,labels):
    plt.figure()
    for i in range(len(d1list)):
        d1 = d1list[i]
        d2 = d2list[i]

        d = np.maximum(d1,d2)
        plt.plot(t,np.sqrt(d),label=labels[i])
    
    plt.plot(t, np.ones_like(t) * (1/(5.0)), '--', label='Safe distance')
    plt.grid()
    plt.legend()
    plt.xlabel('t (s)')
    plt.ylabel(r'$1/d$ (m$^{-2}$)')

def plot_angle_ref(t,theta_d,psi_d,eta):
    theta = eta[:,4]
    psi = eta[:,5]

    fig,ax = plt.subplots(2,1,figsize=(10,8))

    ax[0].plot(t,np.rad2deg(theta_d),label='Reference')
    ax[0].plot(t,np.rad2deg(theta),label='Actual')

    ax[1].plot(t,np.rad2deg(psi_d),label='Reference')
    ax[1].plot(t,np.rad2deg(psi),label='Actual')

    ax[0].grid()
    ax[1].grid()

    ax[0].legend()
    ax[1].legend()

    ax[1].set_xlabel('t (s)')
    ax[0].set_ylabel('theta (deg)')
    ax[1].set_ylabel('psi (deg)')

def plot_state(t,u,v,w,p,q,r):
    _, ax1 = plt.subplots(3,1,figsize=(10,8))
    ax1[0].plot(t,u)
    ax1[1].plot(t,v)
    ax1[2].plot(t,w)

    ax1[0].set_ylabel('u (m/s)')
    ax1[1].set_ylabel('v (m/s)')
    ax1[2].set_ylabel('w (m/s)')
    ax1[2].set_xlabel('t (s)')

    ax1[0].grid()
    ax1[1].grid()
    ax1[2].grid()

    _, ax2 = plt.subplots(3,1,figsize=(10,8))
    ax2[0].plot(t,p)
    ax2[1].plot(t,q)
    ax2[2].plot(t,r)

    ax2[0].set_ylabel('p (rad/s)')
    ax2[1].set_ylabel('q (rad/s)')
    ax2[2].set_ylabel('r (rad/s)')
    ax2[2].set_xlabel('t (s)')

    ax2[0].grid()
    ax2[1].grid()
    ax2[2].grid()

def plot_pos_ort(t,eta):
    x = eta[:,0]
    y = eta[:,1]
    z = eta[:,2]
    phi = eta[:,3]
    theta = eta[:,4]
    psi = eta[:,5]

    _, ax1 = plt.subplots(3,1,figsize=(10,8))
    ax1[0].plot(t,x)
    ax1[1].plot(t,y)
    ax1[2].plot(t,z)

    ax1[0].set_ylabel('x (m)')
    ax1[1].set_ylabel('y (m)')
    ax1[2].set_ylabel('z (m)')
    ax1[2].set_xlabel('t (s)')

    ax1[0].grid()
    ax1[1].grid()
    ax1[2].grid()

    _, ax2 = plt.subplots(3,1,figsize=(10,8))
    ax2[0].plot(t,np.rad2deg(phi))
    ax2[1].plot(t,np.rad2deg(theta))
    ax2[2].plot(t,np.rad2deg(psi))

    ax2[0].set_ylabel('phi (deg)')
    ax2[1].set_ylabel('theta (deg)')
    ax2[2].set_ylabel('psi (deg)')
    ax2[2].set_xlabel('t (s)')

    ax2[0].grid()
    ax2[1].grid()
    ax2[2].grid()