import numpy as np
import numpy.linalg as LA
import matplotlib as mpl
import matplotlib.pyplot as plt
import os
import csv
import xml.etree.ElementTree as ET
#C:\Users\JM246044\workspace\dev\xde\xde\xde\xde\build\out\Release\bin\..\..\..\..\src\aiv\

direc = "C:/Users/JM246044/workspace/dev/xde/xde/xde/xde/"

root = ET.parse(direc+'src/aiv/output.xml').getroot()

font = {'family' : 'sans-serif',
        'weight' : 'normal',
        'size'   : 10}

mpl.rc('font', **font)

#plt.ion()

# with open(direc+'/interpdata_ts.csv', 'rb') as csvfile:
    # treader = csv.reader(csvfile, delimiter=',',quotechar='|',quoting=csv.QUOTE_NONE)
    # tlist = list(treader)
    # rows = len(tlist)
    # cols = len(tlist[0][0:-1]) # ignoring last character
    # interp_tab = np.zeros((rows,cols))
    # for i in range(len(tlist)):
        # interp_tab[i] = np.array([float(t) for t in tlist[i][0:-1]])

for chld in root:

    pl_filename = 'pl_ts_'+ chld.tag + '.csv'
    ctrl_filename = 'ctrl_ts_' + chld.tag + '.csv'
    real_filename = 'real_ts_' + chld.tag + '.csv'

    with open(direc+'build/out/Release/bin/'+pl_filename, 'rb') as csvfile:
        treader = csv.reader(csvfile, delimiter=',',quotechar='|',quoting=csv.QUOTE_NONE)
        tlist = list(treader)
        rows = len(tlist)-1
        cols = len(tlist[0][0:-1])
        print 'mpl size ', rows, cols
        mpl_tab = np.zeros((rows,cols))
        for i in range(rows):
            mpl_tab[i] = np.array([float(t) for t in tlist[i][0:-1]])

    with open(direc+'build/out/Release/bin/'+ctrl_filename, 'rb') as csvfile:
        treader = csv.reader(csvfile, delimiter=',',quotechar='|',quoting=csv.QUOTE_NONE)
        tlist = list(treader)
        rows = len(tlist)-1
        cols = len(tlist[0][0:-1])
        print 'ctrl size ', rows, cols
        ctrl_tab = np.zeros((rows,cols))
        for i in range(rows):
    #        print tlist[i][0:-1]
            ctrl_tab[i] = np.array([float(t) for t in tlist[i][0:-1]])

    with open(direc+'build/out/Release/bin/'+real_filename, 'rb') as csvfile:
        treader = csv.reader(csvfile, delimiter=',',quotechar='|',quoting=csv.QUOTE_NONE)
        tlist = list(treader)
        rows = len(tlist)-1
        cols = len(tlist[0][0:-1])
        print 'real size', rows, cols
        real_tab = np.zeros((rows,cols))
        for i in range(rows):
    #        print tlist[i][0:-1]
            real_tab[i] = np.array([float(t) for t in tlist[i][0:-1]])

    rows = min(min(mpl_tab.shape[0], real_tab.shape[0]), ctrl_tab.shape[0])
    print rows

    #print points_tab[0,:]

    #time = mpl_tab[0:rows,0]
    simTime = mpl_tab[0:rows,0]

    mpl_v = mpl_tab[0:rows,4]
    mpl_w = mpl_tab[0:rows,5]

    mpl_x = mpl_tab[0:rows,1]
    mpl_y = mpl_tab[0:rows,2]
    mpl_t = mpl_tab[0:rows,3]

    ctrl_v = ctrl_tab[0:rows,1]
    ctrl_w = ctrl_tab[0:rows,2]

    real_x = real_tab[0:rows,1]
    real_y = real_tab[0:rows,2]
    real_t = real_tab[0:rows,3]

    real_v = real_tab[0:rows,4]
    real_w = real_tab[0:rows,5]
    # real_t1 = real_tab[0:rows,6]
    # real_t2 = real_tab[0:rows,7]
    # real_t3 = real_tab[0:rows,8]
    # real_t4 = real_tab[0:rows,9]
    # real_t5 = real_tab[0:rows,10]
    # real_t6 = real_tab[0:rows,11]


    # mpl_v2 = [0]+[LA.norm(np.array([x2-x1, y2-y1]))/(t2-t1) for x2, y2, x1, y1, t2, t1 in zip(mpl_x[1:], mpl_y[1:],
        # mpl_x[0:-1], mpl_y[0:-1], time[1:], time[0:-1])]
    # real_v2 = [0]+[LA.norm(np.array([x2-x1, y2-y1]))/(t2-t1) for x2, y2, x1, y1, t2, t1 in zip(real_x[1:], real_y[1:],
            # real_x[0:-1], real_y[0:-1], time[1:], time[0:-1])]

    # print 'real velocity shape', real_v.shape
    # print real_v[1:].shape
    # print len(mpl_v2)
    # print len(real_v2)

    # print 'Nr of samples', time.size
    # print 'real_v', real_v.size

    # fig2 = plt.figure()
    # fig2 = plt.figure()
    # ax2 = fig2.gca()
    # fig_v, axarray = plt.subplots(2)
    fig_epose, axarray_epose = plt.subplots(3, sharex=True)
    fig_evel, axarray_evel = plt.subplots(2, sharex=True)

    fig_pose, axarray_pose = plt.subplots(3, sharex=True)

    axarray_epose[0].grid()
    axarray_epose[1].grid()
    axarray_epose[2].grid()
    axarray_evel[0].grid()
    axarray_evel[1].grid()

    axarray_epose[0].plot(simTime, real_x-mpl_x, 'b', label=r'$x-x_{ref}$')
    # axarray_pose[0].plot(simTime, real_x, 'r', label=r'$x$ (actual)')

    axarray_epose[1].plot(simTime, real_y-mpl_y, 'b', label=r'$y-y_{ref}$')
    #axarray_pose[1].plot(simTime, , 'r', label=r'$y (actual)')

    axarray_epose[2].plot(simTime, real_t-mpl_t, 'b', label=r'$\theta-\theta_{ref}$')
    #axarray_pose[2].plot(simTime, , 'r', label=r'actual')

    axarray_pose[0].grid()
    axarray_pose[1].grid()
    axarray_pose[2].grid()

    axarray_pose[0].plot(simTime, mpl_x, 'b', label=r'$x_{ref}$')
    # axarray_pose[0].plot(simTime, real_x, 'r', label=r'$x$ (actual)')

    axarray_pose[1].plot(simTime, mpl_y, 'b', label=r'$y_{ref}$')
    #axarray_pose[1].plot(simTime, , 'r', label=r'$y (actual)')

    axarray_pose[2].plot(simTime, mpl_t, 'b', label=r'$\theta_{ref}$')
    #axarray_pose[2].plot(simTime, , 'r', label=r'actual')


    #plt.hold(True)

    # ax.plot(points_tab[0:215,0], points_tab[0:215,2], 'k.')

    #axarray[0+1].plot(points_tab[0:-1,0], points_tab[0:-1,5], 'b.')
    #axarray[1+1].plot(points_tab[0:-1,0], points_tab[0:-1,6], 'r.')
    #axarray[0+1].set_xlabel('time (s)')
    #axarray[1+1].set_xlabel('time (s)')
    #axarray[0+1].set_ylabel(r'$v (m/s)$')
    #axarray[1+1].set_ylabel(r'$\omega (rad/s)$')

    axarray_evel[0].plot(simTime, mpl_v, 'b', label=r'$u_{ref}[0]$ (planner output 1)')
    axarray_evel[0].plot(simTime, ctrl_v, 'g', label=r'$u[0]$ (controller output 1)')
    axarray_evel[0].plot(simTime, real_v, 'r', label=r'actual lin vel')
    #axarray[0+1].plot(time, real_v2, 'g', label='real lin2 sp')
    #axarray[0+1].plot(time, real_t1, 'r', label='t1')
    #axarray[0+1].plot(time, real_t2, 'b', label='t2')
    #axarray[0+1].plot(time, real_t3, 'g', label='t3')
    #axarray[1+1].plot(time, real_t4, 'r', label='t4')
    #axarray[1+1].plot(time, real_t5, 'b', label='t5')
    #axarray[1+1].plot(time, real_t6, 'g', label='t6')
    axarray_evel[1].plot(simTime, mpl_w, 'b', label=r'planner output')
    axarray_evel[1].plot(simTime, ctrl_w, 'g', label=r'ctrller output')
    axarray_evel[1].plot(simTime, real_w, 'r', label=r'actual vel')
    #axarray[0+1].set_xlabel('time (s)')
    #axarray[1+1].set_xlabel('time (s)')
    #axarray[0+1].set_ylabel(r'$v (m/s)$')
    #axarray[1+1].set_ylabel(r'$\omega (rad/s)$')

    fig = plt.figure()
    ax = fig.gca()
    ax.axis('equal')
    ax.grid()

    k = 1
    idxp = 0
    rho = 0.2

    plt_mpl_robot_c = plt.Circle(
           (mpl_x[0], mpl_y[0]), # position
           rho, # radius
           color='b',
           ls = 'solid',
           fill=False)

    xy = np.array(
            [[rho*np.cos(mpl_t[0])+mpl_x[0], \
            rho*np.sin(mpl_t[0])+mpl_y[0]],
            [rho*np.cos(mpl_t[0]-2.5*np.pi/3.0)+mpl_x[0], \
            rho*np.sin(mpl_t[0]-2.5*np.pi/3.0)+mpl_y[0]],
            [rho*np.cos(mpl_t[0]+2.5*np.pi/3.0)+mpl_x[0], \
            rho*np.sin(mpl_t[0]+2.5*np.pi/3.0)+mpl_y[0]]])

    plt_mpl_robot_t = plt.Polygon(xy, color='b', fill=True, alpha=0.2)

    plt_real_robot_c = plt.Circle(
           (real_x[0], real_y[0]), # position
           rho, # radius
           color='r',
           ls = 'solid',
           fill=True,
           alpha=0.1)

    plt_real_robot_c2 = plt.Circle(
           (real_x[0], real_y[0]), # position
           rho, # radius
           color='r',
           ls = 'solid',
           fill=False)

    xy = np.array(
            [[rho*np.cos(real_t[0])+real_x[0], \
            rho*np.sin(real_t[0])+real_y[0]],
            [rho*np.cos(real_t[0]-2.5*np.pi/3.0)+real_x[0], \
            rho*np.sin(real_t[0]-2.5*np.pi/3.0)+real_y[0]],
            [rho*np.cos(real_t[0]+2.5*np.pi/3.0)+real_x[0], \
            rho*np.sin(real_t[0]+2.5*np.pi/3.0)+real_y[0]]])


    plt_real_robot_t = plt.Polygon(xy, color='r', fill=True, alpha=0.5)
    ax.add_artist(plt_mpl_robot_c)
    ax.add_artist(plt_mpl_robot_t)
    ax.add_artist(plt_real_robot_c)
    ax.add_artist(plt_real_robot_c2)
    ax.add_artist(plt_real_robot_t)
    k=1

    idx_range = range(1, rows, 15)

    for i, idx in zip(simTime[1::15], idx_range):
        plt_mpl_robot_c.center = mpl_x[idx], mpl_y[idx]
        xy = np.array(
                [[rho*np.cos(mpl_t[idx])+mpl_x[idx], \
                rho*np.sin(mpl_t[idx])+mpl_y[idx]],
                [rho*np.cos(mpl_t[idx]-2.5*np.pi/3.0)+mpl_x[idx], \
                rho*np.sin(mpl_t[idx]-2.5*np.pi/3.0)+mpl_y[idx]],
                [rho*np.cos(mpl_t[idx]+2.5*np.pi/3.0)+mpl_x[idx], \
                rho*np.sin(mpl_t[idx]+2.5*np.pi/3.0)+mpl_y[idx]]])
        plt_mpl_robot_t.set_xy(xy)

        plt_real_robot_c.center = real_x[idx], real_y[idx]
        plt_real_robot_c2.center = real_x[idx], real_y[idx]
        xy = np.array(
                [[rho*np.cos(real_t[idx])+real_x[idx], \
                rho*np.sin(real_t[idx])+real_y[idx]],
                [rho*np.cos(real_t[idx]-2.5*np.pi/3.0)+real_x[idx], \
                rho*np.sin(real_t[idx]-2.5*np.pi/3.0)+real_y[idx]],
                [rho*np.cos(real_t[idx]+2.5*np.pi/3.0)+real_x[idx], \
                rho*np.sin(real_t[idx]+2.5*np.pi/3.0)+real_y[idx]]])
        plt_real_robot_t.set_xy(xy)

        if idx == idx_range[-1]:
            ax.plot(mpl_x[0:idx], mpl_y[0:idx], color='b', label='planned trajectory')
            ax.plot(real_x[0:idx], real_y[0:idx], color='r', label='actual robot trajectory')
        else:
            ax.plot(mpl_x[0:idx], mpl_y[0:idx], color='b')
            ax.plot(real_x[0:idx], real_y[0:idx], color='r')


    #    if i-2 > k*(.4/1.0):
    #        ax.plot(mpl_x[idxp:idx], mpl_y[idxp:idx], label = '{}'.format(k))
    #        ax.plot(real_x[idxp:idx], real_y[idxp:idx], label = '{}'.format(k))
    #        idxp = idx+1
    #        k+=1
    #        axarray[0+1].plot([i, i], [0, 1.25], '--k')
    #        axarray[1+1].plot([i, i], [-6, 6], '--k')
    #        axarray[0+1].plot([i, i], [0, 1.25], '--k')
    #        axarray[1+1].plot([i, i], [-6, 6], '--k')
        ax.relim()
        ax.autoscale_view(True, True, True)
        fig.canvas.draw()

    # hand, lab = axarray_pose[2].get_legend_handles_labels()
    # axarray_pose[2].legend(hand, lab, ncol=1, prop={'size':10}, loc=4)

    # ax.set_ylim([-0.7, 6.5])
    ax.set_xlabel(r'$x\ (m)$')
    ax.set_ylabel(r'$y\ (m)$')
    ax.set_title('Trajectory')

    axarray_epose[2].set_xlabel(r'$time\ (s)$')
    axarray_epose[0].set_ylabel(r'$x\ (m)$')
    axarray_epose[1].set_ylabel(r'$y\ (m)$')
    axarray_epose[2].set_ylabel(r'$\theta\ (rad)$')
    axarray_epose[0].set_title('Position X')
    axarray_epose[1].set_title('Position Y')
    axarray_epose[2].set_title('Yaw')
    #axarray_epose[0].set_xlim([0.0, 24.0])
    #axarray_epose[1].set_xlim([0.0, 24.0])
    #axarray_epose[2].set_xlim([0.0, 24.0])

    axarray_pose[2].set_xlabel(r'$time\ (s)$')
    axarray_pose[0].set_ylabel(r'$x\ (m)$')
    axarray_pose[1].set_ylabel(r'$y\ (m)$')
    axarray_pose[2].set_ylabel(r'$\theta\ (rad)$')
    axarray_pose[0].set_title('Position X')
    axarray_pose[1].set_title('Position Y')
    axarray_pose[2].set_title('Yaw')
    #axarray_pose[0].set_xlim([0.0, 24.0])
    #axarray_pose[1].set_xlim([0.0, 24.0])
    #axarray_pose[2].set_xlim([0.0, 24.0])

    #axarray[0+1].set_xlabel(r'$time (s)$')
    axarray_evel[1].set_xlabel(r'$time\ (s)$')#, fontsize=20)
    axarray_evel[0].set_ylabel(r'$v\ (m/s)$')
    axarray_evel[1].set_ylabel(r'$\omega\ (rad/s)$')
    axarray_evel[0].set_title('Linear velocity')
    axarray_evel[1].set_title('Angular velocity')
    #axarray_evel[0].set_xlim([0.0, 24.0])
    #axarray_evel[1].set_xlim([0.0, 24.0])

    hand, lab = axarray_evel[1].get_legend_handles_labels()
    axarray_evel[1].legend(hand, lab, ncol=1, prop={'size':12}, loc=1)

    hand, lab = axarray_epose[0].get_legend_handles_labels()
    axarray_epose[0].legend(hand, lab, ncol=1, prop={'size':12}, loc=2)
    hand, lab = axarray_epose[1].get_legend_handles_labels()
    axarray_epose[1].legend(hand, lab, ncol=1, prop={'size':12}, loc=2)
    hand, lab = axarray_epose[2].get_legend_handles_labels()
    axarray_epose[2].legend(hand, lab, ncol=1, prop={'size':12}, loc=2)

    hand, lab = axarray_pose[0].get_legend_handles_labels()
    axarray_pose[0].legend(hand, lab, ncol=1, prop={'size':12}, loc=2)
    hand, lab = axarray_pose[1].get_legend_handles_labels()
    axarray_pose[1].legend(hand, lab, ncol=1, prop={'size':12}, loc=2)
    hand, lab = axarray_pose[2].get_legend_handles_labels()
    axarray_pose[2].legend(hand, lab, ncol=1, prop={'size':12}, loc=2)

    hand, lab = ax.get_legend_handles_labels()
    ax.legend(hand, lab, ncol=1, prop={'size':12}, loc=2)

    #axarray[1+1].set_ylim(-6,6)

    #fig2.set_size_inches(1.2*18.5/2.54,1.2*10.5/2.54)

    #fig2.savefig(direc+'/ang.pdf', bbox_inches='tight', dpi=300)
    #fig2.savefig(direc+'/ang.png', bbox_inches='tight', dpi=300)
    fig.savefig(direc+'build/out/Release/bin/'+'path'+ chld.tag +'.pdf', bbox_inches='tight', dpi=300)
    fig.savefig(direc+'build/out/Release/bin/'+'path'+ chld.tag +'.png', bbox_inches='tight', dpi=300)
    fig_evel.savefig(direc+'build/out/Release/bin/'+'vw'+ chld.tag +'.pdf', bbox_inches='tight', dpi=300)
    fig_evel.savefig(direc+'build/out/Release/bin/'+'vw'+ chld.tag +'.png', bbox_inches='tight', dpi=300)
    fig_epose.savefig(direc+'build/out/Release/bin/'+'exytheta'+ chld.tag +'.pdf', bbox_inches='tight', dpi=300)
    fig_epose.savefig(direc+'build/out/Release/bin/'+'exytheta'+ chld.tag +'.png', bbox_inches='tight', dpi=300)

    fig_pose.savefig(direc+'build/out/Release/bin/'+'xytheta'+ chld.tag +'.pdf', bbox_inches='tight', dpi=300)
    fig_pose.savefig(direc+'build/out/Release/bin/'+'xytheta'+ chld.tag +'.png', bbox_inches='tight', dpi=300)

# plt.show()
# raw_input("Press anything to quit")
