import numpy as np
import numpy.linalg as LA
import matplotlib as mpl
import matplotlib.pyplot as plt
import os
import csv
import xml.etree.ElementTree as ET
from optparse import OptionParser
#C:\Users\JM246044\workspace\dev\xde\xde\xde\xde\build\out\Release\bin\..\..\..\..\src\aiv\

def wrapTo2Pi(angle):
    """ Map angles (:math:`\\theta \in R`) to unsigned angles
    (:math:`\\theta \in [0, 2\pi)`).

    .. note:: Method not used.
    """
    while angle < 0.0:
        angle += 2*np.pi
    while angle >= 2*np.pi:
        angle -= 2*np.pi
    return 2.*np.pi+angle if angle < 0.0 else angle

def wrapToPi(angle):
    """ Map angles (:math:`\\theta \in R`) to signed angles
    (:math:`\\theta \in [-pi, +pi)`).
    """
    while angle < -np.pi:
        angle += 2*np.pi;
    while angle >= np.pi:
        angle -= 2*np.pi;
    return angle;


def add_cmdline_options(parser):
        parser.add_option("-f", "--frames", dest='genFrames',
                action='store_true', default=False,
                help='save each frame for posterior video generation')
        #parser.add_option('-r', '--robotsradius', dest='rho', default=0.4, action='store', type='float', help='')
        return

#scriptname = sys.argv[0]

parser = OptionParser()
add_cmdline_options(parser)
(options, args) = parser.parse_args()


font = {'family' : 'sans-serif',
        'weight' : 'normal',
        'size'   : 10}

mpl.rc('font', **font)

class Robots:
    def __init__(self, myID, direc):

        self.id = myID

        with open(direc+'build/out/Release/bin/pl_ts_AdeptLynx'+ str(myID) + '.csv', 'rb') as csvfile:
            treader = csv.reader(csvfile, delimiter=',',quotechar='|',quoting=csv.QUOTE_NONE)
            tlist = list(treader)
            rows = len(tlist)-1 # last line can be incomplete (e.g. if program stopped with ctrl-c)
            cols = len(tlist[0][0:-1]) # last column has nothing, only \n
            self.plTabl = np.zeros((rows,cols))
            for i in range(rows):
                self.plTabl[i] = np.array([float(t) for t in tlist[i][0:-1]])

        with open(direc+'build/out/Release/bin/ctrl_ts_AdeptLynx' + str(myID) + '.csv', 'rb') as csvfile:
            treader = csv.reader(csvfile, delimiter=',',quotechar='|',quoting=csv.QUOTE_NONE)
            tlist = list(treader)
            rows = len(tlist)-1 # last line can be incomplete (e.g. if program stopped with ctrl-c)
            cols = len(tlist[0][0:-1]) # last column has nothing, only \n
            self.ctrlTabl = np.zeros((rows,cols))
            for i in range(rows):
                self.ctrlTabl[i] = np.array([float(t) for t in tlist[i][0:-1]])

        with open(direc+'build/out/Release/bin/real_ts_AdeptLynx' + str(myID) + '.csv', 'rb') as csvfile:
            treader = csv.reader(csvfile, delimiter=',',quotechar='|',quoting=csv.QUOTE_NONE)
            tlist = list(treader)
            rows = len(tlist)-1 # last line can be incomplete (e.g. if program stopped with ctrl-c)
            cols = len(tlist[0][0:-1]) # last column has nothing, only \n
            self.realTabl = np.zeros((rows,cols))
            for i in range(rows):
                self.realTabl[i] = np.array([float(t) for t in tlist[i][0:-1]])
        
        self.rows = min(min(self.plTabl.shape[0], self.realTabl.shape[0]), self.ctrlTabl.shape[0])

        self.resize()

    def resize(self):
        if self.plTabl.shape[0] < self.rows:
            for _ in range(self.rows-self.plTabl.shape[0]):
                self.plTabl = np.append(self.plTabl, [self.plTabl[-1,:]], axis=0)
        elif self.plTabl.shape[0] > self.rows:
            for _ in range(self.plTabl.shape[0]-self.rows):
                self.plTabl = np.delete(self.plTabl, -1, axis=0)
        if self.ctrlTabl.shape[0] < self.rows:
            for _ in range(self.rows-self.ctrlTabl.shape[0]):
                self.ctrlTabl = np.append(self.ctrlTabl, [self.ctrlTabl[-1,:]], axis=0)
        elif self.ctrlTabl.shape[0] > self.rows:
            for _ in range(self.ctrlTabl.shape[0]-self.rows):
                self.ctrlTabl = np.delete(self.ctrlTabl, -1, axis=0)
        if self.realTabl.shape[0] < self.rows:
            for _ in range(self.rows-self.realTabl.shape[0]):
                self.realTabl = np.append(self.realTabl, [self.realTabl[-1,:]], axis=0)
        elif self.realTabl.shape[0] > self.rows:
            for _ in range(self.realTabl.shape[0]-self.rows):
                self.realTabl = np.delete(self.realTabl, -1, axis=0)

    def planLinVelTS(self):
        return self.plTabl[:,4]
    def planAngVelTS(self):
        return self.plTabl[:,5]
    def planLinAccelTS(self):
        return self.plTabl[:,6]
    def planAngAccelTS(self):
        return self.plTabl[:,7]
    def planXTS(self):
        return self.plTabl[:,1]
    def planYTS(self):
        return self.plTabl[:,2]
    def planThetaTS(self):
        return self.plTabl[:,3]
    def ctrlLinVelTS(self):
        return self.ctrlTabl[:,1]
    def ctrlAngVelTS(self):
        return self.ctrlTabl[:,2]
    def realLinAccelTS(self):
        return self.realTabl[:,7]
    def realAngAccelTS(self):
        return self.realTabl[:,8]
    def realLinVelTS(self):
        return self.realTabl[:,5]
    def realAngVelTS(self):
        return self.realTabl[:,6]
    def realXTS(self):
        return self.realTabl[:,2]
    def realYTS(self):
        return self.realTabl[:,3]
    def realThetaTS(self):
        return self.realTabl[:,4]
    def xErrTS(self):
        # return self.realTabl[:,2] - self.plTabl[:,1]
        # return self.realTabl[:,2] - np.concatenate((np.array([self.realTabl[0,2]]), np.roll(self.plTabl[:,1], 1)[1:]))
        # Rolling planned vector for accounting for simulation step time times velocity
        return self.realTabl[:,2] - np.concatenate((np.roll(self.plTabl[:,1], -1)[:-1], np.array([self.realTabl[-1,2]])))
    def yErrTS(self):
        # return self.realTabl[:,3] - self.plTabl[:,2]
        # return self.realTabl[:,3] - np.concatenate((np.array([self.realTabl[0,3]]), np.roll(self.plTabl[:,2], 1)[1:]))
        # Rolling planned vector for accounting for simulation step time times velocity
        return self.realTabl[:,3] - np.concatenate((np.roll(self.plTabl[:,2], -1)[:-1], np.array([self.realTabl[-1,3]])))
    def posErrTS(self):
        return np.array([np.sqrt(ex**2+ey**2) for ex, ey in zip(self.xErrTS(), self.yErrTS())])
    def thetaErrTS(self):
        return np.array([wrapToPi(a-b) for a, b in zip(self.realTabl[:,4], self.plTabl[:,3])])
    def realRadTS(self):
        return self.realTabl[:,1]

    def plot(self, fig, time):
        ax = fig.gca()
        simTime = self.plTabl[:,0]
        self.plCirc = plt.Circle((self.planXTS()[time], self.planYTS()[time]), self.realRadTS()[time], color=plColors[self.id], ls = 'solid', fill=False)
        self.reCirc1 = plt.Circle((self.realXTS()[time], self.realYTS()[time]), self.realRadTS()[time], color=reColors[self.id], ls = 'solid', fill=True, alpha=0.1)
        self.reCirc2 = plt.Circle((self.realXTS()[time], self.realYTS()[time]), self.realRadTS()[time], color=reColors[self.id], ls = 'solid', fill=False)

        triaVert = np.array(
            [[self.realRadTS()[time]*np.cos(self.planThetaTS()[time])+self.planXTS()[time], \
            self.realRadTS()[time]*np.sin(self.planThetaTS()[time])+self.planYTS()[time]],
            [self.realRadTS()[time]*np.cos(self.planThetaTS()[time]-2.5*np.pi/3.0)+self.planXTS()[time], \
            self.realRadTS()[time]*np.sin(self.planThetaTS()[time]-2.5*np.pi/3.0)+self.planYTS()[time]],
            [self.realRadTS()[time]*np.cos(self.planThetaTS()[time]+2.5*np.pi/3.0)+self.planXTS()[time], \
            self.realRadTS()[time]*np.sin(self.planThetaTS()[time]+2.5*np.pi/3.0)+self.planYTS()[time]]])

        self.plTria = plt.Polygon(triaVert, color=plColors[self.id], fill=True, alpha=0.2)

        triaVert = np.array(
            [[self.realRadTS()[time]*np.cos(self.realThetaTS()[time])+self.realXTS()[time], \
            self.realRadTS()[time]*np.sin(self.realThetaTS()[time])+self.realYTS()[time]],
            [self.realRadTS()[time]*np.cos(self.realThetaTS()[time]-2.5*np.pi/3.0)+self.realXTS()[time], \
            self.realRadTS()[time]*np.sin(self.realThetaTS()[time]-2.5*np.pi/3.0)+self.realYTS()[time]],
            [self.realRadTS()[time]*np.cos(self.realThetaTS()[time]+2.5*np.pi/3.0)+self.realXTS()[time], \
            self.realRadTS()[time]*np.sin(self.realThetaTS()[time]+2.5*np.pi/3.0)+self.realYTS()[time]]])

        self.reTria = plt.Polygon(triaVert, color=reColors[self.id], fill=True, alpha=0.5)

        ax.add_artist(self.plCirc)
        ax.add_artist(self.reCirc1)
        ax.add_artist(self.reCirc2)
        ax.add_artist(self.plTria)
        ax.add_artist(self.reTria)

        if time == range(len(simTime))[-1]:
            ax.plot(self.planXTS()[0:time], self.planYTS()[0:time], color=plColors[self.id], label='planned trajectory')
            ax.plot(self.realXTS()[0:time], self.realYTS()[0:time], color=reColors[self.id], label='actual robot trajectory')
        else:
            ax.plot(self.planXTS()[0:time], self.planYTS()[0:time], color=plColors[self.id])
            ax.plot(self.realXTS()[0:time], self.realYTS()[0:time], color=reColors[self.id])

    def updatePlot(self, fig, time):
        simTime = self.plTabl[:,0]
        self.plCirc.center = self.planXTS()[time], self.planYTS()[time]
        self.reCirc1.center = self.realXTS()[time], self.realYTS()[time]
        self.reCirc2.center = self.realXTS()[time], self.realYTS()[time]
        triaVert = np.array(
            [[self.realRadTS()[time]*np.cos(self.planThetaTS()[time])+self.planXTS()[time], \
            self.realRadTS()[time]*np.sin(self.planThetaTS()[time])+self.planYTS()[time]],
            [self.realRadTS()[time]*np.cos(self.planThetaTS()[time]-2.5*np.pi/3.0)+self.planXTS()[time], \
            self.realRadTS()[time]*np.sin(self.planThetaTS()[time]-2.5*np.pi/3.0)+self.planYTS()[time]],
            [self.realRadTS()[time]*np.cos(self.planThetaTS()[time]+2.5*np.pi/3.0)+self.planXTS()[time], \
            self.realRadTS()[time]*np.sin(self.planThetaTS()[time]+2.5*np.pi/3.0)+self.planYTS()[time]]])
        self.plTria.set_xy(triaVert)
        triaVert = np.array(
            [[self.realRadTS()[time]*np.cos(self.realThetaTS()[time])+self.realXTS()[time], \
            self.realRadTS()[time]*np.sin(self.realThetaTS()[time])+self.realYTS()[time]],
            [self.realRadTS()[time]*np.cos(self.realThetaTS()[time]-2.5*np.pi/3.0)+self.realXTS()[time], \
            self.realRadTS()[time]*np.sin(self.realThetaTS()[time]-2.5*np.pi/3.0)+self.realYTS()[time]],
            [self.realRadTS()[time]*np.cos(self.realThetaTS()[time]+2.5*np.pi/3.0)+self.realXTS()[time], \
            self.realRadTS()[time]*np.sin(self.realThetaTS()[time]+2.5*np.pi/3.0)+self.realYTS()[time]]])
        self.reTria.set_xy(triaVert)
        #

        ax.plot(self.planXTS()[0:time], self.planYTS()[0:time], color=plColors[self.id]) #, label='planned trajectory')
        ax.plot(self.realXTS()[0:time], self.realYTS()[0:time], color=reColors[self.id]) #, label='actual robot trajectory')


class Obstacles:
    def __init__(self, myID, direc):

        self.id = myID

        with open(direc+'build/out/Release/bin/obst_ts_Obst' + str(myID) + '.csv', 'rb') as csvfile:
            treader = csv.reader(csvfile, delimiter=',',quotechar='|',quoting=csv.QUOTE_NONE)
            tlist = list(treader)
            rows = len(tlist)-1 # last line can be incomplete (e.g. if program stopped with ctrl-c)
            cols = len(tlist[0][0:-1]) # last column has nothing, only \n
            self.realTabl = np.zeros((rows,cols))
            for i in range(rows):
                self.realTabl[i] = np.array([float(t) for t in tlist[i][0:-1]])
        
        self.rows = self.realTabl.shape[0]
        # self.resize()

    def resize(self):
        if self.realTabl.shape[0] < self.rows:
            for _ in range(self.rows-self.realTabl.shape[0]):
                self.realTabl = np.append(self.realTabl, [self.realTabl[-1,:]], axis=0)
        elif self.realTabl.shape[0] > self.rows:
            for _ in range(self.realTabl.shape[0]-self.rows):
                self.realTabl = np.delete(self.realTabl, -1, axis=0)

    def realLinVelTS(self):
        return self.realTabl[:,5]
    def realAngVelTS(self):
        return self.realTabl[:,6]
    def realXTS(self):
        return self.realTabl[:,2]
    def realYTS(self):
        return self.realTabl[:,3]
    def realThetaTS(self):
        return self.realTabl[:,4]
    def realRadTS(self):
        return self.realTabl[:,1]

    def plot(self, fig, time):
        ax = fig.gca()
        self.innerCirc = plt.Circle((self.realXTS()[time], self.realYTS()[time]), self.realRadTS()[time], color='k', ls = 'solid', fill=True,  alpha=0.3)
        #self.outerCircs = plt.Circle((self.realXTS()[time], self.realYTS()[time]), rho+self.realRadTS()[time], color='k', ls = 'dashed', fill=False, alpha=0.3)
        
        ax.add_artist(self.innerCirc)
        #ax.add_artist(self.outerCirc)

    def updatePlot(self, fig, time):
        ax = fig.gca()
        self.innerCirc.center = self.realXTS()[time], self.realYTS()[time]
        #self.outerCirc.center = self.realXTS()[time], self.realYTS()[time]


direc = "C:/Users/JM246044/workspace/dev/xde/xde/xde/xde/"

try:
    os.mkdir(direc+'build/out/Release/bin/images/')
except OSError:
    print('Probably the output directory '+direc+'build/out/Release/bin/images/ already exists, going to overwrite content')

root = ET.parse(direc+'src/aiv/config.xml').getroot()
nbOfRobots = len(root.find('aivs'))

root = ET.parse(direc+'src/aiv/config.xml').getroot()
obstacles = root.find('obstacles')
nbOfObstacles = len(obstacles)

timeDim = -1
simulInfo = [[]]*(nbOfRobots+nbOfObstacles)

for idx in range(nbOfRobots):
    simulInfo[idx] = Robots(idx, direc)
    timeDim = simulInfo[idx].rows if simulInfo[idx].rows < timeDim or timeDim < 0.0 else timeDim

print 'TIME DIM', timeDim

plColors = [[i/2.0, 0.4, (1.0-i)/2.0] for i in np.linspace(0.0, 1.0, nbOfRobots)]
reColors = [[1.0 - i/2.0, 0.15, 1.0 - (1.0-i)/2.0] for i in np.linspace(0.0, 1.0, nbOfRobots)]

for idx in range(nbOfObstacles):
    simulInfo[idx+nbOfRobots] = Obstacles(idx, direc)

for sRSInfo in simulInfo:
    if sRSInfo.rows != timeDim:
        sRSInfo.rows = timeDim
        sRSInfo.resize()

########################################## PLOTS ####################################################

simTime = simulInfo[0].plTabl[:,0]

r2RDist = []
rPairs = []

for sRSInfo in simulInfo[0:nbOfRobots]:

    ########## POSE ERROR PLOT
    figErrPose, axArrayErrPose = plt.subplots(2, sharex=True)

    axArrayErrPose[0].grid()
    axArrayErrPose[1].grid()

    axArrayErrPose[0].set_ylim([-0.015, 0.015])
    axArrayErrPose[1].set_ylim([-0.1, 0.1])

    axArrayErrPose[0].plot(simTime, sRSInfo.posErrTS(), 'b', label=r'$\|[x_{err}\ y_{err}]^T\|$')

    axArrayErrPose[1].plot(simTime, sRSInfo.thetaErrTS(), 'b', label=r'$\theta_{err}$')

    hand, lab = axArrayErrPose[0].get_legend_handles_labels()
    axArrayErrPose[0].legend(hand, lab, ncol=1, prop={'size':10}, loc=2)
    hand, lab = axArrayErrPose[1].get_legend_handles_labels()
    axArrayErrPose[1].legend(hand, lab, ncol=1, prop={'size':10}, loc=2)

    figErrPose.savefig(direc+'build/out/Release/bin/images/exytheta'+ str(sRSInfo.id) +'.pdf', bbox_inches='tight', dpi=300)
    figErrPose.savefig(direc+'build/out/Release/bin/images/exytheta'+ str(sRSInfo.id) +'.png', bbox_inches='tight', dpi=300)
    ##########

    ########## VELOCITY PLOT
    figVel, axArrayVel = plt.subplots(2, sharex=True)

    axArrayVel[0].grid()
    axArrayVel[1].grid()

    axArrayVel[0].plot(simTime, sRSInfo.planLinVelTS(), 'b', label=r'$u_{ref}[0]$ (planner output 1)')
    axArrayVel[0].plot(simTime, sRSInfo.ctrlLinVelTS(), 'g', label=r'$u[0]$ (ctrller output 1)')
    axArrayVel[0].plot(simTime, sRSInfo.realLinVelTS(), 'r', label=r'actual lin vel')

    axArrayVel[1].plot(simTime, sRSInfo.planAngVelTS(), 'b', label=r'planner output')
    axArrayVel[1].plot(simTime, sRSInfo.ctrlAngVelTS(), 'g', label=r'ctrller output')
    axArrayVel[1].plot(simTime, sRSInfo.realAngVelTS(), 'r', label=r'actual vel')
    maxAngVel = max(max(max(sRSInfo.planAngVelTS()), max(sRSInfo.ctrlAngVelTS())), max(sRSInfo.realAngVelTS()))
    maxAngVel = maxAngVel if maxAngVel < 8.0 else 8.0
    minAngVel = min(min(min(sRSInfo.planAngVelTS()), min(sRSInfo.ctrlAngVelTS())), min(sRSInfo.realAngVelTS()))
    minAngVel = minAngVel if maxAngVel > -8.0 else -8.0
    axArrayVel[1].set_ylim([maxAngVel, minAngVel])

    hand, lab = axArrayVel[1].get_legend_handles_labels()
    axArrayVel[1].legend(hand, lab, ncol=1, prop={'size':10}, loc=1)

    figVel.savefig(direc+'build/out/Release/bin/images/vw'+ str(sRSInfo.id) +'.pdf', bbox_inches='tight', dpi=300)
    figVel.savefig(direc+'build/out/Release/bin/images/vw'+ str(sRSInfo.id) +'.png', bbox_inches='tight', dpi=300)
    ##########

    ########## ACCELERATION PLOT

    # compute approx. real acceleration from real velecity
    linVel = sRSInfo.realLinVelTS()
    approxLinAccelTS = [(linVel[1] - linVel[0])/(simTime[1] - simTime[0])] +\
            [(post - ant)/(tp - ta) for post, ant, tp, ta in zip(linVel[2:], linVel[0:-2], simTime[2:], simTime[0:-2])] +\
            [(linVel[-1] - linVel[-2])/(simTime[-1] - simTime[-2])]
    angVel = sRSInfo.realAngVelTS()
    approxAngAccelTS = [(angVel[1] - angVel[0])/(simTime[1] - simTime[0])] +\
            [(post - ant)/(tp - ta) for post, ant, tp, ta in zip(angVel[2:], angVel[0:-2], simTime[2:], simTime[0:-2])] +\
            [(angVel[-1] - angVel[-2])/(simTime[-1] - simTime[-2])]

    figAccel, axArrayAccel = plt.subplots(2, sharex=True)

    axArrayAccel[0].grid()
    axArrayAccel[1].grid()

    axArrayAccel[0].plot(simTime, sRSInfo.planLinAccelTS(), 'b', label=r'$u_{ref}[0]$ (planner linaccel)')
    axArrayAccel[0].plot(simTime, approxLinAccelTS, 'r', label=r'actual linaccel')

    axArrayAccel[1].plot(simTime, sRSInfo.planAngAccelTS(), 'b', label=r'planner angaccel')
    axArrayAccel[1].plot(simTime, approxAngAccelTS, 'r', label=r'actual angaccel')

    hand, lab = axArrayAccel[1].get_legend_handles_labels()
    axArrayAccel[1].legend(hand, lab, ncol=1, prop={'size':10}, loc=1)

    figAccel.savefig(direc+'build/out/Release/bin/images/aalpha'+ str(sRSInfo.id) +'.pdf', bbox_inches='tight', dpi=300)
    figAccel.savefig(direc+'build/out/Release/bin/images/aalpha'+ str(sRSInfo.id) +'.png', bbox_inches='tight', dpi=300)
    ##########

    ########## COMPUTE INTEROBOT DISTS
    for otherSRSInfo in simulInfo[sRSInfo.id+1:nbOfRobots]:

        rPairs += [(sRSInfo.id, otherSRSInfo.id)]

        myXTS = sRSInfo.realXTS()
        myYTS = sRSInfo.realYTS()
        otherXTS = otherSRSInfo.realXTS()
        otherYTS = otherSRSInfo.realYTS()
        myRadTS = sRSInfo.realRadTS()
        otherRadTS = otherSRSInfo.realRadTS()

        r2RDist += [[np.sqrt(dx**2+dy**2)-(myRad+otherRad) for dx, dy, myRad, otherRad in zip(myXTS - otherXTS, myYTS - otherYTS, myRadTS, otherRadTS)]]

########## INTERROBOT DISTANCES PLOT
figR2RDist = plt.figure()
axR2RDist = figR2RDist.gca()

for dist, pair in zip(r2RDist, rPairs):
    axR2RDist.plot(simTime, dist, label = r'$d(R_{0},R_{1})-\rho_{0}-\rho_{1}$'.format(pair[0], pair[1]))

axR2RDist.grid()
axR2RDist.set_xlabel('time (s)')
axR2RDist.set_ylabel('Inter-robot distance (m)')
axR2RDist.set_title('Inter-robot distances throughout simulation')
handles, labels = axR2RDist.get_legend_handles_labels()
axR2RDist.legend(handles, labels, loc=1, ncol=4, prop={'size':10})

figR2RDist.set_size_inches(1.0*18.5/2.54,1.0*6.5/2.54)
figR2RDist.savefig(direc+'build/out/Release/bin/images/interr.png', bbox_inches='tight', dpi=300)
figR2RDist.savefig(direc+'build/out/Release/bin/images/interr.pdf', bbox_inches='tight', dpi=300)

########## PATHS PLOT ######################################################################
fig = plt.figure()
ax = fig.gca()
ax.axis('equal')
#ax.grid()

# [obst.plot(fig, 0.0) for obst in obsts]
[sRSInfo.plot(fig, 0.0) for sRSInfo in simulInfo]

timeRange = range(len(simTime))[0::10] if options.genFrames == True else [range(len(simTime))[-1]]

for t, index in zip(timeRange, range(len(timeRange))):

    #[obst.updatePlot(fig, t) for obst in obsts]
    [sRSInfo.updatePlot(fig, t) for sRSInfo in simulInfo]

    ax.relim()
    ax.autoscale_view(True, True, True)
    # ax.set_ylim([-10, 10])
    # ax.set_xlim([-10, 10])
    fig.canvas.draw()

    # if t == timeRange[-1]:
    #     handles, labels = ax.get_legend_handles_labels()
    #     ax.legend(handles, labels, loc=1, ncol=3)

    fig.savefig(direc+'build/out/Release/bin/images/path'+str(index)+'.png', bbox_inches='tight', dpi=300)
    #fig.savefig(direc+'build/out/Release/bin/images/path'+str(t)+'.pdf', bbox_inches='tight', dpi=300)

[sRSInfo.updatePlot(fig, len(simTime)-1) for sRSInfo in simulInfo]

ax.relim()
ax.autoscale_view(True, True, True)
fig.canvas.draw()

#handles, labels = ax.get_legend_handles_labels()
#ax.legend(handles, labels, loc=1, ncol=3)

fig.savefig(direc+'build/out/Release/bin/images/path.png', bbox_inches='tight', dpi=300)
fig.savefig(direc+'build/out/Release/bin/images/path.pdf', bbox_inches='tight', dpi=300)
