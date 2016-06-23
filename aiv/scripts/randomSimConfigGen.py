import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom
import sys
from optparse import OptionParser
import os
import numpy as np

def wrapToPi(angle):
    """ Map angles (:math:`\\theta \in R`) to signed angles
    (:math:`\\theta \in [-pi, +pi)`).
    """
    while angle < -np.pi:
        angle += 2*np.pi;
    while angle >= np.pi:
        angle -= 2*np.pi;
    return angle;

def _frange(initial, final, step):
		""" Float point range function with round at the int(round(1./step))+4 decimal position
		"""
		_range = []
		n = 0
		while n*step+initial < final:
			_range += [round(n*step+initial, 4+int(round(1./step)))]
			n+=1
		return _range

###############################################################################
# Boundary
###############################################################################
class Boundary(object):
	def __init__(self, x, y):
		self.x_min = x[0]
		""" Min bound in the x direction.
		"""
		self.x_max = x[1]
		""" Max bound in the x direction.
		"""
		self.y_min = y[0]
		""" Min bound in the y direction.
		"""
		self.y_max = y[1]
		""" Max bound on y direction.
		"""

def add_cmdline_options(parser):

	parser.add_option('-s', '--simspeed', dest='simSpeed', default=1.0, action='store', type='float', help='simulation time / wallclock time (if simspeed=1, 1 simulated second takes 1 real second). The max speed will be limited by the PC specs')

	parser.add_option('-t', '--timestep', dest='timeStep', default=0.01, action='store', type='float', help='XDE parameter specifying the period of physics evaluation (discrete step)')

	parser.add_option("-d", "--startdelay", dest='startDelay', default=1., action='store', type='float', help='Delay in simulated seconds before starting to apply the control input')

	parser.add_option('-D', '--basedirec', dest='direc', help='path to XDE project', metavar='PATH', default='C:/Users/JM246044/workspace/dev/xde/xde/xde/xde/')

	parser.add_option('-b', '--robots', dest='nRobots', default=3, action='store', type='int', help='number of robots')

	parser.add_option('-o', '--obstacles', dest='nObsts', default=6, action='store', type='int', help='number of obstacles')

	parser.add_option('-c', '--comphorizon', dest='timeC', action='store', type='float', help='computation time horizon', default=0.3)

	parser.add_option('-p', '--planhorizon', dest='timeP', default=1.2, action='store', type='float', help='planning time horizon')

	parser.add_option("-w", "--waitforthreads", dest='waitForThreads', action='store_true', default=False, help='waith for planning threads to complete planning even when computing horizon time-out is triggered')

	parser.add_option('-N', '--timesamplingopt', dest='nSamples', default=14, action='store', type='int', help='number of time samples used for optimization')

	parser.add_option('-k', '--knots', dest='nKnots', default=5, action='store', type='int', help='number of internal knots')

	parser.add_option('-l', '--lastsecmindist', dest='terminationDist', default=0.3, action='store', type='float',help='minimal distance left for completing the last section of the planning (in meters)')

	parser.add_option('-i', '--irdist', dest='interRobDist', default=0.05, action='store', type='float', help='minimal allowed distance between two robots (in meters)')

	parser.add_option('-O', '--rodist', dest='robotObstDist', default=0.05, action='store', type='float', help='minimal allowed distance between a robot and a obstacle (in meters)')

	parser.add_option('-v', '--deviation', dest='deps', default=5.0, action='store', type='float', help='path deviation from initial one when dealing with conflict (in meters)')

	parser.add_option('-r', '--detectionradius', dest='detectionRadius', default=6.0, action='store', type='float', help='detection radius within which the robot can detect an obstacle (in meters)')

	parser.add_option("-m", "--optmethod", action="store", type="string", dest="optimizer", default='SLSQP')

	parser.add_option('-x', '--xtol', dest='xtol', default=1E-6, action='store', type='float', help='optimization tolerance for parameters')
	parser.add_option('-a', '--ftol', dest='ftol', default=0.0, action='store', type='float', help='optimization tolerance for cost function')
	parser.add_option('-e', '--eqtol', dest='eqtol', default=1E-4, action='store', type='float', help='optimization tolerance for equations constraints')
	parser.add_option('-q', '--ineqtol', dest='ineqtol', default=1E-4, action='store', type='float', help='optimization tolerance for inequations constraints')

	parser.add_option("-C", "--ctrltype", action="store", type="string", dest="controller", default='NCGPC')

	parser.add_option("-U", "--ctrlmaxout1", action="store", type="float", dest="cU1", default=3.0)

	parser.add_option("-u", "--ctrlmaxout2", action="store", type="float", dest="cU2", default=18.0)

	parser.add_option("-f", "--predictionhor", action="store", type="float", dest="predicHor", default=0.3)

	# parser.add_option("-R", "--numderivepsfbd", action="store", type="float", dest="numDerivEpsFBD", default=1e-6)
	parser.add_option("-T", "--numderivepscd", action="store", type="float", dest="numDerivEpsCD", default=1e-6)

	parser.add_option("-X", "--firstguessmixcte", action="store", type="float", dest="firstGuessMixCte", default=0.1)

	return

def randRobots(no, boundary, rad):

	arb_max_no = int(round((boundary.x_max-boundary.x_min)*(boundary.y_max-boundary.y_min)/(np.pi/2.*rad**2)/10.))
	if no > arb_max_no:
		print('Too many obstacles for the given boundary.')
		print('Using {:.0f} obstacles instead of {:.0f}.'.format(arb_max_no, no))
		no = arb_max_no

	def _isrobotok(robots, c, r, cf):
		""" Vefify if random generated obstacle is ok (not touching another obstacle)
		"""
		# print "LEN ROB: ", len(robots)
		if len(robots) > 0:
			for robot in robots:
				if (c[0]-robot[0][0])**2 + (c[1]-robot[0][1])**2 < (r+robot[1]+0.5)**2 or \
					(cf[0]-robot[3][0])**2 + (cf[1]-robot[3][1])**2 < (r+robot[1]+0.5)**2 :
					# print "robot is not ok"
					return False
			if (cf[0] > boundary.x_max or cf[0] < boundary.x_min or cf[1] > boundary.y_max or cf[1] > boundary.y_min):
				# print "robot is not ok"
				return False
		return True

	resol = 0.001 # meters

	x_range_center = boundary.x_min/2. + boundary.x_max/2.
	y_range_center = boundary.y_min/2. + boundary.y_max/2.

	x_range = _frange(boundary.x_min+rad, x_range_center-2, resol) + _frange(x_range_center+2, boundary.x_max-rad, resol)
	y_range = _frange(boundary.y_min+rad, y_range_center-2, resol) + _frange(y_range_center+2, boundary.y_max-rad, resol)
	# t_range = _frange(-np.pi, np.pi - resol, resol)
	dist_range = _frange(6, 16.0, resol)

	robots = []
	i=0
	while i < no:
		np.random.seed()
		r = rad
		
		x = np.random.choice(x_range)
		y = np.random.choice(y_range)

		if x < x_range_center and y < y_range_center:
			t = np.random.choice(_frange(0.5, 0.8, resol))
		elif x > x_range_center and y > y_range_center:
			t = np.random.choice(_frange(-2.84, -2.2, resol))
		elif x > x_range_center and y < y_range_center:
			t = np.random.choice(_frange(2.2, 2.84, resol))
		elif x < x_range_center and y > y_range_center:
			t = np.random.choice(_frange(-0.8, -0.5, resol))
		else:
			t = np.random.choice(_frange(-np.pi, np.pi - resol, resol))

		dist = np.random.choice(dist_range)

		xf = np.cos(t)*dist+x
		yf = np.sin(t)*dist+y

		# print [x, y], r, [xf, yf]
		# raw_input()
		# input("Press Enter to continue...")

		# if _isrobotok(robots, [x, y], r, [xf, yf]):
			# print "ROBOT OK"
		robots += [([x, y], r, t, [xf, yf])]
		i += 1
	return robots

def randObsts(no, boundary, robots, min_radius=0.15, max_radius=0.40):

	arb_max_no = int(round((boundary.x_max-boundary.x_min)*(boundary.y_max-boundary.y_min)/(np.pi/2.*min_radius**2)/10.))
	print 'arb_max_no', arb_max_no
	print 'no', no
	if no > arb_max_no:
		print('Too many obstacles for the given boundary.')
		print('Using {:.0f} obstacles instead of {:.0f}.'.format(arb_max_no, no))
		no = arb_max_no

	def _isobstok(obsts, robots, c, r):
		""" Vefify if random generated obstacle is ok (not touching another obstacle)
		"""
		if len(obsts) > 0:
			for obst in obsts:
				if (c[0]-obst[0][0])**2 + (c[1]-obst[0][1])**2 < (r+obst[1])**2:
					return False
			for robot in robots:
				if (c[0]-robot[0][0])**2 + (c[1]-robot[0][1])**2 < (r+robot[1]+2.0)**2 or \
					(c[0]-robot[3][0])**2 + (c[1]-robot[3][1])**2 < (r+robot[1]+2.0)**2:
					return False
		return True

	resol = 0.0001 # meters

	radius_range = _frange(min_radius, max_radius, resol)
	x_range = _frange(boundary.x_min+max_radius, boundary.x_max-max_radius, resol)
	y_range = _frange(boundary.y_min+max_radius, boundary.y_max-max_radius, resol)

	obsts = []
	i=0
	while i < no:
		np.random.seed()
		x = np.random.choice(x_range)
		y = np.random.choice(y_range)
		r = np.random.choice(radius_range)
		if _isobstok(obsts, robots, [x, y], r):
			obsts += [([x, y], r)]
			i += 1
	return obsts

parser = OptionParser()
add_cmdline_options(parser)
(options, args) = parser.parse_args()

print options

# Parse existing config.xml file
root = ET.parse(options.direc+'src/aiv/config.xml').getroot()



xml_root = ET.Element('root')

ET.SubElement(xml_root, 'simspeed').text = str(options.simSpeed);
timestep = ET.SubElement(xml_root, 'timestep')
timestep.text = str(options.timeStep);
timestep.set('unit', 's')
timeoffset = ET.SubElement(xml_root, 'timeoffset')
timeoffset.text = str(options.startDelay);
timeoffset.set('unit', 's')
aivs = ET.SubElement(xml_root, 'aivs')
obsts = ET.SubElement(xml_root, 'obstacles')
mpmeth = ET.SubElement(xml_root, 'mpmethod')
mpmeth.set('type', 'receding horizon with termination')
ctrller = ET.SubElement(xml_root, 'controller')
ctrller.set('type', options.controller)

robotsInfo = randRobots(options.nRobots, Boundary([-7.0, 7.0], [-7.0, 7.0]), 2.5*0.19664)

# robotsInfo = [([5.656854, -5.656854], 2.5*0.19664, 2.35619449, [-5.656854, 5.656854]),
	# ([-5.656854, -5.656854], 2.5*0.19664, 0.785398, [5.0, 6.0]), ([0.0, -8.0], 2.5*0.19664, 1.5708, [0.0, 8.0])]

robotsInfo = robotsInfo[0:options.nRobots]

obstsInfo = randObsts(options.nObsts, Boundary([-5.0, 5.0], [-5.0, 5.0]), robotsInfo, 0.4, 1.5)

print obstsInfo
print robotsInfo

robotsColors = ['xde/GreenOpaque', 'xde/YellowOpaque', 'xde/RedOpaque', 'xde/BlueOpaque', 'xde/OrangeOpaque']

for i in range(len(robotsInfo)):
	cent = robotsInfo[i][0]
	rad = robotsInfo[i][1]

	aiv = ET.SubElement(aivs, 'aiv') 
	
	aux = ET.SubElement(aiv, 'frame')
	ET.SubElement(aux, 'cadmodel').text = options.direc+'share/resources/scenes/adeptlynx/frame.dae'
	ET.SubElement(aux, 'mass').text = '55.0'
	ET.SubElement(aux, 'color').text = robotsColors[i%len(robotsColors)]

	aux = ET.SubElement(aiv, 'leftdrivewheel')
	ET.SubElement(aux, 'cadmodel').text = options.direc+'share/resources/scenes/adeptlynx/drivewheel.dae'
	ET.SubElement(aux, 'mass').text = '1.5'
	ET.SubElement(aux, 'color').text = 'xde/RedOpaque'
	ET.SubElement(aux, 'radius').text = '0.096845'
	ET.SubElement(aux, 'material').text = 'rubber'

	aux = ET.SubElement(aiv, 'rightdrivewheel')
	ET.SubElement(aux, 'cadmodel').text = options.direc+'share/resources/scenes/adeptlynx/drivewheel.dae'
	ET.SubElement(aux, 'mass').text = '1.5'
	ET.SubElement(aux, 'color').text = 'xde/BlueOpaque'
	ET.SubElement(aux, 'radius').text = '0.096845'
	ET.SubElement(aux, 'material').text = 'rubber'

	aux = ET.SubElement(aiv, 'leftfrontfreewheel')
	ET.SubElement(aux, 'cadmodel').text = options.direc+'share/resources/scenes/adeptlynx/freewheel.dae'
	ET.SubElement(aux, 'mass').text = '0.5'
	ET.SubElement(aux, 'color').text = 'xde/YellowOpaque'
	ET.SubElement(aux, 'radius').text = '0.0375'
	ET.SubElement(aux, 'material').text = 'rubber'

	aux = ET.SubElement(aiv, 'leftbackfreewheel')
	ET.SubElement(aux, 'cadmodel').text = options.direc+'share/resources/scenes/adeptlynx/freewheel.dae'
	ET.SubElement(aux, 'mass').text = '0.5'
	ET.SubElement(aux, 'color').text = 'xde/YellowOpaque'
	ET.SubElement(aux, 'radius').text = '0.0375'
	ET.SubElement(aux, 'material').text = 'rubber'

	aux = ET.SubElement(aiv, 'rightfrontfreewheel')
	ET.SubElement(aux, 'cadmodel').text = options.direc+'share/resources/scenes/adeptlynx/freewheel.dae'
	ET.SubElement(aux, 'mass').text = '0.5'
	ET.SubElement(aux, 'color').text = 'xde/YellowOpaque'
	ET.SubElement(aux, 'radius').text = '0.0375'
	ET.SubElement(aux, 'material').text = 'rubber'

	aux = ET.SubElement(aiv, 'rightbackfreewheel')
	ET.SubElement(aux, 'cadmodel').text = options.direc+'share/resources/scenes/adeptlynx/freewheel.dae'
	ET.SubElement(aux, 'mass').text = '0.5'
	ET.SubElement(aux, 'color').text = 'xde/YellowOpaque'
	ET.SubElement(aux, 'radius').text = '0.0375'
	ET.SubElement(aux, 'material').text = 'rubber'

	ET.SubElement(aiv, 'track').text = '0.19664'

	ET.SubElement(aiv, 'detectionradius').text = str(options.detectionRadius)

	aux = ET.SubElement(aiv, 'initpose')

	aux2 = ET.SubElement(aux, 'x')
	aux2.set('unit', 'm')
	aux2.text = str(robotsInfo[i][0][0])

	aux2 = ET.SubElement(aux, 'y')
	aux2.set('unit', 'm')
	aux2.text = str(robotsInfo[i][0][1])

	aux2 = ET.SubElement(aux, 'theta')
	aux2.set('unit', 'rad')
	aux2.text = str(robotsInfo[i][2])

	aux = ET.SubElement(aiv, 'initvelo')

	aux2 = ET.SubElement(aux, 'linear')
	aux2.set('unit', 'mps')
	aux2.text = "0.0"

	aux2 = ET.SubElement(aux, 'angular')
	aux2.set('unit', 'radps')
	aux2.text = "0.0"

	aux = ET.SubElement(aiv, 'goalpose')

	aux2 = ET.SubElement(aux, 'x')
	aux2.set('unit', 'm')
	aux2.text = str(robotsInfo[i][3][0])

	aux2 = ET.SubElement(aux, 'y')
	aux2.set('unit', 'm')
	aux2.text = str(robotsInfo[i][3][1])

	aux2 = ET.SubElement(aux, 'theta')
	aux2.set('unit', 'rad')
	aux2.text = str(robotsInfo[i][2])
	
	aux = ET.SubElement(aiv, 'goalvelo')

	aux2 = ET.SubElement(aux, 'linear')
	aux2.set('unit', 'mps')
	aux2.text = "0.0"

	aux2 = ET.SubElement(aux, 'angular')
	aux2.set('unit', 'radps')
	aux2.text = "0.0"

	aux = ET.SubElement(aiv, 'maxvelo')

	aux2 = ET.SubElement(aux, 'linear')
	aux2.set('unit', 'mps')
	aux2.text = "1.0"

	aux2 = ET.SubElement(aux, 'angular')
	aux2.set('unit', 'radps')
	aux2.text = "8.0"

	aux = ET.SubElement(aiv, 'maxacc')

	aux2 = ET.SubElement(aux, 'linear')
	aux2.set('unit', 'mps2')
	aux2.text = "0.8"

	aux2 = ET.SubElement(aux, 'angular')
	aux2.set('unit', 'radps2')
	aux2.text = "2.0"

	aux = ET.SubElement(aiv, 'dynmodelparam')

	ET.SubElement(aux, 'p1').text = " 0.04200441"
	ET.SubElement(aux, 'p2').text = " 0.27468742"
	ET.SubElement(aux, 'p3').text = "-0.01248822"
	ET.SubElement(aux, 'p4').text = " 1.00119437"
	ET.SubElement(aux, 'p5').text = " 0.00545974"
	ET.SubElement(aux, 'p6').text = " 1.03107639"
	
for i in range(options.nObsts):
	
	circular = ET.SubElement(obsts, 'circular')

	ET.SubElement(circular, 'cadmodel').text = options.direc+'share/resources/scenes/cilinder_.dae'
	# ET.SubElement(circular, 'scale').text = str(obstsInfo[i][1])

	aux = ET.SubElement(circular, 'cmposition')

	aux2 = ET.SubElement(aux, 'x')
	aux2.set('unit', 'm')
	aux2.text = str(obstsInfo[i][0][0])
	aux2 = ET.SubElement(aux, 'y')
	aux2.set('unit', 'm')
	aux2.text = str(obstsInfo[i][0][1])
	aux2 = ET.SubElement(aux, 'z')
	aux2.set('unit', 'm')
	aux2.text = "0.3"

	aux = ET.SubElement(circular, 'radius')
	aux.set('unit', 'm')
	aux.text = str(obstsInfo[i][1])

	ET.SubElement(circular, 'mass').text = '55.0'
	ET.SubElement(circular, 'material').text = 'rubber'
	ET.SubElement(circular, 'color').text = 'xde/BlackOpaque'


aux = ET.SubElement(mpmeth, 'comphorizon')
aux.set('unit', 's')
aux.text = str(options.timeC)

aux = ET.SubElement(mpmeth, 'planninghorizon')
aux.set('unit', 's')
aux.text = str(options.timeP)

ET.SubElement(mpmeth, 'waitforthread').text = 'true' if options.waitForThreads else 'false'
ET.SubElement(mpmeth, 'sampling').text = str(options.nSamples)
ET.SubElement(mpmeth, 'interknots').text = str(options.nKnots)
aux = ET.SubElement(mpmeth, 'terminationdist')
aux.text = str(options.terminationDist)
aux.set('unit', 'm')
aux = ET.SubElement(mpmeth, 'interrobotsafetydist')
aux.text = str(options.interRobDist)
aux.set('unit', 'm')

aux = ET.SubElement(mpmeth, 'robotobstaclesafetydist')
aux.text = str(options.robotObstDist)
aux.set('unit', 'm')

aux.set('unit', 'm')
aux = ET.SubElement(mpmeth, 'conflictfreepathdeviation')
aux.text = str(options.deps)
aux.set('unit', 'm')

# ET.SubElement(mpmeth, 'numderivativeepsfbd').text = str(options.numDerivEpsFBD)

ET.SubElement(mpmeth, 'numderivativeepscd').text = str(options.numDerivEpsCD)

ET.SubElement(mpmeth, 'firstguessmixcte').text = str(options.firstGuessMixCte)

optimizer = ET.SubElement(mpmeth, 'optimizer')
optimizer.set('type', options.optimizer)

aux = ET.SubElement(optimizer, 'maxiteration')
ET.SubElement(aux , 'first').text = '100'
ET.SubElement(aux , 'inter').text = '100'
ET.SubElement(aux , 'last').text = '100'

ET.SubElement(optimizer , 'xtolerance').text = str(options.xtol)
ET.SubElement(optimizer , 'ftolerance').text = str(options.ftol)
ET.SubElement(optimizer , 'eqtolerance').text = str(options.eqtol)
ET.SubElement(optimizer , 'ineqtolerance').text = str(options.ineqtol)

threshold = ET.SubElement(ctrller, 'threshold')
ET.SubElement(threshold, 'u1').text = str(options.cU1)
ET.SubElement(threshold, 'u2').text = str(options.cU2)

ET.SubElement(ctrller, 'predictionhorizon').text = str(options.predicHor)


tree = ET.ElementTree(xml_root)
# tree.write('output.xml')

f = open(options.direc + 'src/aiv/config.xml', 'w')
f.write(minidom.parseString(ET.tostring(xml_root, encoding='utf-8')).toprettyxml(indent="\t"))

# os.chdir(options.direc + 'build/out/Release/bin/')
# os.system(options.direc + 'build/out/Release/bin/AIV-simpleapp.exe')