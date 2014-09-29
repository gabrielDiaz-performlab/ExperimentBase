"""
This is designed to load in a system configuration file based on the name of the computer it's being run
on (as determined by platform.node()). If that file does not exist, defaultSys.cfg is loaded. It also sets up
that system in a way that tries not to be tied to a particular experiment to avoid each experiment replicating
this work (potentially with subtle differences). Finally, it reads in an experiment config file, which must conform
with the configspec in expCfgSpec.ini.

V 1.0 Matthew H. Tong (adapted from other code from Dmitry Kit)
V 1.1 Matthew H. Tong - Made it so VRLabConfig could be created without an experiment config, making it easier to use purely to handle the system config.
"""

import platform
import os.path
import sys
import viz
#Set the resources path
viz.res.addPath('resources')
sys.path.append('utils')
from configobj import ConfigObj
from configobj import flatten_errors
from validate import Validator
import vizact

class VRLabConfig:
	"""
	This class represents the configuration for a particular experiment. It has two main components: VRLabConfig.sysCfg
	and VRLabConfig.expCfg that contain the system configuration and the experiment configuration respectively.
	
	VRLabCong.start(): Final initialization after viz.go()
	
	VRLabConfig.sysCfg: The system configuration (a ConfigObj)
	VRLabConfig.expCfg: The experiment configuration (a ConfigObj)
	
	VRLabConfig.writables: A list of variables to be written to the .mov each frame
	VRLabConfig.HMD: The HMD (defined in this file). Is still an HMD even if not using HMD, just disabled.
	VRLabConfig.writer: The DVR writer. None if the writer is not being used or has not been initialized.
	VRLabConfig.eyeTrackingCal: The eyetracking interface. None if we aren't eyetracking
	VRLabConfig.camera: The virtual camera, positioned by headtracking. None if no such camera.
	VRLabConfig.bodyCam: A second virtual camera or tracker
	VRLabConfig.phase_space_markers: Markers for phasespace. Undefined if use_phasespace is false.
	VRLabConfig.phasespace_server: Server for phasespace. Undefined if use_phasespace is false.
	
	VRLabConfig.use_phasespace: True if phasespace is enabled
	VRLabConfig.use_HMD: True if HMD is being used (otherwise configure just for screen)
	VRLabConfig.use_hiball: True if hiball is enabled.
	VRLabConfig.use_eyetracking: True if eyetracking is enabled
	VRLabConfig.use_DVR: True if DVR should record the experiment
	"""
	
	def __init__(self, expCfgName = ""):
		"""
		Opens and interprets both the system config (as defined by the <platform>.cfg file) and the experiment config
		(as defined by the file in expCfgName). Both configurations MUST conform the specs given in sysCfgSpec.ini and
		expCfgSpec.ini respectively. It also initializes the system as specified in the sysCfg.
		"""
		
		self.writables = list()
		if expCfgName:
			self.__createExpCfg(expCfgName)
		else:
			self.expCfg = None
		self.__createSysCfg()
		
		for pathName in self.sysCfg['set_path']:
			viz.res.addPath(pathName)
		
		self.__setupSystem()
		
	def __createSysCfg(self):
		"""
		Set up the system config section (sysCfg)
		"""
		sysCfgName = platform.node()+".cfg"
		if not(os.path.isfile(sysCfgName)):
			sysCfgName = "defaultSys.cfg"
		print "Loading system config file: " + sysCfgName
		sysCfg = ConfigObj(sysCfgName, configspec='sysCfgSpec.ini', raise_errors = True)
		validator = Validator()
		sysCfgOK = sysCfg.validate(validator)
		if sysCfgOK == True:
			print "System config file parsed correctly"
		else:
			print 'System config file validation failed!'
			res = sysCfg.validate(validator, preserve_errors=True)
			for entry in flatten_errors(sysCfg, res):
			# each entry is a tuple
				section_list, key, error = entry
				if key is not None:
					section_list.append(key)
				else:
					section_list.append('[missing section]')
				section_string = ', '.join(section_list)
				if error == False:
					error = 'Missing value or section.'
				print section_string, ' = ', error
			sys.exit(1)
		self.sysCfg = sysCfg
		#self.writables.append(self.sysCfg)
		
	def __createExpCfg(self, expCfgName):
		"""
		Set up the experiment config section (expCfg)
		"""
		print "Loading experiment config file: " + expCfgName
		
		# This is where the parser is called.
		expCfg = ConfigObj(expCfgName, configspec='expCfgSpec.ini', raise_errors = True, file_error = True)

		validator = Validator()
		expCfgOK = expCfg.validate(validator)
		if expCfgOK == True:
			print "Experiment config file parsed correctly"
		else:
			print 'Experiment config file validation failed!'
			res = expCfg.validate(validator, preserve_errors=True)
			for entry in flatten_errors(expCfg, res):
			# each entry is a tuple
				section_list, key, error = entry
				if key is not None:
					section_list.append(key)
				else:
					section_list.append('[missing section]')
				section_string = ', '.join(section_list)
				if error == False:
					error = 'Missing value or section.'
				print section_string, ' = ', error
			sys.exit(1)
		if expCfg.has_key('_LOAD_'):
			for ld in expCfg['_LOAD_']['loadList']:
				print 'Loading: ' + ld + ' as ' + expCfg['_LOAD_'][ld]['cfgFile']
				curCfg = ConfigObj(expCfg['_LOAD_'][ld]['cfgFile'], configspec = expCfg['_LOAD_'][ld]['cfgSpec'], raise_errors = True, file_error = True)
				validator = Validator()
				expCfgOK = curCfg.validate(validator)
				if expCfgOK == True:
					print "Experiment config file parsed correctly"
				else:
					print 'Experiment config file validation failed!'
					res = curCfg.validate(validator, preserve_errors=True)
					for entry in flatten_errors(curCfg, res):
					# each entry is a tuple
						section_list, key, error = entry
						if key is not None:
							section_list.append(key)
						else:
							section_list.append('[missing section]')
						section_string = ', '.join(section_list)
						if error == False:
							error = 'Missing value or section.'
						print section_string, ' = ', error
					sys.exit(1)
				expCfg.merge(curCfg)
		
		self.expCfg = expCfg
		
		
		
	def __setupSystem(self):
		
		# Set up the wiimote
		################################################################
		################################################################
		##  Misc. Design specific items here.
		if( self.sysCfg['use_wiimote']):
			# Create wiimote holder
			self.wiimote = 0
			self._connectWiiMote()
		
		#Set up the HMD
		if self.sysCfg['use_hmd']:
			self.hmd = HMD(self.sysCfg['hmd'], True)
			self.use_HMD = True
		else:
			self.hmd = HMD(self.sysCfg['hmd'], False)
			self.use_HMD = False
		
		
		viz.window.setFullscreenMonitor(self.sysCfg['displays'])
		
		viz.setMultiSample(self.sysCfg['antiAliasPasses'])
		viz.MainWindow.clip(0.01 ,200)
		viz.vsync(1)
		
		viz.setOption("viz.glfinish", 1)
		viz.setOption("viz.dwm_composition", 0)
		
		#Set up recording
		if self.sysCfg['use_DVR'] == 1:
			self.use_DVR = True
		else:
			self.use_DVR = False
		self.writer = None #Will get initialized later when the system starts
		
		#Set up the eye tracking callibration/configuration (eyeTrackingCal)
		
		if self.sysCfg['use_eyetracking']:
			self.use_eyeTracking = True
			if self.sysCfg['hmd']['type'] == 'nvis':
			
				import EyeTrackerCalibrationNVIS_MT
				self.eyeTrackingCal = EyeTrackerCalibrationNVIS_MT.EyeTrackerCalibrationNVIS(self.sysCfg['eyetracker']['settingsDir'])
				self.eyeTrackingCal.changeDisplayNum(self.sysCfg['displays'][0])
				
				
				print "Eye tracking enabled using NVIS visor"
			else:
				print "Error in VRLabConfig: Eye-tracking not setup for this HMD."
				sys.exit(1)
		else:
			self.use_eyeTracking = False
			self.eyeTrackingCal = None
			
		#self.writables.append(self.eyeTrackingCal)
			
		self.mocap = None
		self.bodyCam = None
		
		if self.sysCfg['use_phasespace']:
			
			from mocapInterface import phasespaceInterface			
			self.mocap = phasespaceInterface(self.sysCfg);
				
			self.use_phasespace = True
		else:
			self.use_phasespace = False
		
		if self.sysCfg['use_hiball']:
			from HiBallCameraMT import HiBallCamera
			#self.mocap = HiBallCamera(self.sysCfg['hiball']['origin'], self.sysCfg['hiball']['scale'], None, None, self.sysCfg, None);
			self.mocap = HiBallCamera(self.sysCfg['hiball']['origin'], particle=None, sensorNum=self.sysCfg['hiball']['headCam'], attachTo=viz.MainView, preTrans = self.sysCfg['hiball']['preTransHead'])
			if self.sysCfg['hiball']['bodyCam'] != -1:
				self.bodyCam = HiBallCamera(self.sysCfg['hiball']['origin'], particle=None, sensorNum=self.sysCfg['hiball']['bodyCam'], attachTo=None, preTrans = self.sysCfg['hiball']['preTransBody'])
			else:
				self.bodyCam = None
			self.use_hiball = True
		else:
			self.use_hiball = False
		

		self.writables.append(self.mocap)
		self.writables.append(self.bodyCam)
		
		if self.sysCfg['use_fullscreen']:
			viz.go(viz.FULLSCREEN)
		else:
			viz.go()
		
		self._setWinPriority(priority=5)
		
			
	def start(self):
		if self.use_DVR:
			print "Starting DVR"
			from DVRwriter				import DVRwriter
			from datetime 				import datetime
			
			metadata = 'unused per-file meta data' #Can be filled in with useful metadata if desired.
			
			if None == self.writer: #need to lazy initialize this because it has to be called after viz.go()
				sz = viz.window.getSize()
				self.now = datetime.now()
				nameRoot = '%s%d.%d.%d.%d-%d' % (self.sysCfg['writer']['outFileDir'], self.now.year, self.now.month, self.now.day, self.now.hour, self.now.minute)
				outFile = '%s.%s' % (nameRoot, self.sysCfg['writer']['outFileName'])
				self.expCfg.filename = '%s.expCfg.cfg' % (nameRoot)
				self.sysCfg.filename = '%s.sysCfg.cfg' % (nameRoot)
				
				
				if 'L' == self.sysCfg['eyetracker']['eye']: 
					viewport = (0,      0,sz[0]/2,sz[1])
				else:               
					viewport = (sz[0]/2,0,sz[0]/2,sz[1])
				#fi
				
				print "OutfileName:" + self.sysCfg['writer']['outFileName']
				print "Metadata:" + metadata
				print "Viewport:" + str(viewport)
				print "Eyetracking:" + str(self.use_eyeTracking)
				
				self.writer = DVRwriter(outFile, metadata, viewport, self.use_eyeTracking)
				self.expCfg.write()
				self.sysCfg.write()
				
			self.writer.turnOn()
		
			
	def __record_data__(self, e):
		
		if self.use_DVR and self.writer != None:
			#print "Writing..."
			self.writer.write(self.writables)
	
	def _connectWiiMote(self):
		
		wii = viz.add('wiimote.dle')#Add wiimote extension
		
		# Replace old wiimote
		if( self.wiimote ):
			print 'Wiimote removed.'
			self.wiimote.remove()
			
		self.wiimote = wii.addWiimote()# Connect to first available wiimote
		
		vizact.onexit(self.wiimote.remove) # Make sure it is disconnected on quit
		
		self.wiimote.led = wii.LED_1 | wii.LED_4 #Turn on leds to show connection
	
	def _setWinPriority(self,pid=None,priority=1):
		
		""" Set The Priority of a Windows Process.  Priority is a value between 0-5 where
			2 is normal priority.  Default sets the priority of the current
			python process but can take any valid process ID. """
			
		import win32api,win32process,win32con
		
		priorityclasses = [win32process.IDLE_PRIORITY_CLASS,
						   win32process.BELOW_NORMAL_PRIORITY_CLASS,
						   win32process.NORMAL_PRIORITY_CLASS,
						   win32process.ABOVE_NORMAL_PRIORITY_CLASS,
						   win32process.HIGH_PRIORITY_CLASS,
						   win32process.REALTIME_PRIORITY_CLASS]
		if pid == None:
			pid = win32api.GetCurrentProcessId()
		
		handle = win32api.OpenProcess(win32con.PROCESS_ALL_ACCESS, True, pid)
		win32process.SetPriorityClass(handle, priorityclasses[priority])
	

class HMD:
	class HMDStats:
		def __init__(self, offset, overlap):
			self.offset = offset
			self.overlap = overlap
	
	def __init__(self, cfg, enabled=1):
		
		type = cfg['type']
		offset = cfg['offset']
		overlap = cfg['overlap']
		fov = cfg['fov']
		#self.hmdstats = self.HMDStats(offset, overlap)
		self.enabled = enabled
		
		if enabled:
			
			if 'nvis' == type:
				
				import nvis
				self.hmd = nvis.nvisorSX111()
				
				#vizconfig.register(self.hmd)
				print "HMD type: NVIS SX111"
				
				print '*** Offset:' + str( viz.MainWindow.getViewOffset() )
			elif 'oculus' == type:
			
				import oculus
				self.hmd = oculus.Rift()
				print "HMD type: Oculus Rift"
				#vizconfig.register(self.hmd)
				#self.updateOverlap(self.hmdstats.overlap-self.hmd._overlap)
			
			else:
				print "Unsupported HMD type when starting HMD"
				sys.exit(1)
			
			import vizconfig
			vizconfig.register(self.hmd)
			
			if(overlap > -1 ):
					self.updateOverlap(self.hmdstats.overlap-self.hmd._overlap)
			else:
				defaultOverlap = 100
				self.hmdstats = self.HMDStats(viz.MainWindow.getViewOffset, defaultOverlap)
				
			#print 'After: ' + str(viz.MainWindow.getViewOffset()) + str(viz.MainWindow.stereoOverlap())
			#self.hmdstats = self.HMDStats(viz.MainWindow.getViewOffset, viz.MainWindow.stereoOverlap)
			
			
		else:
			self.hmd = None
			print "HMD disabled"
			
			
		
	def updateOverlap(self, amount):
		if self.enabled:
			self.hmd._overlap = self.hmd._overlap + amount	
			offset = (self.hmd._hfov - self.hmd._overlap) / 2.0
			viz.MainWindow.setViewOffset(viz.Matrix.euler(-offset,0.0,self.hmd._leftRollShift), eye=viz.LEFT_EYE)
			viz.MainWindow.setViewOffset(viz.Matrix.euler( offset,0.0,self.hmd._rightRollShift),eye=viz.RIGHT_EYE)
			self.hmdstats.offset = offset
			self.hmdstats.overlap = self.hmd._overlap
			print "Overlap", self.hmdstats.overlap, "Offset", self.hmdstats.offset
			
		