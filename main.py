"""
Runs an experiment.
"""

import viz
import viztask
import vrlabConfig
import vizshape
import vizact
from drawNumberFromDist import *
import visEnv
import physEnv
import ode
import datetime
from ctypes import * # eyetrackka


expConfigFileName = 'badmintonTest.cfg'
#expConfigFileName = 'shortTest.cfg'
#expConfigFileName = 'fullTest.cfg'



ft = .3048
inch = 0.0254
m = 1
eps = .01
nan = float('NaN')

# Create a globally accessible soundbank.
# To access within a member function, 
# import the global variable with 'global soundbank'

vizact.onkeydown( 'm', viz.window.screenCapture, 'image.bmp' )

class soundBank():
	def __init__(self):
		
		################################################################
		################################################################
		## Register sounds.  It makes sense to do it once per experiment.
		
		self.bounce = viz.addAudio('/Resources/bounce.wav')
		self.buzzer = viz.addAudio('/Resources/BUZZER.wav')
		self.bubblePop = viz.addAudio('/Resources/bubblePop3.wav')
		self.cowbell = viz.addAudio('/Resources/cowbell.wav')
		self.highdrip = viz.addAudio('/Resources/highdrip.wav')
		self.gong = viz.addAudio('/Resources/gong.wav')
		#return soundBank

soundBank = soundBank()

class Experiment(viz.EventClass):
	
	"""
	Experiment manages the basic operation of the experiment.
	"""
	
	def __init__(self, config):
		
		# Event classes can register their own callback functions
		# This makes it possible to register callback functions (e.g. activated by a timer event)
		# within the class (that accept the implied self argument)
		# eg self.callbackFunction(arg1) would receive args (self,arg1)
		# If this were not an eventclass, the self arg would not be passed = badness.
		
		viz.EventClass.__init__(self)
		
		##############################################################
		##############################################################
		## Use config to setup hardware, motion tracking, frustum, eyeTrackingCal.
		##  This draws upon the system config to setup the hardware / HMD
		
		self.config = config 

		# Eventually, self.config.writables is passed to DVRwriter
		# self.config.writables is a list
		# dvrwriter will attempt to run .getOutput on every member of the list
		# One could then add experiment, theBall, theRacquet, eyeTrackingCal to the list, assuming 
		# they include the member function .getOutput().
		# I prefer to do all my data collection in one place: experiment.getOutput()
		
		self.config.writables = [self]
		
		################################################################
		################################################################
		## Set states
		
		self.inCalibrateMode = False
		self.inHMDGeomCheckMode = False
		self.setEnabled(False)
		self.test_char = None

		################################################################
		################################################################
		# Create visual and physical objects (the room)
	
		self.room = visEnv.room(config)
		
		# self.room.physEnv 
		self.hmdLinkedToView = False
		
		################################################################
		################################################################
		# Build block and trial list
		
		self.blockNumber = 0;
		self.trialNumber = 0;
		self.inProgress = True;
		
		self.blocks_bl = []
		
		for bIdx in range(len(config.expCfg['experiment']['blockList'])):
			self.blocks_bl.append(block(config,bIdx));
		
		self.currentTrial = self.blocks_bl[self.blockNumber].trials_tr[self.trialNumber]
		
#		################################################################
#		################################################################
#		##  Misc. Design specific items here.
#		if( config.sysCfg['use_wiimote ']):
#			# Create wiimote holder
#			self.wiimote = 0
#			self.connectWiiMote()
		
		if( config.wiimote ):
			self.registerWiimoteActions()
		
		# Setup launch trigger
		self.launchKeyIsCurrentlyDown = False
		
		self.minLaunchTriggerDuration = config.expCfg['experiment']['minLaunchTriggerDuration']
		
		# maxFlightDurTimerID times out balls a fixed dur after launch
		self.maxFlightDurTimerID = viz.getEventID('maxFlightDurTimerID') # Generates a unique ID. 
		
		################################################################
		##  LInk up the hmd to the mainview
		
		if( self.config.use_phasespace == True ):
			
			################################################################
			##  Link up the hmd to the mainview
			#if( self.config.mocap.mainViewLinkedToHead ):
			if( self.config.use_HMD and  config.mocap.returnPointerToRigid('hmd') ):
				#self.hmdLinkedToView = True
				self.config.mocap.enableHMDTracking()
				
				#vizact.onkeydown('-',self.config.mocap.disableHMDTracking)
				#vizact.onkeydown('=',self.config.mocap.enableHMDTracking)
			
			################################################################
			##  LInk up the paddle to the mocap
			# If there is a paddle visObj and a paddle rigid...
		
		self.setupPaddle()
				
#		self.discTarg = visEnv.visObj(self.room,'cylinder',[0.5,1])
#		self.discTarg.visNode.setPosition([0,1,20])
#		self.discTarg.visNode.setAxisAngle(1,0,0,90)
#		self.discTarg.applyVisToPhys()
		
			
		##############################################################
		##############################################################
		## Callbacks and timers
		
		vizact.onupdate(viz.PRIORITY_PHYSICS,self._checkForCollisions)
		
		#self.callback(viz.TIMER_EVENT, self.timer_event)
		self.callback(viz.KEYDOWN_EVENT,  self.onKeyDown)
		self.callback(viz.KEYUP_EVENT, self.onKeyUp)
		self.callback( viz.TIMER_EVENT,self._timerCallback )
		
		self.perFrameTimerID = viz.getEventID('perFrameTimerID') # Generates a unique ID. 
		self.starttimer( self.perFrameTimerID, viz.FASTEST_EXPIRATION, viz.FOREVER)
		
		# DVR snaps a shot of the frame, records eye data, and contents of self.writables is written out to the movie
		self.callback(viz.POST_SWAP_EVENT, self.config.__record_data__, viz.PRIORITY_LAST_UPDATE)
	
		# Use text output!
		if( config.sysCfg['use_DVR'] >0):
			
			vizact.ontimer(3,self.checkDVRStatus)
			
			now = datetime.datetime.now()
			dateTimeStr = str(now.year) + '-' + str(now.month) + '-' + str(now.day) + '-' + str(now.hour) + '-' + str(now.minute)
			
			dataOutPutDir = config.sysCfg['writer']['outFileDir']
			
			self.expDataFile = open(dataOutPutDir + 'exp_data-' + dateTimeStr + '.txt','a')
			
			if( self.config.sysCfg['use_eyetracking']):
				self.eyeDataFile = open(dataOutPutDir + 'eye_data-' + dateTimeStr + '.txt','a')
			
			vizact.onupdate(viz.PRIORITY_LAST_UPDATE,self.writeDataToText)
		
		# Create an event flag object
		# This var is set to an int on every frame
		# The int saves a record of what was happening on that frame
		# It can be configured to signify the start of a trial, the bounce of a ball, or whatever
		
		self.eventFlag = eventFlag()
		
	def _timerCallback(self,timerID):
		
		
		if( timerID == self.maxFlightDurTimerID ):
			
			
			#print 'Ball timed out. Removing ball!'
			
			self.currentTrial.removeBall()
			self.room.standingBox.visible( viz.TOGGLE )
			self.endTrial()
		
	def _checkForCollisions(self):
		
		thePhysEnv = self.room.physEnv;
		
		if( thePhysEnv.collisionDetected == False ): 
			# No collisions this time!
			return
		
		theFloor = self.room.floor
		theBackWall = self.room.wall_NegZ
		theBall = self.currentTrial.ballObj		
		thePaddle = self.room.paddle
		
		for idx in range(len(thePhysEnv.collisionList_idx_physNodes)):
			
			physNode1 = thePhysEnv.collisionList_idx_physNodes[idx][0]
			physNode2 = thePhysEnv.collisionList_idx_physNodes[idx][1]
			
			# BALL / FLOOR
			
			if( theBall > 0 ):
				if( self.currentTrial.ballHasBouncedOnFloor == False and
					(physNode1 == theFloor.physNode and physNode2 == theBall.physNode or 
					physNode1 == theBall.physNode and physNode2 == theFloor.physNode )):
						
					self.eventFlag.setStatus(3)
					
					self.currentTrial.ballHasBouncedOnFloor = True 
					 
					# This is an example of how to get contact information
					bouncePos_XYZ,normal,depth,geom1,geom2 = thePhysEnv.contactObjects_idx[0].getContactGeomParams()
					
					self.currentTrial.ballOnPaddlePos_XYZ = bouncePos_XYZ
					
					#print 'Ball has hit the ground.'
					soundBank.bounce.play()
					
					# Compare pre-bounce flight dur with predicted pre-bounce flight dur
					actualPreBounceFlightDur =  float(viz.getFrameTime()) - self.currentTrial.launchTime
					durationError = self.currentTrial.predictedPreBounceFlightDur - actualPreBounceFlightDur
					self.currentTrial.flightDurationError = durationError 
					
					print 'Predicted: ' + str(self.currentTrial.predictedPreBounceFlightDur)
					print 'Actual   : ' + str(actualPreBounceFlightDur)
					
					print 'Flight duration error: ' + str(durationError)
					
				# BALL / PADDLE
				if( self.currentTrial.ballHasHitPaddle == False and
					(physNode1 == thePaddle.physNode and physNode2 == theBall.physNode or 
					physNode1 == theBall.physNode and physNode2 == thePaddle.physNode )):
						
					self.eventFlag.setStatus(4)
					self.currentTrial.ballHasHitPaddle = True
					
					soundBank.cowbell.play()
					
					# self.ballObj.physNode.setStickUponContact( room.paddle.physNode.geom )
					if( theBall.physNode.queryStickyState(thePaddle.physNode) ):
					
						theBall.visNode.setParent(thePaddle.visNode)
						collPoint_XYZ = theBall.physNode.collisionPosLocal_XYZ
						theBall.visNode.setPosition(collPoint_XYZ)
						
						self.currentTrial.ballOnPaddlePosLoc_XYZ = collPoint_XYZ
						
						# If you don't set position in this way (on the next frame using vizact.onupdate),
						# then it doesn't seem to update correctly.  
						# My guess is that this is because the ball's position is updated later on this frame using
						# visObj.applyPhysToVis()
						
						vizact.onupdate(viz.PRIORITY_LINKS,theBall.visNode.setPosition,collPoint_XYZ[0],collPoint_XYZ[1],collPoint_XYZ[2])

				if( physNode1 == theBackWall.physNode and physNode2 == theBall.physNode or 
					physNode1 == theBall.physNode and physNode2 == theBackWall.physNode):
					
					self.eventFlag.setStatus(5)
					#print 'Ball has hit the back wall.'
					
					#currentTrial.removeBall()
					soundBank.bounce.play()

	def start(self):
		
		##This is called when the experiment should begin.
		self.setEnabled(True)
		self.config.start()

	def toggleEyeCalib(self):
		"""
		Toggles the calibration for eye tracking.
		Note, that for this to work, toggling 
		# self.config.camera must turn off your world model
		# This is setup in testRoom.init().
		
		# Example of what's needed in testRoom.init
		self.room = viz.addGroup()
		self.model = viz.add('pit.osgb',parent = self.room)
		"""
		
		if not self.config.mocap:
			pass
		#elif (self.config.mocap.isOn() == 1): 
		elif( self.config.mocap.mainViewUpdateAction == False):
			#self.config.mocap.turnOff()
			self.config.mocap.enableHMDTracking()
			#self.config.mocap.turnOn()
		else:
			self.config.mocap.disableHMDTracking()
			
		viz.mouse.setOverride(viz.TOGGLE)
		self.config.eyeTrackingCal.toggleCalib()
		
		self.inCalibrateMode = not self.inCalibrateMode
		
		if self.inCalibrateMode:
			viz.clearcolor(.5, .5, .5)
			viz.MainView.setPosition(0,0,0)
			viz.MainView.setAxisAngle(0, 1, 0, 0)
			viz.MainView.velocity([0,0,0]);
			
		else:
			viz.clearcolor(0, 0, 0)
			
		if self.room:
			#self.room.visible(viz.disable)
			self.room.walls.visible(viz.TOGGLE)
			self.room.objects.visible(viz.TOGGLE)
		
			
	def createCamera(self):
		"""
		Head camera is generally initialized as part of the system calls. Additional changes should be added here.
		"""
		pass		
		
	def onKeyDown(self, key):
		"""
		Interactive commands can be given via the keyboard. Some are provided here. You'll likely want to add more.
		"""
		mocapSys = self.config.mocap;
		
		if( self.config.use_phasespace == True ):
			hmdRigid = mocapSys.returnPointerToRigid('hmd')
			paddleRigid = mocapSys.returnPointerToRigid('paddle')
		else:
			hmdRigid = []
			paddleRigid = []
		
		##########################################################
		##########################################################
		## Keys used in the default mode
		
		if 'c' == key: # and self.config.eyeTrackingCal != None: # eyeTrackingCal is the where the interace to the eyetracker calib lives
			self.toggleEyeCalib()
			# a bit of a hack.  THe crossahair / calib ponit in viewpoint mapping is a bit off
			# until you hit a key.  So, I'm doing that for you.
			
			self.config.eyeTrackingCal.updateOffset('s')
			self.config.eyeTrackingCal.updateOffset('w')
			
			
		if (self.inCalibrateMode is False):
			
			if key == 'M':
				
				# Toggle the link between the HMD and Mainview
				if( mocapSys ):
					if( mocapSys.mainViewUpdateAction ):
						mocapSys.disableHMDTracking()
					else:
						mocapSys.enableHMDTracking()
			elif key == 'p':
				mocapSys.resetRigid('paddle')
			elif key == 'P':
				mocapSys.saveRigid('paddle')
			elif key == 'h':
				mocapSys.resetRigid('hmd')
			elif key == 'H':
				mocapSys.saveRigid('hmd')
			elif key == 'W':
				self.connectWiiMote()
			elif key == 'v':
				self.launchKeyDown()
				
			elif key == 'D':
				
				dvrWriter = self.config.writer;
				dvrWriter.toggleOnOff()
			
					
	
		##########################################################
		##########################################################
		## Eye-tracker calibration mode
		
		if self.inCalibrateMode:
			if key == 'w':
				self.config.eyeTrackingCal.updateOffset('w')
			elif key == 's':
				self.config.eyeTrackingCal.updateOffset('s')
			elif key == 'i':
				self.config.eyeTrackingCal.updateDelta('w')
			elif key == 'k':
				self.config.eyeTrackingCal.updateDelta('s')
			elif key == 'j':
				self.config.eyeTrackingCal.updateDelta('a')
			elif key == 'l':
				self.config.eyeTrackingCal.updateDelta('d')
	
	
	def onKeyUp(self,key):
				
		if( key == 'v'):
			self.launchKeyUp()
			

	def launchKeyDown(self):
		
		if( self.inProgress == True and   # Experiment ongoing
					self.launchKeyIsCurrentlyDown == False and
					self.currentTrial.ballInRoom == False ): # There is not already a ball
					
					# Start timing trigger duration 
					# At end of trigger, launch the ball.
					self.launchKeyIsCurrentlyDown = True
					self.timeLaunchKeyWasPressed = viz.tick()
					self.room.standingBox.visible( viz.TOGGLE )
					self.currentTrial.placeBall(self.room)
					
	
	def launchKeyUp(self):			
			
		if( self.launchKeyIsCurrentlyDown == True ):
			
			self.launchKeyIsCurrentlyDown = False
			triggerDuration = viz.tick() - self.timeLaunchKeyWasPressed
			ballReadyToLaunch = False
			
			if( self.currentTrial.ballInRoom == True and
				self.currentTrial.ballInInitialState == True and
				self.currentTrial.ballLaunched == False):
						
				# Ball is ready to launch
				
				if( triggerDuration <= self.minLaunchTriggerDuration ):
				
					# Trigger not held long enough for a launch
					soundBank.cowbell.play()
					self.room.standingBox.visible( viz.TOGGLE )
					self.currentTrial.removeBall()
					#print 'Launch aborted'
			
				if( triggerDuration >= self.minLaunchTriggerDuration ):
					
					self.eventFlag.setStatus(1)
					#print 'Ball launched'
				
					self.currentTrial.launchBall();
					self.starttimer(self.maxFlightDurTimerID,self.currentTrial.ballFlightMaxDur);
	
			else:
				
				return

	def getOutput(self):
		
		"""
		Returns a string describing the current state of the experiment, useful for recording.
		"""
		
		# Legend:
		# ** for 1 var
		# () for 2 vars
		# [] for 3 vars
		# <> for 4 vars
		# @@ for 16 vars (view and projection matrices)
		
		#### Eventflag
		# 1 ball launched
		# 3 ball has hit floor 
		# 4 ball has hit paddle
		# 5 ball has hit back wall
		# 6 ball has timed out
				
		outputString = '* frameTime %f * ' % (viz.getFrameTime())
		
		outputString = outputString + '* inCalibrateBool %f * ' % (self.inCalibrateMode)
		
		outputString = outputString + '* eventFlag %f * ' % (self.eventFlag.status)
		
		outputString = outputString + '* trialType %s * ' % (self.currentTrial.trialType)
		
		viewPos_XYZ = viz.MainView.getPosition()
		outputString = outputString + '[ viewPos_XYZ %f %f %f ] ' % (viewPos_XYZ[0],viewPos_XYZ[1],viewPos_XYZ[2])
		
		viewMat = viz.MainView.getMatrix()
		
		viewQUAT_XYZW = viewMat.getQuat()
		
		outputString = outputString + '< viewQUAT_WXYZ %f %f %f %f > ' % (viewQUAT_XYZW [0],viewQUAT_XYZW [1],viewQUAT_XYZW [2],viewQUAT_XYZW[3])
		
		################################################
		################################################
		#### Racquet data
		
		paddlePos_XYZ = []
		paddleQUAT_XYZW = []
		paddleAngVel_XYZ = []
		
		if( self.room.paddle ):
			
			paddlePos_XYZ = self.room.paddle.visNode.getPosition()
			paddleMat = self.room.paddle.visNode.getMatrix()
			paddleQUAT_XYZW = paddleMat.getQuat()
			
		else:
			paddlePos_XYZ = [nan,nan,nan]
			paddleQUAT_WXYZ = [nan,nan,nan,nan]
			
		outputString = outputString + '[ paddlePos_XYZ %f %f %f ] ' % (paddlePos_XYZ[0],paddlePos_XYZ[1],paddlePos_XYZ[2])
		
		outputString = outputString + '< paddleQUAT_WXYZ %f %f %f %f > ' % (paddleQUAT_XYZW[0],paddleQUAT_XYZW[1],paddleQUAT_XYZW[2],paddleQUAT_XYZW[3])
		
		################################################
		################################################
		#### BALL DATA

		if( self.currentTrial.ballInRoom  ):
		# If ball is in room, set ball position and velocity	
			
			theBall = self.currentTrial.ballObj;
			
			ballPos_XYZ = theBall.visNode.getPosition()
			outputString = outputString + '[ ballPos_XYZ %f %f %f ] ' % (ballPos_XYZ [0],ballPos_XYZ[1],ballPos_XYZ [2])
			
			ballVel_XYZ = theBall.getVelocity()
			outputString = outputString + '[ ballVel_XYZ %f %f %f ] ' % (ballVel_XYZ[0],ballVel_XYZ[1],ballVel_XYZ[2])
			
			ballPix_XYDist = viz.MainWindow.worldToScreen(ballPos_XYZ,viz.LEFT_EYE)
			outputString = outputString + '[ ballPix_XYDist %f %f %f ] ' % (ballPix_XYDist[0],ballPix_XYDist[1],ballPix_XYDist[2])
			
		else:
			
			# Ball not in room.  set pos / vel to Nan
			ballPos_XYZ = [nan,nan,nan]
			outputString = outputString + '[ ballPos_XYZ %f %f %f ] ' % (ballPos_XYZ [0],ballPos_XYZ [1],ballPos_XYZ [2])
			
			ballVel_XYZ = [nan,nan,nan]
			outputString = outputString + '[ ballVel_XYZ %f %f %f ] ' % (ballVel_XYZ [0],ballVel_XYZ [1],ballVel_XYZ [2])
			
			ballPix_XYDist = [nan,nan,nan]
			outputString = outputString + '[ ballPix_XYDist %f %f %f ] ' % (ballPix_XYDist [0],ballPix_XYDist [1],ballPix_XYDist [2])
			
		if( self.eventFlag.status == 1 ):
		# Launch mode.  Print initial conditions.
			
			outputString = outputString + '* ballDiameter %f * ' % (self.currentTrial.ballDiameter)
			outputString = outputString + '* ballGravity %f * ' % (self.currentTrial.gravity)
			outputString = outputString + '* ballPassingLoc %f * ' % (self.currentTrial.passingLoc)
			outputString = outputString + '* ballElasticity %f * ' % (self.currentTrial.ballElasticity)
			outputString = outputString + '* ballBounceDist %f * ' % (self.currentTrial.bounceDist)
			outputString = outputString + '* ballBounceSpeed %f * ' % (self.currentTrial.bounceSpeed)
			outputString = outputString + '* ballLaunchHeight %f * ' % (self.currentTrial.launchHeight)
			outputString = outputString + '* ballLaunchDistance %f * ' % (self.currentTrial.launchDistance)
			outputString = outputString + '* ballApproachAngleDegs %f * ' % (self.currentTrial.approachAngleDegs)
			
			outputString = outputString + '[ ballBounceLoc_XYZ %f %f %f ] ' % (self.currentTrial.ballBounceLoc_XYZ[0],self.currentTrial.ballBounceLoc_XYZ[1],self.currentTrial.ballBounceLoc_XYZ[2])
			outputString = outputString + '[ ballInitialPos_XYZ %f %f %f ] ' % (self.currentTrial.initialPos_XYZ[0],self.currentTrial.initialPos_XYZ[1],self.currentTrial.initialPos_XYZ[2])
			outputString = outputString + '[ initialVelocity_XYZ %f %f %f ] ' % (self.currentTrial.initialVelocity_XYZ[0],self.currentTrial.initialVelocity_XYZ[1],self.currentTrial.initialVelocity_XYZ[2])
		
		elif( self.eventFlag.status == 3 ):
			# Error, in seconds, between predicted bounce time and actual bounce time
			outputString = outputString + '* flightDurationError %f * ' % (self.currentTrial.flightDurationError)
		elif( self.eventFlag.status == 4 ):
			#print 'Event flag 4!  Writing paddle position!'
			outputString = outputString + '[ ballOnPaddlePosLoc_XYZ %f %f %f ] ' % (self.currentTrial.ballOnPaddlePosLoc_XYZ[0],self.currentTrial.ballOnPaddlePosLoc_XYZ[1],self.currentTrial.ballOnPaddlePosLoc_XYZ[2])
		
		# The end of the trial
		if( (self.eventFlag.status == 6 or self.eventFlag.status == 7) ): 
			
			# For convenicne, this ensures the var  ballOnPaddlePosLoc_XYZ will show up once per  trial
			if ( self.currentTrial.ballHasHitPaddle == False ):
				outputString = outputString + '[ ballOnPaddlePosLoc_XYZ %f %f %f ] ' % (nan, nan, nan)
				#print '***** [ ballOnPaddlePosLoc_XYZ %f %f %f ] ' % (nan, nan, nan)
		
		##  MATRICES
		
		viewMat = viz.MainWindow.getMatrix(viz.LEFT_EYE)
		invViewMat = viewMat.inverse()
		
		outputString = outputString + '@ invViewMat %s @ '  % (str(invViewMat.get()))
		
		proMat = viz.MainWindow.getProjectionMatrix(viz.LEFT_EYE)
		invProMat  = proMat.inverse()
		outputString = outputString + '@ invProMat %s @ '  % (str(invProMat.get()))
	
		return outputString #%f %d' % (viz.getFrameTime(), self.inCalibrateMode)
						
		
#	def getOutput(self):
#		
#
#		"""
#		Returns a string describing the current state of the experiment, useful for recording.
#		"""
#		
#		# Legend:
#		# ** for 1 var
#		# () for 2 vars
#		# [] for 3 vars
#		# <> for 4 vars
#		# @@ for 16 vars (view and projection matrices)
#		
#		#### Eventflag
#		# 1 ball launched
#		# 3 ball has hit floor 
#		# 4 ball has hit paddle
#		# 5 ball has hit back wall
#		# 6 ball has timed out
#				
#		outputString = '* frameTime %f * ' % (viz.getFrameTime())
#		
#		outputString = outputString + '* inCalibrateBool %f * ' % (self.inCalibrateMode)
#		
#		outputString = outputString + '* eventFlag %f * ' % (self.eventFlag.status)
#		
#		outputString = outputString + '* trialType %s * ' % (self.currentTrial.trialType)
#		
#		viewPos_XYZ = viz.MainView.getPosition()
#		outputString = outputString + '[ viewPos_XYZ %f %f %f ] ' % (viewPos_XYZ[0],viewPos_XYZ[1],viewPos_XYZ[2])
#		
#		viewQUAT_XYZW = viz.MainView.getQuat()
#		outputString = outputString + '< viewQUAT_XYZW %f %f %f %f > ' % (viewQUAT_XYZW [0],viewQUAT_XYZW [1],viewQUAT_XYZW [2],viewQUAT_XYZW[3])
#		
#		################################################
#		################################################
#		#### Racquet data
#		
#		paddlePos_XYZ = []
#		paddleQUAT_XYZW = []
#		paddleAngVel_XYZ = []
#		
#		if( self.room.paddle ):
#			
#			paddlePos_XYZ = self.room.paddle.visNode.getPosition()
#			paddleQUAT_XYZW = self.room.paddle.visNode.getQuat()
#			
#			#paddleVel_XYZ = self.room.paddle.getVelocity()
#			#paddleAngVel_XYZ = self.room.paddle.getAngularVelocity()
#			
#			
#		else:
#			paddlePos_XYZ = [nan,nan,nan]
#			paddleQUAT_XYZW = [nan,nan,nan,nan]
#			paddleVel_XYZ = [nan,nan,nan]
#			paddleAngVel_XYZ = [nan,nan,nan]
#			
#		outputString = outputString + '[ paddlePos_XYZ %f %f %f ] ' % (paddlePos_XYZ[0],paddlePos_XYZ[1],paddlePos_XYZ[2])
#		outputString = outputString + '< paddleQUAT_XYZW %f %f %f %f > ' % (paddleQUAT_XYZW[0],paddleQUAT_XYZW[1],paddleQUAT_XYZW[2],paddleQUAT_XYZW[3])
#		
#		#outputString = outputString + '[ paddleVel_XYZ %f %f %f ] ' % (paddleVel_XYZ[0],paddleVel_XYZ[1],paddleVel_XYZ[2])
#		#outputString = outputString + '[ paddleAngVel_XYZ %f %f %f ] ' % (paddleAngVel_XYZ[0],paddleAngVel_XYZ[1],paddleAngVel_XYZ[2])
#		
#		################################################
#		################################################
#		#### BALL DATA
#
#		
#		if( self.currentTrial.ballInRoom  ):
#		# If ball is in room, set ball position and velocity	
#			
#			theBall = self.currentTrial.ballObj;
#			
#			ballPos_XYZ = theBall.visNode.getPosition()
#			outputString = outputString + '[ ballPos_XYZ %f %f %f ] ' % (ballPos_XYZ [0],ballPos_XYZ [1],ballPos_XYZ [2])
#			
#			ballVel_XYZ = theBall.getVelocity()
#			#theBall.visNode.getVelocity()
#			outputString = outputString + '[ ballVel_XYZ %f %f %f ] ' % (ballVel_XYZ[0],ballVel_XYZ[1],ballVel_XYZ[2])
#			
#			ballPix_XY = viz.MainWindow.worldToScreen(ballPos_XYZ,viz.LEFT_EYE)
#			outputString = outputString + '[ ballPix_XY %f %f %f ] ' % (ballPix_XY [0],ballPix_XY [1],ballPix_XY [2])
#			
#		else:
#			
#			# Ball not in room.  set pos / vel to Nan
#			ballPos_XYZ = [nan,nan,nan]
#			outputString = outputString + '[ ballPos_XYZ %f %f %f ] ' % (ballPos_XYZ [0],ballPos_XYZ [1],ballPos_XYZ [2])
#			
#			ballVel_XYZ = [nan,nan,nan]
#			outputString = outputString + '[ ballVel_XYZ %f %f %f ] ' % (ballVel_XYZ [0],ballVel_XYZ [1],ballVel_XYZ [2])
#			
#			ballPix_XY = [nan,nan,nan]
#			outputString = outputString + '[ ballPix_XY %f %f %f ] ' % (ballPix_XY [0],ballPix_XY [1],ballPix_XY [2])
#			
#		if( self.eventFlag.status == 1 ):
#		# Launch mode.  Print initial conditions.
#			
#			outputString = outputString + '* ballDiameter %f * ' % (self.currentTrial.ballDiameter)
#			outputString = outputString + '* ballGravity %f * ' % (self.currentTrial.gravity)
#			outputString = outputString + '* ballPassingLoc %f * ' % (self.currentTrial.passingLoc)
#			outputString = outputString + '* ballElasticity %f * ' % (self.currentTrial.ballElasticity)
#			outputString = outputString + '* ballBounceDist %f * ' % (self.currentTrial.bounceDist)
#			outputString = outputString + '* ballBounceSpeed %f * ' % (self.currentTrial.bounceSpeed)
#			outputString = outputString + '* ballLaunchHeight %f * ' % (self.currentTrial.launchHeight)
#			outputString = outputString + '* ballLaunchDistance %f * ' % (self.currentTrial.launchDistance)
#			outputString = outputString + '* ballApproachAngleDegs %f * ' % (self.currentTrial.approachAngleDegs)
#			
#			outputString = outputString + '[ ballBounceLoc_XYZ %f %f %f ] ' % (self.currentTrial.ballBounceLoc_XYZ[0],self.currentTrial.ballBounceLoc_XYZ[1],self.currentTrial.ballBounceLoc_XYZ[2])
#			outputString = outputString + '[ ballInitialPos_XYZ %f %f %f ] ' % (self.currentTrial.initialPos_XYZ[0],self.currentTrial.initialPos_XYZ[1],self.currentTrial.initialPos_XYZ[2])
#			outputString = outputString + '[ initialVelocity_XYZ %f %f %f ] ' % (self.currentTrial.initialVelocity_XYZ[0],self.currentTrial.initialVelocity_XYZ[1],self.currentTrial.initialVelocity_XYZ[2])
#		
##				elif( self.eventFlag == 3 ):
##					# bouncePos_XYZ
##					print 'Event flag 3!  Writing ball position!'
##					outputString = outputString + '[ ballOnPaddlePos_XYZ %f %f %f ] ' % (self.currentTrial.ballOnPaddlePos_XYZ[0],self.currentTrial.ballOnPaddlePos_XYZ[1],self.currentTrial.ballOnPaddlePos_XYZ[2])
#			
#		elif( self.eventFlag.status == 4 ):
#			#print 'Event flag 4!  Writing paddle position!'
#			outputString = outputString + '[ ballOnPaddlePosLoc_XYZ %f %f %f ] ' % (self.currentTrial.ballOnPaddlePosLoc_XYZ[0],self.currentTrial.ballOnPaddlePosLoc_XYZ[1],self.currentTrial.ballOnPaddlePosLoc_XYZ[2])
#		
#		# The end of the trial
#		if( (self.eventFlag.status == 6 or self.eventFlag.status == 7) ): 
#			
#			# For convenience, this ensures the var  ballOnPaddlePosLoc_XYZ will show up once per trial
#			if ( self.currentTrial.ballHasHitPaddle == False ):
#				outputString = outputString + '[ ballOnPaddlePosLoc_XYZ %f %f %f ] ' % (nan, nan, nan)
#				print '***** [ ballOnPaddlePosLoc_XYZ %f %f %f ] ' % (nan, nan, nan)
#				
#		##  MATRICES
#		
#		viewMat = viz.MainWindow.getMatrix(viz.LEFT_EYE)
#		invViewMat = viewMat.inverse()
#		
#		outputString = outputString + '@ invViewMat %s @ '  % (str(invViewMat.get()))
#		
#		proMat = viz.MainWindow.getProjectionMatrix(viz.LEFT_EYE)
#		invProMat  = proMat.inverse()
#		outputString = outputString + '@ invProMat %s @ '  % (str(invProMat.get()))
#		#outputString = outputString + '@ invProMat %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f @ '  % (invProMat.get())
#		
#		return outputString #%f %d' % (viz.getFrameTime(), self.inCalibrateMode)
#	
#	def vizQuatToMatlabQuat(self,vizQuat):
#		
#		# Viz is in ABCW
#		
#		# I want WABC, with Z and Y axes flipped
#		oldTrans = viz.Transform()
#		oldTrans.setQuat(vizQuat)		
#		oldTrans.postAxisAngle(1,0,0,-90) # rotate -90 degs around X
#		return oldTrans.getQuat()
#		
##		# negate Z axis
##		
###		
###		#newTrans = viz.Transform()
###		
###		newTransVec = [];
###		
###		for rIdx in range(3):
###			newTransVec.append(oldTrans.val(rIdx,0))
###			newTransVec.append(oldTrans.val(rIdx,1))
###			newTransVec.append(oldTrans.val(rIdx,3))
###			newTransVec.append(oldTrans.val(rIdx,2))
###		
###		newTrans = viz.Transform()
###		newTrans.setQuat(newTransVec)
##		
##		print 'VizQ: ' + str(oldTrans.getAxisAngle())
##		print 'NewQ: ' + str(newTrans.getAxisAngle())
##		
##		a=1
		
		
	def getEyeData(self):
		
		# Legend:
		# ** for 1 var
		# () for 2 vars
		# [] for 3 vars
		# <> for 4 vars
		# @@ for 16 vars (view and projection matrices)

		outputString = ''		
		
		
		
		#pyArr = viz.add('arrington.dle')
		#eyeA = pyArr.EYE_A;
		

		class VPX_RealType(Structure):
			 _fields_ = [("x", c_float),("y", c_float)]

		## Position
		arrington = self.config.eyeTrackingCal.arrington;
		
		#eyePos_XY = pyArr.getPosition(self.arrington.RAW)
		#outputString = outputString + '( eyePos_XY %f %f ) ' %(eyePos_XY[0],eyePos_XY[1])
		
		Eye_A  = c_int(0)

		eyePos = VPX_RealType()
		eyePosPointer = pointer(eyePos)
		
		arrington.VPX_GetGazePoint2(Eye_A ,eyePosPointer)
		outputString = outputString + '( eyePos %f  %f ) ' % (eyePos.x, eyePos.y)
		
		## Time
		eyeTime = c_double();
		eyeTimePointer = pointer(eyeTime)		
		arrington.VPX_GetDataTime2(Eye_A ,eyeTimePointer)
		outputString = outputString + '* eyeDataTime %f * ' % eyeTime.value
		
		## Quality
		eyeQual = c_int();
		eyeQualPointer = pointer(eyeQual)		
		arrington.VPX_GetDataQuality2(Eye_A ,eyeQualPointer)
		outputString = outputString + '* eyeQuality %i * ' % eyeQual.value
		
		return outputString
		
	def endTrial(self):
		
		endOfTrialList = len(self.blocks_bl[self.blockNumber].trials_tr)
		
		#print 'Ending block: ' + str(self.blockNumber) + 'trial: ' + str(self.trialNumber)
		
		if( self.trialNumber < endOfTrialList ):
			
			recalAfterTrial_idx = self.blocks_bl[self.blockNumber].recalAfterTrial_idx
			
			if( recalAfterTrial_idx.count(self.trialNumber ) > 0):
				soundBank.gong.play()
				vizact.ontimer2(0,0,self.toggleEyeCalib)

			# Increment trial 
			self.trialNumber += 1
			self.killtimer(self.maxFlightDurTimerID)
			
			self.eventFlag.setStatus(6)
			
		if( self.trialNumber == endOfTrialList ):
			
			# Increment block
			
			# arg2 of 1 allows for overwriting eventFlag 6 (new trial)
			self.eventFlag.setStatus(7,True) 
			
			self.blockNumber += 1
			self.trialNumber = 0
			
			# Increment block or end experiment
			if( self.blockNumber == len(self.blocks_bl) ):
				
				# Run this once on the next frame
				# This maintains the ability to record one frame of data
				vizact.ontimer2(0,0,self.endExperiment)
				return
				
		if( self.inProgress ):
				
			print 'Starting block: ' + str(self.blockNumber) + ' Trial: ' + str(self.trialNumber)
			self.currentTrial = self.blocks_bl[self.blockNumber].trials_tr[self.trialNumber]
	
	def writeDataToText(self):
		
		# Only write data is the experiment is ongoing
		if( self.inProgress is False ):
			return
			
		now = datetime.datetime.now()
		dateTimeStr = str(now.hour) + ':' + str(now.minute) + ':' + str(now.second) + ':' + str(now.microsecond)
		
		expDataString = self.getOutput()
		self.expDataFile.write(expDataString + '\n')
		
		if( self.config.sysCfg['use_eyetracking']):
			
			eyeDataString = self.getEyeData()
			self.eyeDataFile.write(eyeDataString + '\n')
		
		# Eyetracker data
	
	def registerWiimoteActions(self):
				
		wii = viz.add('wiimote.dle')#Add wiimote extension
		
		vizact.onsensordown(self.config.wiimote,wii.BUTTON_B,self.launchKeyDown) 
		vizact.onsensorup(self.config.wiimote,wii.BUTTON_B,self.launchKeyUp) 
		
		if( self.config.use_phasespace == True ):
			
			mocapSys = self.config.mocap;
		
			vizact.onsensorup(self.config.wiimote,wii.BUTTON_DOWN,mocapSys.resetRigid,'hmd') 
			vizact.onsensorup(self.config.wiimote,wii.BUTTON_UP,mocapSys.saveRigid,'hmd') 
			
			vizact.onsensorup(self.config.wiimote,wii.BUTTON_LEFT,mocapSys.resetRigid,'paddle') 
			vizact.onsensorup(self.config.wiimote,wii.BUTTON_RIGHT,mocapSys.saveRigid,'paddle') 
			
			vizact.onsensorup(self.config.wiimote,wii.BUTTON_A,self.printQuats)
			
	
	def endExperiment(self):
		# If recording data, I recommend ending the experiment using:
		#vizact.ontimer2(.2,0,self.endExperiment)
		# This will end the experiment a few frame later, making sure to get the last frame or two of data
		# This could cause problems if, for example, you end the exp on the same that the ball dissapears
		# ...because the eventflag for the last trial would never be recorded
		
		#end experiment
		print 'end experiment'
		self.inProgress = False
		soundBank.gong.play()
		
	def printQuats(self):
		
		viewMat = viz.MainView.getMatrix()
		viewQUAT_XYZW = viewMat.getQuat()
		
		#outputString = outputString + '< viewQUAT_XYZW %f %f %f %f > ' % (viewQUAT_XYZW [3],viewQUAT_XYZW [0],viewQUAT_XYZW [1],viewQUAT_XYZW[2])
		string = '< viewQUAT_WXYZ %f %f %f %f > ' % (viewQUAT_XYZW [3],viewQUAT_XYZW [0],viewQUAT_XYZW [2],viewQUAT_XYZW[1])
		print string
		
		paddleMat = self.room.paddle.visNode.getMatrix()
		paddleQUAT_XYZW = paddleMat.getQuat()
		string = '< paddleQUAT_XYZW %f %f %f %f > ' % (paddleQUAT_XYZW[3],paddleQUAT_XYZW[0],paddleQUAT_XYZW[2],paddleQUAT_XYZW[1])
		print string
	
	def checkDVRStatus(self):
	
		dvrWriter = self.config.writer;
		
		if( dvrWriter.isPaused == 1 ):
			print '************************************ DVR IS PAUSED ************************************'

	def setupPaddle(self):
		# Performs several functions
		# Creates either a fake paddle, a visual paddle, or a vis/phy/mocap paddle
		
		# FOr debugging. Creates a fake paddle in teh center of the room
		if( self.config.expCfg['experiment']['useFakePaddle'] ):
				
#				if(any("paddle" in idx for idx in self.room.visObjNames_idx)):
#					print 'removed paddle'
#					self.room.paddle.remove()
					
				# Put a fake stationary paddle in the room
				paddleSize = [.15, 1.5]
				self.room.paddle = visEnv.visObj(self.room,'cylinder_Z',paddleSize)
				self.room.paddle.enablePhysNode()
				self.room.paddle.visNode.setPosition([0,1.6,-1])
				#self.room.paddle.visNode.setAxisAngle(1,0,0,90)
				self.room.paddle.visNode.color([0,1,0])
				self.room.paddle.applyVisToPhys()
				self.room.paddle.visNode.alpha(.5)
				
				return
		
		paddleRigid  = self.config.mocap.returnPointerToRigid('paddle')
		
		# If there is a visObj paddle and a paddle rigid, link em up!
		if any("paddle" in idx for idx in self.room.visObjNames_idx): 
			
#			if(paddleRigid ):
#				paddle = self.room.paddle
#				# Link the visObj to the rigid 
#				paddle.setMocapRigidBody(self.config.mocap,'paddle')
#				paddle.toggleUpdateWithRigid()
#				paddle.visNode.alpha(0)
#				paddle.enablePhysNode()
#				paddle.toggleUpdatePhysWithVis()
#				paddle.setPosition([0,1.5,0])
#				
#						# Try to add the squash model
#				pObj = viz.addChild('squash.osgb')
#				
#				pObj.setEuler(0,0,180)
#				pObj.setParent(self.room.paddle.visNode)
#				pObj.setScale([.9,.9,.9])
#				pObj.setPosition(0,0.5,0,viz.ABS_PARENT)
#		
			
			if(paddleRigid ):
				paddle = self.room.paddle
				# Link the visObj to the rigid 
				paddle.setMocapRigidBody(self.config.mocap,'paddle')
				paddle.toggleUpdateWithRigid()
				#paddle.visNode.alpha(0)
				paddle.enablePhysNode()
				paddle.toggleUpdatePhysWithVis()
				paddle.setPosition([0,1.5,0])
				#paddle.visNode.setEuler(0,180,0)
				#vizact.onupdate(10,paddle.visNode.setEuler,[0,90,0],viz.ABS_LOCAL)
		
		
		
		#paddle.obj.visible(viz.ON)					
		#visEnv.drawMarkerSpheres(self.room,self.config.mocap)
		
############################################################################################################
############################################################################################################

class eventFlag(viz.EventClass):
	
	def __init__(self):
		
		################################################################
		##  Eventflag
		
		# 1 ball launched
		# 2 * not used * 
		# 3 ball has hit floor
		# 4 ball has hit paddle
		# 5 ball has hit back wall
		# 6 trial end
		# 7 block end
		
		viz.EventClass.__init__(self)
		
		self.status = 0
		self.lastFrameUpdated = viz.getFrameNumber()
		self.currentValue = 0
		
		# On every frame, self.eventFlag should be set to 0
		# This should happen first, before any timer object has the chance to overwrite it!
		vizact.onupdate(viz.PRIORITY_FIRST_UPDATE,self._resetEventFlag)
		
	def setStatus(self,status,overWriteBool = False):
		
		if( self.lastFrameUpdated != viz.getFrameNumber() ):
			
			#print 'Setting status to' + str(status)
			
			self.status = status
			self.lastFrameUpdated = viz.getFrameNumber()
			
		elif( overWriteBool is True and self.lastFrameUpdated == viz.getFrameNumber() ):
			
			#print 'Overwrite from status ' + str(self.status) + ' to ' + str(status)
			
			self.status = status
			self.lastFrameUpdated = viz.getFrameNumber()
		
			
		elif( self.lastFrameUpdated == viz.getFrameNumber() ):
			
			#print 'Stopped attempt to overwrite status of ' + str(self.status) + ' with ' + str(status) + ' [overWriteBool=False]'
			pass
		
	def _resetEventFlag(self):
		
		#This should run before timers, eyeTrackingCal.  Called using <vizact>.onupdate
		if( self.lastFrameUpdated == viz.getFrameNumber() ):
			print 'Did not reset! Status already set to ' + str(self.status)
		else:
			self.status = 0; # 0 Means nothing is happening
			
		
class block():
	def __init__(self,config=None,blockNum=1):
			
		# Each block will have a block.trialTypeList
		# This list is a list of strings of each trial type
		# Types included and frequency of types are defined in the config
		# Currently, these trial types are automatically randomized
		# e.g. 't1,t2,t2,t2,t1'
		
		self.blockName = config.expCfg['experiment']['blockList'][blockNum]

	#    Kinds of trial in this block
		
		# trialTypeList enumerates the types of trials
		self.trialTypesInBlock = config.expCfg['blocks'][self.blockName]['trialTypesString'].split(',')
		# The number of each type of trial
		self.numOfEachTrialType_type = map(int,config.expCfg['blocks'][self.blockName]['trialTypeCountString'].split(','));
		
		# THe type of each trial
		# _tr indicates that the list is as long as the number of trials
		self.trialTypeList_tr = []
		
		for typeIdx in range(len(self.trialTypesInBlock)):
			for count in range(self.numOfEachTrialType_type[typeIdx]):
				self.trialTypeList_tr.append(self.trialTypesInBlock[typeIdx])
		
		# Randomize trial order
		from random import shuffle
		shuffle(self.trialTypeList_tr)
		
		self.numTrials = len(self.trialTypeList_tr)
		self.recalAfterTrial_idx = config.expCfg['blocks'][self.blockName]['recalAfterTrial']
		
		self.trials_tr = []
		
		for trialNumber in range(self.numTrials):
			
			## Get trial info
			trialObj = trial(config,self.trialTypeList_tr[trialNumber])
				
			##Add the body to the list
			self.trials_tr.append(trialObj)

			## Create a generator this will loop through the balls
			#nextBall = viz.cycle(balls); 
		
class trial(viz.EventClass):
	def __init__(self,config=None,trialType='t1'):
		
		#viz.EventClass.__init__(self)
		
		self.trialType = trialType

		## State flags
		self.ballInRoom = False; # Is ball in room?
		self.ballInInitialState = False; # Is ball ready for launch?
		self.ballLaunched = False; # Has a ball been launched?  Remains true after ball disappears.
		self.ballHasBouncedOnFloor = False;
		self.ballHasHitPaddle = False;
		
		## Trial event data
		self.ballOnPaddlePos_XYZ = []
		self.ballOnPaddlePosLoc_XYZ = []
		
		## Timer objects
		self.timeSinceLaunch = [];
		
		self.ballObj = -1;
		
		### Below this is all the code used to generate ball trajectories
		self.ballFlightMaxDur = float(config.expCfg['experiment']['ballFlightMaxDur'])
		
		#  Set ball color.
		try:
			self.ballColor_RGB = map(float,config.expCfg['trialTypes'][self.trialType]['ballColor_RGB'])
		except:
			print 'Using def color'
			self.ballColor_RGB = map(float,config.expCfg['trialTypes']['default']['ballColor_RGB'])
		
		# The rest of variables are set below, by drawing values from distributions
#		
		self.ballDiameter_distType = []
		self.ballDiameter_distParams = []
		self.ballDiameter = []
		
		self.gravity_distType = []
		self.gravity_distParams = []
		self.gravity = []
		
		self.passingLoc_distType = []
		self.passingLoc_distParams = []
		self.passingLoc = []
		
		self.ballElasticity_distType = []
		self.ballElasticity_distParams = []
		self.ballElasticity = []
		
		self.bounceDist_distType = []
		self.bounceDist_distParams = []
		self.bounceDist = []
		
		self.bounceSpeed_distType = []
		self.bounceSpeed_distParams = []
		self.bounceSpeed = []
		
		self.launchHeight_distType = []
		self.launchHeight_distParams = []
		self.launchHeight = []
		
		self.launchDistance_distType = []
		self.launchDistance_distParams = []
		self.launchDistance = []
		
		self.approachAngleDegs_distType = []
		self.approachAngleDegs_distParams = []
		self.approachAngleDegs = []
		
		self.ballBounceLoc_XYZ = [0,0,0]
		self.initialPos_XYZ = [0,0,0]
		self.initialVelocity_XYZ = [0,0,0]
		
		self.flightDurationError = []
		
		##########################################################################
		##########################################################################
		# Go into config file and define values 
		# When a distribution is specified, select a value from the distribution
		
		variablesInATrial = config.expCfg['trialTypes']['default'].keys()
		
		for varIdx in range(len(variablesInATrial)):
			if "_distType" in variablesInATrial[varIdx]:
			
				varName = variablesInATrial[varIdx][0:-9]

				
				try:
					distType, distParams, value = self._setValueOrUseDefault(config,varName)
				except:
					print 'Error in main.trial.init.drawNumberFromDist()'
					print 'Variable name is: ' + varName
				try:
					exec( 'self.' + varName + '_distType = distType' )
					exec( 'self.' + varName + '_distParams = distParams' )
					exec( 'self.' + varName + '_distType = distType' )
					# Draw value from a distribution
					exec( 'self.' + varName + ' = drawNumberFromDist( distType , distParams);' )
				except:
					a=1
				
		
		##########################################################################
		##########################################################################
		##  Calculate ball trajectory
		
		#######################################
		## Initial positions
		
		import math
		
		self.ballBounceLoc_XYZ[0] = math.sin(math.radians(self.approachAngleDegs))*self.bounceDist;
		self.ballBounceLoc_XYZ[1] = self.ballDiameter/2
		self.ballBounceLoc_XYZ[2] = math.cos(math.radians(self.approachAngleDegs))*self.bounceDist;
		
		self.initialPos_XYZ[0] = math.sin(math.radians(self.approachAngleDegs))*self.launchDistance;
		self.initialPos_XYZ[1] = self.launchHeight
		self.initialPos_XYZ[2] = math.cos(math.radians(self.approachAngleDegs))*self.launchDistance;
		
		##  Account for room offset
		
		#self.initialPos_XYZ[0] = self.initialPos_XYZ[0] + config.expCfg['room']['translateRoom_X']
		#self.initialPos_XYZ[2] = self.initialPos_XYZ[2] + config.expCfg['room']['translateRoom_Z']
		
		#self.ballBounceLoc_XYZ[0] = self.ballBounceLoc_XYZ[0] + config.expCfg['room']['translateRoom_X']
		#self.ballBounceLoc_XYZ[2] = self.ballBounceLoc_XYZ[2] + config.expCfg['room']['translateRoom_Z']
		
		#######################################
		## Set and record velocities
		
		# vel = sqrt( v^2+2ad )
		# tmp.initBallVelZ = sqrt( pow(tmp.bounceSpeed,2)+ 2*-9.8f*(tmp.initBallZ-tmp.ballBounceZ));
		
		self.initialVelocity_XYZ[1] = math.sqrt( (self.bounceSpeed*self.bounceSpeed)+ 
			2 * -self.gravity * (self.initialPos_XYZ[1]-self.ballBounceLoc_XYZ[1]));
			
		durationOfPreBounceFlight = (self.bounceSpeed - self.initialVelocity_XYZ[1]) / -self.gravity
		
		self.initialVelocity_XYZ[0] = (self.ballBounceLoc_XYZ[0]-self.initialPos_XYZ[0]) / durationOfPreBounceFlight;
		self.initialVelocity_XYZ[2] = (self.ballBounceLoc_XYZ[2]-self.initialPos_XYZ[2]) / durationOfPreBounceFlight;
		
		# For debugging, compare these values!
		self.predictedPreBounceFlightDur = durationOfPreBounceFlight
		self.launchTime = []
		
	def removeBall(self):
		
		self.ballObj.remove()
		self.ballObj = -1
		
		self.ballInRoom = False
		self.ballInInitialState = False
		self.ballLaunched = False
		
		#print 'Cleaned up ball'

	def _setValueOrUseDefault(self,config,paramPrefix):
		
		try:
			#print paramPrefix
			# Try to values from the subsection [[trialType]]
			distType = config.expCfg['trialTypes'][self.trialType][paramPrefix + '_distType']
			distParams = config.expCfg['trialTypes'][self.trialType][paramPrefix +'_distParams']
			
		except:
			# print 'Using default: **' + paramPrefix + '**'
			# Try to values from the subsection [['default']]
			distType = config.expCfg['trialTypes']['default'][paramPrefix + '_distType'];
			distParams = config.expCfg['trialTypes']['default'][paramPrefix + '_distParams'];
		
		
		value = drawNumberFromDist(distType,distParams)
	
		
		return distType,distParams,value
			

	def placeBall(self,room):
	
		
		print self.initialPos_XYZ
		
		self.ballObj = visEnv.visObj(room,'sphere',self.ballDiameter/2,self.initialPos_XYZ,self.ballColor_RGB)

		self.ballObj.toggleUpdateWithPhys()
		self.ballObj.setVelocity([0,0,0])
		
		
		#print 'BALL ELASTICITY:' + str(self.ballElasticity)
		self.ballObj.physNode.setBounciness(self.ballElasticity)
		self.ballObj.physNode.disableMovement() # Makes it stand in place

		# Costly, in terms of computation
		#self.ballObj.projectShadows(self.ballObj.parentRoom.floor.visNode)
		
		# Register it as something that will stick to the paddle
		self.ballObj.physNode.setStickUponContact( room.paddle.physNode.geom )
		
		self.ballInRoom = True
		self.ballInInitialState = True
		self.ballLaunched = False 
		self.ballPlacedOnThisFrame = True
		
	def launchBall(self):
		
		if( self.ballObj == False ):
			print 'No ball present.'
			return
		
		self.ballObj.physNode.enableMovement()
		self.ballObj.setVelocity(self.initialVelocity_XYZ)
		
		self.ballInRoom = True
		self.ballInInitialState = False
		self.ballLaunched = True
		
		self.launchTime = viz.getFrameTime()
	
################################################################################################################   
################################################################################################################
################################################################################################################
##  Here's where the magic happens!

experimentConfiguration = vrlabConfig.VRLabConfig(expConfigFileName)


## vrlabConfig uses config to setup hardware, motion tracking, frustum, eyeTrackingCal.
##  This draws upon the system config to setup the hardware / HMD


## The experiment class initialization draws the room, sets up physics, 
## and populates itself with a list of blocks.  Each block contains a list of trials
experimentObject = Experiment(experimentConfiguration)
experimentObject.start()

#pV = experimentObject.room.paddle.visNode
#eA = vizact.onupdate(8,pV.setEuler,[0,90,0],viz.ABS_LOCAL)

#pObj = experimentObject.room.paddle.obj
#pObj.setPosition(0,0.54,0,viz.ABS_PARENT)

#visEnv.drawMarkerSpheres(experimentObject.room,experimentObject.config.mocap)

if( experimentObject.hmdLinkedToView == False ):
	
	print 'Head controlled by mouse/keyboard. Initial viewpoint set in vrLabConfig _setupSystem()'
	
	viz.MainView.setPosition(-3,2,-3)
	#viz.MainView.setPosition([experimentObject.room.wallPos_NegX +.1, 2, experimentObject.room.wallPos_NegZ +.1])
	viz.MainView.lookAt([0,2,-2])