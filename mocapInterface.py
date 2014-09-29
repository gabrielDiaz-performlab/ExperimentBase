import viz
import vizshape
import vizact

# not a vizard module.  My own.

import math
from OWL import *

from os import rename


## marker index is the standard marker number
## When identifying a marker that has been registered on the server,
# use Marker ID.  Marker ID is a function of index and tracker number.



def markerNumToID(tracker, index):
	'''# The markerNumToID macro takes a tracker and an index and produces a marker 
	# ID. The index is generally a sequential number sequence starting at 0, 
	# with a new number for each marker defined. Remember that the 
	# MARKER macro merely produces a marker ID.
	# this BIT SHIFTs LEFT BY 12 BITS IS BINARY MULTIPLICATION BY (2 ** 12)'''
	
	return ( ( tracker << 12 ) | (index) );

class rigidObject(viz.EventClass):
	
	def __init__(self,trackerIdx,filePath,fileName,avgMarkerList_midx = [0],rigidOffset_worldXYZ = [0,0,0]):
		
		self.filePath = filePath
		self.fileName = fileName
		
		self.trackerIdx = trackerIdx
		self.markerID_midx = []
		self.markerPos_midx_localXYZ = []

		self.serverData = False
		
		# List of markers to be averaged over
		self.avgMarkerList_midx = avgMarkerList_midx
		
		# Used to pre-translate rigid body by a set amount
		# Here, it is converted from vizard to phasespace coordinates
		self.rigidOffset_worldXYZ = [-rigidOffset_worldXYZ[2],rigidOffset_worldXYZ[1],-rigidOffset_worldXYZ[0]]
		
		# B/C Phasespace has a flipped Z axis
		# transform mats must be converted.  
		# Be sure to explicitly keep track of the format
		# transformViz or transformPS
		
		self.transformViz = 0
		
		# Create rigid tracker		
		self._loadDefaults()
		
		# Create rigidTracker
		owlTrackeri( self.trackerIdx, OWL_CREATE, OWL_RIGID_TRACKER );
		
		# Populate rigidTracker 
		for i in range( len(self.markerID_midx) ):
			
			# set markers
			owlMarkeri(  markerNumToID(self.trackerIdx, i), OWL_SET_LED, self.markerID_midx[i] )
			# set marker positions
			owlMarkerfv( markerNumToID(self.trackerIdx, i), OWL_SET_POSITION, self.markerPos_midx_localXYZ[i])
		
		# Activate tracking for rigidTracker
		owlTracker( self.trackerIdx, OWL_ENABLE )
		print 'Enabling ' + self.fileName
			
		# Flush requests and check for errors
		if not owlGetStatus():    
			#owl_print_error("error in point tracker setup", owlGetError())
			errorCode = owlGetError();
			print("Error in point tracker setup.", errorCode)
			return 0

	def _loadDefaults(self):

		import os.path
		openfile = [];
		openfile = open( self.filePath + self.fileName, 'r' );
		
		lineData = openfile.readlines();

		parsedData = [];

		markerID = [];
		markerPos = [];
		
		count = 0;
		
		for currentLine in lineData:
			
			tempLineDataList = currentLine.split(',');
			
			markerID.append( int(tempLineDataList[0]) );
			
			markerPos.append( map( float, tempLineDataList[1].split() ) );
			
			count += 1;
			
		#rof
			
		self.markerID_midx = markerID
		self.markerPos_midx_localXYZ = markerPos
		
		openfile.close()
		
		print 'Mocap: Read ' + str(count) + ' lines from the rigid body file.'
		if count == 0: print 'This is likely to cause OWL.init() to fail'
	
	def getMarkerPositions(self,alPSMarkers_midx):
		
		# Build list of updated marker positions in newPosWorldCoords
		newPos_midx_GlobalXYZ = [];
		
		for oldMarkerIdx in range( 0, len(self.markerID_midx), 1 ):

			for newMarkerIdx in range( 0, len(alPSMarkers_midx), 1 ):
				
				# Confused?  See markerNumToID macro at top!
				oldRigidIDConverted = markerNumToID( self.trackerIdx, oldMarkerIdx );
				
				if( alPSMarkers_midx[newMarkerIdx].id == oldRigidIDConverted ):
					
					newPos_midx_GlobalXYZ.append( [ alPSMarkers_midx[newMarkerIdx].x, 
												alPSMarkers_midx[newMarkerIdx].y,
												alPSMarkers_midx[newMarkerIdx].z ] );
												
				#fi
				
			#rof
			
		#rof
		
		if( len(newPos_midx_GlobalXYZ) < len(self.markerID_midx ) ):
			
			print "getMarkerPositions:  Error. Could not see all markers."
			
			for i in range(len(alPSMarkers_midx)):
				print ("  Visible Marker IDs: " + str(alPSMarkers_midx[i].id) );
			
			return -1
			#rof
		else:
			return newPos_midx_GlobalXYZ
	
	def saveNewDefaults(self):
		
		fileObject = open(self.filePath + 'temp.rb','w')#self.fileName , 'w')
		
		# Get old marker id's and new positions
		oldRigidID_midx = self.markerID_midx
		newRigidPos_midx_xyz = self.markerPos_midx_localXYZ
		
		for idx in range(len(newRigidPos_midx_xyz )):
			posString = str(newRigidPos_midx_xyz[idx][0]) + ' ' + str(newRigidPos_midx_xyz[idx][1]) + ' ' + str(newRigidPos_midx_xyz[idx][2]);
			newLine = str(oldRigidID_midx[idx]) + ', ' + posString + '\n'
			fileObject.write(newLine);
			
		fileObject.close();
		
		#from shutil import move
		#move(self.filePath + 'temp.rb', self.filePath + self.fileName)

		from os import remove
		from shutil import move

		remove(self.filePath + self.fileName)
		move(self.filePath + 'temp.rb', self.filePath + self.fileName)

		print "Rigid body definition written to file"	
		
	def resetRigid( self, alPSMarkers_midx  ):
		
		# Build list of updated marker positions in newPosWorldCoords
		newPos_midx_GlobalXYZ = [];
		
		# Find markers that belong to the rigid tracker.
		# Their most position sensed on this frame is stored in newPos_midx_GlobalXYZH
		for oldMarkerIdx in range( 0, len(self.markerID_midx), 1 ):
			for newMarkerIdx in range( 0, len(alPSMarkers_midx), 1 ):
				
				# markerNumToID macro defined at top of file!
				oldRigidIDConverted = markerNumToID( self.trackerIdx, oldMarkerIdx );
				
				if( alPSMarkers_midx[newMarkerIdx].id == oldRigidIDConverted ):
					
					newPos_midx_GlobalXYZ.append( [ alPSMarkers_midx[newMarkerIdx].x, 
												alPSMarkers_midx[newMarkerIdx].y,
												alPSMarkers_midx[newMarkerIdx].z ] );

		# Check to make sure all markers were seen
		if( len(newPos_midx_GlobalXYZ) < len(self.markerID_midx ) ):
			
			print "ResetRigid:  Error. Could not see all markers."
			
			for i in range(len(alPSMarkers_midx)):
				print ("  Visible Marker IDs: " + str(alPSMarkers_midx[i].id) );
			#rof
			
			return
			
		##################################################################################
		#  Here's where we set the origin and frame of reference for a rigid body
		# The origin is set by averaging over markers in avgMarkerList_midx
		
		markerCount = 0;
		center_GlobalXYZ = [0,0,0]
		
		for mIdx in range(len(self.avgMarkerList_midx)):
			mNum = self.avgMarkerList_midx[mIdx]
			center_GlobalXYZ[0] = center_GlobalXYZ[0] + newPos_midx_GlobalXYZ[mNum][0]
			center_GlobalXYZ[1] = center_GlobalXYZ[1] + newPos_midx_GlobalXYZ[mNum][1]
			center_GlobalXYZ[2] = center_GlobalXYZ[2] + newPos_midx_GlobalXYZ[mNum][2]
		
		center_GlobalXYZ[0] = center_GlobalXYZ[0] / len(self.avgMarkerList_midx)
		center_GlobalXYZ[1] = center_GlobalXYZ[1] / len(self.avgMarkerList_midx)
		center_GlobalXYZ[2] = center_GlobalXYZ[2] / len(self.avgMarkerList_midx)
				
		# Convert from world-based to rigid-body based frame of reference
		newPosRigidCoords_mIdx_LocalXYZ = []
		
		for i in range( 0, len(newPos_midx_GlobalXYZ), 1 ):
			# ... also, offset origin by self.rigidOffset_worldXYZ ( in world coordinates ) 
			newPosRigidCoords_mIdx_LocalXYZ.append( [ newPos_midx_GlobalXYZ[i][0]-center_GlobalXYZ[0] + self.rigidOffset_worldXYZ[0],
										newPos_midx_GlobalXYZ[i][1]-center_GlobalXYZ[1] + self.rigidOffset_worldXYZ[1],
										newPos_midx_GlobalXYZ[i][2]-center_GlobalXYZ[2] + self.rigidOffset_worldXYZ[2], ] );
										
		#rof
			
		# Update rigid body definition on the owl server
		# and in this rigid body
		owlTracker( self.trackerIdx, OWL_DISABLE );
		
		for i in range( 0, len(newPosRigidCoords_mIdx_LocalXYZ), 1 ):
			
			owlMarkerfv( markerNumToID(self.trackerIdx, i), OWL_SET_POSITION, newPosRigidCoords_mIdx_LocalXYZ[i] )
			self.markerPos_midx_localXYZ[i] = newPosRigidCoords_mIdx_LocalXYZ[i];
			
		#rof
			
		owlTracker( self.trackerIdx, OWL_ENABLE );
		
		if not owlGetStatus():
			
			print ( "ResetRigid: Could not enable rigid body. OwlGetStatus returned: ", owlGetError() );
			
			exit();
		
		#fi
		
		print "Rigid body definition updated on server."
	
	def rotateRigid(self,rotateByDegs_XYZ):

		# create a temp vizard 3d object 
		# rotate, use vertices to redefine rigid object
		
		viz.startLayer(viz.POINTS)
		for idx in range(len(self.markerPos_midx_localXYZ)):
			viz.vertex(self.markerPos_midx_localXYZ[idx][0],self.markerPos_midx_localXYZ[idx][1],self.markerPos_midx_localXYZ[idx][2])
		
		tempRigidObject = viz.endLayer()
		tempRigidObject.visible( viz.OFF ) #Make the object invisible.
		
		# Update rigid body definition on the owl server
		tempRigidObject.setEuler( rotateByDegs_XYZ,viz.RELATIVE)		
		
		owlTracker(self.trackerIdx,OWL_DISABLE)

		count = 0
		for i in xrange(len(self.markerID_midx)):
			owlMarkerfv(markerNumToID(self.trackerIdx, i), OWL_SET_POSITION, tempRigidObject.getVertex(i,viz.RELATIVE))
			self.markerPos_midx_localXYZ[count] = tempRigidObject.getVertex(i,viz.RELATIVE)
			count +=1
			
		owlTracker(self.trackerIdx, OWL_ENABLE);
		
		tempRigidObject.remove() #Remove the object.
		
	

class phasespaceInterface(viz.EventClass):
	
	def __init__(self, config=None):
		''' Initializes an interface to the phasespace server. 
		Also sets up rigid body and marker tracker objects. '''
		
		viz.EventClass.__init__(self)
		
		self.config = config;
		self.origin = []
		self.scale = []
		self.serverAddress = []
		self.showEyePosition = 0
		self.updateActionPaused = 0
		self.markerSeenThisRound = [];
		
		self.allRigidBodyObjects = [];
		self.alPSMarkers_midx = []; # Note that these are vectors of Phasespace marker objects
		
		# a list of marker IDs on the server.
		# These ID's are converted to server ID's using the macro
		# markerNumToID.  
		self.markerServerID_mIdx = [] 
		
		# the index of the tracker used for free-floating markers
		self.markerTrackerIdx = 0 
		
		self.frame 			   		= 0
		self.last_tick 		   		= 0
		self.off 			   		= True
		self.markersUsedInRigid 	= [];
		
		self.mainViewUpdateAction = False
		self.rigidHMDIdx = -1
		#self.mainViewLinkedToHead 	= False
		
		if config==None:
			print('***Debug mode***')
			self.phaseSpaceFilePath = 'Resources/'
			self.origin 	= [0,0,0];
			self.scale 		= [0.001,0.001,0.001];
			self.serverAddress = '192.168.1.230';
			
			self.rigidFileNames_ridx= ['hmd-nvisMount.rb','paddle-hand.rb']
			self.rigidAvgMarkerList_rIdx_mId = '[1,2],[3,5]'
			self.rigidOffsetMM_ridx_WorldXYZ = '[0,0,0],[0,0,0]'
			
			#self.rigidBodyShapes_ridx = ['sphere','cylinder']
			#self.rigidBodySizes_ridx = [[.1],[.03,.09]]
			#self.rigidBodyToggleVisibility_ridx = [0,1]
			
			self.owlParamMarkerCount = 20
			self.owlParamFrequ = OWL_MAX_FREQUENCY
			self.owlParamInterp = 0
			self.owlParamMarkerCondThresh = 50
			self.owlParamPostProcess = 0
			self.owlParamModeNum = 3
			
		else:
			
			self.phaseSpaceFilePath 	= 'Resources/'
			self.origin 				= self.config['phasespace']['origin']
			self.scale 					= self.config['phasespace']['scale']
			self.serverAddress 			= self.config['phasespace']['phaseSpaceIP']
			self.rigidFileNames_ridx	= self.config['phasespace']['rigidBodyList']

			self.rigidOffsetMM_ridx_WorldXYZ = eval(self.config['phasespace']['rigidOffsetMM_ridx_WorldXYZ'])
			self.rigidAvgMarkerList_rIdx_mId = eval(self.config['phasespace']['rigidAvgMarkerList_rIdx_mId'])
			
			self.owlParamModeNum	= self.config['phasespace']['owlParamModeNum']
			
			self.owlParamMarkerCount = self.config['phasespace']['owlParamMarkerCount']
			self.owlParamFrequ = self.config['phasespace']['owlParamFrequ'] 
			self.owlParamInterp = self.config['phasespace']['owlParamInterp']
			self.owlParamMarkerCondThresh = self.config['phasespace']['owlParamMarkerCondThresh']
			self.owlParamRigidCondThresh = self.config['phasespace']['owlParamRigidCondThresh']
			self.owlParamPostProcess = self.config['phasespace']['owlParamPostProcess']
		
		flags = 'OWL_MODE'+ str(self.owlParamModeNum)
		
		if( self.owlParamPostProcess ):
			flags = flags + '|OWL_POSTPROCESS'
		
		initCode = owlInit(self.serverAddress,eval(flags))
		#initCode = owlInit(self.serverAddress,0)
		
		if (initCode < 0): 
			print "Mocap: Could not connect to OWL Server"
			exit()
		else:
			print '**** OWL Initialized with flags: ' + flags + ' ****'
		
		if not owlGetStatus():
			print "Mocap: could not enable OWL_STREAMING, owlGetStatus returned: ", owlGetError()
			exit();
			
		####################################################################################################################
		####################################################################################################################
		# Set server parameters
		
		# set default frequency
		if( self.owlParamFrequ == 0 ):
			
			self.owlParamFrequ = OWL_MAX_FREQUENCY;
			
		#fi
		
		owlSetFloat(OWL_FREQUENCY, self.owlParamFrequ)
		# start streaming
		owlSetInteger(OWL_STREAMING, OWL_ENABLE)
		owlSetInteger(OWL_INTERPOLATION, self.owlParamInterp)

		######################################################################
		######################################################################
		# Create rigid objects
		
		# for all rigid bodies passed into the init function...
		for rigidIdx in range(len(self.rigidFileNames_ridx)):
			# initialize rigid and store in self.allRigidBodyObjects
			
			rigidOffsetMM_WorldXYZ  = [0,0,0]
			rigidAvgMarkerList_mId = [0]
			
			if( len(self.rigidOffsetMM_ridx_WorldXYZ) < rigidIdx ):
				print 'Rigid offset not set! Using offset of [0,0,0]'
			else:
				rigidOffsetMM_WorldXYZ = self.rigidOffsetMM_ridx_WorldXYZ[rigidIdx]
			
			if( len(self.rigidAvgMarkerList_rIdx_mId) < rigidIdx ):
				print 'Average markers not provided! Using default (marker 0)'
			else:
				rigidAvgMarkerList_mId  = self.rigidAvgMarkerList_rIdx_mId[rigidIdx]
			
			
			
			self.allRigidBodyObjects.append( rigidObject(rigidIdx,self.phaseSpaceFilePath,self.rigidFileNames_ridx[rigidIdx],rigidAvgMarkerList_mId,rigidOffsetMM_WorldXYZ ))
			
		### Track markers not on rigid bodies 
		# Fill allRigidBodyObjects with server data
		
		numRigidMarkers = 0;
		self.markersUsedInRigid = []
		
		# for each rigid object...
		for rIdx in range(len(self.allRigidBodyObjects)):
			
			# create a list of the markers used rigid bodies
			markersUsedInThisRigid = self.allRigidBodyObjects[rIdx].markerID_midx 
			self.markersUsedInRigid.extend( markersUsedInThisRigid );
			
			# add this
			for mIdx in range(len(markersUsedInThisRigid)):
				self.markerServerID_mIdx.append( markerNumToID(self.allRigidBodyObjects[rIdx].trackerIdx, mIdx ) )
			
		#print markersUsedInRigid;
		if( self.markersUsedInRigid > self.owlParamMarkerCount ): 'Mocap: More markers used by rigid bodies than in owlParamMarkerCount.'
		
		#####################################################################
		######################################################################
		# One tracker contains all loose marker objects
		
		# Create tracker
		self.markerTrackerIdx = len(self.allRigidBodyObjects)
		owlTrackeri(self.markerTrackerIdx, OWL_CREATE, OWL_POINT_TRACKER)
		
		markerCount = 0;
		
		for markerNum in xrange(self.owlParamMarkerCount):
			
			# if marker not already used in rigid
			if self.markersUsedInRigid.count(markerNum) == 0: 

				# ...then add it to the loose marker tracker				
				owlMarkeri(markerNumToID(self.markerTrackerIdx,markerCount), OWL_SET_LED, markerNum)
				self.markerServerID_mIdx.append( markerNumToID(self.markerTrackerIdx,markerCount) )
				
				#print 'Mocap: marker: ' + str(markerNum) + ' is unattached, and was added to a tracker.'
				markerCount = markerCount+1
			else:
				pass
				#print 'Mocap: marker: ' + str(markerNum) + ' is attached to a rigid body'
				
#		self.markerTrackerIdx = len(self.allRigidBodyObjects)
#		
#		owlTrackeri(self.markerTrackerIdx, OWL_CREATE, OWL_POINT_TRACKER)
#		
#		markerCount = 0;
#		for i in xrange(self.owlParamMarkerCount):
#			
#			# if marker not already used in rigid...
#			if self.markersUsedInRigid.count(i) == 0: 
#				
#				owlMarkeri(markerNumToID(self.markerTrackerIdx,i), OWL_SET_LED, i)
#				self.markerServerID_mIdx.append( markerNumToID(self.markerTrackerIdx,markerCount) )
#				
#				print 'Mocap: marker: ' + str(i) + ' is unattached, and was added to a tracker.'
#				markerCount += 1;
#			else:
#				pass
#				#print 'Mocap: marker: ' + str(i) + ' is attached to a rigid body'
		
		owlTracker(self.markerTrackerIdx, OWL_ENABLE);
	
		if not owlGetStatus():
			print "HelmetHandPhaseSpace, could not enable OWL_STREAMING, owlGetStatus returned: ", owlGetError()
			exit();

		# Setup a timer to update owl server
		self.callback(viz.TIMER_EVENT, self.refreshMarkerPositions)
		self.starttimer(0,0,viz.FOREVER)
		self.turnOn()
	
	# end init
	######################################################################
	######################################################################
	def returnPointerToRigid(self,fileName):
		
		#  Accepts partial filenames, such as 'hmd' or 'paddle'
		#  Will return the first match found.
		
		for rigidIdx in range(len(self.rigidFileNames_ridx)):
			if( self.rigidFileNames_ridx[rigidIdx].find(fileName) > -1 ):
			#if( self.rigidFileNames_ridx[rigidIdx] == fileName):
				return self.allRigidBodyObjects[rigidIdx]
				
		print 'returnPointerToRigid: Could not find ' + fileName
		return 0
		
	def returnPointerToMarker(self,markerIdx):
	
		# Uses marker Idx to find the marker's position
		
		for mIdx in range(len(self.alPSMarkers_midx)):
			if( self.alPSMarkers_midx[mIdx].id == self.markerServerID_mIdx[markerIdx] ):
				return self.alPSMarkers_midx[mIdx]
		
		print 'returnPointerToMarker: Could not find marker number' + str(markerIdx)
		return 0
	
	def getMarkerPosition(self,markerID):
		# Returns marker position in VIZARD frame of reference
		
		for mIdx in range(len(self.alPSMarkers_midx)):
			if( self.alPSMarkers_midx[mIdx].id == self.markerServerID_mIdx[markerID] ):
				
				pos_XYZ = [self.alPSMarkers_midx[mIdx].x, self.alPSMarkers_midx[mIdx].y, self.alPSMarkers_midx[mIdx].z]
				
				condition = self.alPSMarkers_midx[mIdx].cond
				
				if( condition > 0 and condition < self.owlParamMarkerCondThresh ):
					return self.psPosToVizPos(pos_XYZ)
				else:
					# print 'Bad condition'
					return 0
				
	def checkForRigid(self,fileName):
		return( self.returnPointerToRigid(fileName) )
		
	def __del__(self):
		self.quit()
			
	def turnOff(self):
		
		if( self.mainViewUpdateAction ):
			self.disableHMDTracking()
			self.updateActionPaused = 1;
			
		self.off = 1
		self.setEnabled(False)
		

	def turnOn(self):
		
		self.off = 0
		self.setEnabled(True)
		
		if( self.updateActionPaused ):
			self.enableHMDTracking()
			self.updateActionPaused = 0;

	def isOn(self):
		return not self.off

	def quit(self):
		print "Disconnecting"
		owlDone()

	def getOutput(self):
		
		return ( ' PhaseSpace: ' + `viz.MainView.getPosition()` + ' ' + `viz.MainView.getQuat()` );
	
	#fed

	
	
	def refreshMarkerPositions( self, num ):
		
		if (num == 0):
			
			# These are bools for whether to look for a rigid body
			self.markerSeenThisRound = []
			numMarkersSeenThisRound = 1;
			markersSeen = [];
			#self.alPSMarkers_midx = []
			
			# There is a buffer of rigidsSeen and markersSeen
			# Empty this buffer and use latest information.
			while numMarkersSeenThisRound:
				
				markersSeen = owlGetMarkers();
				numMarkersSeenThisRound = len( markersSeen );
				numMarkersSeenLastRound = len( self.alPSMarkers_midx );
				
				if( numMarkersSeenThisRound > 0 ):
					
					tempMarkerVector = [];
					
					if( numMarkersSeenLastRound == 0 or len(self.alPSMarkers_midx) < self.owlParamMarkerCount ):
						
						# Until all markers have been seen at least once, 
						# self.allMarkers will less than the length of owlParamMarkerCount
						# e
						self.alPSMarkers_midx = markersSeen;
						
						#print ('mocap.refreshMarkerPositions: Getting marker data for first time.');
					
					else:
						
						for idx in range( 0, numMarkersSeenThisRound, 1 ):
							
							#self.alPSMarkers_midx = markersSeen;

							currentMarkersCondition = markersSeen[idx].cond
							
							# Run a quality check!
							if( currentMarkersCondition > 0 and currentMarkersCondition < self.owlParamMarkerCondThresh ):
							
								# If the marker was seen
								# The highter the .cond, the poorer the track quality
								self.alPSMarkers_midx[idx] = markersSeen[idx]; # Update the marker position
								self.markerSeenThisRound.append(idx)
							#fi
									
						#rof
					
					#fi
			
				#fi
				
			#elihw
	
				numRigidsSeenThisRound = 1;
				rigidsSeen = []
				pose = []
				
				while numRigidsSeenThisRound:
				
					rigidsSeen = owlGetRigids()			
					numRigidsSeenThisRound = len(rigidsSeen);
					numRigidsSeenLastRound = len( self.allRigidBodyObjects );
					
					if( numRigidsSeenThisRound > 0 ):
						
						if( numRigidsSeenLastRound == 0 ):
							# This runs the first frame of mocap only
							self.allRigidBodyObjects = rigidsSeen
							print ('mocap.refreshMarkerPositions: Getting rigid data for first time.');
						else:							
							# This runs on all subsequent frames in which a rigid body has been seen
							for rSeenIdx in range(0,len(rigidsSeen)):
								
								if( rigidsSeen[rSeenIdx].cond > 0 and  rigidsSeen[rSeenIdx].cond < self.owlParamMarkerCondThresh ):									
									
									pose = rigidsSeen[rSeenIdx].pose
									transformViz = self.psPoseToVizTransform(pose)
									#print transformViz
									self.allRigidBodyObjects[rSeenIdx].transformViz  = transformViz
								#else:
									#print 'Problem!'
									
			
	def resetRigid( self, fileName ):
		
		
		rigidBody = self.returnPointerToRigid( fileName );
		
		if( rigidBody ):
			
			#if( self.mainViewUpdateAction ):
			rigidBody.resetRigid( self.alPSMarkers_midx );
		else:
			
			print ('Error: Rigid body not initialized');
			
		#fi
		
	#fed
		
	def saveRigid(self,fileName):
		rigidBody = self.returnPointerToRigid(fileName)
		
		if(rigidBody):
			rigidBody.saveNewDefaults()
		else: print 'Error: Rigid body not initialized'
	
	def rotateRigid(self,fileName,rotateByDegs_XYZ):
		
		rigidBody = self.returnPointerToRigid(fileName)
		
		if(rigidBody):
			rigidBody.rotateRigid(rotateByDegs_XYZ)
		else: print 'Error: Rigid body not initialized'
		
		
#	def getOrientation(self,fileName):
#		
#		rigidBody = self.returnPointerToRigid(fileName)
#		
#		if(rigidBody):
#			rigidBody.getOrientation(self.alPSMarkers_midx)
#		else: print 'Rigid body not initialized'
	
	def toggleEyeProbes(self):
		
		hmdRigid = self.returnPointerToRigid('hmd')
		
		if( hmdRigid ):
			print 'Toggling eye probes'
			hmdRigid.toggleEyeMarkers()
		else:
			print 'No HMD rigid body'
	
	def enableHMDTracking(self):
		
		if( self.mainViewUpdateAction is False):
			print 'phaseSpaceInterface.enableHMDTracking: Mainview now updated to match *hmd*.rb position and orientation'
			self.mainViewUpdateAction = vizact.onupdate(viz.PRIORITY_FIRST_UPDATE,self.updateMainViewWithRigid,'hmd')
	
	def disableHMDTracking(self):
		
		if( self.mainViewUpdateAction ):
			print 'phaseSpaceInterface.enableHMDTracking: Mainview fred from *hmd*.rb'
			vizact.removeEvent(self.mainViewUpdateAction)
			self.mainViewUpdateAction = False
		
		#self.mainViewUpdateAction.
		
	def updateMainViewWithRigid(self,fileName):
	
		#print viz.getFrameNumber()
		rigidBody = self.returnPointerToRigid(fileName)
		
		if(rigidBody):
			
			# Transform is updated in refreshmarkerpositions
			transformViz  = rigidBody.transformViz
			#print transformViz  
			
			if transformViz:
				#print 'mocap.updateMainViewWithRigid: updating with transform'
				viz.MainView.setMatrix(transformViz)
			else:
				# No server data
				# This should only occur on startup
				# when rigid body has been registered
				# but system hasn't yet recieved data
				#print 'No transform'
				return
				
		else: print 'mocapINterface.updateMainViewWithRigid() Rigid body not initialized'

	def printMarkerIDs(self):
		
		for mIdx in range(len(self.alPSMarkers_midx)):
			
			print 'ps   ' + str(self.alPSMarkers_midx[mIdx].id)
			print 'mine ' + str(self.markerServerID_mIdx[mIdx])
			
			#print 'returnPointerToMarker: Could not find marker number' + str(markerID)
	
	def psPoseToVizTransform(self,psPose):
		
		# Set rigid body transformation matrix
		pos_XYZ = [  psPose[0]*self.scale[0] + self.origin[0],
		 psPose[1]*self.scale[1] + self.origin[1],
		-psPose[2]*self.scale[2] + self.origin[2] ];
		
		# PS quats are WABC
		# Viz wants ABCW
		quat_ABCW = [ psPose[4], psPose[5], -psPose[6], -psPose[3] ];
		
		transformViz = viz.Transform()
		transformViz.setQuat(quat_ABCW)
		transformViz.postTrans(pos_XYZ)
		transformViz.postAxisAngle(0,1,0,90)
		
		return transformViz
		
	def psPosToVizPos(self,posPS_XYZ):
		
		# FLip Z axis and rotate basis CCW 90 degs
		
		# Set rigid body transformation matrix
		pos_XYZ = [  posPS_XYZ[0]*self.scale[0] + self.origin[0],
		 posPS_XYZ[1]*self.scale[1] + self.origin[1],
		-posPS_XYZ[2]*self.scale[2] + self.origin[2] ];
		
		transformViz = viz.Transform()
		transformViz.postTrans(pos_XYZ)
		transformViz.postAxisAngle(0,1,0,90)
		
		return transformViz.getPosition()

if __name__ == "__main__":
	
	import vizact
	
	environment = viz.addChild('piazza.osgb')
	environment.setPosition(2.75,0,-.75)
	
	## Setup the phasespace server
	mocap = phasespaceInterface();
	
	mocap.enableHMDTracking()
	
	# Auto-update the mainview with the rigid body position
	# Note the use of 'hmd' as a partial string.
	# This is used to find a rigid body file that contains this string
	# In my case, hmd-oculus.rb
	#vizact.onupdate(viz.PRIORITY_FIRST_UPDATE,mocap.updateMainViewWithRigid,'hmd')
	#vizact.onupdate(viz.PRIORITY_FIRST_UPDATE,mocap.getMarkerPosition,1)
	
	#import oculus
	#hmd = oculus.Rift()
	import nvis
	hmd = nvis.nvisorSX111()
	
	hmdRigid = mocap.allRigidBodyObjects[mocap.rigidHMDIdx];			
	
	viz.vsync(viz.OFF)
	
	viz.window.setFullscreenMonitor([1,2]) 
	viz.setMultiSample(4)
	viz.MainWindow.clip(0.01 ,200)
	
	viz.go(viz.FULLSCREEN)
	
	vizact.onkeydown( 'h', mocap.resetRigid, 'hmd' );
	vizact.onkeydown( 'H', mocap.saveRigid, 'hmd' );
	vizact.onkeydown( 'p', mocap.resetRigid, 'paddle' );
	vizact.onkeydown( 'P', mocap.saveRigid, 'paddle' );
	vizact.onkeydown( 'e', mocap.enableHMDTracking);
