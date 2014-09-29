import viz
import vizshape
import vizact

# not a vizard module.  My own.

import math
from OWL import *

from os import rename

# Marker ID returned by owlGetMarkers() depends on 
# the ID of the rigid body to which it is attached
# and the the rigid_marker number (from zero to N, where N = num of markers attached to rigid body)
# This takes tracker num and rigid-marker id and converts.

def convertIDtoServerID(tracker, index):
	
	## BIT SHIFT LEFT BY 12 BITS IS BINARY MULTIPLICATION BY (2 ** 12)
	return ( ( tracker << 12 ) | (index) );

class rigidBody(viz.EventClass):
	
	def __init__(self,trackerIdx,filePath,fileName):
		
		self.filePath = filePath
		self.fileName = fileName
		
		self.trackerIdx = trackerIdx
		self.markerID_midx = []
		self.markerPos_midx_localXYZ = []

		self.serverData = False
		
		# Used to pre-translate rigid body by a set amount
		self.offset_XYZ = [0,0,0]
		
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
			owlMarkeri(  convertIDtoServerID(self.trackerIdx, i), OWL_SET_LED, self.markerID_midx[i] )
			# set marker positions
			owlMarkerfv( convertIDtoServerID(self.trackerIdx, i), OWL_SET_POSITION, self.markerPos_midx_localXYZ[i])
		
		# Activate tracking for rigidTracker
		owlTracker( self.trackerIdx, OWL_ENABLE )
		print 'Enabling ' + self.fileName
			
		# Flush requests and check for errors
		if not owlGetStatus():    
			#owl_print_error("error in point tracker setup", owlGetError())
			errorCode = owlGetError();
			print("Error in point tracker setup.", errorCode)
			return 0
		
#		if( room ):
#			#self._createvisNode(self.visNodeShape,self.visNodeSize)
#			pass
#		else():
#			pass
		
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
	
#	def getTransform(self,origin,scale):
#		
#			transformViz = viz.Transform()
#			transformViz.setQuat(rot)
#			transformViz.postTrans(pos)
#			transformViz.postAxisAngle(0,1,0,90)
#			
#			if transformViz is None:
#				print 'Nonetype'
#			else:
#				return transformViz
				
	def getMarkerPositions(self,allMarkers_midx):
		
		# Build list of updated marker positions in newPosWorldCoords
		newPos_midx_GlobalXYZ = [];
		
		for oldMarkerIdx in range( 0, len(self.markerID_midx), 1 ):

			for newMarkerIdx in range( 0, len(allMarkers_midx), 1 ):
				
				# Confused?  See convertIDtoServerID macro at top!
				oldRigidIDConverted = convertIDtoServerID( self.trackerIdx, oldMarkerIdx );
				
				if( allMarkers_midx[newMarkerIdx].id == oldRigidIDConverted ):
					
					newPos_midx_GlobalXYZ.append( [ allMarkers_midx[newMarkerIdx].x, 
												allMarkers_midx[newMarkerIdx].y,
												allMarkers_midx[newMarkerIdx].z ] );
												
				#fi
				
			#rof
			
		#rof
		
		if( len(newPos_midx_GlobalXYZ) < len(self.markerID_midx ) ):
			
			print "getMarkerPositions:  Error. Could not see all markers."
			
			for i in range(len(allMarkers_midx)):
				print ("  Visible Marker IDs: " + str(allMarkers_midx[i].id) );
			
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
		
	def resetRigid( self, allMarkers_midx ):
		
		#newPos_midx_GlobalXYZ = self.getMarkerPositions( allMarkers_midx )
		
		# Build list of updated marker positions in newPosWorldCoords
		newPos_midx_GlobalXYZ = [];
		
		# Find markers that belong to the rigid tracker.
		# Collect their position in newPos_midx_GlobalXYZH
		for oldMarkerIdx in range( 0, len(self.markerID_midx), 1 ):
			for newMarkerIdx in range( 0, len(allMarkers_midx), 1 ):
				
				# convertIDtoServerID macro defined at top of file!
				oldRigidIDConverted = convertIDtoServerID( self.trackerIdx, oldMarkerIdx );
				
				if( allMarkers_midx[newMarkerIdx].id == oldRigidIDConverted ):
					
					newPos_midx_GlobalXYZ.append( [ allMarkers_midx[newMarkerIdx].x, 
												allMarkers_midx[newMarkerIdx].y,
												allMarkers_midx[newMarkerIdx].z ] );
												
				#fi
				
			#rof
			
		#rof
			
		if( len(newPos_midx_GlobalXYZ) < len(self.markerID_midx ) ):
			
			print "ResetRigid:  Error. Could not see all markers."
			
			for i in range(len(allMarkers_midx)):
				print ("  Visible Marker IDs: " + str(allMarkers_midx[i].id) );
			#rof
			
			return
		
		####################################################################################
		##  Here's where we set the COM / center of rotatnoi for the rigid body
		
		if(self.fileName.find('hmd-nvisMount')> -1 ):
			
			# calculate the center of the rigid body using the first and last marker
			center_GlobalXYZ = [(newPos_midx_GlobalXYZ[1][0] + newPos_midx_GlobalXYZ[2][0])/2,
					  (newPos_midx_GlobalXYZ[1][1] + newPos_midx_GlobalXYZ[2][1])/2,
					  (newPos_midx_GlobalXYZ[1][2] + newPos_midx_GlobalXYZ[2][2])/2]
			
			# Move the center of mass in a world reference frame
			# to the locatnoi of the eye
			
			center_GlobalXYZ = center_GlobalXYZ + [0,-7.6,0]; 
	
					  
		elif( self.fileName.find('oculus')> -1 ):
			
			print 'mocapInterface.rigidBody.resetRigid: For '+ self.fileName + ' using marker 0 as COM'
			
			# calculate the center of the rigid body using the first and last marker
			center_GlobalXYZ = [newPos_midx_GlobalXYZ[0][0],newPos_midx_GlobalXYZ[0][1],newPos_midx_GlobalXYZ[0][2]+30]
					  
		if( self.fileName.find('paddle-rac')> -1 ):
			
			self.offset_XYZ = [0, -.1 ,0];
			print 'mocapInterface.rigidBody.resetRigid: For '+ self.fileName + ' using mean of markers 1 and 4 as COM'
			
			# middle of markers down the Y axis by .038
			
			# calculate the center of the rigid body using the first and last marker
			center_GlobalXYZ = [(newPos_midx_GlobalXYZ[1][0] + newPos_midx_GlobalXYZ[4][0])/2,
					  (newPos_midx_GlobalXYZ[1][1] + newPos_midx_GlobalXYZ[4][1])/2,
					  (newPos_midx_GlobalXYZ[1][2] + newPos_midx_GlobalXYZ[4][2])/2]

		if( self.fileName.find('squash')> -1 ):
			
			self.offset_XYZ = [0, -.2 ,0];
			print 'mocapInterface.rigidBody.resetRigid: For '+ self.fileName + ' using mean of markers 1 and 4 as COM'
			
			# middle of markers down the Y axis by .038
			
			# calculate the center of the rigid body using the first and last marker
			center_GlobalXYZ = [(newPos_midx_GlobalXYZ[1][0] + newPos_midx_GlobalXYZ[4][0])/2,
					  (newPos_midx_GlobalXYZ[1][1] + newPos_midx_GlobalXYZ[4][1])/2,
					  (newPos_midx_GlobalXYZ[1][2] + newPos_midx_GlobalXYZ[4][2])/2]
			
		else:
			print 'mocapInterface.rigidBody.resetRigid: For '+ self.fileName + ' using mean of all markers as COM'
			
			sum = []
			center_GlobalXYZ = [[3]*3]*3
			
			for dim in range(3):
				sum = 0
				
				for mIdx in range(len(newPos_midx_GlobalXYZ)):
					sum = sum+newPos_midx_GlobalXYZ[mIdx][dim]
				
				center_GlobalXYZ[dim] = sum / len(newPos_midx_GlobalXYZ)
				
		# Convert to rigid body coordinates, relative to center
		newPosRigidCoords_mIdx_LocalXYZ = []
		
		for i in range( 0, len(newPos_midx_GlobalXYZ), 1 ):
			
			newPosRigidCoords_mIdx_LocalXYZ.append( [ newPos_midx_GlobalXYZ[i][0]-center_GlobalXYZ[0] + self.offset_XYZ[0]*1000,
										newPos_midx_GlobalXYZ[i][1]-center_GlobalXYZ[1] + self.offset_XYZ[1]*1000,
										newPos_midx_GlobalXYZ[i][2]-center_GlobalXYZ[2] + self.offset_XYZ[2]*1000, ] );
										
		#rof
			
		# Update rigid body definition on the owl server
		# and in this rigid body
		owlTracker( self.trackerIdx, OWL_DISABLE );
		
		
		for i in range( 0, len(newPosRigidCoords_mIdx_LocalXYZ), 1 ):
			
			owlMarkerfv( convertIDtoServerID(self.trackerIdx, i), OWL_SET_POSITION, newPosRigidCoords_mIdx_LocalXYZ[i] )
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
			owlMarkerfv(convertIDtoServerID(self.trackerIdx, i), OWL_SET_POSITION, tempRigidObject.getVertex(i,viz.RELATIVE))
			self.markerPos_midx_localXYZ[count] = tempRigidObject.getVertex(i,viz.RELATIVE)
			count +=1
			
		owlTracker(self.trackerIdx, OWL_ENABLE);
		
		tempRigidObject.remove() #Remove the object.
		
	

class phasespaceInterface(viz.EventClass):
	
	def __init__(self, config=None):
		
		viz.EventClass.__init__(self)
		
		self.config = config;
		self.origin = []
		self.scale = []
		self.serverAddress = []
		self.showEyePosition = 0
		self.updateActionPaused = 0
		self.allRigids_ridx = [];
		self.allMarkers_midx = [];
		self.markerSeenThisRound = [];
		
		# a list of marker IDs on the server.
		# These ID's are converted to server ID's using the macro
		# convertIDtoServerID.  
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
			
			self.rigidFileNames_ridx= ['hmd-oculus.rb','paddle-hand.rb']
			
			#self.rigidBodyShapes_ridx = ['sphere','cylinder']
			#self.rigidBodySizes_ridx = [[.1],[.03,.09]]
			#self.rigidBodyToggleVisibility_ridx = [0,1]
			
			self.owlParamMarkerCount = 20
			self.owlParamFrequ = OWL_MAX_FREQUENCY
			self.owlParamInterp = 0
			self.owlParamMarkerCondThresh = 50
			self.owlParamPostProcess = 0
			self.owlParamModeNum = 1
		else:
			self.phaseSpaceFilePath 	= 'Resources/'
			self.origin 				= self.config['phasespace']['origin']
			self.scale 					= self.config['phasespace']['scale']
			self.serverAddress 			= self.config['phasespace']['phaseSpaceIP']
			self.rigidFileNames_ridx	= self.config['phasespace']['rigidBodyList']
			
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
		# Create rigid trackers
		
		for rigidIdx in range(len(self.rigidFileNames_ridx)):
			self.allRigids_ridx.append(rigidBody(rigidIdx,self.phaseSpaceFilePath,self.rigidFileNames_ridx[rigidIdx]))
			
		### Track markers not on rigid bodies 
		# Fill allRigids_ridx with server data
		# Count up markers on rigid bodies, while building a list
		
		numRigidMarkers = 0;
		self.markersUsedInRigid = []
		
		for rIdx in range(len(self.allRigids_ridx)):
			
			# This gets the list .markerID_midx and appends it to markersUsedInRigid
			markersUsedInThisRigid = self.allRigids_ridx[rIdx].markerID_midx 
			self.markersUsedInRigid.extend( markersUsedInThisRigid );
			
			for mIdx in range(len(markersUsedInThisRigid)):
				self.markerServerID_mIdx.append( convertIDtoServerID(rIdx,mIdx) )
			
		#print markersUsedInRigid;
		if( self.markersUsedInRigid > self.owlParamMarkerCount ): 'Mocap: More markers used by rigid bodies than in owlParamMarkerCount.'
		
		#####################################################################
		######################################################################

		self.markerTrackerIdx = len(self.allRigids_ridx)
		
		owlTrackeri(self.markerTrackerIdx, OWL_CREATE, OWL_POINT_TRACKER)
		
		markerCount = 0;
		
		for i in xrange(self.owlParamMarkerCount):
			# if marker not already used in rigid...
			if self.markersUsedInRigid.count(i) == 0: 
				
				owlMarkeri(convertIDtoServerID(self.markerTrackerIdx,markerCount), OWL_SET_LED, i)
				
				self.markerServerID_mIdx.append( convertIDtoServerID(self.markerTrackerIdx,markerCount) )
				
				print 'Mocap: marker: ' + str(i) + ' is unattached, and was added to a tracker.'
				markerCount += 1;
			else:
				pass
				
				#print 'Mocap: marker: ' + str(i) + ' is attached to a rigid body'
				
#		self.markerTrackerIdx = len(self.allRigids_ridx)
#		
#		owlTrackeri(self.markerTrackerIdx, OWL_CREATE, OWL_POINT_TRACKER)
#		
#		markerCount = 0;
#		for i in xrange(self.owlParamMarkerCount):
#			
#			# if marker not already used in rigid...
#			if markersUsedInRigid.count(i) == 0: 
#				
#				owlMarkeri(convertIDtoServerID(self.markerTrackerIdx,i), OWL_SET_LED, i)
#				
#				self.markerServerID_mIdx.append( convertIDtoServerID(self.markerTrackerIdx,markerCount) )
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
				return self.allRigids_ridx[rigidIdx]
				
		print 'returnPointerToRigid: Could not find ' + fileName
		return 0
		
	def returnPointerToMarker(self,markerID):
	
		#  Accepts partial filenames, such as 'hmd' or 'paddle'
		#  Will return the first match found.
		
		for mIdx in range(len(self.allMarkers_midx)):
			if( self.allMarkers_midx[mIdx].id == self.markerServerID_mIdx[markerID] ):
				return self.allMarkers_midx[mIdx]
		
		print 'returnPointerToMarker: Could not find marker number' + str(markerID)
		return 0
	
	def getMarkerPosition(self,markerID):
		# Returns marker position in VIZARD frame of reference
		
		for mIdx in range(len(self.allMarkers_midx)):
			if( self.allMarkers_midx[mIdx].id == self.markerServerID_mIdx[markerID] ):
				
				pos_XYZ = [self.allMarkers_midx[mIdx].x, self.allMarkers_midx[mIdx].y, self.allMarkers_midx[mIdx].z]
				
				condition = self.allMarkers_midx[mIdx].cond
				
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
			#self.allMarkers_midx = []
			
			# There is a buffer of rigidsSeen and markersSeen
			# Empty this buffer and use latest information.
			while numMarkersSeenThisRound:
				
				markersSeen = owlGetMarkers();
				numMarkersSeenThisRound = len( markersSeen );
				numMarkersSeenLastRound = len( self.allMarkers_midx );
				
				if( numMarkersSeenThisRound > 0 ):
					
					tempMarkerVector = [];
					
					if( numMarkersSeenLastRound == 0 or len(self.allMarkers_midx) < self.owlParamMarkerCount ):
						
						# Until all markers have been seen at least once, 
						# self.allMarkers will less than the length of owlParamMarkerCount
						# e
						self.allMarkers_midx = markersSeen;
						
						#print ('mocap.refreshMarkerPositions: Getting marker data for first time.');
					
					else:
						
						for idx in range( 0, numMarkersSeenThisRound, 1 ):
							
							#self.allMarkers_midx = markersSeen;

							currentMarkersCondition = markersSeen[idx].cond
							
							# Run a quality check!
							if( currentMarkersCondition > 0 and currentMarkersCondition < self.owlParamMarkerCondThresh ):
							
								# If the marker was seen
								# The highter the .cond, the poorer the track quality
								self.allMarkers_midx[idx] = markersSeen[idx]; # Update the marker position
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
					numRigidsSeenLastRound = len( self.allRigids_ridx );
					
					if( numRigidsSeenThisRound > 0 ):
						
						if( numRigidsSeenLastRound == 0 ):
							# This runs the first frame of mocap only
							self.allRigids_ridx = rigidsSeen
							print ('mocap.refreshMarkerPositions: Getting rigid data for first time.');
						else:							
							# This runs on all subsequent frames in which a rigid body has been seen
							for rSeenIdx in range(0,len(rigidsSeen)):
								
								if( rigidsSeen[rSeenIdx].cond > 0 and  rigidsSeen[rSeenIdx].cond < self.owlParamMarkerCondThresh ):									
									
									pose = rigidsSeen[rSeenIdx].pose
									transformViz = self.psPoseToVizTransform(pose)
									#print transformViz
									self.allRigids_ridx[rSeenIdx].transformViz  = transformViz
								#else:
									#print 'Problem!'
									
			
	def resetRigid( self, fileName ):
		
		
		rigidBody = self.returnPointerToRigid( fileName );
		
		if( rigidBody ):
			
			#if( self.mainViewUpdateAction ):
			rigidBody.resetRigid( self.allMarkers_midx );
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
#			rigidBody.getOrientation(self.allMarkers_midx)
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
		
		for mIdx in range(len(self.allMarkers_midx)):
			
			print 'ps   ' + str(self.allMarkers_midx[mIdx].id)
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
	
	hmdRigid = mocap.allRigids_ridx[mocap.rigidHMDIdx];			
	
	viz.window.setFullscreenMonitor([1,2]) 
	viz.setMultiSample(4)
	viz.MainWindow.clip(0.01 ,200)
	
	viz.go(viz.FULLSCREEN)
	
	vizact.onkeydown( 'h', mocap.resetRigid, 'hmd' );
	vizact.onkeydown( 'H', mocap.saveRigid, 'hmd' );
	vizact.onkeydown( 'p', mocap.resetRigid, 'paddle' );
	vizact.onkeydown( 'P', mocap.saveRigid, 'paddle' );
	vizact.onkeydown( 'e', mocap.enableHMDTracking);
