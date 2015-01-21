'''Classes for fetching and handling data from phasespace.'''

import collections
import logging
import OWL
import random
import threading
import time
import viz


ERROR_MAP = {
    OWL.OWL_NO_ERROR: 'No Error',
    OWL.OWL_INVALID_VALUE: 'Invalid Value',
    OWL.OWL_INVALID_ENUM: 'Invalid Enum',
    OWL.OWL_INVALID_OPERATION: 'Invalid Operation',
}

class Error(Exception):
    pass

class OwlError(Error):
    def __init__(self, msg):
        self.msg = 'setting up {}'.format(msg)
        self.err = OWL.owlGetError()

    def __str__(self):
        return '{}: OWL error {} (0x{:04x})'.format(
            self.msg, ERROR_MAP.get(self.err, self.err), self.err)


Marker = collections.namedtuple('Marker', 'pos cond')
Pose = collections.namedtuple('Pose', 'pos quat cond')


class PointTracker(viz.EventClass):
    '''Track a set of markers on the phasespace server.'''

    def __init__(self, index, marker_ids):
        super(PointTracker, self).__init__()

        self.marker_ids = marker_ids
        self._index = index
        self._lock = threading.Lock()
        self._raw_markers = [Marker(pos=(0, 0, 0), cond=-1) for _ in marker_ids]
        self._markers = [Marker(pos=(0, 0, 0), cond=-1) for _ in marker_ids]
        self._targets = [None for _ in marker_ids]

        # schedule an update event for following marker data.
        def update(event):
            paired = None
            with self._lock:
                paired = zip(self._markers, self._targets)
            for marker, target in paired:
                if target is not None and 0 < marker.cond < 100:
                    target.setPosition(marker.pos)
        self.callback(viz.UPDATE_EVENT, update)

    def get_markers(self):
        '''Get a dictionary of all tracked markers.

        Returns
        -------
        {int: Marker} :
            A dictionary mapping phasespace marker ids to marker information.
        '''
        with self._lock:
            return dict(zip(self.marker_ids, self._markers))

    def get_marker(self, marker_id):
        '''Get a marker tuple for the current state of the given marker_id.

        Parameters
        ----------
        marker_id : int
            The phasespace marker id to get.

        Raises
        ------
        ValueError :
            If the given marker_id is not in this tracker.

        Returns
        -------
        Marker :
            A marker tuple with current location information. The marker tuple
            has .pos and .cond attributes.
        '''
        with self._lock:
            return self._markers[self.marker_ids.index(marker_id)]

    def update_markers(self, offset, marker, raw_marker):
        '''Update the state of the given marker for this tracker.

        This should really only be called by the Phasespace object.

        Parameters
        ----------
        offset : int
            The offset (within this tracker) of the marker to update.
        marker : Marker
            A marker tuple with updated location information.
        raw_marker : Marker
            A marker tuple with unscaled phasespace data.
        '''
        with self._lock:
            self._markers[offset] = marker
            self._raw_markers[offset] = raw_marker

    def link_marker(self, marker_id, target):
        '''Link an object to a particular marker for continual updates.

        Parameters
        ----------
        marker_id : int
            Phasespace ID of a marker to link.
        target : any
            An object to link to the given marker.

        Raises
        ------
        ValueError :
            If the given marker_id is not in this tracker.
        '''
        with self._lock:
            self._targets[self.marker_ids.index(marker_id)] = target


class RigidTracker(PointTracker):
    '''Track a rigid body on the phasespace server.'''

    def __init__(self, index, marker_ids, center_marker_ids, localOffest_xyz):
        super(RigidTracker, self).__init__(index, marker_ids)

        self.center_marker_ids = center_marker_ids
        self._pose = Pose(pos=(0, 0, 0), quat=(1, 0, 0, 0), cond=-1)
        self._transform = viz.Transform()
        self._localOffset = [0,0,0]
        
        self.filepath = []
        self.filename = []
    
    def _loadPoseFromFile(self):
        ''' Set marker positions to positions contained in a specific *.rb file
        '''
        import os.path
        openfile = [];
        openfile = open( self.filepath + self.filename, 'r' );
        
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

    def _getLocalPositions(self):
        
        ''' Returns markerPositions within a local frame of reference
        Returns
        -------
        A list of tuples
        '''
        
        logging.info('Getting local marker positions %s', self.marker_ids)
        
        globalPositions = []
        localPositions = []
        com = []
        
        with self._lock:
            for m in self.marker_ids:
                marker = self._raw_markers[self.marker_ids.index(m)]
                if marker is None or not 0 < marker.cond < 100:
                    logging.error('missing marker %d for reset', m)
                    return
                globalPositions.append(marker.pos)
                if m in self.center_marker_ids:
                    com.append(marker.pos)

        # compute center of mass
        cx = sum(x for x, _, _ in com) / len(com)
        cy = sum(y for _, y, _ in com) / len(com)
        cz = sum(z for _, _, z in com) / len(com)
        logging.info('body center: (%s, %s, %s)', cx, cy, cz)
    

        # Construct marker_map, a dictionary of pos tuples
        # using marker ID's as keys
        localPositions = []
        for i, (x, y, z) in enumerate(globalPositions):
            localPositions.append((x - cx, y - cy, z - cz))
            
        return localPositions
            
    def get_pose(self):
        '''Return the current pose for our rigid body.

        Returns
        -------
        Pose :
            A pose tuple for our body. The pose tuple contains .pos, .quat, and
            .cond attributes.
        '''
        with self._lock:
            return self._pose

    def get_transform(self):
        '''Get a Vizard-compatible transform matrix for this rigid body.

        Returns
        -------
        matrix :
            A transform that can be applied to vizard objects.
        '''
        with self._lock:
            return self._transform

    def link_pose(self, target):
        '''Link an object to this rigid body for continual updates.

        Parameters
        ----------
        target : viz.Object
            An object to link to this rigid tracker.
        '''
        self.callback(
            viz.UPDATE_EVENT, lambda e: target.setMatrix(self.get_transform()))

    def update_pose(self, pose):
        '''Update the pose (and transform) for this rigid body.
        Note that self._localOffset is implemented here.
        This should really only be called by the Phasespace object.

        Parameters
        ----------
        pose : Pose
            The new pose information for this rigid body.
        '''
        with self._lock:
            self._pose = pose

            # update the vizard transform matrix. 
            # The pose has already been converted into the Vizard coordinate system
            self._transform.makeIdent()
            self._transform.setQuat(pose.quat)
            
            # Implement offset
            newPos = tuple(sum(t) for t in zip(pose.pos , tuple(self._localOffset) ) )
            
            self._transform.postTrans(newPos)
            
    def save(self):
        
        '''Save rigid body out to *.rb file
        '''
        
        fileObject = open(self.filepath + 'temp.rb','w')
        
        localPositions = self._getLocalPositions()
                            
        # Get old marker id's and new positions
        oldRigidID_midx = self.getLocalPositions()
        
        for idx in range(len(oldRigidID_midx)):
            posString = str(oldRigidID_midx[idx][0]) + ' ' + str(oldRigidID_midx[idx][1]) + ' ' + str(oldRigidID_midx[idx][2]);
            newLine = str(self.marker_ids[idx]) + ', ' + posString + '\n'
            fileObject.write(newLine);
            
        fileObject.close();
        
        #from shutil import move
        #move(self.filePath + 'temp.rb', self.filePath + self.fileName)

        from os import remove
        from shutil import move

        remove(self.filepath + self.filename)
        move(self.filepath + 'temp.rb', self.filepath + self.filename)

        print "Rigid body definition written to file"
         # reset phasespace marker positions
        
    def reset(self):
        
        '''Reset this rigid body based on the current locations of its markers.
        '''

        logging.info('resetting rigid body %s', self.marker_ids)

        localPositions = self._getLocalPositions()
                
        OWL.owlTracker(self._index, OWL.OWL_DISABLE)
        
        for i, (x, y, z) in enumerate(localPositions):
        
            OWL.owlMarkerfv(OWL.MARKER(self._index, i),
                            OWL.OWL_SET_POSITION,
                            [x,y,z])
                            
        OWL.owlTracker(self._index, OWL.OWL_ENABLE)


class Phasespace(viz.EventClass):
    '''Handle the details of getting mocap data from phasespace.

    Parameters
    ----------
    self.serverAddress : str
        The hostname for the phasespace server.
    self.owlParamFrequ : float
        Poll phasespace this many times per second. Defaults to 100.
    scale : (float, float, float)
        Scale raw phasespace position data by this amount. The default is to
        scale positions by 0.001, thus converting from mm to meters.
    offset : (float, float, float)
        Translate scaled phasespace position data by this amount.
    postprocess : bool
        Enable phasespace postprocessing. The default is no postprocessing.
    slave : bool
        Run our OWL client in slave mode. The default is to run OWL in master mode.
    '''

    UPDATE_TIMER = 0
    
    def __init__(self, config = None):
        
        super(Phasespace, self).__init__()
        
        self.rbTrackers_rbIdx = []; # A list full of rigidBodyTrackers
        self.markerTrackers_mIdx = []; # Note that these are vectors of Phasespace marker objects
        
        if config==None:
            
            print('***Debug mode***')
            self.phaseSpaceFilePath = 'Resources/'
            self.origin 	= [0,0,0];
            self.scale 		= [0.001,0.001,0.001];
            self.serverAddress = '192.168.1.230';
            
            self.rigidFileNames_ridx= ['hmd-nvisMount.rb','paddle-hand.rb']
            self.rigidAvgMarkerList_rIdx_mId = [[1,2],[3,5]]
            self.rigidOffsetMM_ridx_WorldXYZ = [[0,0,0],[0,0,0]]
                        
            self.owlParamMarkerCount = 20
            self.owlParamFrequ = OWL.OWL_MAX_FREQUENCY
            self.owlParamInterp = 0
            self.owlParamMarkerCondThresh = 50
            self.owlParamPostProcess = 0
            
            self.owlParamModeNum = 1
            print '**** Using default MODE #' + str(self.owlParamModeNum) + ' ****'
            
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

        flags = 0
        
        if self.owlParamPostProcess:
            flags |= OWL.OWL_POSTPROCESS

        if OWL.owlInit(self.serverAddress, flags) < 0:
            raise OwlError('phasespace')

        OWL.owlSetFloat(OWL.OWL_FREQUENCY, OWL.OWL_MAX_FREQUENCY)
        OWL.owlSetInteger(OWL.OWL_STREAMING, OWL.OWL_ENABLE)
        OWL.owlSetInteger(OWL.OWL_INTERPOLATION, self.owlParamInterp)
        
        self.trackers = []
        
        ######################################################################
        ######################################################################
        # Create rigid objects
        #head = mocap.track_rigid('Resources/hmd-nvisMount.rb', center_markers=(1,2))
        
        # for all rigid bodies passed into the init function...
        for rigidIdx in range(len(self.rigidFileNames_ridx)):
            # initialize rigid and store in self.rbTrackers_rbIdx
            
            rigidOffsetMM_WorldXYZ  = [0,0,0]
            rigidAvgMarkerList_mIdx   = [0]
            
            if( len(self.rigidOffsetMM_ridx_WorldXYZ) < rigidIdx ):
                print 'Rigid offset not set! Using offset of [0,0,0]'
            else:
                rigidOffsetMM_WorldXYZ = self.rigidOffsetMM_ridx_WorldXYZ[rigidIdx]
            
            if( len(self.rigidAvgMarkerList_rIdx_mId) < rigidIdx ):
                print 'Average markers not provided! Using default (marker 0)'
            else:
                rigidAvgMarkerList_mIdx  = self.rigidAvgMarkerList_rIdx_mId[rigidIdx]
            
            #rigidOffsetMM_WorldXYZ = self.rigidOffsetMM_ridx_WorldXYZ[rigidIdx]
            
            #rigidNameAndPathStr = self.phaseSpaceFilePath + self.rigidFileNames_ridx[rigidIdx]
            #self.rbTrackers_rbIdx.append( self.track_rigid(rigidNameAndPathStr , rigidAvgMarkerList_mId ) ) # rigidOffsetMM_WorldXYZ
            
            self.rbTrackers_rbIdx.append( self.track_rigid(self.rigidFileNames_ridx[rigidIdx],rigidAvgMarkerList_mIdx, rigidOffsetMM_WorldXYZ))
        
            #def track_rigid(self, filename, markers=None, center_markers=None):

        ###########################################################################
        self._updated = viz.tick()
        self._thread = None
        self._running = False

    def __del__(self):
        '''Clean up our connection to the phasespace server.'''
        OWL.owlDone()

    def start_thread(self):
        self._running = True
        self._thread = threading.Thread(target=self.update_thread)
        self._thread.start()
        self.callback(viz.EXIT_EVENT, self.stop_thread)

    def stop_thread(self):
        self._running = False
        if self._thread:
            self._thread.join()
            self._thread = None

    def update_thread(self):
        while self._running:
            self.update()
            elapsed = viz.tick() - self._updated
            wait = 1. / self.owlParamFrequ - elapsed
            while wait < 0:
                wait += 1. / self.owlParamFrequ
            #time.sleep(wait)

    def start_timer(self):
        self.callback(viz.TIMER_EVENT, self.update_timer)
        self.starttimer(Phasespace.UPDATE_TIMER, 0, viz.FOREVER)

    def update_timer(self, timer_id):
        if timer_id == Phasespace.UPDATE_TIMER:
            self.update()

    def update(self):
        '''Update our knowledge of the current data from phasespace.'''
        now = viz.tick()
        #logging.info('%dus elapsed since last phasespace update',
        #             1000000 * (now - self._updated))
        self._updated = now

        rigids = OWL.owlGetRigids()
        markers = OWL.owlGetMarkers()
        err = OWL.owlGetError()
        if err != OWL.OWL_NO_ERROR:
            hex = '0x%x' % err
            logging.debug(
                'OWL error %s (%s) getting marker data',
                ERROR_MAP.get(err, hex), hex)
            return

        sx, sy, sz = self.scale
        ox, oy, oz = self.origin
        
        def psPoseToVizardPose(x, y, z): # converts Phasespace pos to vizard cond
            return sz * z + oz, sy * y + oy, sx * x + ox
        def psQuatToVizardQuat(w, a, b, c): # converts Phasespace quat to vizard quat
            return c, b, a, -w

        for marker in markers:
            t, o = marker.id >> 12, marker.id & 0xfff
            x, y, z = marker.x, marker.y, marker.z
            self.trackers[t].update_markers(o,
                Marker(pos=psPoseToVizardPose(x, y, z), cond=marker.cond),
                Marker(pos=(x, y, z), cond=marker.cond))

        for rigid in rigids:
            self.trackers[rigid.id].update_pose(Pose(
                pos=psPoseToVizardPose(*rigid.pose[0:3]),
                quat=psQuatToVizardQuat(*rigid.pose[3:7]),
                cond=rigid.cond))

    def get_markers(self):
        '''Get a dictionary of all current marker locations.

        Returns
        -------
        dict :
            A dictionary that maps phasespace marker ids to Marker tuples.
        '''
        markers = {}
        for tracker in self.trackers:
            markers.update(tracker.get_markers())
        return markers

    def track_points(self, markers):
        '''Track a set of markers using phasespace.

        Parameters
        ----------
        markers : int or sequence of int
            If this is an integer, specifies the number of markers we ought to
            track. If this is a sequence of integers, it specifies the IDs of
            the markers to track.

        Returns
        -------
        PointTracker :
            A PointTracker instance to use for tracking the markers in question.
        '''
        if isinstance(markers, int):
            markers = range(markers)
        marker_ids = sorted(markers)

        # set up tracker using owl libraries.
        index = len(self.trackers)
        OWL.owlTrackeri(index, OWL.OWL_CREATE, OWL.OWL_POINT_TRACKER)
        for i, marker_id in enumerate(marker_ids):
            OWL.owlMarkeri(OWL.MARKER(index, i), OWL.OWL_SET_LED, marker_id)
        OWL.owlTracker(index, OWL.OWL_ENABLE)
        if OWL.owlGetStatus() == 0:
            raise OwlError('point tracker (index {})'.format(index))

        tracker = PointTracker(index, marker_ids)
        self.trackers.append(tracker)
        return tracker

    def track_rigid(self, markers_orFilename=None, center_markers=None, localOffest_xyz = [0,0,0]):
        '''Add a rigid-body tracker to the phasespace workload.

        Parameters
        ----------
        markers_orFilename : varied
            This parameter describes the markers that make up the rigid body. It
            can take several forms:

            - An integer, n. This will result in tracking markers 0 through n-1.
            - A sequence of integers, (n1, n2, ...). This will use markers n1,
              n2, ... to track the rigid body.
            - A dictionary, {n1: (x, y, z), n2: ...}. This will use markers n1,
              n2, ... to track the rigid body, using relative marker offsets
              specified by the values in the dictionary.
            - A string. The rigid body configuration will be loaded from the
              file named in the string.

            If markers_orFilename is an integer or sequence of integers, then random marker
            offsets will be assigned initially; call the .reset() method on the
            rigid tracker to reassign the marker offsets.

        center_markers : sequence of int
            This parameter specifies which marker ids to use for computing the
            center of the rigid body. If this is None, all markers will be
            used.

        Returns
        -------
        RigidTracker :
            A RigidTracker instance to use for tracking this rigid body.
        '''
        def random_offsets():
            return [random.random() - 0.5 for _ in range(3)]

        marker_map = {}

        # If markers_orFilename is a dictionary
        if isinstance(markers_orFilename, dict):
            marker_map = markers_orFilename

        # If markers_orFilename is an int
        if isinstance(markers_orFilename, int):
            for i in range(markers_orFilename):
                marker_map[i] = random_offsets()

        # If markers_orFilename is a tuple
        if isinstance(markers_orFilename, (tuple, list, set)):
            for i in markers_orFilename:
                marker_map[i] = random_offsets()
        
        # If markers_orFilename is a string
        if isinstance(markers_orFilename, str):
            
            fileLocation = self.phaseSpaceFilePath + markers_orFilename
            print "Initializing rigid body: " + fileLocation 
            
            # load rigid body configuration from a file.
            with open(fileLocation) as handle:
                for line in handle:
                    id, x, y, z = line.replace(',', '').strip().split()
                    marker_map[int(id)] = float(x), float(y), float(z)
                    
        marker_map = sorted(marker_map.iteritems())
        marker_ids = tuple(i for i, _ in marker_map)

        # set up tracker using owl libraries.
        # Marker locations have been read in from file
        index = len(self.trackers)
        OWL.owlTrackeri(index, OWL.OWL_CREATE, OWL.OWL_RIGID_TRACKER)
        for i, (marker_id, pos) in enumerate(marker_map):
            OWL.owlMarkeri(OWL.MARKER(index, i), OWL.OWL_SET_LED, marker_id)
            OWL.owlMarkerfv(OWL.MARKER(index, i), OWL.OWL_SET_POSITION, pos)
        OWL.owlTracker(index, OWL.OWL_ENABLE)
        if OWL.owlGetStatus() == 0:
            raise OwlError('rigid tracker (index {})'.format(index))
        
        tracker = RigidTracker(index,marker_ids, center_markers or marker_ids, localOffest_xyz)

        if isinstance(markers_orFilename, str):
            tracker.filepath = self.phaseSpaceFilePath
            tracker.filename = markers_orFilename
            
        self.trackers.append(tracker)
        return tracker
        
    def get_rigidTracker(self,fileName):
        '''
        Accepts a partial filenames, such as 'hmd' or 'paddle'
        Will return a pointer to ta rigidTracker
        This tracker is the first rigid body that contains this string
        '''
        
        
        for rigidIdx in range(len(self.rigidFileNames_ridx)):
            if( self.rigidFileNames_ridx[rigidIdx].find(fileName) > -1 ):
            #if( self.rigidFileNames_ridx[rigidIdx] == fileName):
                return self.rbTrackers_rbIdx[rigidIdx]
                
        print 'returnPointerToRigid: Could not find ' + fileName
        return 0
    
    def get_MarkerPos(self,markerIdx):
        ''' Returns marker position in vizard format
        '''
        markerTrackers_mIdx = self.get_markers()
        return markerTrackers_mIdx[markerIdx].pos
                
    def get_MarkerCond(self,markerIdx):
        '''Returns marker condition
        
        '''            
        markerTrackers_mIdx = self.get_markers()
        return markerTrackers_mIdx[markerIdx].cond
            

    def resetRigid( self, fileName ):
        '''Finds the rigid body corresponding to this filename.
        Resets marker positions based upon current position
        '''
        
        rigidBody = self.get_rigidTracker( fileName );
        
        if( rigidBody ):
            
            #if( self.mainViewUpdateAction ):
            rigidBody.reset()
        else:
            
            print ('Error: Rigid body not initialized');
            
        #fi
        
    #fed
        
    def saveRigid(self,fileName):
        '''Finds the rigid body corresponding to this filename.
        Saves marker positions out to *.rb file
        '''
        
        rigidBody = self.get_rigidTracker(fileName)
        
        if(rigidBody):
            rigidBody.save()
        else: print 'Error: Rigid body not initialized'

