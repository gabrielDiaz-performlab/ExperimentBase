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

    def __init__(self, index, marker_ids, center_marker_ids):
        super(RigidTracker, self).__init__(index, marker_ids)

        self.center_marker_ids = center_marker_ids
        self._pose = Pose(pos=(0, 0, 0), quat=(1, 0, 0, 0), cond=-1)
        self._transform = viz.Transform()

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

        This should really only be called by the Phasespace object.

        Parameters
        ----------
        pose : Pose
            The new pose information for this rigid body.
        '''
        with self._lock:
            self._pose = pose

            # update the vizard transform matrix. requires some magical
            # swizzling of dimensions between phasespace and vizard!
            self._transform.makeIdent()
            self._transform.setQuat(pose.quat)
            self._transform.postTrans(pose.pos)

    def save(self, filename):
        '''Write the current pose for this rigid body out to a file.

        Parameters
        ----------
        filename : str
            The name of the file to write containing our rigid body information.
            (Typically this file ends with a .rb extension.)
        '''
        with open(filename, 'w') as handle:
            for i in self.marker_ids:
                handle.write('{}, {} {} {}\n'.format(i, *self.get_marker(i).pos))

    def reset(self):
        '''Reset this rigid body based on the current locations of its markers.
        '''
        logging.info('resetting rigid body %s', self.marker_ids)

        # assemble marker positions
        positions = []
        com = []
        with self._lock:
            for m in self.marker_ids:
                marker = self._raw_markers[self.marker_ids.index(m)]
                if marker is None or not 0 < marker.cond < 100:
                    logging.error('missing marker %d for reset', m)
                    return
                positions.append(marker.pos)
                if m in self.center_marker_ids:
                    com.append(marker.pos)

        # compute center of mass
        cx = sum(x for x, _, _ in com) / len(com)
        cy = sum(y for _, y, _ in com) / len(com)
        cz = sum(z for _, _, z in com) / len(com)
        logging.info('body center: (%s, %s, %s)', cx, cy, cz)

        # reset phasespace marker positions
        OWL.owlTracker(self._index, OWL.OWL_DISABLE)
        for i, (x, y, z) in enumerate(positions):
            OWL.owlMarkerfv(OWL.MARKER(self._index, i),
                            OWL.OWL_SET_POSITION,
                            (x - cx, y - cy, z - cz))
        OWL.owlTracker(self._index, OWL.OWL_ENABLE)


class Phasespace(viz.EventClass):
    '''Handle the details of getting mocap data from phasespace.

    Parameters
    ----------
    server_name : str
        The hostname for the phasespace server.
    frame_rate : float
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

    def __init__(self, server_name, frame_rate=200.,
                 scale=(0.001, 0.001, 0.001), offset=(0, 0, 0),
                 postprocess=False, slave=False):
        super(Phasespace, self).__init__()

        flags = 0
        if postprocess:
            flags |= OWL.OWL_POSTPROCESS
        if slave:
            flags |= OWL.OWL_SLAVE

        if OWL.owlInit(server_name, flags) < 0:
            raise OwlError('phasespace')

        OWL.owlSetFloat(OWL.OWL_FREQUENCY, OWL.OWL_MAX_FREQUENCY)
        OWL.owlSetInteger(OWL.OWL_STREAMING, OWL.OWL_ENABLE)

        self.scale = scale
        self.offset = offset
        self.trackers = []

        self.frame_rate = frame_rate
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
            wait = 1. / self.frame_rate - elapsed
            while wait < 0:
                wait += 1. / self.frame_rate
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
        ox, oy, oz = self.offset
        def transform(x, y, z):
            return sz * z + oz, sy * y + oy, sx * x + ox
        def swizzle(w, a, b, c):
            return c, b, a, -w

        for marker in markers:
            t, o = marker.id >> 12, marker.id & 0xfff
            x, y, z = marker.x, marker.y, marker.z
            self.trackers[t].update_markers(o,
                Marker(pos=transform(x, y, z), cond=marker.cond),
                Marker(pos=(x, y, z), cond=marker.cond))

        for rigid in rigids:
            self.trackers[rigid.id].update_pose(Pose(
                pos=transform(*rigid.pose[0:3]),
                quat=swizzle(*rigid.pose[3:7]),
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

    def track_rigid(self, markers=None, center_markers=None):
        '''Add a rigid-body tracker to the phasespace workload.

        Parameters
        ----------
        markers : varied
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

            If markers is an integer or sequence of integers, then random marker
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
        if isinstance(markers, dict):
            marker_map = markers
        if isinstance(markers, int):
            for i in range(markers):
                marker_map[i] = random_offsets()
        if isinstance(markers, (tuple, list, set)):
            for i in markers:
                marker_map[i] = random_offsets()
        if isinstance(markers, str):
            # load rigid body configuration from a file.
            with open(markers) as handle:
                for line in handle:
                    id, x, y, z = line.replace(',', '').strip().split()
                    marker_map[int(id)] = float(x), float(y), float(z)
        marker_map = sorted(marker_map.iteritems())
        marker_ids = tuple(i for i, _ in marker_map)

        # set up tracker using owl libraries.
        index = len(self.trackers)
        OWL.owlTrackeri(index, OWL.OWL_CREATE, OWL.OWL_RIGID_TRACKER)
        for i, (marker_id, pos) in enumerate(marker_map):
            OWL.owlMarkeri(OWL.MARKER(index, i), OWL.OWL_SET_LED, marker_id)
            OWL.owlMarkerfv(OWL.MARKER(index, i), OWL.OWL_SET_POSITION, pos)
        OWL.owlTracker(index, OWL.OWL_ENABLE)
        if OWL.owlGetStatus() == 0:
            raise OwlError('rigid tracker (index {})'.format(index))

        tracker = RigidTracker(index, marker_ids, center_markers or marker_ids)
        self.trackers.append(tracker)
        return tracker
