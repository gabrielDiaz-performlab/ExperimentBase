import viz
import dvr_arr as dvr

class DVRwriter(viz.EventClass):
	def __init__(self, filename, metadata, viewport, noeye=False):
		viz.EventClass.__init__(self)
		
		self.callback(viz.EXIT_EVENT, self.__exit__)
		
		self.metadata = metadata
		
		self.dvr = dvr.create(1)
		rc = dvr.open(self.dvr, filename, viewport, noeye)
		
		self.isPaused = 0;
		
	def __exit__(self):
		dvr.close(self.dvr, self.metadata)
		
	def toggleOnOff(self):
		
		if( self.isPaused == 0 ):
			print '***Pausing DVR***'
			dvr.stop(self.dvr)
			self.isPaused = 1
			
		elif( self.isPaused == 1):
			print '***Unpausing DVR***'
			dvr.start(self.dvr)
			self.isPaused = 0
			
	def turnOff(self):
		
		dvr.stop(self.dvr)
		self.isPaused = 1
		
	def turnOn(self):
		
		dvr.start(self.dvr)
		self.isPaused = 0
		
			
	def write(self, writables):
		
		lst = []
		for w in writables:
			if (None != w):
				output = w.getOutput()
				if None != output and '' != output:
					lst.append(output)

		dvr.post(self.dvr, '\t'.join(lst)+'\n')
	