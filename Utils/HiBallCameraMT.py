import viz
import vizmat
import vizact
import math

from Vector import Vector


class HiBallCamera(viz.EventClass):
	def __init__(self, offset, particle=None, sensorNum=0, attachTo=viz.MainView, preTrans = [0, 0, 0.1778]) :
		viz.EventClass.__init__(self)

		self.offset 	= offset
		self.particle   = particle
		self.sensorNum = sensorNum
		self.attachTo = attachTo
		self.preTran = preTrans
		
		self.off 			   = True

		self.vrpn = viz.add('vrpn6.dle')
		self.tracker = self.vrpn.addTracker('Tracker0@192.168.1.6', sensorNum) 
		self.tracker.swapPos([1,3,2]) 
		self.tracker.swapQuat([-1,-3,-2,4])
		
		self.preMat = vizmat.Transform()
		self.preMat.preEuler([-90, 0, 0]);
		self.preMat.preTrans(self.preTran);
		
		self.postMat = vizmat.Transform();
		self.postMat.postTrans(offset);
		
		vizact.ontimer(0, self.updateView)
		self.pos = [0,0,0]
		self.rot = [0, 0, 0, 0]
		self.turnOn()
		
	def __del__(self):
		self.quit()

	def turnOff(self):
		self.setEnabled(False)
		self.off = 1

	def turnOn(self):
		self.off = 0
		self.setEnabled(True)

	def isOn(self):
		return not self.off

	def quit(self):
		print "Quiting"

	def getOutput(self):
		#pos = viz.MainView.getPosition();
		#print ' HiBall: ', pos[2]
		return ' HiBall' + `self.sensorNum` + ':' + `self.pos` + ' ' + `self.rot`


	def updateView(self):
		
		data = self.tracker.getData()
		
		#print data;
		
		pos=data[:3];
		
		
		rot = data[3:7];
		
		
		m = viz.Transform()
		m.setQuat(rot)
		m.postTrans(pos)
		m.postAxisAngle(0,1,0,90)
		m.preMult(self.preMat);
		m.postMult(self.postMat);
		
		if self.attachTo != None and not self.off:
			self.attachTo.setMatrix(m)

		#self.extra_draw.draw()
		if self.particle != None:
			pos = m.getTrans()
			self.particle.moveTo(Vector([pos[0], self.offset[1], pos[2]]))
		#print "pos: ", pos, "quat: ", rot
		self.pos = pos
		self.rot = rot
		self.matrix = m

	

###############################################################################
if __name__ == "__main__":
	pass