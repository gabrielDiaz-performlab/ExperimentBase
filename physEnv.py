import ode
import viz
import vizact


# The physical environment
class physEnv(viz.EventClass):
	def __init__(self):
		
		viz.EventClass.__init__(self)
		
		print 'physEnv.init(): Frame-rate hardcoded at 1/60!'
		
		self.frameRate = 1.0/60
		
		if( type(self.frameRate) is not float ):
			print 'physEnv.init(): frame-rate must be a float!'
			return
			
		# Keep track of physnodes in here
		self.physNodes_phys = []
		
		# This will be turned to TRUE when a collision has been detected
		self.collisionDetected = False
		
		# ODE initialization steps
		self.world = ode.World()
		
		print 'physEnv.init(): FIX:  Grav hardcoded at 9.8. Should accept gravity as a parameter, or include a function to change gravity'
		self.world.setGravity( [0,-9.8,0] )
		
		#self.world.setCFM(0.00001)
		#self.world.setERP(0.05)
		
		self.world.setCFM(0.00001)
		self.world.setERP(0.1)
		
		#self.world.setContactSurfaceLayer(0.001)
		

		##bounce_vel is the minimum incoming velocity to cause a bounce
		
		# Collision space where geoms live and collisions are simulated
		# 0 for a 'simple' space (faster and less accurate), 1 for a hash space
		self.space = ode.Space(1) 
		self.minBounceVel = .2 # min vel to cause a bounce
		
		####  A better description: 
		##Spaces are containers for geom objects that are the actual objects tested for collision. 
		##For the collision detection a Space is the same as the World for the dynamics simulation, and a geom object corresponds to a body object. 
		##For the pure dynamics simulation the actual shape of an object doesn't matter, you only have to know its mass properties. 
		##However, to do collision detection you need to know what an object actually looks like, and this is what's the difference between a body and a geom.

		# A joint group for the contact joints that are generated whenever two bodies collide
		self.jointGroup = ode.JointGroup()
		self.collisionList_idx = []
		self.contactJoints_idx = []
		self.contactObjects_idx = []
		# A list of non-collision joints, such as fixed joints, etc
		self.joints_jIdx = []
		
		############################################################################################
		############################################################################################
		## Contact/collision functions
		
		vizact.onupdate( viz.PRIORITY_PHYSICS, self.stepPhysics)
		#vizact.onupdate( viz.PRIOR, self.emptyContactGroups)
		
	def makePhysNode(self,type,pos=[0,0,0],size=[]):
		
		newPhysNode = physNode(self.world,self.space,type,pos,size)
		self.physNodes_phys.append(newPhysNode)
		
		#print 'Tried to make type ' + type + '.  Made type ' + str(type(newPhysNode)
		# Store the physnode in the list of physnodes
		#print 'physEnv.makePhysNode:  What happens to the list of physnodes when a physnode is erased?  Does the list update itself?'
		
		return newPhysNode 

	def stepPhysics(self):
		
#		self.emptyCollisionBuffer()
#		self.space.collide(self,self.detectCollisions)
#		self.world.step( 1.0/60  )
		
		self.emptyCollisionBuffer()
		
		numCycles = 20
		
		timeStep = (1.0/60) / numCycles
		
		for idx in range(numCycles):
			self.space.collide(self,self.detectCollisions)
			self.world.step( timeStep  )
		
		 # New collisions are now stored in self.contactJoints_idx
		 # They can be accessed using physEnv.getCollisions()
		
	def returnPointerToPhysNode(self,geomOrBody):
		
		# Accept a body or geom and return pointer to the phys node
		
		if( geomOrBody == ode.Body):
			print '*1'
		
		if( type(geomOrBody) == ode.Body):
			print '*2'
			
		if( type(geomOrBody) == ode.GeomObject or
			type(geomOrBody) == ode.GeomBox or
			type(geomOrBody) == ode.GeomCapsule or
			type(geomOrBody) == ode.GeomCCylinder or
			type(geomOrBody) == ode.GeomCylinder or
			type(geomOrBody) == ode.GeomPlane or
			type(geomOrBody) == ode.GeomRay or
			type(geomOrBody) == ode.GeomSphere or
			type(geomOrBody) == ode.GeomTriMesh ):
			
			'Searching geom'
			
			for idx in range(len(self.physNodes_phys)):
				if( self.physNodes_phys[idx].geom == geomOrBody ):
					return self.physNodes_phys[idx]
			print 'physEnv.returnPointerToPhysNode(): Geom not found in physNodes_phys'
				
		elif( type(geomOrBody) == ode.Body ):
			'Searching body'
			for idx in range(len(self.physNodes_phys)):
				if( self.physNodes_phys[idx].body == geomOrBody ):
					return self.physNodes_phys[idx]
			print 'physEnv.returnPointerToPhysNode(): Body not found in physNodes_phys'
		
		else:
			print 'physEnv.returnPointerToPhysNode(): Function accepts only geoms or body types.  You provided a ' + str(type(geomOrBody))
	
	def emptyCollisionBuffer(self):
		# This functino is explicit to make it clear 
		# that there is a buffer that should be emptied on each iteration
		
		self.jointGroup.empty()
		self.collisionList_idx_physNodes = []
		self.contactJoints_idx = []
		self.contactObjects_idx = []
		self.collisionDetected = False
		
	def getCollisions(self):
		
		# By default, getCollisions should be queried once on each iteration through the main loop
		# In the future the phys Env may be divorced from the mainloop, 
		# allowing for multiple runs of the phys engine at a finer temporal scale
		
		return self.contactJoints_idx
		
	def detectCollisions(self,thePhysEnv, geom1, geom2):
		
		 #  This callback is called whenever two objects may potentially collide. 
		 # The callback function then has to do a proper collision test and has to create contact joints whenever a collision has occured. 
		
		# Check if the objects do collide
		contactObjectList_idx = ode.collide(geom1, geom2)
		
		# Create contact joints
		for contactObject in contactObjectList_idx:
			
			body1 = geom1.getBody()
			body2 = geom2.getBody()
			
			# physNode objects have extra parameters attached to them, like bouncincess and friction
			physNode1 = self.returnPointerToPhysNode(geom1)
			physNode2 = self.returnPointerToPhysNode(geom2)
			
			#  Note that, for some reason, the ball is always geom  1
			if (physNode1 is None) is False and (physNode2 is None) is False:
				
				#####################################################################
				## Funct(geom): Return physnode that geom corresponds to
				## Bounce!  Calculate dynamics of bounce 
				
				self.collisionDetected = True

				#####################################################################
				
				self.collisionList_idx_physNodes.append([physNode1,physNode2])
				contactInfo = ode.collide(geom1,geom2)

				#####################################################################
				#####################################################################
				#  Should it stick?
				# <physNode> stickTo_gIdx is a list of pointers to geoms
				# that the node should stick to via fixed joint
				
#				## exit without doing anything if the two bodies are connected by a joint
#				# because this includes type contact joint, thsi prevents multiple contacts/collisions
#				if( body1 and body2 and ode.areConnected(body1, body2)):
#					print 'already connected'
#					return;


				for gIdx in range(len(physNode1.stickTo_gIdx)):
					if( physNode1.stickTo_gIdx[gIdx] == geom2 ):
						
						physNode1.disableCollisions()
						physNode1.disableMovement()
					
						# Ball always seems to be be the first geom
						physNode1.collisionPosLocal_XYZ = body2.getPosRelPoint(body1.getPosition())
				else:
					
					# This determines the dynamics of this particular collision / contact
					contactObject.setBounce(physNode1.bounciness * physNode2.bounciness ) # Coefficient of restitution
					
					#print str(viz.getFrameNumber()) + ': bounciness (1,2,Both): ' + str(physNode1.bounciness) + ' ' + str(physNode2.bounciness) + ' ' + str(physNode1.bounciness * physNode2.bounciness )
					
					# setBounceVel DOES NOT INFLUENCE BOUNCINESS.  it is the min vel needed for bounce to occur
					contactObject.setBounceVel(self.minBounceVel)
					
					contactObject.setMu(physNode1.friction * physNode2.friction ) # Friction
					
					# store for later
					self.contactObjects_idx.append(contactObject)
					
					#  Add joint to the contact group
					contactJoint = ode.ContactJoint(thePhysEnv.world, self.jointGroup, contactObject)
					
					# Create contact joint
					contactJoint.attach(body1, body2)
					self.contactJoints_idx.append(contactJoint)
					
				
		#####################################################		
		# Note that empyContactGroups is called automatically on each iteration 
		# This is necessary.
		### in physEnv.init(): vizact.onupdate( viz.PRIORITY_LAST_UPDATE, self.emptyContactGroups)
		#####################################################

class physNode():

	def __init__(self,world,space,shape,pos,size=[],bounciness = 1,friction = 0,vertices = None,indices = None):
		
		self.geom  = 0
		self.body = 0
		
		self.parentWorld = []
		self.parentSpace = []
		
		self.bounciness = bounciness;
		self.friction = friction;
		
		# A list of bodies that it will stick to upon collision
		self.stickTo_gIdx = []
		self.collisionPosLocal_XYZ = []
		
		if shape == 'plane':
			
			# print 'phsEnv.createGeom(): type=plane expects pos=ABCD,and NO size. SIze is auto infinite.'
			self.geom = ode.GeomPlane(space, [pos[0],pos[1],pos[2]], pos[3])
			self.parentSpace = space
			# No more work needed
			
		elif shape == 'sphere':
			
			#print 'Making sphere physNode'
			# print 'phsEnv.createGeom(): type=sphere expects pos=XYZ, and size=RADIUS'
			
			################################################################################################
			################################################################################################
			# Define the Body: something that moves as if under the 
			# influence of environmental physical forces
			
			self.geomMass = ode.Mass()
			
			# set sphere properties automatically assuming a mass of 1 and self.radius 
			mass = 1.0
			self.geomMass.setSphereTotal(mass, size) 
			
			self.body = ode.Body(world)
			self.parentWorld = world
			self.body.setMass(self.geomMass) # geomMass or 1 ? 
			self.body.setPosition(pos)
			
			# Define the Geom: a geometric shape used to calculate collisions
			#size = radius!
			self.geom = ode.GeomSphere(space,size)
			self.geom.setBody( self.body )
			self.parentSpace = space
			
			################################################################################################
			################################################################################################
		#elif shape == 'cylinder':
		elif('cylinder' in shape):
			#print 'Making cylinder physNode'
			
			# print 'phsEnv.createGeom(): type=sphere expects pos=XYZ, and size=RADIUS'
			
			################################################################################################
			################################################################################################
			# Define the Body: something that moves as if under the 
			# influence of environmental physical forces
			radius = size[1]
			length = size[0]
			
			self.geomMass = ode.Mass()
			
			# set sphere properties automatically assuming a mass of 1 and self.radius 
			mass = 1.0
						  
			if( shape[-2:] == '_X'):
				direction =  1
			elif(shape[-2:] == '_Y'):
				direction =  2
			else:
				direction =  3 # direction - The direction of the cylinder (1=x axis, 2=y axis, 3=z axis) 
			
			self.geomMass.setCylinderTotal(mass,direction,radius,length)
			
			self.body = ode.Body(world)
			self.parentWorld = world
			self.body.setMass(self.geomMass) # geomMass or 1 ? 
			self.body.setPosition(pos)
			
			# Define the Geom: a geometric shape used to calculate collisions
			#size = radius!
			self.geom = ode.GeomCylinder(space,radius,length)
			self.geom.setPosition(pos)
			
			self.geom.setBody( self.body )
			
			# This bit compensates for a problem with ODE
			# Note how the body is created in line with any axis
			# When I wrote this note, it was in-line with Y (direction=2)
			# The geom, however, can only be made in-line with the Z axis
			# This creates an offset to bring the two in-line
			vizOffsetTrans = viz.Transform()
			
			if( shape[-2:] == '_X'):
				vizOffsetTrans.setAxisAngle(1,0,0,90)
			elif(shape[-2:] == '_Y'):
				vizOffsetTrans.setAxisAngle(0,0,1,90)
			
			vizOffsetQuat = vizOffsetTrans.getQuat()
			
			odeRotMat = self.vizQuatToRotationMat(vizOffsetQuat)
			
			#print self.geom.getRotation()
			
			self.geom.setOffsetRotation(odeRotMat)
			
			self.parentSpace = space
			
		elif shape == 'box':
			
			################################################################################################
			################################################################################################
			# Define the Body: something that moves as if under the 
			# influence of environmental physical forces
			
			length = size[1]
			width = size[2]
			height = size[0]
			
			self.geomMass = ode.Mass()
			
			# set sphere properties automatically assuming a mass of 1 and self.radius 
			mass = 1.0
			
			self.geomMass.setBoxTotal(mass,length,width,height)
			
			self.body = ode.Body(world)
			self.parentWorld = world
			self.body.setMass(self.geomMass) # geomMass or 1 ? 
			self.body.setPosition(pos)
			
			# Define the Geom: a geometric shape used to calculate collisions
			#size = radius!
			self.geom = ode.GeomBox(space,[length,width,height])
			self.geom.setPosition(pos)
			
			self.geom.setBody( self.body )
			
			self.parentSpace = space
		
		elif shape == 'trimesh':
			
			if( vertices == None or indices == None):
				print 'physNode.init(): For trimesh, must pass in vertices and indices'
				
			self.body = ode.Body(world)
			self.parentWorld = world
			self.body.setMass(self.geomMass) # geomMass or 1 ? 
			self.body.setPosition(pos)
	
			triMeshData = ode.TrisMeshData()
			triMeshData.build(vertices, indices)
			self.geom = ode.GeomTriMesh(td, space)
			self.geom.setBody(self.body)
			
			## Set parameters for drawing the trimesh
			body.shape = "trimesh"
			body.geom = self.geom
			
			self.parentSpace = space
			
		else:
			print 'physEnv.physNode.init(): ' + str(type) +' not implemented yet!'
			return
			pass
		
	
	def remove(self):
		
		#self.parentRoom.physEnv.removeGeom(self.physGeom)
		
		self.geom.setBody(None)
		self.parentSpace.remove(self.geom)
		
		#self.parentWorld.remove()
		#dBodyDestroy(dBodyID);
		
		# Remove kinematic body
		del self.body
		self.body = 0
		
	def removeBody(self):
		
		del self.body
		self.body = 0
		
	def getQuaternion(self):
		
		# Note that vizard's quats are in format xyzw
		# however, ODE's quats are wxyz
		# here, we convert!
		
		if( self.body ):
			odeFormQuat = self.body.getQuaternion()
		elif( self.geom ):
			odeFormQuat = self.geom.getQuaternion()
		else:
			print 'No physnode!'
			return
			
		vizFormQuat = [ odeFormQuat[1], odeFormQuat[2], odeFormQuat[3], odeFormQuat[0]]
		
		return vizFormQuat 
		
	def updateWithTransform(self,transform):
		
		newPos = transform.getPosition()
		newQuat = transform.getQuat()
		
		self.setQuaternion(newQuat)
		self.setPosition(newPos)
		
	def setQuaternion(self,vizFormQuat):
		
		# Note that vizard's quats are in format xyzw
		# This function expects that format.
		# however, ODE's quats are wxyz
		# here, we convert!
		
		odeFormQuat = [ vizFormQuat[3], vizFormQuat[0], vizFormQuat[1], vizFormQuat[2]]
		
		if( self.body ):
			self.body.setQuaternion(odeFormQuat)
		
#		if( self.geom ):
#			self.geom.setQuaternion(odeFormQuat)
		
	def setPosition(self,pos):
		
		if( self.body ):
			self.body.setPosition(pos)
		
#		if( self.geom ):
#			self.geom.setPosition(pos)
		
	def setVelocity(self,vel_XYZ):
		self.setLinearVel(vel_XYZ)
		
	def setLinearVel(self,vel_XYZ):
		self.body.setLinearVel(vel_XYZ)
		
	def setBounciness(self,bounciness):
		self.bounciness = bounciness
		
	def setFriction(self,friction):
		self.friction = friction
	
	def enableMovement(self):
		self.body.setDynamic()
	
	def disableMovement(self):
		
		self.body.setLinearVel([0,0,0])
		self.body.setKinematic()
		
	def setStickUponContact(self,geom):
		
		# Prevent duplicates
		for idx in range(len(self.stickTo_gIdx)):
			if ( self.stickTo_gIdx[idx] == geom ):
				return
		
		# Add to the list
		self.stickTo_gIdx.append(geom)
		pass
	
	def queryStickyState(self, physNodeIn): 
		
		# returns a true if it is set to stick to the physNode
		for idx in range(len(self.stickTo_gIdx)):
			if ( self.stickTo_gIdx[idx] == physNodeIn.geom ):
				return 1
				
		return 0
	
	def enableCollisions(self):
		self.geom.enable()
	
	def disableCollisions(self):
		self.geom.disable()
		
	def vizQuatToRotationMat(self,quat):
		
		# Converts a quat in the form WXYZ (Vizard)
		# to a rotation matrix
		
		W = quat[0]
		X = quat[1]
		Y = quat[2]
		Z = quat[3]
		
		
		xx      = X * X;
		xy      = X * Y;
		xz      = X * Z;
		xw      = X * W;

		yy      = Y * Y;
		yz      = Y * Z;
		yw      = Y * W;

		zz      = Z * Z;
		zw      = Z * W;

		rotMat = [0]*10
		rotMat[0]  = 1 - 2. * ( yy + zz );
		rotMat[1]  =     2. * ( xy - zw );
		rotMat[2]  =     2. * ( xz + yw );

		rotMat[3]  =     2. * ( xy + zw );
		rotMat[4]  = 1 - 2. * ( xx + zz );
		rotMat[5]  =     2. * ( yz - xw );

		rotMat[6]  =     2. * ( xz - yw );
		rotMat[7]  =     2. * ( yz + xw );
		rotMat[8]  = 1 - 2. * ( xx + yy );
		
		return rotMat
		
	
if __name__ == "__main__":

	import vizact
	import visEnv

	viz.window.setFullscreenMonitor([1]) 
	viz.setMultiSample(4)
	viz.MainWindow.clip(0.01 ,200)

	viz.go(viz.FULLSCREEN)
	viz.MainView.setPosition([-5,2,10.75])
	viz.MainView.lookAt([0,2,0])

	print str( viz.phys.getGravity() )

	viz.vsync(1)

	# Configure the simulation using VRLABConfig
	# This also enables mocap

	# Create environment
	room = visEnv.room()

	ballInitialPos = [0,3,0]
	ballInitialVel = [0,0,0]

	ball = visEnv.visObj(room,'sphere',.07,ballInitialPos)
	ball.toggleUpdateWithPhys() # Visobj are inanimate until either tied to a physics or motion capture object
	ball.setBounciness(0.5)
	ball.physNode.enableMovement() # Turns on kinematics
