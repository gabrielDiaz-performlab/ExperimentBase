        
from visEnv import *
        


import vizact
useConfig = True

    
def launchBall1(ball):
    
    launchVel_XYZ = [ 0,5,0 ]

    
    #vf**2=vo2 +2ad ->  d = -vi**2/2a
    
    ball.maxHeight =  -launchVel_XYZ[1]**2/(2*-9.8)
    
    #print 'predicted max height: ' + str(ball.maxHeight)
    
    ball.setPosition([0,ball.size,15])
    #ball.setVelocity(launchVel_XYZ)
    ball.physNode.setVelocity(launchVel_XYZ) 
    ball.enableMovement()
    
def launchBall2(ball):
    
    launchVel_XYZ = [ 0,5,-10 ]
    #vf**2=vo2 +2ad ->  d = -vi**2/2a
    ball.setPosition([0,1.7,15])

    ball.physNode.setVelocity(launchVel_XYZ) 
    ball.enableMovement()

def dropBall(ball,pos):
    
    ball.setPosition(pos)
    ball.physNode.setVelocity([0,0,0]) 
    ball.physNode.enableMovement()
        
        
		
if useConfig:
	
	#########################################################
	#########################################################
	# Configure the simulation using VRLABConfig
	# This also enables mocap tracking of markers and 
	# rigid bodies specified in config expConfigName
	
	
	expConfigName = 'bayesBallRac.cfg'
	import vrlabConfig
	config = vrlabConfig.VRLabConfig(expConfigName)
	room = room(config )

	#########################################################
	# Link mainview to hmd rigid body
	# note that 'hmd' is used to search rigid body files for the partial string
	# in my current setup, it will find hmd-oculus.rb and link to that
	
	if( config.use_phasespace and config.use_HMD and config.mocap.returnPointerToRigid('hmd') ):
			
			config.mocap.enableHMDTracking()
			
			vizact.onkeydown('-',config.mocap.disableHMDTracking)
			vizact.onkeydown('=',config.mocap.enableHMDTracking)
			
			vizact.onkeydown( 'h', config.mocap.resetRigid, 'hmd' );
			vizact.onkeydown( 'H', config.mocap.saveRigid, 'hmd' );
	else:
		
		viz.MainView.setPosition([room.wallPos_NegX +.1, 2, room.wallPos_NegZ +.1])
		viz.MainView.lookAt([0,2,0])
		
#        ##  Create mocap marker spheres - 1 per LED
#        markerVisObjList_idx = []
#        for idx in range(7,config.sysCfg['phasespace']['owlParamMarkerCount']):
#            markerVisObjList_idx.append(mocapMarkerSphere(config.mocap,room,idx))
	
	
	#########################################################
	# This block for debugging physics

	viz.MainView.setPosition([0,10,15])
	viz.MainView.lookAt([0,0,15])
	
	# Put a fake stationary paddle in the room
	paddleSize = [0.04,0.24,0.34]
	
	room.paddle.visNode.setPosition([0,2,15])
	room.paddle.visNode.setAxisAngle(1,0,0,115)
	room.paddle.visNode.color([0,1,0])
	room.paddle.applyVisToPhys()
	room.paddle.visNode.alpha(.5)
   
	# Link ball to physics engine
	
	ballInitialPos = [0,5,15]
	ballInitialVel = [0,0,0]
	
	#visObj(room,shape,size,position=[0,.25,-3],color=[.5,0,0])):
	ball = visObj(room,'sphere',.15,ballInitialPos,[0,1,1])
	ball.visNode.alpha(1)
	ball.setVelocity(ballInitialVel)
	ball.toggleUpdateWithPhys()
	ball.physNode.enableMovement()
	
	vizact.onkeydown('b',dropBall,ball,[0,7,15])
	vizact.onkeydown('f',dropBall,ball,[0,7,17])
	vizact.onkeydown('h',dropBall,ball,[0,7,13])
	vizact.onkeydown('t',dropBall,ball,[2.5,7,15])
	vizact.onkeydown('g',dropBall,ball,[-2.5,7,15])

	#########################################################
	# Register some keypresses
  
	#vizact.onkeydown('8',ball.toggleUpdateWithPhys)
	#vizact.onkeydown('9',ball.remove)
	
	#viz.MainView.setPosition([0,10,15])
	#viz.MainView.lookAt([0,0,15])
	
else:
	
	#########################################################
	##  Just create the basic visual environment
	
	viz.window.setFullscreenMonitor([1]) 
	viz.setMultiSample(4)
	viz.MainWindow.clip(0.01 ,200)
	
	viz.go(viz.FULLSCREEN)
   
	viz.MainView.setPosition([-5,2,10.75])
	viz.MainView.lookAt([0,2,0])        
	viz.vsync(1)
	room = room()