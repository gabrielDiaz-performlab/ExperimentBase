import viz
import vizshape
import vizmat
import math
import ode
import physEnv
import vizact
import Shadow

ft = .3048
inch = 0.0254
m = 1
eps = .01
yard = 1.09361
nan = float('NaN')
            
class room():
    def __init__(self,config=None):

        self.roomvisNode = viz.addGroup()
        self.walls = viz.addGroup()
        self.objects = viz.addGroup()
        
        ##################################
        ## Physical environment
        self.physEnv = physEnv.physEnv()
        ##################################
        
        if config==None:
            print 'No config'
            self.texPath = 'Resources/'
            self.roomWidth = 50.0; #self.wallPos_PosX-self.wallPos_NegX
            self.roomLength = 50.0; #self.wallPos_PosZ-self.wallPos_NegZ
            
            self.translateOnZ = 2.0;
            self.translateOnX = 0.0;
            
            self.wallPos_PosZ = self.roomLength/2 + self.translateOnZ;
            self.wallPos_NegZ = -self.roomLength/2 + self.translateOnZ;            
            self.wallPos_PosX = self.roomWidth/2 + self.translateOnX;
            self.wallPos_NegX = -self.roomWidth/2 + self.translateOnX;
            self.ceilingHeight = 25.0;
            self.drawStandingBox = False
        else:
            
            self.texPath = config.expCfg['experiment']['texturePath'] #'Resources/'
            
            roomSize_WHL = map(float,config.expCfg['room']['roomSize_WHL'])
            
            self.roomWidth = roomSize_WHL[0]
            self.ceilingHeight = roomSize_WHL[1]
            self.roomLength = roomSize_WHL[2]
            
            self.translateOnX = float(config.expCfg['room']['translateRoom_X'])
            self.translateOnZ = float(config.expCfg['room']['translateRoom_Z'])
            
            self.wallPos_PosZ = self.roomLength/2 + self.translateOnZ;
            self.wallPos_NegZ = -self.roomLength/2 + self.translateOnZ;            
            self.wallPos_PosX = self.roomWidth/2 + self.translateOnX;
            self.wallPos_NegX = -self.roomWidth/2 + self.translateOnX;
            
            self.drawStandingBox = config.expCfg['experiment']['drawStandingBox']
            
            self.isLeftHanded = float(config.expCfg['experiment']['isLeftHanded'])
            
            if( self.drawStandingBox  ):
                self.standingBoxOffset_X = config.expCfg['room']['standingBoxOffset_X']
                self.standingBoxSize_WHL = map(float, config.expCfg['room']['standingBoxSize_WHL'])
            
            self.shiftWorldRelToUser_XYZ = [0,0,0]
                    
            ####################################################################
            ##  Fill room with objects 
            
            self.visObjNames_idx = config.expCfg['visObj']['visObjVarNames']
            self.visObjShapes_idx = config.expCfg['visObj']['visObjShapes']
            self.visObjSizes_idx = eval(config.expCfg['visObj']['visObjSizesString'])
        
            self.fillWithVisObj()
            
            self.shadowSource = []
            self.lightSource = []
            self.setLighting()
            
            ####################################################################
        
        texScale = 1;
        
        wallTexPath = self.texPath  + 'tile_slate.jpg'
        floorTexPath = self.texPath + 'tile_wood.jpg'
        
        planeABCD = [0,-1,0,-self.ceilingHeight]
        self.ceiling = wall(self.physEnv,[self.roomWidth,self.roomLength],[1,0,0,-90],
                                [self.translateOnX,self.ceilingHeight,self.translateOnZ],
                                wallTexPath,texScale,planeABCD);        
                                
        planeABCD = [0,0,-1,-self.wallPos_PosZ]
        self.wall_PosZ = wall(self.physEnv,[self.ceilingHeight,self.roomWidth],[0,0,1,90],
                                [self.translateOnX,self.ceilingHeight/2, self.wallPos_PosZ],
                                wallTexPath,texScale,planeABCD);
       
        planeABCD = [0,0,1,self.wallPos_NegZ] 
        self.wall_NegZ = wall(self.physEnv,[self.roomWidth,self.ceilingHeight],[0,1,0,180],
                                [self.translateOnX,self.ceilingHeight/2, self.wallPos_NegZ],
                                wallTexPath,texScale,planeABCD);
        
        planeABCD = [-1,0,0,-self.wallPos_PosX] 
        self.wall_PosX = wall(self.physEnv,[self.roomLength,self.ceilingHeight],[0,1,0,90],
                                [self.wallPos_PosX,self.ceilingHeight/2,self.translateOnZ ],
                                wallTexPath,texScale,planeABCD);
        
        planeABCD = [1,0,0,self.wallPos_NegX] 
        self.wall_NegX = wall(self.physEnv,[self.roomLength,self.ceilingHeight],[0,-1,0,90],
                                [self.wallPos_NegX,self.ceilingHeight/2,self.translateOnZ ],
                                wallTexPath,texScale,planeABCD);
       
        planeABCD = [0,1,0,0]
        self.floor = wall(self.physEnv,[self.roomWidth,self.roomLength],[1,0,0,90],
                                [self.translateOnX,0, self.translateOnZ],
                                floorTexPath,texScale,planeABCD);
                                
        self.floor.visNode.setParent(self.walls)
        self.ceiling.visNode.setParent(self.walls)
        self.wall_PosZ.visNode.setParent(self.walls)
        self.wall_NegZ.visNode.setParent(self.walls)
        self.wall_PosX.visNode.setParent(self.walls)
        self.wall_NegX.visNode.setParent(self.walls)
        
        self.walls.setParent( self.roomvisNode )
        self.objects.setParent( self.roomvisNode )
        
        if( self.drawStandingBox ):
            self.createStandingBox()
        
    def fillWithVisObj(self):
        # This little bit of code fills the room with objects specified in the config file
        
        
        for idx in range(len(self.visObjNames_idx)):
                if( len(self.visObjNames_idx[idx])>0 ):
                    execString = 'self.' + self.visObjNames_idx[idx] + ' = visObj(self,self.visObjShapes_idx[idx],self.visObjSizes_idx[idx])'
                    print 'VisEnv:Room: Added ' + self.visObjNames_idx[idx]
                    exec(execString)
        
    def createStandingBox(self):
        
            
            # Draw the standing box
            
            boxSizeInfo = [ self.standingBoxSize_WHL[0], self.standingBoxSize_WHL[1],self.standingBoxSize_WHL[2]]
            
            self.standingBox = vizshape.addBox( boxSizeInfo,color=viz.GREEN,splitFaces = True,back=True)
            self.standingBox.emissive([0,1,0])
            self.standingBox.alpha(0.5)
            
            if( self.isLeftHanded ): self.standingBoxOffset_X *= -1;
            
            self.standingBox.setPosition(float(-self.standingBoxOffset_X),self.standingBoxSize_WHL[1]/2,.01)            

            self.standingBox.color(1,0,0,node='back')            
            self.standingBox.emissive(1,0,0,node='back')
            self.standingBox.alpha(0.7,node='back')

            self.standingBox.setParent(self.objects)
            #self.standingBox.disable(viz.CULLING)
            self.standingBox.disable(viz.CULL_FACE)

            
        
   
        
    def setLighting(self):
        
        viz.MainView.getHeadLight().disable()
        #viz.MainView.get
        self.lightSource = viz.addLight() 
        self.lightSource.enable() 
        self.lightSource.position(0, self.ceilingHeight, 0) 
        self.lightSource.spread(180) 
        self.lightSource.intensity(2)
    
        
        ### ADD A SHADOW
        #SHADOW_RES = 256*10
        SHADOW_RES = 100*10
        SHADOW_PROJ_POS = (0, self.ceilingHeight, 0)
        SHADOW_AREA = (self.roomWidth,self.roomLength)
        
        #Create shadow projector
        self.shadowSource = Shadow.ShadowProjector(size=SHADOW_RES,pos=SHADOW_PROJ_POS,area=SHADOW_AREA)

class wall():
    def __init__(self,physEnv,dimensions,axisAngle,position,texPath,texScale,planeABCD):
        
        # A wall object invludes a specialized visNode
        # This visNode is actually a texQuad

        ################################################################################################
        ################################################################################################
        ## Set variables
        
        self.dimensions = dimensions;
        self.axisAngle = axisAngle;
        self.position = position;
        self.texPath = texPath;
        self.texScale = texScale;
        
        ################################################################################################
        ################################################################################################
        ##  Create visNode: a texture quad
        
        self.visNode = viz.addTexQuad()
        self.visNode.setScale(dimensions[0],dimensions[1])
        self.visNode.setPosition(position)
        self.visNode.setAxisAngle(axisAngle)
        self.visNode.disable(viz.DYNAMICS)
        self.visNode.enable([viz.LIGHTING,viz.CULL_FACE])

        # Put texture on the quad  
        matrix = vizmat.Transform()
        matrix.setScale([dimensions[0]/texScale,dimensions[1]/texScale,texScale])
        
        self.visNode.texmat(matrix)
        
        self.texObj = viz.addTexture(texPath)
        self.texObj.wrap(viz.WRAP_T, viz.REPEAT)
        self.texObj.wrap(viz.WRAP_S, viz.REPEAT)
        self.visNode.texture(self.texObj)
        
        ################################################################################################
        ################################################################################################
        ##  Create physNode plane
        
        self.physNode = physEnv.makePhysNode('plane',planeABCD)
        
class visObj(viz.EventClass):
    def __init__(self,room,shape,size,position=[0,.25,-3],color=[.5,0,0],alpha = 1):
       
        
        ################################################################################################
        ################################################################################################
        ## Set variables
        
        self.elasticity = 1;
        self.color_3f = color
        self.position = position
        self.shape = shape
        self.alpha = alpha
        self.isDynamic = 0
        self.isVisible = 1
        self.inFloorCollision = 0
        
        # Note that size info is particular to the shape
        # For ball, just a radius
        # for box, lenght width and height
        # etc.
        
        self.size = size
        self.parentRoom = room;
        
        self.visNode = 0
        self.physNode = 0
        self.obj = []
        
        ################################################################################################
        ################################################################################################
        ## Variables related to automated updating with physics or motion capture
        
        self.applyVisToPhysAction = 0
        self.updatingPhysWithVis = 0
        
        self.updateAction = 0
        self.updatingWithPhys = 0
        
        self.mocapDevice = 0
        self.updatingWithMocap = 0
        
        self.rigidBodyFile = 0
        self.markerNumber = -1
        
        ################################################################################################
        ################################################################################################
        ## Create visual object
        
        self.makeBasicVisNode()
        #self.visNode.color(self.color_3f)
        self.setColor(self.color_3f)
        self.visNode.visible(True)

        ## Create physical object
        self.visNode.dynamic() # This command speeds up rendering, supposedly    
            
        #self.updateAction = vizact.onupdate(viz.PRIORITY_LINKS, self.applyVisToPhys)
        
    def __del__(self):
        
        print viz.getFrameNumber()
        
        # Remove physical component
        self.physNode.remove()
        
        # Stop updating visNode
        if( self.updateAction ):
            self.updateAction.remove()
            
        # Remove visual component
        self.visNode.remove()
     
    def remove(self):
        self.__del__()
        
    def makeBasicVisNode(self):
        
        # Returns a pointer to a vizshape object
        # This is added to the room.objects parent
        newvisNode = []
        
        if(self.shape == 'box' ):
            #print 'Making box visNode'
            
            if( type(self.size) == float or len(self.size) !=3): 
                print '**********Invalid size for box'
                print 'Check rigidBodySizesString.  Expected 3 val for box: height,width,length.'
                print 'Got: ' + str(self.size)
                import winsound
                winsound.Beep(1000,250)
            lwh = [self.size[1],self.size[2],self.size[0]]
            newvisNode = vizshape.addBox(lwh ,alpha = self.alpha,color=viz.RED)
            
        elif(self.shape == 'sphere'):
            
            if( type(self.size) == list and len(self.size) == 1 ):
                self.size = float(self.size[0])
                
            if( type(self.size) != float):  # accept a float
                
                print '**********Invalid size for sphere'
                print 'Check rigidBodySizesString.  Expected 1 val for sphere: radius'
                print 'Got: ' + str(self.size)
                import winsound
                winsound.Beep(1000,250)
            
            #print 'Making sphere visNode'
            newvisNode = vizshape.addSphere(radius = float(self.size), alpha = self.alpha,color=viz.BLUE,slices=10, stacks=10)
        
        elif('cylinder' in self.shape):
            
            if( type(self.size) == float or len(self.size) !=2): 
                
                print '**********Invalid size for cylinder'
                print 'Check rigidBodySizesString.  Expected 2 val for cylinder: height,radius'
                print 'Got: ' + str(self.size)
                import winsound
                winsound.Beep(1000,250)
                
            #print 'Making cylinder visNode'
            
            if( self.shape[-2:] == '_X' or self.shape[-2:] == '_Y' or self.shape[-2:] == '_Z' ):
                axisString = 'vizshape.AXIS' + self.shape[-2:]
                print axisString + axisString + axisString + axisString
                evalString = 'vizshape.addCylinder(height=self.size[0],radius=self.size[1], alpha = self.alpha,color=viz.BLUE,axis=' + axisString + ')'
                
                newvisNode = eval(evalString)
            else:
                newvisNode = vizshape.addCylinder(height=self.size[0],radius=self.size[1], alpha = self.alpha,color=viz.BLUE, axis = vizshape.AXIS_Y )
            
        if( newvisNode ) :
                    
            self.visNode = newvisNode
            
        else:
            
            print 'vizEnv.room.makeBasicVisNode(): Unable to create visNode'
            import winsound
            winsound.Beep(1000,250)
        
        #if(self.parentRoom):
        newvisNode.setParent(self.parentRoom.objects)

    def enablePhysNode(self):

        ## Create physical object
        self.physNode = self.parentRoom.physEnv.makePhysNode(self.shape,self.position,self.size)
        self.setVelocity([0,0,0])
        self.physNode.disableMovement()
        
    def setVelocity(self,velocity):
        
        #self.visNode.setVelocity(velocity)
        if( self.physNode.body ):
            self.physNode.body.setLinearVel(velocity)
    
    def getVelocity(self):
        
        #self.visNode.setVelocity(velocity)
        if( self.physNode.body ):
            return self.physNode.body.getLinearVel()
    
    def getAngularVelocity(self):
        
        #self.visNode.setVelocity(velocity)
        if( self.physNode.body ):
            return self.physNode.body.getAngularVel()
            
    def setPosition(self,position):

        self.physNode.setPosition(position)
        self.visNode.setPosition(position)
        
    def setColor(self,color3f):
        
        self.visNode.color(color3f)
        #self.visNode.ambient(color3f)
        #self.visNode.specular(color3f)
    
    #$def _onTimer(self,timerNum):
    
    def setBounciness(self,bounciness):
        
        self.physNode.setBounciness(bounciness)
    
#    def makeMarkerSphere(self,targetVisNode):
#        
#        for mIdx in range(len(markersUsedInThisRigid)):
#            self.markerServerID_mIdx.append( convertIDtoServerID(rIdx,mIdx) )
#        pass
        
    def projectShadows(self,targetVisNode):
    
        #add avatar as shadow caster
        self.parentRoom.shadowSource.addCaster(self.visNode)

        #Add ground as shadow receiver
        self.parentRoom.shadowSource.addReceiver(targetVisNode)
        
    def setMocapMarker(self,mocap,markerIndex):        
       
       self.markerObject = mocap.returnPointerToMarker(markerIndex)
    
    def setMocapRigidBody(self,mocap,rigidBodyFileString):        
       
       self.rigidBodyFile = mocap.returnPointerToRigid(rigidBodyFileString)
    
    def removeUpdateAction(self):
        
        if( self.updateAction ):
            self.updateAction.remove()
            self.updateAction = 0
            
            if( self.updatingWithMocap):
                self.applyVisToPhysAction.remove()
                self.applyVisToPhysAction = 0
                
    def toggleUpdateWithRigid(self):
        
        self.removeUpdateAction()
                
        if( self.rigidBodyFile ):
            if( self.updatingWithMocap == 0 ):
                print 'Now updating with mocap'
                
                self.updatingWithMocap = True
                self.updateAction = vizact.onupdate(viz.PRIORITY_FIRST_UPDATE, self.applyRigidToVis)
                
#                if( self.physNode ):
#                    self.applyVisToPhysAction = vizact.onupdate(viz.PRIORITY_FIRST_UPDATE, self.applyVisToPhys)
                
            else:
                print 'Not updating with mocap'
                self.updatingWithMocap = False
                self.updateAction.remove()
        else:
            self.updateAction = 0
            print 'No phyNode defined'  

        

    def toggleUpdateWithMarker(self):
        
        self.removeUpdateAction()
            
        if( self.markerNumber > -1  ):
            
            if( self.updatingWithMocap == False ):
                #print 'Now updating with mocap'
                self.updatingWithMocap = True
                self.updateAction = vizact.onupdate(viz.PRIORITY_FIRST_UPDATE, self.applyMarkerToVis)
            else:
                self.updatingWithMocap = False
                self.updateAction.remove()
        else:
            self.updateAction = 0
            print 'No marker defined'
    
    def toggleUpdateWithPhys(self):
        
        self.removeUpdateAction()
        
        # Create a physNode
        if( self.physNode == 0 ):
            self.enablePhysNode();
        
        # Update with physics    
        if( self.updatingWithPhys == False ):
            print 'Now updating with physics'
            self.updatingWithPhys = True
            self.updateAction = vizact.onupdate(viz.PRIORITY_FIRST_UPDATE, self.applyPhysToVis)
            #self.physNode.enableMovement()
        else:
            print 'No longer updating with physics'
            self.updatingWithPhys = False
            self.physNode.disableMovement() #If you don't disble the physics component, it will keep on moving in the physworld
        
   
    def disableUpdateWithPhys(self):
        
        if( self.updateAction and self.updatingWithPhys ):
            
            #self.physNode.disableMovement() #If you don't disble the physics component, it will keep on moving in the physworld
            self.updateAction.remove()
            #self.updateAction.remove()
            self.updateAction = 0
            self.updatingWithPhys = False
    
    def toggleUpdatePhysWithVis(self):
        
        #self.removeUpdateAction()
        
        if( self.updatingPhysWithVis == False ):
            self.updatingPhysWithVis = True
            self.applyVisToPhysAction = vizact.onupdate(viz.PRIORITY_FIRST_UPDATE, self.applyVisToPhys)
            
        else:
            self.updatingPhysWithVis = False
            self.updateAction.remove()
        
        
    def applyMarkerToVis(self):
        
        if( self.markerNumber > -1 and self.mocapDevice ):
            
            pos_XYZ = self.mocapDevice.getMarkerPosition(self.markerNumber)
            
            if( pos_XYZ ):
                self.visNode.visible(viz.ON)
                pos_XYZ = [pos_XYZ[0], pos_XYZ[1], pos_XYZ[2]]
                #print 'Marker pos: ' + str(pos)
                self.visNode.setPosition(pos_XYZ)
            else:
                # Marker not seen
                self.visNode.visible(viz.OFF)
                return
        else:
            #print 'visEnv.updateWithMocap: No mocap, or no marker number set!'
            return
             
     
    def applyVisToMarker(self):
        
        #print str(viz.getFrameNumber())
        #self.physNode.setPosition(self.visNode.getPosition())
        #self.physNode.setQuaternion(self.visNode.getQuat())
        
        pos_XYZ = self.visNode.getPosition()
        pos_XYZ[2] = -pos_XYZ[2]
        self.physNode.setPosition(pos_XYZ)
    
    def applyVisToPhys(self):
        
        transformMat = self.visNode.getMatrix()
        self.physNode.updateWithTransform(transformMat)
    
    def applyPhysToVis(self):
        
        self.visNode.setPosition(self.physNode.geom.getPosition())
        self.visNode.setQuat(self.physNode.getQuaternion())
    
    def applyRigidToVis(self):
        
        if( self.rigidBodyFile ):
           
            transformViz = self.rigidBodyFile.transformViz 
            #print transformViz
            #self.visNode.setMatrix(transformViz)
            
            if(transformViz ):
                self.visNode.setMatrix(transformViz)
                
            else:
                return
                print 'visObj.updateWithMocap(): PS server does not have valid rigid body data.  Its likely that it was not detected.'
        else:
            print 'visEnv.updateWithMocap: No rigid body set!'
            return
        
    def removeUpdateAction(self):
        if( self.updateAction ):
            self.updateAction.remove()
            self.updateAction = 0
            
            self.updatingPhysWithVis = False
            self.updatingWithMocap = False
            self.updatingWithPhys = False
        
class mocapMarkerSphere(visObj):
    def __init__(self,mocap,room,markerNum):
        #super(visObj,self).__init__(room,'sphere',.04,[0,0,0],[.5,0,0],1)

        position = [0,0,0]
        shape = 'sphere'
        color=[.5,0,0]
        size = [.015]
        
        visObj.__init__(self,room,shape,size,position,color)
        
        #self.physNode.enableMovement()
        self.markerNumber = markerNum
        self.mocapDevice = mocap
        self.toggleUpdateWithMarker()

def drawMarkerSpheres(room,mocap):
    
     ##  Create mocap marker spheres - 1 per LED
    markerVisObjList_idx = []
    
    for idx in range(0,29):
            
            print 'visEnv.mocapMarkerSphere: Drawing marker ' + str(idx)
            markerVisObjList_idx.append(mocapMarkerSphere(mocap,room,idx))

if __name__ == "__main__":

    import vizact
    useConfig = True
    
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
                
        # Draw markers where the spehres are
        drawMarkerSpheres(room,config.mocap)
        
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
        
#    #Add a world axis with X,Y,Z labels
#    world_axes = vizshape.addAxes()
#    X = viz.addText3D('X',pos=[1.1,0,0],color=viz.RED,scale=[0.3,0.3,0.3],parent=world_axes)
#    Y = viz.addText3D('Y',pos=[0,1.1,0],color=viz.GREEN,scale=[0.3,0.3,0.3],align=viz.ALIGN_CENTER_BASE,parent=world_axes)
#    Z = viz.addText3D('Z',pos=[0,0,1.1],color=viz.BLUE,scale=[0.3,0.3,0.3],align=viz.ALIGN_CENTER_BASE,parent=world_axes)



