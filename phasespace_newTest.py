# python imports
import logging
import sys

# vizard imports
import nvis
import viz
import vizconfig

# local imports
import phasespaceNew as phasespace


def main():
    
    viz.vsync(viz.ON)
    viz.window.setFullscreenMonitor([2]) 
    viz.setMultiSample(4)
    viz.MainWindow.clip(0.01 ,200)

    viz.go(viz.FULLSCREEN)

    environment = viz.addChild('piazza.osgb')
    environment.setPosition(2.75,0,-.75)

    mocap = phasespace.Phasespace()

    head = mocap.get_rigidTracker('hmd')

    # DOES NOT ACCOUNT FOR RIGID BODY OFFSET DURING RESET
        
    #mocap.track_rigid('Resources/hmd-nvisMount.rb', center_markers=(1,2))
    head.link_pose(viz.MainView)

    #glove = mocap.track_points([8, 9, 10])

    paddle = mocap.track_rigid({
      17:(0, 0, 1),
      19:(0, 0, 0),
      20:(1, 0, 0),
      22:(0, 1, 0),
    })

    mocap.start_thread()
    #mocap.start_timer()

    def log_mocap(timer_id):
        #print 'glove#9 pos {0[1]:.2f} {0[1]:.2f} {0[2]:.2f}'.format(*glove.get_marker(9))
        print 'head    pos {0[0]:.2f} {0[1]:.2f} {0[2]:.2f}, quat {1[0]:.3f} {1[1]:.3f} {1[2]:.3f} {1[3]:.3f}'.format(*head.get_pose())
        print 'main    ' + str(viz.MainView.getPosition())
        #print 'paddle  pos {0[0]:.2f} {0[1]:.2f} {0[2]:.2f}, quat {1[0]:.3f} {1[1]:.3f} {1[2]:.3f} {1[3]:.3f}'.format(*paddle.get_pose())

    viz.callback(viz.TIMER_EVENT, log_mocap)
    viz.starttimer(0, 1, viz.FOREVER)

    def keydown(key):
        
        if( key == 'h' ) :
            head.reset()
        if( key == 'H' ) :
            head.save()            
        elif( key == '1' ):

            print 'Marker Pos: ' + str(mocap.get_MarkerPos(1))
        

    viz.callback(viz.KEYDOWN_EVENT, keydown)


if __name__ == '__main__':
    logging.basicConfig(
        stream=sys.stdout,
        level=logging.DEBUG,
        format='%(levelname)1s %(asctime)s %(message)s',
    )
    main()
