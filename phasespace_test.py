# python imports
import logging
import sys

# vizard imports
import nvis
import viz
import vizconfig

# local imports
import phasespace


def main():
    viz.setOption('viz.fullscreen.monitor', 1)
    viz.setOption('viz.window.width', 2 * 640)
    viz.setOption('viz.window.height', 480)
    viz.setMultiSample(4)
    viz.MainWindow.clip(0.01, 500)

    vizconfig.register(nvis.nvisorSX111())

    viz.go(viz.FULLSCREEN)

    piazza = viz.addChild('piazza.osgb')

    mocap = phasespace.Mocap('192.168.1.230')

    head = mocap.track_rigid('Resources/hmd-nvis.rb', center_markers=(0, 5))
    head.link_pose(viz.MainView)

    glove = mocap.track_points([8, 9, 10])

    paddle = mocap.track_rigid({
      17:(0, 0, 1),
      19:(0, 0, 0),
      20:(1, 0, 0),
      22:(0, 1, 0),
    })

    mocap.start_thread()
    #mocap.start_timer()

    def log_mocap(timer_id):
        print 'glove#9 pos {0[1]:.2f} {0[1]:.2f} {0[2]:.2f}'.format(*glove.get_marker(9))
        print 'head    pos {0[0]:.2f} {0[1]:.2f} {0[2]:.2f}, quat {1[0]:.3f} {1[1]:.3f} {1[2]:.3f} {1[3]:.3f}'.format(*head.get_pose())
        print 'paddle  pos {0[0]:.2f} {0[1]:.2f} {0[2]:.2f}, quat {1[0]:.3f} {1[1]:.3f} {1[2]:.3f} {1[3]:.3f}'.format(*paddle.get_pose())

    viz.callback(viz.TIMER_EVENT, log_mocap)
    viz.starttimer(0, 1, viz.FOREVER)

    def keydown(*args):
        head.reset()
        paddle.reset()

    viz.callback(viz.KEYDOWN_EVENT, keydown)


if __name__ == '__main__':
    logging.basicConfig(
        stream=sys.stdout,
        level=logging.DEBUG,
        format='%(levelname)1s %(asctime)s %(message)s',
    )
    main()
