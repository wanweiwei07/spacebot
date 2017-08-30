# show in panda3d
from spacebot import spacebotplot
from spacebot import spacebot
from direct.showbase.ShowBase import ShowBase
from panda3d.core import *
from panda3d.bullet import BulletSphereShape
import pandactrl.pandageom as pandageom
import pandactrl.pandactrl as pandactrl
import numpy as np
from communication import tcpserver as ts

import thread

base = pandactrl.World(camp=[10000, 10000, 10000], lookatp=[0, 0, 700])

sbot = spacebot.Spacebot()
sbot.goinitpose()

armjnts = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])
sbot.movearmfk(armjnts)
spacebotmnp = spacebotplot.genmnp(sbot)
spacebotmnp.reparentTo(base.render)

try:
    thread.start_new_thread(ts.simple_tcp_server, ("127.0.0.1", 50307, sbot))
except Exception as e:
    print e

def updateworld(sbmnp, task):
    sbmnp[0].removeNode()
    sbmnp[0] = spacebotplot.genmnp(sbot)
    sbmnp[0].reparentTo(base.render)
    return task.cont

base.taskMgr.add(updateworld, "updateworld", extraArgs=[[spacebotmnp]], appendTask=True)

base.run()