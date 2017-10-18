# show in panda3d
from spacebot import spacebotplot
from spacebot import spacebot
from direct.showbase.ShowBase import ShowBase
from panda3d.core import *
from panda3d.bullet import BulletSphereShape
import pandactrl.pandageom as pandageom
import pandactrl.pandactrl as pandactrl
import numpy as np
import os
from communication import tcpserver as ts

import thread

base = pandactrl.World(camp=[10000, 10000, 10000], lookatp=[0, 0, 700])

sbot = spacebot.Spacebot()
sbot.goinitpose()

armjnts = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])
sbot.movearmfk(armjnts)
spacebotmnp = spacebotplot.genmnp(sbot)
spacebotmnp.reparentTo(base.render)

this_dir, this_filename = os.path.split(__file__)
bottom_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "spacebot/models/egg", "bottom.egg"))
bottom_model = loader.loadModel(bottom_filepath)
bottom_node = NodePath("bottom")
bottom_model.instanceTo(bottom_node)
bottom_node.reparentTo(base.render)

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