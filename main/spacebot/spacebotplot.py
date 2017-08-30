import os

import numpy as np
from panda3d.core import *

import pandactrl.pandageom as pg

def plotstick(pandanp, spacebot, rgba = [.5,0,0,1]):
    """
    plot the stick model of the spacebot robot in panda3d

    :param pandanp: a panda3d nodepath
    :param spacebot:
    :return: null

    author: weiwei
    """

    # plot base
    pg.plotDumbbell(pandanp, spos=Vec3(0,0,0), epos=spacebot.arm[0]['linkpos'],
                    thickness=20, rgba=rgba)
    # plot arm
    i = 0
    while i != -1:
        pg.plotDumbbell(pandanp, spos=spacebot.arm[i]['linkpos'], epos=spacebot.arm[i]['linkend'],
                               thickness=20, rgba=rgba)
        i = spacebot.arm[i]['child']

def genmnp(spacebot):
    """
    generate a panda3d nodepath for the spacebot
    mnp indicates this function generates a mesh nodepath

    :param spacebot:
    :return: a nodepath which is ready to be plotted using plotmesh

    author: weiwei
    """

    spacebotmnp = NodePath("spacebotmnp")

    this_dir, this_filename = os.path.split(__file__)

    # chest0-2, head1 (neck is not plotted)
    shoulderr_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "models/egg", "shoulder_r.egg"))
    shouldery_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "models/egg", "shoulder_y.egg"))
    shoulderp_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "models/egg", "shoulder_p.egg"))
    elbow1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "models/egg", "elbow_1.egg"))
    elbow2_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "models/egg", "elbow_2.egg"))
    wristp_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "models/egg", "wrist_p.egg"))
    wristy_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "models/egg", "wrist_y.egg"))
    wristr_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "models/egg", "wrist_r.egg"))

    shoulderr_model  = loader.loadModel(shoulderr_filepath)
    shouldery_model  = loader.loadModel(shouldery_filepath)
    shoulderp_model = loader.loadModel(shoulderp_filepath)
    elbow1_model = loader.loadModel(elbow1_filepath)
    elbow2_model = loader.loadModel(elbow2_filepath)
    wristp_model = loader.loadModel(wristp_filepath)
    wristy_model = loader.loadModel(wristy_filepath)
    wristr_model = loader.loadModel(wristr_filepath)

    shoulderr_nodepath = NodePath("shoulderr")
    shouldery_nodepath = NodePath("shouldery")
    shoulderp_nodepath = NodePath("shoulderp")
    elbow1_nodepath = NodePath("elbow1")
    elbow2_nodepath = NodePath("elbow2")
    wristp_nodepath = NodePath("wristp")
    wristy_nodepath = NodePath("wristy")
    wristr_nodepath = NodePath("wristr")

    # 0
    shoulderr_model.instanceTo(shoulderr_nodepath)
    shoulderr_nodepath.setMat(Mat4.identMat())
    shoulderr_nodepath.setColor(.5,.5,1,1)
    shoulderr_nodepath.reparentTo(spacebotmnp)
    # 1
    shouldery_model.instanceTo(shouldery_nodepath)
    spacebotshouldery_rotmat = pg.cvtMat4(spacebot.arm[0]['rotmat'], spacebot.arm[0]['linkpos'])
    shouldery_nodepath.setMat(spacebotshouldery_rotmat)
    shouldery_nodepath.setColor(.5,1,.5,1)
    shouldery_nodepath.reparentTo(spacebotmnp)
    # 2
    shoulderp_model.instanceTo(shoulderp_nodepath)
    spacebotshoulderp_rotmat = pg.cvtMat4(spacebot.arm[1]['rotmat'], spacebot.arm[1]['linkpos'])
    shoulderp_nodepath.setMat(spacebotshoulderp_rotmat)
    shoulderp_nodepath.setColor(.5,.5,.5,1)
    shoulderp_nodepath.reparentTo(spacebotmnp)
    # 3
    elbow1_model.instanceTo(elbow1_nodepath)
    spacebotelbow1_rotmat = pg.cvtMat4(spacebot.arm[2]['rotmat'], spacebot.arm[2]['linkpos'])
    elbow1_nodepath.setMat(spacebotelbow1_rotmat)
    elbow1_nodepath.setColor(.5,.5,.5,1)
    elbow1_nodepath.reparentTo(spacebotmnp)
    # 4
    elbow2_model.instanceTo(elbow2_nodepath)
    spacebotelbow2_rotmat = pg.cvtMat4(spacebot.arm[3]['rotmat'], spacebot.arm[3]['linkpos'])
    elbow2_nodepath.setMat(spacebotelbow2_rotmat)
    elbow2_nodepath.setColor(.5,.7,.3,1)
    elbow2_nodepath.reparentTo(spacebotmnp)
    # 5
    wristp_model.instanceTo(wristp_nodepath)
    spacebotwristp_rotmat = pg.cvtMat4(spacebot.arm[4]['rotmat'], spacebot.arm[4]['linkpos'])
    wristp_nodepath.setMat(spacebotwristp_rotmat)
    wristp_nodepath.setColor(.5,.5,.5,1)
    wristp_nodepath.reparentTo(spacebotmnp)
    # 6
    wristy_model.instanceTo(wristy_nodepath)
    spacebotwristy_rotmat = pg.cvtMat4(spacebot.arm[5]['rotmat'], spacebot.arm[5]['linkpos'])
    wristy_nodepath.setMat(spacebotwristy_rotmat)
    wristy_nodepath.setColor(.5,1,.5,1)
    wristy_nodepath.reparentTo(spacebotmnp)
    # 7
    wristr_model.instanceTo(wristr_nodepath)
    spacebotwristr_rotmat = pg.cvtMat4(spacebot.arm[6]['rotmat'], spacebot.arm[6]['linkpos'])
    wristr_nodepath.setMat(spacebotwristr_rotmat)
    wristr_nodepath.setColor(.5,.5,1,1)
    wristr_nodepath.reparentTo(spacebotmnp)

    return spacebotmnp