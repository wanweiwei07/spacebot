import numpy as np
import exceptions as ep
import utils.robotmath as rm
import pandactrl.pandactrl as pandactrl
import pandactrl.pandageom as pg

class Spacebot():
    def __init__(self):
        """
        initiate the robot, handside indicates which is the end-effector
        :param hndside:
        """

        self.__name = 'spacebot'
        # initjnts has 9 elements where the first seven are for the seven joints from left to right
        # the last two are the sates of the end-effectors
        self.__initjnts = np.array([0,0,0,0,0,0,0,0,0]);
        self.__arm = self.__initlj()
        self.__targetjoints = [0,1,2,3,4,5,6]
        self.__base = self.__arm[0]
        self.__ee = self.__arm[6]
        self.goinitpose()

    @property
    def name(self):
        # read-only property
        return self.__name

    @property
    def initjnts(self):
        # read-only property
        return self.__initjnts

    @property
    def arm(self):
        # read-only property
        return self.__arm

    @property
    def base(self):
        # read-only property
        return self.__base

    @property
    def targetjoints(self):
        # read-only property
        return self.__targetjoints

    def getlinkendframe(self, jointid):
        """
        get the local frame at the end of a link in Mat4 format

        author: weiwei
        """

        return pg.cvtMat4(self.__arm[jointid]['rotmat'], self.__arm[jointid]['linkend'])

    def getlinkstartframe(self, jointid):
        """
        get the local frame at the start of a link in Mat4 format

        author: weiwei
        """

        return pg.cvtMat4(self.__arm[jointid]['rotmat'], self.__arm[jointid]['linkpos'])


    def __initlj(self):
        """
        Init arm links and joints

        ## note
        x is facing forward, y is facing left, z is facing upward
        each element of lj is a dictionary
        lj[i]['linkpos'] indicates the position of a link
        lj[i]['linkvec'] indicates the vector of a link that points from start to end
        lj[i]['rotmat'] indicates the frame of this link
        lj[i]['rotax'] indicates the rotation axis of the link
        lj[i]['rotangle'] indicates the rotation angle of the link around the rotax
        lj[i]['linkend'] indicates the end position of the link (passively computed)

        ## more note:
        lj[1]['linkpos'] is the position of the first joint
        lj[i]['linkend'] is the same as lj[i+1]['linkpos'],
        I am keeping this value for the eef (end-effector)

        ## even more note:
        joint is attached to the linkpos of a link
        for the first link, the joint is fixed and the rotax = 0,0,0

        :return:
        lj:
            a list of dictionaries with each dictionary holding name, mother, child,
        linkpos (link position), linkvec (link vector), rotmat (orientation), rotax (rotation axis of a joint),
        rotangle (rotation angle of the joint around rotax)
        linkend (the end position of a link) is computed using rotmat, rotax, linkpos, and linklen
        lj means link and joint, the joint attached to the link is at the linkend

        author: weiwei
        """

        lj = [dict() for i in range(7)]
        rngsafemargin = 0

        # the 0th link and joint
        lj[0]['name'] = 'link0'
        lj[0]['mother'] = -1
        lj[0]['child'] = 1
        lj[0]['linkpos'] = np.array([0,1145,0])
        lj[0]['linkvec'] = np.array([0, 0, 676])
        lj[0]['rotax'] = np.array([0,1,0])
        lj[0]['rotangle'] = 0
        lj[0]['rotmat'] = rm.rodrigues(lj[0]['rotax'], lj[0]['rotangle'])
        lj[0]['linkend'] = np.dot(lj[0]['rotmat'], lj[0]['linkvec'])+lj[0]['linkpos']

        # the 1st joint and link
        lj[1]['name'] = 'link1'
        lj[1]['mother'] = 0
        lj[1]['child'] = 2
        lj[1]['linkpos'] = lj[0]['linkend']
        lj[1]['linkvec'] = np.array([-496,0,0])
        lj[1]['rotax'] = np.array([0,0,1])
        lj[1]['rotangle'] = 0
        lj[1]['rotmat'] = np.dot(lj[0]['rotmat'], rm.rodrigues(lj[1]['rotax'], lj[1]['rotangle']))
        lj[1]['linkend'] = np.dot(lj[1]['rotmat'], lj[1]['linkvec'])+lj[1]['linkpos']
        lj[1]['rngmin'] = -(360-rngsafemargin)
        lj[1]['rngmax'] = +(360-rngsafemargin)

        # the 2nd joint and link
        lj[2]['name'] = 'link2'
        lj[2]['mother'] = 1
        lj[2]['child'] = 3
        lj[2]['linkpos'] = lj[1]['linkend']
        lj[2]['linkvec'] = np.array([-580.696,0,4040])
        lj[2]['rotax'] = np.array([-1,0,0])
        lj[2]['rotangle'] = 0
        lj[2]['rotmat'] = np.dot(lj[1]['rotmat'], rm.rodrigues(lj[2]['rotax'], lj[2]['rotangle']))
        lj[2]['linkend'] = np.dot(lj[2]['rotmat'], lj[2]['linkvec'])+lj[2]['linkpos']
        lj[2]['rngmin'] = -(360-rngsafemargin)
        lj[2]['rngmax'] = +(360-rngsafemargin)

        # the 3rd joint and link
        lj[3]['name'] = 'link3'
        lj[3]['mother'] = 2
        lj[3]['child'] = 4
        lj[3]['linkpos'] = lj[2]['linkend']
        lj[3]['linkvec'] = np.array([0,0,-4040])
        lj[3]['rotax'] = np.array([-1,0,0])
        lj[3]['rotangle'] = 0
        lj[3]['rotmat'] = np.dot(lj[2]['rotmat'], rm.rodrigues(lj[3]['rotax'], lj[3]['rotangle']))
        lj[3]['linkend'] = np.dot(lj[3]['rotmat'], lj[3]['linkvec'])+lj[3]['linkpos']
        lj[3]['rngmin'] = -(360-rngsafemargin)
        lj[3]['rngmax'] = +(360-rngsafemargin)

        # the 4th joint and link
        lj[4]['name'] = 'link4'
        lj[4]['mother'] = 3
        lj[4]['child'] = 5
        lj[4]['linkpos'] = lj[3]['linkend']
        lj[4]['linkvec'] = np.array([-496,0,0])
        lj[4]['rotax'] = np.array([-1,0,0])
        lj[4]['rotangle'] = 0
        lj[4]['rotmat'] = np.dot(lj[3]['rotmat'], rm.rodrigues(lj[4]['rotax'], lj[4]['rotangle']))
        lj[4]['linkend'] = np.dot(lj[4]['rotmat'], lj[4]['linkvec'])+lj[4]['linkpos']
        lj[4]['rngmin'] = -(360-rngsafemargin)
        lj[4]['rngmax'] = +(360-rngsafemargin)

        # the 5th joint and link
        lj[5]['name'] = 'link5'
        lj[5]['mother'] = 4
        lj[5]['child'] = 6
        lj[5]['linkpos'] = lj[4]['linkend']
        lj[5]['linkvec'] = np.array([0,0,-676])
        lj[5]['rotax'] = np.array([0,0,-1])
        lj[5]['rotangle'] = 0
        lj[5]['rotmat'] = np.dot(lj[4]['rotmat'], rm.rodrigues(lj[5]['rotax'], lj[5]['rotangle']))
        lj[5]['linkend'] = np.dot(lj[5]['rotmat'], lj[5]['linkvec'])+lj[5]['linkpos']
        lj[5]['rngmin'] = -(360-rngsafemargin)
        lj[5]['rngmax'] = +(360-rngsafemargin)

        # the 6th joint and link
        lj[6]['name'] = 'link6'
        lj[6]['mother'] = 5
        lj[6]['child'] = -1
        lj[6]['linkpos'] = lj[5]['linkend']
        lj[6]['linkvec'] = np.array([0,-1145,0])
        lj[6]['rotax'] = np.array([0,-1,0])
        lj[6]['rotangle'] = 0
        lj[6]['rotmat'] = np.dot(lj[5]['rotmat'], rm.rodrigues(lj[6]['rotax'], lj[6]['rotangle']))
        lj[6]['linkend'] = np.dot(lj[6]['rotmat'], lj[6]['linkvec'])+lj[6]['linkpos']
        lj[6]['rngmin'] = -(360-rngsafemargin)
        lj[6]['rngmax'] = +(360-rngsafemargin)

        return lj

    def movearmfk(self, armjnts):
        """
        move the joints of lj specified by targetjoints using forward kinematics, waist is not included

        :param armjnts: a 1-by-6 ndarray where each element indicates the angle of a joint (in degree)
        :return: null

        author: weiwei
        """

        counter = 0
        for i in self.__targetjoints:
            self.arm[i]['rotangle'] = armjnts[counter]
            counter += 1
        self.__updatefk()

    def goinitpose(self):
        """
        move the robot to initial pose

        :return: null

        author: weiwei
        """

        self.movearmfk(self.initjnts)

    def gettcp(self):
        """
        get the tcppos, and tcprot of the speficied armid

        :return: [tcppos, tcprot] in nparray
        """

        tcppos = self.arm[-1]['linkend']
        tcprot = self.arm[-1]['rotmat']
        return [tcppos, tcprot]

    def getarmjnts(self):
        """
        get the target joints of the specified armid

        :return: armjnts: a 1-by-6 numpy ndarray

        author: weiwei
        """

        armjnts = np.zeros(len(self.__targetjoints))
        counter = 0
        for i in self.__targetjoints:
            armjnts[counter] = self.arm[i]['rotangle']
            counter += 1

        return armjnts

    def chkrng(self):
        """
        check if the given armjnts is inside the oeprating range of the arm

        :param armjnts: a 1-by-x numpy ndarray indicating the targejoints of a manipulator
        :return: True or False indicating inside the range or not

        author: weiwei
        """

        counter = 0
        for i in self.__targetjoints:
            if self.arm[counter] < self.arm[i]["rngmin"] or self.arm[counter] > self.arm[i]["rngmax"]:
                print "Joint "+ str(i) + " of the " + armid + " arm is out of range"
                print "Angle is " + str(self.arm[counter])
                print "Range is (" + str(self.arm[i]["rngmin"]) + ", " + str(self.arm[i]["rngmax"]) + ")"
                return False
            counter += 1

        return True

    def __updatefk(self):
        """
        Update the structure of arm links and joints
        Note that this function should not be called explicitly
        It is automatically invoked by functions like movexxx

        :param armlj: the robot structure

        author: weiwei
        """

        self.arm[0]['rotmat'] = rm.rodrigues(self.arm[0]['rotax'], self.arm[0]['rotangle'])
        self.arm[0]['linkend'] = np.dot(self.arm[0]['rotmat'], self.arm[0]['linkvec'])+self.arm[0]['linkpos']
        i = 1
        while i != -1:
            j = self.arm[i]['mother']
            self.arm[i]['linkpos'] = self.arm[j]['linkend']
            self.arm[i]['rotmat'] = np.dot(self.arm[j]['rotmat'], rm.rodrigues(self.arm[i]['rotax'], self.arm[i]['rotangle']))
            self.arm[i]['linkend'] = np.dot(self.arm[i]['rotmat'], self.arm[i]['linkvec']) + self.arm[i]['linkpos']
            i = self.arm[i]['child']
        return self.arm

if __name__=="__main__":

    # show in panda3d
    import spacebotplot

    base = pandactrl.World(camp = [10000,10000,10000], lookatp = [0,0,700])

    sbot = Spacebot()
    sbot.goinitpose()
    # spacebotplot.plotstick(base.render, sbot)

    armjnts = np.array([30, 30, 30, 30, 30, 30, 30, 0, 0])
    sbot.movearmfk(armjnts)
    spacebotmnp = spacebotplot.genmnp(sbot)
    spacebotmnp.reparentTo(base.render)

    base.run()