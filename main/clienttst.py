import socket

class SpaceSocket():
    """

        author: weiwei
    """

    def __init__(self, ip, port):
        ip = ip
        port = port
        self.__buffersize = 2048
        self.__s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__s.connect((ip, port))
        self.__connected = False

    @property
    def consts(self):
        # read-only property
        return self.__connected

    def initialize(self):
        if not  self.__connected:
            self.__connected = self.__send('connect', 'connected!')
            if self.__connected:
                print "Robot is initialized!"
            else:
                print "Failed to initialize robot!"
        else:
            print "Robot already initialized!"

    def gooff(self):
        if self.__connected:
            self.__connected = self.__send('disconnect', 'disconnected!')
            print "Robot is back to off!"
        else:
            print "No initialized robot found!"

    def movejnts(self, armjnts):
        """
        move all joints of the robot

        :param armjnts 1by9
        :return: bool

        author: weiwei
        """

        lj0 = str(armjnts[0])
        lj1 = str(armjnts[1])
        lj2 = str(armjnts[2])
        lj3 = str(armjnts[3])
        lj4 = str(armjnts[4])
        lj5 = str(armjnts[5])
        lj6 = str(armjnts[6])
        lj7 = str(armjnts[7])
        lj8 = str(armjnts[8])
        sep = ','

        message = 'movejoints' + sep + lj0 + sep + lj1 + sep + lj2 + sep + \
                  lj3 + sep + lj4+ sep + lj5 + sep + lj6 + sep + lj7 + sep + lj8
        self.__send(message, 'jointsmoved!')

    def __send(self, message, confirmmessage):
        """
        confirm pairs:
        connect connected!
        disconnect disconnected!

        :param message:
        :param confirmmessage:
        :return:
        author: weiwei
        """

        self.__s.send(message)
        data = self.__s.recv(self.__buffersize)
        if data == confirmmessage:
            return True
        else:
            return False

    def __del__(self):
        self.__s.close()

if __name__ == '__main__':
    ip = '127.0.0.1'
    port = 50307
    ss = SpaceSocket(ip, port)

    import time
    import numpy as np
    tic = time.clock()
    ss.initialize()
    toc = time.clock()
    print toc-tic
    angle = 0
    while True:
        time.sleep(.1)
        if (angle/20) % 2 == 0:
            ss.movejnts(np.array([angle,0,angle,0,angle,0,0,0,1]))
        if (angle/20) % 2 == 1:
            ss.movejnts(np.array([angle,0,angle,0,angle,0,0,1,0]))
        angle = angle +1