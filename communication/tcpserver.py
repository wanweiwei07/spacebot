import SocketServer

class EchoHandler(SocketServer.BaseRequestHandler):
    robot = None

    def __init__(self, request, client_address, server):
        SocketServer.BaseRequestHandler.__init__(self, request, client_address, server)

    def handle(self):
        while True:
            try:
                request_msg = self.request.recv(2048)
                if request_msg:
                    print "Message from: %s" % str(self.client_address)
                    print request_msg
                else:
                    continue
                if request_msg == "connect":
                    self.initializerobot()
                elif request_msg == "disconnect":
                    self.finalizerobot()
                    return
                elif request_msg[:10] == "movejoints":
                    # data structure: 7 for joints, 2 for two hands
                    # the values are separated by ","
                    # the char at 11 is ","
                    param_strlist = request_msg[11:].split(",")
                    param_list = map(float, param_strlist)
                    self.movejoints(param_list)
            except Exception, ex:
                print 'e', ex,

    def initializerobot(self):
        self.robot.goinitpose()
        print "Sending feedback..."
        self.request.send("connected!")

    def finalizerobot(self):
        self.robot.goinitpose()
        print "Sending feedback..."
        self.request.send("disconnected!")

    def movejoints(self, param_list):
        self.robot.movearmfk(param_list)
        print "Sending feedback..."
        self.request.send("jointsmoved!")


class ThreadedTCPServer(SocketServer.ThreadingMixIn, SocketServer.TCPServer):
    def serve_forever(self, robot):
        self.RequestHandlerClass.robot = robot
        SocketServer.TCPServer.serve_forever(self)


def simple_tcp_server(ip, port, robot):
    tcp_server = ThreadedTCPServer((ip, port), RequestHandlerClass=EchoHandler)
    try:
        print "server start"
        tcp_server.serve_forever(robot)
    except KeyboardInterrupt, err:
        print "server close"
        tcp_server.server_close()