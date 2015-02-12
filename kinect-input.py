"""

Crazyflie controlled by Kinect via OSC from openFrameworks.

"""

import time, sys
from threading import Thread

#FIXME: Has to be launched from within the example folder
sys.path.append("../lib")
import cflib
from cflib.crazyflie import Crazyflie

import logging
logging.basicConfig(level=logging.ERROR)

import OSC

#-------------------------------------------------------------------------

class ReceiveOSC:

    def __init__(self):
        # tupple with ip, port
        receive_address = 'localhost', 8000

        # OSC Server
        self.s = OSC.OSCServer(receive_address) # basic
        ##s = OSC.ThreadingOSCServer(receive_address) # threading
        ##s = OSC.ForkingOSCServer(receive_address) # forking

        # this registers a 'default' handler (for unmatched messages)
        self.s.addDefaultHandlers()

    def receive(self):
        #instantiate Input class
        inp = Input(available[0][0])

        # add a function call that gets data sent via OSC
        self.s.addMsgHandler("/drone/position", inp.drive_handler)

        # check handlers added
        print "\nRegistered Callback-functions are :"
        for addr in self.s.getOSCAddressSpace():
            print addr

        # Start OSCServer
        print "\nStarting OSCServer. Use ctrl-C to quit."
        th = Thread( target = self.s.serve_forever )
        th.start()

        try :
            while 1 :
                time.sleep(0)

        except KeyboardInterrupt :
            # disconnect radio
            inp._cf.close_link()

            # close server
            print "\nClosing OSCServer."
            self.s.close()
            
            # close thread
            print "Waiting for Server-thread to finish"
            th.join()

            print "Done"

    # define a message-handler function for the server to call
    def print_handler(self, addr, tags, stuff, source):
        print "---"
        print "received new osc msg from %s" % OSC.getUrlStr(source)
        print "with addr : %s" % addr
        print "typetags %s" % tags
        print "data %s" % stuff

#-------------------------------------------------------------------------

class Input:

    def __init__(self, link_uri):
        self._cf = Crazyflie()

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.open_link(link_uri)

        print "Connecting to %s" % link_uri

        # input values continuously modified
        self.thrust = 10000
        self.roll = 0

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        #Thread(target=self._ramp_motors).start()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the speficied address)"""
        print "Connection to %s failed: %s" % (link_uri, msg)

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print "Connection to %s lost: %s" % (link_uri, msg)

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print "Disconnected from %s" % link_uri

    # handler called by OSC server
    def drive_handler(self, addr, tags, stuff, source):
        if (stuff[0] == 9):
            self.thrust = 0
            self.roll = 0
            pitch = 0
            yawrate = 0
        else:
            '''# decrease roll
            if (stuff[0] == 1):
                pitch = 0
                self.roll += 1
                yawrate = 0
            # increase roll
            elif (stuff[0] == 2):
                pitch = 0
                self.roll -= 1
                yawrate = 0
            # maintin current roll
            else:
                pitch = 0
                yawrate = 0'''

            # increase thrust
            if (stuff[1] == 1):
                self.thrust -= 200
                pitch = 0
                yawrate = 0
            # decrease thrust
            elif (stuff[1] == 2):
                self.thrust -= 70
                pitch = 0
                yawrate = 0
            # decrease thrust
            elif (stuff[1] == 3):
                self.thrust += 400
                pitch = 0
                yawrate = 0
            # decrease thrust
            elif (stuff[1] == 4):
                self.thrust += 10
                pitch = 0
                yawrate = 0
            # maintin current thrust
            else:
                pitch = 0
                yawrate = 0

            if (self.thrust > 50000):
                self.thrust = 50000

        # set input values
        self._cf.commander.send_setpoint(self.roll, pitch, yawrate, self.thrust)

#-------------------------------------------------------------------------

if __name__ == '__main__':

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Scan for Crazyflies and use the first one found
    print "\nScanning interfaces for Crazyflies..."    
    available = cflib.crtp.scan_interfaces()
        
    # list available connection
    print "Crazyflies found:"
    for i in available:
        print i[0]

    # check radio connection
    if len(available) > 0:
        osc = ReceiveOSC()
        osc.receive()
    else:
        print "No Crazyflies found, cannot run example"