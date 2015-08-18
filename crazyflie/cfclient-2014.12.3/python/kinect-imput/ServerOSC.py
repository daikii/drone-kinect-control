"""

Crazyflie controlled by Kinect via OSC from openFrameworks.

OSC server.

"""

import OSC


class ServerOSC:

    # tupple with ip, port
    receive_address = 'localhost', 8000
    
    # OSC Server. three different types of server.
    s = OSC.OSCServer(receive_address) # basic
    ##s = OSC.ThreadingOSCServer(receive_address) # threading
    ##s = OSC.ForkingOSCServer(receive_address) # forking

    def __init__(self, link_uri):
        # this registers a 'default' handler (for unmatched messages), 
        # an /'error' handler, an '/info' handler.
        # And, if the client supports it, a '/subscribe' & '/unsubscribe' handler
        s.addDefaultHandlers()

    # define a message-handler function for the server to call.
    def printing_handler(addr, tags, stuff, source):
        print "---"
        print "received new osc msg from %s" % OSC.getUrlStr(source)
        print "with addr : %s" % addr
        print "typetags %s" % tags
        print "data %s" % stuff
        print "---"

    def receive():
        # add a function call that gets data sent via OSC
        s.addMsgHandler("/mouse/position", printing_handler)

        # check handlers added
        print "Registered Callback-functions are :"
        for addr in s.getOSCAddressSpace():
            print addr

        # Start OSCServer
        print "\nStarting OSCServer. Use ctrl-C to quit."
        st = threading.Thread( target = s.serve_forever )
        st.start()

        try :
            while 1 :
                time.sleep(1)

        except KeyboardInterrupt :
            print "\nClosing OSCServer."
            s.close()
            print "Waiting for Server-thread to finish"
            st.join() ##!!!
            print "Done"