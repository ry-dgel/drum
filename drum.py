# Original code by Mark Orchard-Webb,
# Tweaked by:
#            Robert Turner, 2019.12.09
#            Rigel Zifkin, 2020.08.09

#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!#
# The functions in this module should not be directly  #
# accessed by students. Please use the safe drum code. #
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!#

import serial
import threading
import queue
import numpy
import time

class Vibrating_Plate:
    D_NONE = 0
    D_INFO = 1
    D_OPEN = 2
    D_ALL = 3

    def __init__(self,debug=D_ALL):
        self.handle = False
        self.debug = debug
        
        # Attemp to open device by trying all COM ports
        for i in range(2,10):
            try:
                device = "COM%d" % (i)
                if self.D_OPEN & self.debug: print("Attempting '%s'"%(device))
                self.handle = serial.Serial(device,115200) # 115200 = Data Rate
            except :
                print("except")
                continue
            break
        if not self.handle:
            # Raise an error if no device is found
            raise RuntimeError("No Device Found")
        print("Got device %s" % (device))

        # Open queue for threaded communication with device.
        self.queue = queue.Queue()
        t = threading.Thread(target=self.reader)
        t.daemon = True
        t.start()
        self.debug_print("thread started")
        while False == self.timeout_for(b"reset",timeout=1):
            self.debug_print("resetting")
            self.handle.write(b"reset\n")

    def _reader(self):
        self.debug_print("reader starts")
        while True:
            str = self.handle.readline()
            self.queue.put(str)

    # Wait for message matching "str", with max wait time "timeout"
    def _timeout_for(self,str,timeout):
        while True:
            try:
                resp = self.queue.get(block=True,timeout=timeout)
                self.debug_print("after get")
                self.queue.task_done()
                rc = resp.find(str)  
                if 0 == rc : return resp
                print("UnMatched: %s" % (resp.decode()))
            except queue.Empty:
                self.debug_print("exception")
                return False

    # Wait for message matching "str" with no max time
    def _wait_for(self,str,timeout=None):
        while True:
            resp = self.queue.get()
            self.queue.task_done()
            if 0:
                print((b"wait_for() read: "),)
                print((resp),)
            if "ERR:" == resp[0:4]:
                print(resp)
                raise RuntimeError
            rc = resp.find(str)  
            if 0 == rc : return resp
            print(b"Unmatched: %s" % (resp),)

    # Set debug mode? Not really sure yet.
    def _debug(self):
        self.handle.write(b"debug\n");
        self.wait_for(b"debug")
        return;

    # Get the device status
    def _status(self):
        self.handle.write(b"status\n");
        self.wait_for(b"status")
        return;        

    # Sets the pulse time for the angular motor
    def _angular_set(self,dt):
        command = b"a_set %f\n" % (dt)
        self.handle.write(command);
        resp = self.wait_for(b"a_set")
        words = resp.split(b" ")
        return int(words[1])/48e3 # samples/samplerate

    # Sets the pulse time for the radial motor
    def _radial_set(self,dt):
        command = b"r_set %f\n" % (dt)
        self.handle.write(command);
        resp = self.wait_for(b"r_set")
        words = resp.split(b" ")
        return int(words[1])/48e3 # samples/samplerate

    # Sends the angular motor the given number of steps.
    def _angular_go(self,steps):
        command = b"a_go %d\n" % (steps)
        self.handle.write(command);
        self.wait_for(b"a_go")
        return True

    # Sends the radial motor the given number of steps.
    def _radial_go(self,steps):
        command = b"r_go %d\n" % (steps)
        self.handle.write(command);
        self.wait_for(b"r_go")
        return True

    # Returns true if the angular motor is not moving
    def _angular_idle(self):
        self.handle.write(b"a_idle\n")
        resp = self.wait_for(b"a_idle")
        return b"true" == resp[7:11]

    # Returns true if the radial motor is not moving
    def _radial_idle(self):
        self.handle.write(b"r_idle\n")
        resp = self.wait_for(b"r_idle")
        return b"true" == resp[7:11]

    # Home the angular motor
    def _angular_home(self):
        self.handle.write(b"a_home\n")
        resp = self.wait_for(b"a_home")
        return True

    # Home the radial motor
    def _radial_home(self):
        self.handle.write(b"r_home\n")
        resp = self.wait_for(b"r_home")
        return True

    # Sends both motors a given number of steps and waits until they're both
    # done moving
    def _go_and_wait(self,angular,radial):
        while not (self.angular_idle() and self.radial_idle()): True
        if (angular != 0): self.angular_go(angular)
        if (radial != 0): self.radial_go(radial)
        while not (self.angular_idle() and self.radial_idle()): True
        return True

    # Added by Rigel:
    def _debug_print(self, message):
        if self.debug:
            print(message)
