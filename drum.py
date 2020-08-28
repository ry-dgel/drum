# Original code by Mark Orchard-Webb,
# Tweaked by:
#            Robert Turner, 2019.12.09
#            Rigel Zifkin, 2020.08.09
# This code needs comments and probably scrubbing.

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

    def reader(self):
        self.debug_print("reader starts")
        while True:
            str = self.handle.readline()
            self.queue.put(str)

    # Wait for message matching "str", with max wait time "timeout"
    def timeout_for(self,str,timeout):
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
    def wait_for(self,str,timeout=None):
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
    def debug(self):
        self.handle.write(b"debug\n");
        self.wait_for(b"debug")
        return;

    # Get the device status
    def status(self):
        self.handle.write(b"status\n");
        self.wait_for(b"status")
        return;        

    # Sets the pulse time for the angular motor
    def angular_set(self,dt):
        command = b"a_set %f\n" % (dt)
        self.handle.write(command);
        resp = self.wait_for(b"a_set")
        words = resp.split(b" ")
        return int(words[1])/48e3 # samples/samplerate

    # Sets the pulse time for the radial motor
    def radial_set(self,dt):
        command = b"r_set %f\n" % (dt)
        self.handle.write(command);
        resp = self.wait_for(b"r_set")
        words = resp.split(b" ")
        return int(words[1])/48e3 # samples/samplerate

    # Sends the angular motor the given number of steps.
    def angular_go(self,steps):
        command = b"a_go %d\n" % (steps)
        self.handle.write(command);
        self.wait_for(b"a_go")
        return True

    # Sends the radial motor the given number of steps.
    def radial_go(self,steps):
        command = b"r_go %d\n" % (steps)
        self.handle.write(command);
        self.wait_for(b"r_go")
        return True

    # Returns true if the angular motor is not moving
    def angular_idle(self):
        self.handle.write(b"a_idle\n")
        resp = self.wait_for(b"a_idle")
        return b"true" == resp[7:11]

    # Returns true if the radial motor is not moving
    def radial_idle(self):
        self.handle.write(b"r_idle\n")
        resp = self.wait_for(b"r_idle")
        return b"true" == resp[7:11]

    # Home the angular motor
    def angular_home(self):
        self.handle.write(b"a_home\n")
        resp = self.wait_for(b"a_home")
        return True

    # Home the radial motor
    def radial_home(self):
        self.handle.write(b"r_home\n")
        resp = self.wait_for(b"r_home")
        return True

    # Sends both motors a given number of steps and waits until they're both
    # done moving
    def go_and_wait(self,angular,radial):
        while not (self.angular_idle() and self.radial_idle()): True
        if (angular != 0): self.angular_go(angular)
        if (radial != 0): self.radial_go(radial)
        while not (self.angular_idle() and self.radial_idle()): True
        return True

    # Gets raw measurement samples from the device
    def get_samples(self):
        self.handle.write(b"get_samples\n")
        resp = self.wait_for(b"get_samples")
        n = int(resp[11:])
        values = numpy.zeros(n)
        for i in range(n):
            values[i] = int(self.queue.get())
            self.queue.task_done()
        return values

    # Returns raw samples while moving the angular motor, not recommended.
    def angular_record(self,steps):
        self.handle.write(b"m_sample\n");
        self.wait_for(b"m_sample")
        self.angular_go(steps)
        while not self.angular_idle(): time.sleep(1)
        values = self.get_samples()
        if abs(steps) != numpy.size(values,axis=0):
            print(b"Odd, samples count (%d) is different from steps taken (%d)" % (numpy.size(values,axis=0),abs(steps)))
        return values

    # Returns raw samples while moving the radial motor, not recommended.
    def radial_record(self,steps):
        self.handle.write(b"m_sample\n");
        self.wait_for(b"m_sample")
        self.radial_go(steps)
        while not self.radial_idle(): time.sleep(1)
        values = self.get_samples()
        if abs(steps) != numpy.size(values,axis=0):
            print(b"Odd, samples count (%d) is different from steps taken (%d)" % (numpy.size(values,axis=0),abs(steps)))
        return values

    # Records samples to then send
    def raw_record(self):
        self.handle.write(b"t_sample\n");
        self.wait_for(b"t_sample")
        time.sleep(1)
        return self.get_samples()

    # Added by Rigel:
    def debug_print(self, message):
        if self.debug:
            print(message)
