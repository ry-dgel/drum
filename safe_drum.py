# Safe control code
# Written by Rigel Zifkin 2020-08-09

# import drum
import dummy_drum as drum # For Testing
import numpy as np
import time
from datetime import datetime

#TODO Determine these values.
ANG_MAX_STEPS = 360 # Number of steps that make a full circle
ANG_MIN_STEPS = 0 # Initial steps in case we want a specific physical angle to be 0 in the future.

RAD_MAX_STEPS = 12000 # Number of steps from the center to corner of plate
# Since we're on a square. If the sensor is all the way in a corner, rotating
# may result in a collision, so we need to be aware of these bounds.
RAD_MAX_SAFE  = 10000 # Number of steps from center to edge of plate
RAD_MIN_STEPS = 0 # Initial steps in case we want a specific radius to be 0.

# By convention any time both positions are used in a single line
# The order should be radial, angular
class SafePlate(drum.Vibrating_Plate):
    def __init__(self, debug=True):
        super().__init__(debug)
        # When initalized try to read the previous position.
        self.radial, self.angular = self.read_position()


    def write_position(self):
        """ Writes the position to a file (position.csv) with a date and time.
            For tracking the position between initializations of the code.
            Will warn if it can't access the working directory.
        """
        try:
            f = open("position.csv", 'w')
        except PermissionError:
            print("""Could not write position to file, make sure you're 
                     running this from a directory in which you have write access.""")
        now = datetime.now()
        f.write("#Previous drumhead position on %s\n" % now.strftime("%Y/%m/%d %H:%M:%S"))
        f.write("#Radial, Angular\n")
        f.write("%d, %d" % (self.radial, self.angular))

    def read_position(self):
        """ Reads the previous location of the scanner from a file (position.csv).
            If the file does not exist, or can't be read, it will default to 0,0.
            In this case, it remind the user to home the scanner.
        """
        # Initialize default position
        pos = [0,0]
        try:
            # Read from the file
            pos = np.genfromtxt("position.csv", delimiter=',', skip_header = 2, dtype=np.intc)
        except (FileNotFoundError, OSError):
            print("No position file found. Position set to 0,0. Consider running drum.home().")

        # Return the radius, and angle
        return pos[0],pos[1]

    def home(self):
        """ Moves the radius and angular motor backwards until they hit the limit switches.
            This defines the (0,0) position of the scanner. 
        """
        #rad_stop, ang_stop = self.read_limits()
        #while not (rad_stop and ang_stop):
        #   if not rad_stop:
                ## Decriment Radial by x steps
        #   if not ang_stop:
                ## Decriment Angular by x steps
        #   rad_stop, ang_stop = self.read_limits()
        #
        self.radial = RAD_MIN_STEPS
        self.angular = ANG_MIN_STEPS

    def read_limits(self):
        """Reads the status of the radial and angular limit switch.

        Returns
        -------
        bool, bool
            Status of the radial and limit switch respectively.
            True indicates that the switch is depressed.
        """
        #TODO
        return False,False

    def move_abs(self, rad_steps, ang_steps):
        """ Steps the radial and angular motors to the position given by their respective
            number of steps relative to zero.

            This method protects the motor and sensor through several precautions:
                * Ensures limits are established on the given positions.
                * Automatically adjusts angular steps so that the actuator never performs
                  more than a full rotation.
                * Calculates the max sensor radius for a given angle, and ensures
                  the sensor does not collide with the walls of the square plate while moving
                  to this position.

        Parameters
        ----------
        rad_steps : int
            The position, in steps, relative to zero to move the radial position to.
        ang_steps : int
            The position, in steps, relative to zero to move the angular position to.

        Raises
        ------
        ValueError
            Will return this error if the radius is attempted to be set outside of the max range,
            or if it's set to a radius that is incompatible with the desired angular position.
        """
        # Ensure integers, or integer like numbers are passed
        rad_steps = int(rad_steps)
        ang_steps = int(ang_steps)

        # Assert radial position is within limits
        if (rad_steps < RAD_MIN_STEPS or rad_steps > RAD_MAX_STEPS):
            raise ValueError("Angular position %d outside range of %d-%d." % 
                            (rad_steps, RAD_MIN_STEPS, RAD_MAX_STEPS))

        if (rad_steps > squine(ang_steps)):
            raise ValueError("Radial position %d outside safe range %d for angle %d" % 
                            (rad_steps, squine(ang_steps), ang_steps))

        # Take absolute position around single rotation of circle
        ang_delta = (ang_steps % ANG_MAX_STEPS) - self.angular
        
        # Flags that modify how the motion should be handled
        retreat = False
        ang_first = False
        # If the radius is outside the safe range and we're rotating
        # we need to first pull in the sensor, then do the rotation
        # and finally set the radius to the correct amount.
        if (ang_delta != 0 and self.radial > RAD_MAX_SAFE):
            retreat = True
            ang_first = True
        # If we're moving outside the safe radial distance, we want to rotate first.
        if (rad_steps > RAD_MAX_SAFE):
            ang_first = True

        # If we're past the safe rotating radius, pull the radius in.
        if retreat:
            self.debug_print("Retreating radius for rotation by %d steps" % ang_delta)
            # If the target radius is within the safe limit, go there
            # otherwise, move to the minimum safe distance.
            if rad_steps < RAD_MAX_SAFE:
                self.rad_move_abs(rad_steps)
            else:
                self.rad_move_abs(RAD_MAX_SAFE)
        # Calculate number of steps needed to move radially.
        # Put here since retreating will change this.
        rad_delta = rad_steps - self.radial

        self.debug_print("Moving to %d, %d" % (self.radial + rad_delta, self.angular + ang_delta))

        # If there is motion to do, send the motors that number of steps
        # and wait until idle.
        if ang_delta != 0:
            self.angular_go(ang_delta)
            # If we need the angular motion to finish before the radial motion starts, wait here.
            if ang_first:
                self.debug_print("Rotating Angle First")
                while not self.angular_idle():
                    time.sleep(0.1)
        if rad_delta != 0:
            self.radial_go(rad_delta)

        #TODO Monitor Limit Switches while moving
        # Wait for all movement to stop.
        while not (self.radial_idle() and self.angular_idle()):
            time.sleep(0.1)

        # Register the new changes
        self.angular += ang_delta
        self.radial += rad_delta
        self.write_position()
        self.debug_print("%d, %d" % (self.radial, self.angular))

    def rad_move_rel(self, steps):
        """ Step the radial motor by a given number of steps.
            Wrapper for move_abs(), see that documentation for more info.
           
        Parameters
        ----------
        steps : int
            The number of steps to actuate the motor.
            Positive number indicates forward motion.
            Negative number indicates backwards motion.
        """
        self.move_rel(steps, 0)

    def rad_move_abs(self, steps):
        """ Steps the radial motor to an absolute position given by 'steps' from zero.
            Wrapper for move_abs(), see that documentation for more info.

        Parameters
        ----------
        steps : int
            The position in steps, relative to zero to move the radial position to.
        """
        self.move_abs(steps, self.angular)

    def ang_move_rel(self, steps):
        """ Step the angular motor by a given number of steps.  
            Wrapper for move_abs(), see that documentation for more info.

        Parameters
        ----------
        steps : int
            The number of steps to actuate the motor.
            # TODO: Check if these directions are correct:
            Positive number indicates clockwise motion.
            Negative number indicates counterclockwise motion.
        """
        self.move_rel(0, steps)

    def ang_move_abs(self, steps):
        """Steps the angular motor to an absolute position given by 'steps' from zero.
           Wrapper for move_abs(), see that documentation for more info.
           
        Parameters
        ----------
        steps : int
            The position in steps, relative to zero to move the angular position to.
        """
        self.move_abs(self.radial, steps)

    def move_rel(self, rad_steps, ang_steps):
        """Step the radial and angular motor by a given number of steps.  
           Wrapper for move_abs(), see that documentation for more info.

        Parameters
        ----------
        rad_steps : int
            The number of steps to actuate the radial motor.
            Positive number indicates forward motion.
            Negative number indicates backwards motion.
        ang_steps : int
            The number of steps to actuate the angular motor.
            # TODO: Check if these directions are correct:
            Positive number indicates clockwise motion.
            Negative number indicates counterclockwise motion.
        """
        self.move_abs(self.radial + rad_steps, self.angular + ang_steps)

    

# For a given angle theta, max radial position is squine(theta)
# Here's a function for getting that as a function of angular steps
def squine(angular):
    """Calculates the absolute maximum radius for a given angular (in steps) position.
       This is effectively the distance between the center of a square and it's perimiter
       at a given angle. 
       As a multiple of half the square's width, for an edge it's exactly 1, 
       for the exact corner it's sqrt(2).

    Parameters
    ----------
    angular : int
        the angular position in steps.

    Returns
    -------
    int
        the maximum position in steps of the radial motor for the sensor to not
        collide with the walls.
    """
    theta = (angular/ANG_MAX_STEPS) * 2 * np.pi
    # Don't care about divide by zero within the following math
    with np.errstate(divide='ignore'):
        return RAD_MAX_SAFE * np.min(np.abs([1/np.cos(theta), 1/np.sin(theta)]))

# Testing will remove
if __name__ == "__main__":
    drum = SafePlate()
    # Set position to 0, 0
    drum.home()
    # Testing basic moves
    drum.move_abs(100,100)
    drum.move_abs(200,200)
    drum.move_abs(100,100)

    # Testing Bounds
    drum.move_abs(100,820)
    drum.move_abs(100,-820)
    try: 
        drum.move_abs(100000, 90)
    except ValueError:
        print("Caught Exception - Radius way to big")
        pass
    
    # Testing Safe Rotation
    drum.radial_set(0.01)
    drum.move_abs(11000, 90)
    try:
        drum.move_abs(10050, 360)
    except ValueError:
        print("Caught Exception - Radius outside safe range")

    drum.move_abs(300,0)
    drum.radial_set(0.1)