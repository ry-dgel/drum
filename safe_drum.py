# Safe control code
# Written by Rigel Zifkin 2020-08-09

# By convention any time both positions are used in a single line
# The order should be radial, angular for polar coordinates
# and x,y for cartesian coordinates

import drum # For real use
#import dummy_drum as drum # For Testing
import numpy as np
import time
from datetime import datetime

ANG_MAX_STEPS = 720 # Number of steps that make a full circle
ANG_MIN_STEPS = 0 # Initial steps in case we want a specific physical angle to be 0 in the future.

RAD_MAX_STEPS = 10000 # Number of steps from the center to corner of plate
# Since we're on a square. If the sensor is all the way in a corner, rotating
# may result in a collision, so we need to be aware of these bounds.
RAD_MAX_SAFE  = 7300 # Number of steps from center to edge of plate
RAD_MIN_STEPS = 0 # Initial steps in case we want a specific radius to be 0.

######################################
# Safe Vibrating Plate Control Class #
######################################
class SafePlate(drum.Vibrating_Plate):
    def __init__(self, debug=True, shape="circle"):
        """ A safe wraper for controlling the vibrating plate lab.
            Everytime an instance is opened, the experiment is homed, this uses limit switches
            to set the apparatus to its (0,0) position.
           
            Moving the equpiment is then done using safe functions.

            WARNING: You should have no reason to access and functions/fields prefixed 
                     with an underscore '_'. Python leaves these fully accessible and you can
                     100% break the equipment if you mess with them.
                     If you really think you need to for some fantastic new functionality that will 
                     make the experiment go better, please contact a TA/Prof first.
            
            To initialize, simply define a new drum object:
            ```
            import safe_drum as sd
            drum = sd.SafePlate()
            ```
            
            This will print out some connection messages, and assuming the device connects
            properly, will home the instrument. If any errors occur during homing, a large
            warning will be displayed, please watch-out for that.
            
            Once homed, you have control of the instrument.  The main moving function is
            `drum.move_abs(rad_steps, ang_steps)` which sets the scanner to an 
            absolute position defined by a radius and angle.

            Both numbers should be in number of steps for the motor with rough conversion:
            1 Radial step ~ 0.01 mm
            1 Angular step ~ 0.5 degrees of rotaton.

            There are a bunch of wrappers to this motion function, allowing for relative 
            motion and cartesian coordinates. See those functions for more info.


        Parameters
        ----------
        debug : bool, optional
            Whether to print verbose debug messages, by default True
        shape : str, optional
            The shape of the plate in the apparatus. Can be one of 
            "circle" or "square", by default circle.
        """
        super().__init__(debug)
        # Home the instrument on every startup, that way we are always at 0,0
        # from the beginning
        self.home()
        self._shape = shape
        self._radial = RAD_MIN_STEPS
        self._angular = ANG_MIN_STEPS

    def get_radial(self):
        return self._radial

    def get_angular(self):
        return self._angular

    def home(self):
        """ Moves the radius and angular motor backwards until they hit the limit switches.
            This defines the minimal position of the scanner along both axes.
        """
        print("Homing Instrument, please wait")
        r_status = self._radial_home()
        a_status = self._angular_home()
        
        self._radial = RAD_MIN_STEPS
        self._angular = ANG_MIN_STEPS
        if not (r_status and a_status):
            if r_status:
                print("Radial Switch Failed.")
            if a_status:
                print("Angular Switch Failed.")
            print("!!!!!!~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~!!!!!!")
            print("Homing Failed, please check the camera and contact technician if needed.")
            print("If the radial switch is obviously not engadged, try running home() again.")
            print("Otherwise, immediatly get help and do not run any other commands.")
            print("!!!!!!~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~!!!!!!")
            input("Press Enter to acknowledge this message...")

        else:
            print("Homing Completed Succesfuly.")

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
        if not self.safe_polar(rad_steps, ang_steps/2):
            if shape is "square":
                raise ValueError("Radial position %d outside safe range %d for angular steps %d" % 
                                 (rad_steps, squine(ang_steps), ang_steps))
            if shape is "circle":
                raise ValueError("Radial position %d outside safe range %d" %
                                 (rad_steps, RAD_MAX_SAFE))

        # Take absolute position around single rotation of circle
        ang_delta = (ang_steps % ANG_MAX_STEPS) - self._angular
        
        if shape is "square":
            # Flags that modify how the motion should be handled
            retreat = False
            ang_first = False
            # If the radius is outside the safe range and we're rotating
            # we need to first pull in the sensor, then do the rotation
            # and finally set the radius to the correct amount.
            safe_dists = squine(np.linspace(self._angular,ang_steps,np.abs(self._angular-ang_steps)+1,dtype=int))
            safe_dist = np.min(safe_dists)
            if (ang_delta != 0 and self._radial > safe_dist):
                retreat = True
            # If we're moving outside the safe radial distance, we want to rotate first.
            if (rad_steps > RAD_MAX_SAFE):
                ang_first = True

            # If we're past the safe rotating radius, pull the radius in.
            if retreat:
                self._debug_print("Retreating radius for rotation by %d steps" % ang_delta)
                # If the target radius is within the safe limit, go there
                # otherwise, move to the minimum safe distance.
                self.rad_move_abs(safe_dist)
            # Calculate number of steps needed to move radially.
            # Put here since retreating will change this.
            rad_delta = rad_steps - self._radial

        self._debug_print("Moving to %d, %d" % (self._radial + rad_delta, self._angular + ang_delta))

        # If there is motion to do, send the motors that number of steps
        # and wait until idle.
        if ang_delta != 0:
            self._angular_go(ang_delta)
            # If we need the angular motion to finish before the radial motion starts, wait here.
            if ang_first:
                self._debug_print("Rotating Angle First")
                while not self._angular_idle():
                    time.sleep(0.1)
        if rad_delta != 0:
            self._radial_go(rad_delta)

        # Wait for all movement to stop.
        while not (self._radial_idle() or self._angular_idle()):
            time.sleep(0.1)

        # Register the new changes
        self._angular += ang_delta
        self._radial += rad_delta
        self._debug_print("Final Position: %d, %d" % (self._radial, self._angular))

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
            The position in steps, relative to zerodrum to move the radial position to.
        """
        self.move_abs(steps, self._angular)

    def ang_move_rel(self, steps):
        """ Step the angular motor by a given number of steps.  
            Wrapper for move_abs(), see that documentation for more info.

        Parameters
        ----------
        steps : int
            The number of steps to actuate the motor.
            Positive number indicates clockwise motion (looking from above).
            Negative number indicates counterclockwise motion (looking from above).
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
        self.move_abs(self._radial, steps)

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
            Positive number indicates clockwise motion (looking from above).
            Negative number indicates counterclockwise motion (looking from above).
        """
        self.move_abs(self._radial + rad_steps, self._angular + ang_steps)

    def cart_move_abs(self, x, y):
        """ Steps the radial and angular motors to a cartesian position (x,y).
            This is done by converting the x and y values to polar coordinates,
            so it may result in slightly different positions due to rounding.

        Parameters
        ----------
        x : int
            The position in steps along the x axis to posotion the sensor.
        y : int 
            The position in steps along the y axis to posotion the sensor.
        Raises
        ------
        ValueError
            Will return an error if the given coordinates are outisde the range set
            by +/- the maximum safe radius.
        """
        # Ensure integers, or integer like numbers are passed
        x = int(x)
        y = int(y)
        
        # Assert values are within range
        if not self.safe_xy(x,y):
            raise ValueError("Position (%d, %d) contains value outside safe range of %d-%d." % 
                              (x,y, -RAD_MAX_SAFE, RAD_MAX_STEPS))

        # Convert x,y to r,theta
        r,theta = xy_to_polar(x,y)
        # Convert to steps
        # This will probably introduce some rounding error...
        radial = int(round(r))
        angular = int(round(theta * 2))

        self.move_abs(radial, angular)

    def cart_move_rel(self, x, y):
        """ Steps the radial and angular motors to move relative to the current position a number
            of steps given (x,y) in cartesian coordinates. 
            This requires converting the current position to cartesian, calculating the new
            position and converting back to polar, and will likely result in a slightly different
            position due to rounding. 

        Parameters
        ----------
        x : int
            The number of steps to move along the x-axis
        y : int
            The number of steps to move along the y-axis
        """
        # Get current position in cartesian
        x_cur, y_cur = polar_to_xy(self._radial, self._angular/2)
        # This will probably introduce some rounding error...
        x_cur = int(round(x_cur))
        y_cur = int(round(y_cur))

        # Compute new absolute cartesian coordinates
        new_x = x + x_cur
        new_y = y + y_cur

        self.cart_move_abs(new_x, new_y)

    def safe_polar(self, radial, angular):
        """ Returns true if the given polar coordinates are safe for the vibrating plate.
            Tip: You can use this to filter a list of coordinates to ensure that no errors occur
                while scanning through them.

        Parameters
        ----------
        r : int or float
            The radius of the given position. To be safe, this value must be within
            the minimal safe radius, and the maximum safe radius for the given angle.
        theta : int or float
            The angle of the given position. This has no limitations as the angle
            will be automatically wrapped if it exceeds a full turn.

        Returns
        -------
        bool
            Returns true if the position is safe, and false otherwise.
        """
        radial = int(round(r))
        angular = int(round(angular))
        if shape is "square":
            return not (radial < RAD_MIN_STEPS or radial > squine(angular))
        elif shape is "circle":
            return not (radial < RAD_MIN_STEPS)

    def safe_xy(self, x,y):
        """ Returns true if the given cartesian coordinates are safe for the vibrating drum.
            Tip: You can use this to filter a list of coordinates to ensure that no errors occur
                while scanning through them.

        Parameters
        ----------
        x : int or float
            The x coordinate of the given position. To be safe this value must be 
            within +/- the maximum safe radius
        y : int or float
            The y coordinate of the given position. To be safe this value must be 
            within +/- the maximum safe radius

        Returns
        -------
        bool
            Returns true if the position is safe, and false otherwise.
        """
        if shape is "square":
            checks = [(pos < -RAD_MAX_SAFE or pos > RAD_MAX_SAFE) for pos in [x,y]]
            return not any(checks)
        if shape is "circle":
            r, theta = xy_to_polar(x,y)
            return r <= RAD_MAX_SAFE
####################
# Helper Functions #
####################
def xy_to_polar(x,y):
    """ Converts a set of cartesian coordinates (x,y) into their polar counterparts (r,theta).
        By convention this will return (0,0) when the input is (0,0) keep that in mind as it may lead
        to interesting behaviour when scanning through cartesian coordinates.
        For more info, read numpy's arctan2 documentation.

    Parameters
    ----------
    x : int or float
        The x coordinate.
    y : int or float
        the y coordinate.

    Returns
    -------
    float,float
        The corresponding polar coordinates.
    """
    r = np.sqrt(x**2 + y**2)
    theta = np.degrees(np.arctan2(y,x))
    return r,theta

def polar_to_xy(r,theta):
    """ Converts a set of polar coordinates (r,theta) into their cartesian counterparts (x,y).

    Parameters
    ----------
    r : int or float
        The radius coordinate.
    theta : int or float
        The angle coordinate.

    Returns
    -------
    float, float
        The corresponding cartesian coordinates.
    """
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return x,y

# For a given angle theta, max radial position is squine(theta)
# Here's a function for getting that as a function of angular steps
# Not good enough since the sensor has some width to it.
# For now gonna try multiplying by (0.91 + 0.09 * cos(2*theta)**2) in order
# to smoothly go from edge at angle 0 to corner at angle 45.
def squine(angular):
    """Calculates the absolute maximum radius for a given angular (in steps) position.
       This is effectively the distance between the center of a square and it's perimiter
       at a given angle. 
       As a multiple of half the square's width, for an edge it's exactly 1, 
       for the exact corner it's sqrt(2). 
       Since the sensor is not a zero-size point, it's width must also be considered.
       This is handled by a multiplicative factor of (0.91 + 0.09 * np.cos(2*theta)**2), which
       reduces the calculated squine value by a factor of 0.91 at the corner to 1.0 at the edge.

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
    multiple = (0.91 + 0.09 * np.cos(2*theta)**2) * RAD_MAX_SAFE
    # Don't care about divide by zero within the following math
    with np.errstate(divide='ignore'):
        return  multiple * np.min(np.abs([1/np.cos(theta), 1/np.sin(theta)]), axis=0)

# Testing will remove
if __name__ == "__main__":
    plate = SafePlate()
    # Test Polar Bounds
    for i in range(0,365,5):
        print(i)
        plate.move_abs(squine(i),i)
    # Test Cartesian Bounds
    for i in range(-7000,7200,200):
        print(i)
        plate.cart_move_abs(i,i)
    plate.move_abs(0,0)
"""
    
    # Set position to 0, 0
    #drum.home()
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
"""