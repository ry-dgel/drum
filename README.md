# drum
## Basic Instructions
To initialize, simply define a new drum object:
``` python
import safe_drum as sd
drum = sd.SafePlate()
```

This will print out some connection messages, and, assuming the device connects
properly, will home the instrument. If any errors occur during homing, a large
warning will be displayed, please watchout for that.

Once homed, you have control of the instrument.  
The main moving function is:
``` python
drum.move_abs(rad_steps, ang_steps)
``` 
which sets the scanner to an absolute position defined by a radius and angle.

See safe_drum.py for more moving functions and documentation. 

## Code hirerarchy
drum.py contains initial code for controlling arduino. Comments and homing functions
were added. Every method had a '\_' prepended to indicate that the students should not
access them directly.

safe_drum.py contains all the safe code, based on the SafePlate class which is the 
interface through which the device should be accessed.

dummy_drum.py is just a dummy version of drum.py for testing purposes and is not needed.
