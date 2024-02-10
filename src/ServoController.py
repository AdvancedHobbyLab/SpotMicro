import smbus, time

class ServoController:
    """Class to control servos through an I2C interface.

    It is based off a servo controller using the PCA9685 chip.
    """
    
    def __init__(self, bus=1, address=0x40):
        """Init function

        Parameters
        ----------
        bus : int
            The I2C bus for the servo controller.
        address : int
            The I2C address for the servo controller.
        """
        self._bus = smbus.SMBus(bus)
        self._address = address
        
        self._bus.write_byte_data(address, 0, 0x20) # enables word writes
        time.sleep(.25)
        self._bus.write_byte_data(address, 0, 0x10) # enable Prescale change as noted in the datasheet
        time.sleep(.25) # delay for reset
        self._bus.write_byte_data(address, 0xfe, 0x79) #changes the Prescale register value for 50 Hz, using the equation in the datasheet.
        self._bus.write_byte_data(address, 0, 0x20) # enables word writes

    class Servo:
        """Class for controlling a single servo."""
        
        def __init__(self, controller, port, offset):
            """Init function, should only be called by ServoController.
            
            Parameters
            ----------
            controller : ServoController
                Reference to the ServoController class that created it.
            port : int
                The port on the servo controller PCB that this servo is connected to.
            offset : float
                The angle (deg.) to offset the zero position to fine-tune the position in software.
            """
            self._controller = controller
            self._offset = offset
            self._angle = 90
            self._min_angle = 45
            self._max_angle = 135
            self._min_pulse = 1.0 / ((1/50.0)/4096 * 1e3)
            self._max_pulse = self._min_pulse * 2

            self._start = 0x6 + 4*port
            self._end   = 0x8 + 4*port

            self._controller._bus.write_word_data(self._controller._address, self._start, 0) # pulse start time = 0us

        def set_angle(self, angle):
            """Set the angle (deg.) of the servo.
            
            Parameters
            ----------
            angle : float
                The new angle (deg.) of the servo.
            """
            self._angle = max(self._min_angle, min(angle, self._max_angle))
            angle_ratio = (self._angle-self._min_angle) / (self._max_angle-self._min_angle) 

            ## (1/freq) / (12 bit resolution) * (pulse length)
            #pulse = (1.0 + angle_ratio) / ((1/50.0)/4096 * 1e3)
            #pulse += self._offset
            pulse = self._min_pulse + (self._max_pulse-self._min_pulse)*angle_ratio
            pulse += self._offset
            
            self._controller._bus.write_word_data(self._controller._address, self._end, int(pulse))

        def zero(self):
            """Set the angle to the zero position.n"""
            self.set_angle(90)

        def set_min_angle(self, angle):
            """Set the minimum angle that the servo can achieve.

            Parameters
            ----------
            angle : float
                The new minimum angle (deg.)
            """
            self._min_angle = angle

        def set_min_pulse(self, pulse):
            """Set the pulse required to achieve the minimum angle.

            Parameters
            ----------
            pulse : float
                The new minimum pulse.
            """
            self._min_pulse = pulse

        def set_max_angle(self, angle):
            """Set the maximum angle that the servo can achieve.

            Parameters
            ----------
            angle : float
                The new maximum angle (deg.)
            """
            self._max_angle = angle

        def set_max_pulse(self, pulse):
            """Set the pulse required to achieve the maximum angle.

            Parameters
            ----------
            pulse : float
                The new maximum pulse.
            """
            self._max_pulse = pulse

        def get_offset(self):
            """Get the offset form the zero position."""
            return self._offset

        def set_offset(self, offset):
            """Set the offset from the zero position.

            Parameters
            ----------
            offset : float
                The new offset (deg.)
            """
            self._offset = offset

    def load_servos(self, ports, offsets = None):
        """Create Servo instances.
        
        Parameters
        ----------
        ports: list
            A list of the port number on the PCB where servos are attached.
        offsets: list
            A list zero position offsets for each servo. Should be the same 
            length as ports or None to use 0 for offsets.

        Returns:
            list : A list of Servo instances that correlates to the list of ports provided.
        """
        servos = []
        for i in range(len(ports)):
            servos.append(self.Servo(self, ports[i], offsets[i]))
        return servos
            

    
