import time
import math
import asyncio
from enum import Enum

from InverseKinematics import InverseKinematics, interpolate, KeyframeKinematics

class Locomotion:
    """Main class for running keyframe animation for SpotMicro.

    This class defines the keyframe animation and provides access functions
    to control the animation for SpotMicro. The animations only define the
    position of the feet and use the inverse kinematic equasions to convert
    them into servo angles which are then sent to the servo controller.
    """
    class Leg(Enum):
        # Enum difinition for the four legs
        FrontLeft = 0
        FrontRight = 1
        BackLeft = 2
        BackRight = 3
    
    def __init__(self, servos):
        """Init function

        Parameters
        ----------
        servos : list
            list of 12 servos, ordered by leg: FrontLeft, FrontRight, BackLeft, BackRight,
            with the servos in each leg ordered by: shoulder, leg, foot.
        """
        self._servos = servos
        self._running = False
        self._keyframes = []
        for i in range(4):
            self._keyframes.append([[0,150, 0], [0,150, 0]])
            
        self._standing = True
        self._forward_factor = 0.0
        self._rotation_factor = 0.0
        self._lean_factor = 0.0
        self._height_factor = 0.0
        

    async def Run(self):
        """Main loop for running keyframe animation.
        
        When executed, this function will start the keyframe animation and continue until
        the Shutdown function is called. It can be run using asyncio to process other events.
        """
        elapsed = 0.0
        gait = [[-10.0, 150.0, 40.0], [-10.0,120.0,40.0], [10.0, 120.0, 40.0], [10.0, 150.0, 40.0], [3.5, 150.0, 40.0], [-3.5, 150, 40]]
        start = time.time()
        
        last_index = -1
        self._running = True
        while self._running:
            elapsed += (time.time()-start)*15
            if elapsed >= len(gait):
                elapsed -= len(gait)
            start = time.time()
            index = math.floor(elapsed)
            ratio = elapsed - index

            if last_index != index:
                self._shift_keyframes()
                
                if self._standing:
                    self._set_standing_keyframes()
                else:
                    angle = 45.0/180.0*math.pi
                    x_rot = math.sin(angle) * self._rotation_factor
                    z_rot = math.cos(angle) * self._rotation_factor

                    angle = (45+gait[index][0])/180.0*math.pi
                    x_rot = x_rot-math.sin(angle) * self._rotation_factor
                    z_rot = z_rot-math.cos(angle) * self._rotation_factor
                    self._keyframes[self.Leg.FrontRight.value][1] = [gait[index][0]*self._forward_factor+x_rot, gait[index][1], gait[index][2]+z_rot]
                    self._keyframes[self.Leg.BackLeft.value][1] = [gait[index][0]*self._forward_factor-x_rot, gait[index][1], gait[index][2]+z_rot]
    
                    adjusted_index = index+3
                    if adjusted_index >= len(gait): adjusted_index -= len(gait)

                    angle = 45.0/180.0*math.pi
                    x_rot = math.sin(angle) * self._rotation_factor
                    z_rot = math.cos(angle) * self._rotation_factor

                    angle = (45+gait[adjusted_index][0])/180.0*math.pi
                    x_rot = x_rot-math.sin(angle) * self._rotation_factor
                    z_rot = z_rot-math.cos(angle) * self._rotation_factor
                    self._keyframes[self.Leg.FrontLeft.value][1] = [gait[adjusted_index][0]*self._forward_factor-x_rot, gait[adjusted_index][1], gait[adjusted_index][2]-z_rot]
                    self._keyframes[self.Leg.BackRight.value][1] = [gait[adjusted_index][0]*self._forward_factor+x_rot, gait[adjusted_index][1], gait[adjusted_index][2]-z_rot]

                last_index = index

            self._InterpolateKeyframes(ratio)

            await asyncio.sleep(0)

        # Set keyframes for standing position
        self._shift_keyframes(elapsed - math.floor(elapsed))
        self._set_standing_keyframes()
        self._standing = True
            
        elapsed = 0.0
        start
        while elapsed < 1:
            elapsed += (time.time()-start)*20
            start = time.time()
            self._InterpolateKeyframes(elapsed)

            await asyncio.sleep(0)

    def set_forward_factor(self, factor):
        """Set forward and backward movement.

        Parameters
        ----------
        factor : float
            Positive values move forward, negative values move back. Should be in the range -1.0 - 1.0.
        """
        self._forward_factor = factor*2

    def set_rotation_factor(self, factor):
        """Set rotation movement.

        Parameters
        ----------
        factor : float
            Positive values rotate right, negative values rotate left. Should be in the range -1.0 - 1.0.
        """
        self._rotation_factor = factor*100

    def set_lean(self, lean):
        """Set the distance that it should lean left to right.

        Parameters
        ----------
        lean : float
            Positive values lean right, negative values lean left. Should be in the range -1.0 - 1.0.
        """
        self._lean_factor = lean*20

    def set_height_offset(self, height):
        """Set the extra distance for the chassis to be off the ground.

        Parameters
        ----------
        height : float
            Should be in the range 0.0-1.0.
        """
        self._height_factor = height*40

    def toggle_standing(self):
        """Toggle the standing keyframes."""
        self._standing = not self._standing

    def Shutdown(self):
        """Shutdown and stop the Run loop."""
        self._running = False

    def _InterpolateKeyframes(self, ratio):
        """Interpolate between the current and next keyframes for each leg and apply the servo positions.

        Parameters
        ----------
        ratio : float
            The ratio of each keyframe in the interpolation. Should be in the range 0.0-1.0.
        """
        keyframes = self._keyframes[self.Leg.FrontRight.value]
        foot, leg, shoulder = KeyframeKinematics([keyframes[0][0], keyframes[0][1]+self._height_factor, keyframes[0][2]-self._lean_factor],
                                                 [keyframes[1][0], keyframes[1][1]+self._height_factor, keyframes[1][2]-self._lean_factor],
                                                 ratio)
        self._FrontRightLeg(foot, leg, shoulder)

        keyframes = self._keyframes[self.Leg.BackLeft.value]
        foot, leg, shoulder = KeyframeKinematics([keyframes[0][0], keyframes[0][1]+self._height_factor, keyframes[0][2]+self._lean_factor],
                                                 [keyframes[1][0], keyframes[1][1]+self._height_factor, keyframes[1][2]+self._lean_factor],
                                                 ratio)
        self._BackLeftLeg(foot, leg, shoulder)

        keyframes = self._keyframes[self.Leg.FrontLeft.value]
        foot, leg, shoulder = KeyframeKinematics([keyframes[0][0], keyframes[0][1]+self._height_factor, keyframes[0][2]+self._lean_factor],
                                                 [keyframes[1][0], keyframes[1][1]+self._height_factor, keyframes[1][2]+self._lean_factor],
                                                 ratio)
        self._FrontLeftLeg(foot, leg, shoulder)
        
        keyframes = self._keyframes[self.Leg.BackRight.value]
        foot, leg, shoulder = KeyframeKinematics([keyframes[0][0], keyframes[0][1]+self._height_factor, keyframes[0][2]-self._lean_factor],
                                                 [keyframes[1][0], keyframes[1][1]+self._height_factor, keyframes[1][2]+self._lean_factor],
                                                 ratio)
        self._BackRightLeg(foot, leg, shoulder)
    
    def _FrontRightLeg(self, foot, leg, shoulder):
        """Helper function for setting servo angles for the front right leg.

        Parameters
        ----------
        foot : float
            Servo angle for foot in degrees.
        leg : float
            Servo angle for leg in degrees.
        shoulder : float
            Servo angle for shoulder in degrees.
        """
        self._servos[2].set_angle(180-foot)
        self._servos[1].set_angle(180-(leg+90))
        self._servos[0].set_angle(shoulder)
    
    def _FrontLeftLeg(self, foot, leg, shoulder):
        """Helper function for setting servo angles for the front left leg.

        Parameters
        ----------
        foot : float
            Servo angle for foot in degrees.
        leg : float
            Servo angle for leg in degrees.
        shoulder : float
            Servo angle for shoulder in degrees.
        """
        self._servos[5].set_angle(foot)
        self._servos[4].set_angle(180-(90-leg))
        self._servos[3].set_angle(180-shoulder)
    
    def _BackRightLeg(self, foot, leg, shoulder):
        """Helper function for setting servo angles for the back right leg.

        Parameters
        ----------
        foot : float
            Servo angle for foot in degrees.
        leg : float
            Servo angle for leg in degrees.
        shoulder : float
            Servo angle for shoulder in degrees.
        """
        self._servos[8].set_angle(180-foot)
        self._servos[7].set_angle(180-(leg+90))
        self._servos[6].set_angle(shoulder)
    
    def _BackLeftLeg(self, foot, leg, shoulder):
        """Helper function for setting servo angles for the back left leg.

        Parameters
        ----------
        foot : float
            Servo angle for foot in degrees.
        leg : float
            Servo angle for leg in degrees.
        shoulder : float
            Servo angle for shoulder in degrees.
        """
        self._servos[11].set_angle(foot)
        self._servos[10].set_angle(180-(90-leg))
        self._servos[9].set_angle(180-shoulder)

    def _shift_keyframes(self, ratio = 0):
        """Shift the next keyframe to be the current keyframe.
        
        Parameters
        ----------
        ratio : float
            Ratio for interpolating keyframes if shifting between keyframe transitions.
        """
        if ratio > 0:
            for i in range(len(self._keyframes)):
                key1 = self._keyframes[i][0]
                key2 = self._keyframes[i][1]
                self._keyframes[i][0] = [interpolate(key1[0], key2[0], ratio), interpolate(key1[1], key2[1], ratio), interpolate(key1[2], key2[2], ratio)]
        else:
            for i in range(len(self._keyframes)):
                self._keyframes[i][0] = self._keyframes[i][1]

    def _set_standing_keyframes(self):
        """Helper function for seting a static standing keyframe"""
        for i in range(4):
            self._keyframes[i][1] = [0, 150, 40]
        
