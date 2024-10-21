#!/usr/bin/env python

import sys
import time
import crtk
import dvrk
import numpy


# Simple application for moving the arm up and down
class simple_up_down_movement:

    def __init__(self, ral, arm_name):
        print(f'Configuring dvrk_arm for {arm_name}')
        self.ral = ral
        self.interval = 0.01
        self.arm = dvrk.arm(ral = ral,
                            arm_name = arm_name,
                            expected_interval = self.interval)

    def home(self):
        self.arm.check_connections()

        print('starting enable')
        if not self.arm.enable(10):
            sys.exit('failed to enable within 10 seconds')
        print('starting home')
        if not self.arm.home(10):
            sys.exit('failed to home within 10 seconds')
        # get current joints just to set size
        print('move to starting position')
        goal = numpy.copy(self.arm.setpoint_jp())
        # go to zero position, for PSM and ECM make sure 3rd joint is past cannula
        goal.fill(0)
        if ((self.arm.name() == 'PSM1') or (self.arm.name() == 'PSM2')
            or (self.arm.name() == 'PSM3') or (self.arm.name() == 'ECM')):
            goal[2] = 0.12
        # move and wait
        print('moving to starting position')
        self.arm.move_jp(goal).wait()
        # try to move again to make sure waiting is working fine, i.e. not blocking
        print('testing move to current position')
        move_handle = self.arm.move_jp(goal)
        print('move handle should return immediately')
        move_handle.wait()
        print('home complete')

    def move_up_down(self, amplitude, duration):
        print(f'Moving arm up and down with amplitude {amplitude} and duration {duration}')
        
        initial_position = numpy.copy(self.arm.setpoint_jp())  # Get current position
        goal = numpy.copy(initial_position)  # Start with the current position
        samples = int(duration / self.interval)   # Number of steps for smooth motion
        
        sleep_rate = self.ral.create_rate(1.0 / self.interval)
        
        for i in range(samples):
            # Move up (increase in z-axis)
            goal[2] = initial_position[2] + amplitude * (1 - numpy.cos(i * numpy.pi / samples))  # Smooth movement
            self.arm.move_jp(goal).wait()
            sleep_rate.sleep()
        
        for i in range(samples):
            # Move down (decrease in z-axis)
            goal[2] = initial_position[2] - amplitude * (1 - numpy.cos(i * numpy.pi / samples))  # Smooth movement
            self.arm.move_jp(goal).wait()
            sleep_rate.sleep()
    
    def run(self):
        self.home()
        # Parameters for the up-down motion
        amplitude = 0.05  # 5 cm movement in z-axis
        duration = 5.0  # total duration of up-down motion
        self.move_up_down(amplitude, duration)

def main():
    arm_name = 'PSM1'
    ral = crtk.ral('dvrk_move_arm')

    arm_mover = simple_up_down_movement(ral, arm_name)
    ral.spin_and_execute(arm_mover.run)

if __name__ == '__main__':
    main()
