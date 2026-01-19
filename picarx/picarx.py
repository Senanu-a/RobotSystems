from robot_hat import Pin, ADC, PWM, Servo, fileDB
from robot_hat import Grayscale_Module, Ultrasonic, utils
import time
import os


def constrain(x, min_val, max_val):
    '''
    Constrains value to be within a range.
    '''
    return max(min_val, min(max_val, x))

class Picarx(object):
    CONFIG = '/opt/picar-x/picar-x.conf'

    DEFAULT_LINE_REF = [1000, 1000, 1000]
    DEFAULT_CLIFF_REF = [500, 500, 500]

    DIR_MIN = -30#!/usr/bin/env python3
from .basic import _Basic_class
from .pwm import PWM
from .servo import Servo
import time
from .filedb import fileDB
import os

# user and User home directory
User = None
UserHome = None
config_file = None


class Robot(_Basic_class):
    """
    Robot class

    This class is for makeing a servo robot with Robot HAT

    There are servo initialization, all servo move in specific speed. servo offset and stuff. make it easy to make a robot.
    All Pi-series robot from SunFounder use this class. Check them out for more details.

    PiSloth: https://github.com/sunfounder/pisloth

    PiArm: https://github.com/sunfounder/piarm

    PiCrawler: https://github.com/sunfounder/picrawler
    """

    move_list = {}
    """Preset actions"""

    max_dps = 428  # dps, degrees per second, genally in 4.8V : 60des/0.14s, dps = 428
    # max_dps = 500
    """Servo max Degree Per Second"""

    def __init__(self, pin_list, db=config_file, name=None, init_angles=None, init_order=None, **kwargs):
        """
        Initialize the robot class

        :param pin_list: list of pin number[0-11]
        :type pin_list: list
        :param db: config file path
        :type db: str
        :param name: robot name
        :type name: str
        :param init_angles: list of initial angles
        :type init_angles: list
        :param init_order: list of initialization order(Servos will init one by one in case of sudden huge current, pulling down the power supply voltage. default order is the pin list. in some cases, you need different order, use this parameter to set it.)
        :type init_order: list
        :type init_angles: list
        """
        super().__init__(**kwargs)
        self.servo_list = []
        self.pin_num = len(pin_list)

        if name == None:
            self.name = 'other'
        else:
            self.name = name

        self.offset_value_name = f"{self.name}_servo_offset_list"
        # offset
        self.db = fileDB(db=db, mode='774', owner=User)
        temp = self.db.get(self.offset_value_name,
                           default_value=str(self.new_list(0)))
        temp = [float(i.strip()) for i in temp.strip("[]").split(",")]
        self.offset = temp

        # parameter init
        self.servo_positions = self.new_list(0)
        self.origin_positions = self.new_list(0)
        self.calibrate_position = self.new_list(0)
        self.direction = self.new_list(1)

        # servo init
        if None == init_angles:
            init_angles = [0]*self.pin_num
        elif len(init_angles) != self.pin_num:
            raise ValueError('init angels numbers do not match pin numbers ')

        if init_order == None:
            init_order = range(self.pin_num)

        for i, pin in enumerate(pin_list):
            self.servo_list.append(Servo(pin))
            self.servo_positions[i] = init_angles[i]
        for i in init_order:
            self.servo_list[i].angle(self.offset[i]+self.servo_positions[i])
            time.sleep(0.15)

        self.last_move_time = time.time()

    def new_list(self, default_value):
        """
        Create a list of servo angles with default value

        :param default_value: default value of servo angles
        :type default_value: int or float
        :return: list of servo angles
        :rtype: list
        """
        _ = [default_value] * self.pin_num
        return _

    def servo_write_raw(self, angle_list):
        """
        Set servo angles to specific raw angles

        :param angle_list: list of servo angles
        :type angle_list: list
        """
        for i in range(self.pin_num):
            self.servo_list[i].angle(angle_list[i])

    def servo_write_all(self, angles):
        """
        Set servo angles to specific angles with original angle and offset

        :param angles: list of servo angles
        :type angles: list
        """
        rel_angles = []  # ralative angle to home
        for i in range(self.pin_num):
            rel_angles.append(
                self.direction[i] * (self.origin_positions[i] + angles[i] + self.offset[i]))
        self.servo_write_raw(rel_angles)

    def servo_move(self, targets, speed=50, bpm=None):
        """
        Move servo to specific angles with speed or bpm

        :param targets: list of servo angles
        :type targets: list
        :param speed: speed of servo move
        :type speed: int or float
        :param bpm: beats per minute
        :type bpm: int or float
        """
        '''
            calculate the max delta angle, multiply by 2 to define a max_step
            loop max_step times, every servo add/minus 1 when step reaches its adder_flag
        '''
        speed = max(0, speed)
        speed = min(100, speed)
        step_time = 10  # ms 
        delta = []
        absdelta = []
        max_step = 0
        steps = []
        # print(f"targets: {targets}")
        # print(f"current:{self.servo_positions}")
        # st = time.time()
        # if self.name == "legs":
        #     print(f"move_interval: {time.time() - self.last_move_time}")
        #     self.last_move_time = time.time()

        for i in range(self.pin_num):
            value = targets[i] - self.servo_positions[i]
            delta.append(value)
            absdelta.append(abs(value))

        # Calculate max delta angle
        max_delta = int(max(absdelta))
        if max_delta == 0:
            time.sleep(step_time/1000)
            return

        # Calculate total servo move time
        if bpm: # bpm: beats per minute
            total_time = 60 / bpm * 1000 # time taken per beat, unit: ms
        else:
            total_time = -9.9 * speed + 1000 # time spent in one step, unit: ms
        # print(f"Total time: {total_time} ms")

        # Calculate max dps
        current_max_dps = max_delta / total_time * 1000 # dps, degrees per second

        # If current max dps is larger than max dps, then calculate a new total servo move time
        if current_max_dps > self.max_dps:
            # print(
            #     f"Current Max DPS {current_max_dps} is too high. Max DPS is {self.max_dps}")
            # print(f"Total time: {total_time} ms")
            # print(f"Max Delta: {max_delta}")
            total_time = max_delta / self.max_dps * 1000
            # print(f"New Total time: {total_time} ms")
        # calculate max step
        max_step = int(total_time / step_time)

        # Calculate all step-angles for each servo
        for i in range(self.pin_num):
            step = float(delta[i])/max_step
            steps.append(step)

        # print(f"usage1: {time.time() - st}")
        # st = time.time()

        # print(f"max_delta: {max_delta}, max_step: {max_step}")
        for _ in range(max_step):
            start_timer = time.time()
            delay = step_time/1000

            for j in range(self.pin_num):
                self.servo_positions[j] += steps[j]
            self.servo_write_all(self.servo_positions)

            servo_move_time = time.time() - start_timer#!/usr/bin/env python3
from .basic import _Basic_class
from .pwm import PWM
from .servo import Servo
import time
from .filedb import fileDB
import os

# user and User home directory
User = None
UserHome = None
config_file = None


class Robot(_Basic_class):
    """
    Robot class

    This class is for makeing a servo robot with Robot HAT

    There are servo initialization, all servo move in specific speed. servo offset and stuff. make it easy to make a robot.
    All Pi-series robot from SunFounder use this class. Check them out for more details.

    PiSloth: https://github.com/sunfounder/pisloth

    PiArm: https://github.com/sunfounder/piarm

    PiCrawler: https://github.com/sunfounder/picrawler
    """

    move_list = {}
    """Preset actions"""

    max_dps = 428  # dps, degrees per second, genally in 4.8V : 60des/0.14s, dps = 428
    # max_dps = 500
    """Servo max Degree Per Second"""

    def __init__(self, pin_list, db=config_file, name=None, init_angles=None, init_order=None, **kwargs):
        """
        Initialize the robot class

        :param pin_list: list of pin number[0-11]
        :type pin_list: list
        :param db: config file path
        :type db: str
        :param name: robot name
        :type name: str
        :param init_angles: list of initial angles
        :type init_angles: list
        :param init_order: list of initialization order(Servos will init one by one in case of sudden huge current, pulling down the power supply voltage. default order is the pin list. in some cases, you need different order, use this parameter to set it.)
        :type init_order: list
        :type init_angles: list
        """
        super().__init__(**kwargs)
        self.servo_list = []
        self.pin_num = len(pin_list)

        if name == None:
            self.name = 'other'
        else:
            self.name = name

        self.offset_value_name = f"{self.name}_servo_offset_list"
        # offset
        self.db = fileDB(db=db, mode='774', owner=User)
        temp = self.db.get(self.offset_value_name,
                           default_value=str(self.new_list(0)))
        temp = [float(i.strip()) for i in temp.strip("[]").split(",")]
        self.offset = temp

        # parameter init
        self.servo_positions = self.new_list(0)
        self.origin_positions = self.new_list(0)
        self.calibrate_position = self.new_list(0)
        self.direction = self.new_list(1)

        # servo init
        if None == init_angles:
            init_angles = [0]*self.pin_num
        elif len(init_angles) != self.pin_num:
            raise ValueError('init angels numbers do not match pin numbers ')

        if init_order == None:
            init_order = range(self.pin_num)

        for i, pin in enumerate(pin_list):
            self.servo_list.append(Servo(pin))
            self.servo_positions[i] = init_angles[i]
        for i in init_order:
            self.servo_list[i].angle(self.offset[i]+self.servo_positions[i])
            time.sleep(0.15)

        self.last_move_time = time.time()

    def new_list(self, default_value):
        """
        Create a list of servo angles with default value

        :param default_value: default value of servo angles
        :type default_value: int or float
        :return: list of servo angles
        :rtype: list
        """
        _ = [default_value] * self.pin_num
        return _

    def servo_write_raw(self, angle_list):
        """
        Set servo angles to specific raw angles

        :param angle_list: list of servo angles
        :type angle_list: list
        """
        for i in range(self.pin_num):
            self.servo_list[i].angle(angle_list[i])

    def servo_write_all(self, angles):
        """
        Set servo angles to specific angles with original angle and offset

        :param angles: list of servo angles
        :type angles: list
        """
        rel_angles = []  # ralative angle to home
        for i in range(self.pin_num):
            rel_angles.append(
                self.direction[i] * (self.origin_positions[i] + angles[i] + self.offset[i]))
        self.servo_write_raw(rel_angles)

    def servo_move(self, targets, speed=50, bpm=None):
        """
        Move servo to specific angles with speed or bpm

        :param targets: list of servo angles
        :type targets: list
        :param speed: speed of servo move
        :type speed: int or float
        :param bpm: beats per minute
        :type bpm: int or float
        """
        '''
            calculate the max delta angle, multiply by 2 to define a max_step
            loop max_step times, every servo add/minus 1 when step reaches its adder_flag
        '''
        speed = max(0, speed)
        speed = min(100, speed)
        step_time = 10  # ms 
        delta = []
        absdelta = []
        max_step = 0
        steps = []
        # print(f"targets: {targets}")
        # print(f"current:{self.servo_positions}")
        # st = time.time()
        # if self.name == "legs":
        #     print(f"move_interval: {time.time() - self.last_move_time}")
        #     self.last_move_time = time.time()

        for i in range(self.pin_num):
            value = targets[i] - self.servo_positions[i]
            delta.append(value)
            absdelta.append(abs(value))

        # Calculate max delta angle
        max_delta = int(max(absdelta))
        if max_delta == 0:
            time.sleep(step_time/1000)
            return

        # Calculate total servo move time
        if bpm: # bpm: beats per minute#!/usr/bin/env python3
from .basic import _Basic_class
from .pwm import PWM
from .servo import Servo
import time
from .filedb import fileDB
import os

# user and User home directory
User = None
UserHome = None
config_file = None


class Robot(_Basic_class):
    """
    Robot class

    This class is for makeing a servo robot with Robot HAT

    There are servo initialization, all servo move in specific speed. servo offset and stuff. make it easy to make a robot.
    All Pi-series robot from SunFounder use this class. Check them out for more details.

    PiSloth: https://github.com/sunfounder/pisloth

    PiArm: https://github.com/sunfounder/piarm

    PiCrawler: https://github.com/sunfounder/picrawler
    """

    move_list = {}
    """Preset actions"""

    max_dps = 428  # dps, degrees per second, genally in 4.8V : 60des/0.14s, dps = 428
    # max_dps = 500
    """Servo max Degree Per Second"""

    def __init__(self, pin_list, db=config_file, name=None, init_angles=None, init_order=None, **kwargs):
        """
        Initialize the robot class

        :param pin_list: list of pin number[0-11]
        :type pin_list: list
        :param db: config file path
        :type db: str
        :param name: robot name
        :type name: str
        :param init_angles: list of initial angles
        :type init_angles: list
        :param init_order: list of initialization order(Servos will init one by one in case of sudden huge current, pulling down the power supply voltage. default order is the pin list. in some cases, you need different order, use this parameter to set it.)
        :type init_order: list
        :type init_angles: list
        """
        super().__init__(**kwargs)
        self.servo_list = []
        self.pin_num = len(pin_list)

        if name == None:
            self.name = 'other'
        else:
            self.name = name

        self.offset_value_name = f"{self.name}_servo_offset_list"
        # offset
        self.db = fileDB(db=db, mode='774', owner=User)
        temp = self.db.get(self.offset_value_name,
                           default_value=str(self.new_list(0)))
        temp = [float(i.strip()) for i in temp.strip("[]").split(",")]
        self.offset = temp

        # parameter init
        self.servo_positions = self.new_list(0)
        self.origin_positions = self.new_list(0)
        self.calibrate_position = self.new_list(0)
        self.direction = self.new_list(1)

        # servo init
        if None == init_angles:
            init_angles = [0]*self.pin_num
        elif len(init_angles) != self.pin_num:
            raise ValueError('init angels numbers do not match pin numbers ')

        if init_order == None:
            init_order = range(self.pin_num)

        for i, pin in enumerate(pin_list):
            self.servo_list.append(Servo(pin))
            self.servo_positions[i] = init_angles[i]
        for i in init_order:
            self.servo_list[i].angle(self.offset[i]+self.servo_positions[i])
            time.sleep(0.15)

        self.last_move_time = time.time()

    def new_list(self, default_value):
        """
        Create a list of servo angles with default value

        :param default_value: default value of servo angles
        :type default_value: int or float
        :return: list of servo angles
        :rtype: list
        """
        _ = [default_value] * self.pin_num
        return _

    def servo_write_raw(self, angle_list):
        """
        Set servo angles to specific raw angles

        :param angle_list: list of servo angles
        :type angle_list: list
        """
        for i in range(self.pin_num):
            self.servo_list[i].angle(angle_list[i])

    def servo_write_all(self, angles):
        """
        Set servo angles to specific angles with original angle and offset

        :param angles: list of servo angles
        :type angles: list
        """
        rel_angles = []  # ralative angle to home
        for i in range(self.pin_num):
            rel_angles.append(
                self.direction[i] * (self.origin_positions[i] + angles[i] + self.offset[i]))
        self.servo_write_raw(rel_angles)

    def servo_move(self, targets, speed=50, bpm=None):
        """
        Move servo to specific angles with speed or bpm

        :param targets: list of servo angles
        :type targets: list
        :param speed: speed of servo move
        :type speed: int or float
        :param bpm: beats per minute
        :type bpm: int or float
        """
        '''
            calculate the max delta angle, multiply by 2 to define a max_step
            loop max_step times, every servo add/minus 1 when step reaches its adder_flag
        '''
        speed = max(0, speed)
        speed = min(100, speed)
        step_time = 10  # ms 
        delta = []
        absdelta = []
        max_step = 0
        steps = []
        # print(f"targets: {targets}")
        # print(f"current:{self.servo_positions}")
        # st = time.time()
        # if self.name == "legs":
        #     print(f"move_interval: {time.time() - self.last_move_time}")
        #     self.last_move_time = time.time()

        for i in range(self.pin_num):
            value = targets[i] - self.servo_positions[i]
            delta.append(value)
            absdelta.append(abs(value))

        # Calculate max delta angle
        max_delta = int(max(absdelta))
        if max_delta == 0:
            time.sleep(step_time/1000)
            return

        # Calculate total servo move time
        if bpm: # bpm: beats per minute
            total_time = 60 / bpm * 1000 # time taken per beat, unit: ms
        else:
            total_time = -9.9 * speed + 1000 # time spent in one step, unit: ms
        # print(f"Total time: {total_time} ms")

        # Calculate max dps
        current_max_dps = max_delta / total_time * 1000 # dps, degrees per second

        # If current max dps is larger than max dps, then calculate a new total servo move time
        if current_max_dps > self.max_dps:
            # print(
            #     f"Current Max DPS {current_max_dps} is too high. Max DPS is {self.max_dps}")
            # print(f"Total time: {total_time} ms")
            # print(f"Max Delta: {max_delta}")
            total_time = max_delta / self.max_dps * 1000
            # print(f"New Total time: {total_time} ms")
        # calculate max step
        max_step = int(total_time / step_time)

        # Calculate all step-angles for each servo
        for i in range(self.pin_num):
            step = float(delta[i])/max_step
            steps.append(step)

        # print(f"usage1: {time.time() - st}")
        # st = time.time()

        # print(f"max_delta: {max_delta}, max_step: {max_step}")
        for _ in range(max_step):
            start_timer = time.time()
            delay = step_time/1000

            for j in range(self.pin_num):
                self.servo_positions[j] += steps[j]
            self.servo_write_all(self.servo_positions)

            servo_move_time = time.time() - start_timer
            # print(f"Servo move: {servo_move_time}")
            delay = delay - servo_move_time
            delay = max(0, delay)
            time.sleep(delay)
            # _dealy_start = time.time()
            # if delay > 0:
            #     while (time.time() - _dealy_start < delay):
            #         pass
        # print(f"usage2: {time.time() - st}, max_steps: {max_step}")

    def do_action(self, motion_name, step=1, speed=50):
        """
        Do prefix action with motion_name and step and speed

        :param motion_name: motion
        :type motion_name: str
        :param step: step of motion
        :type step: int#!/usr/bin/env python3
from .basic import _Basic_class
from .pwm import PWM
from .servo import Servo
import time
from .filedb import fileDB
import os

# user and User home directory
User = None
UserHome = None
config_file = None


class Robot(_Basic_class):
    """
    Robot class

    This class is for makeing a servo robot with Robot HAT

    There are servo initialization, all servo move in specific speed. servo offset and stuff. make it easy to make a robot.
    All Pi-series robot from SunFounder use this class. Check them out for more details.

    PiSloth: https://github.com/sunfounder/pisloth

    PiArm: https://github.com/sunfounder/piarm

    PiCrawler: https://github.com/sunfounder/picrawler
    """

    move_list = {}
    """Preset actions"""

    max_dps = 428  # dps, degrees per second, genally in 4.8V : 60des/0.14s, dps = 428
    # max_dps = 500
    """Servo max Degree Per Second"""

    def __init__(self, pin_list, db=config_file, name=None, init_angles=None, init_order=None, **kwargs):
        """
        Initialize the robot class

        :param pin_list: list of pin number[0-11]
        :type pin_list: list
        :param db: config file path
        :type db: str
        :param name: robot name
        :type name: str
        :param init_angles: list of initial angles
        :type init_angles: list
        :param init_order: list of initialization order(Servos will init one by one in case of sudden huge current, pulling down the power supply voltage. default order is the pin list. in some cases, you need different order, use this parameter to set it.)
        :type init_order: list
        :type init_angles: list
        """
        super().__init__(**kwargs)
        self.servo_list = []
        self.pin_num = len(pin_list)

        if name == None:
            self.name = 'other'
        else:
            self.name = name

        self.offset_value_name = f"{self.name}_servo_offset_list"
        # offset
        self.db = fileDB(db=db, mode='774', owner=User)
        temp = self.db.get(self.offset_value_name,
                           default_value=str(self.new_list(0)))
        temp = [float(i.strip()) for i in temp.strip("[]").split(",")]
        self.offset = temp

        # parameter init
        self.servo_positions = self.new_list(0)
        self.origin_positions = self.new_list(0)
        self.calibrate_position = self.new_list(0)
        self.direction = self.new_list(1)

        # servo init
        if None == init_angles:
            init_angles = [0]*self.pin_num
        elif len(init_angles) != self.pin_num:
            raise ValueError('init angels numbers do not match pin numbers ')

        if init_order == None:
            init_order = range(self.pin_num)

        for i, pin in enumerate(pin_list):
            self.servo_list.append(Servo(pin))
            self.servo_positions[i] = init_angles[i]
        for i in init_order:
            self.servo_list[i].angle(self.offset[i]+self.servo_positions[i])
            time.sleep(0.15)

        self.last_move_time = time.time()

    def new_list(self, default_value):
        """
        Create a list of servo angles with default value

        :param default_value: default value of servo angles
        :type default_value: int or float
        :return: list of servo angles
        :rtype: list
        """
        _ = [default_value] * self.pin_num
        return _

    def servo_write_raw(self, angle_list):
        """
        Set servo angles to specific raw angles

        :param angle_list: list of servo angles
        :type angle_list: list
        """
        for i in range(self.pin_num):
            self.servo_list[i].angle(angle_list[i])

    def servo_write_all(self, angles):
        """
        Set servo angles to specific angles with original angle and offset

        :param angles: list of servo angles
        :type angles: list
        """
        rel_angles = []  # ralative angle to home
        for i in range(self.pin_num):
            rel_angles.append(
                self.direction[i] * (self.origin_positions[i] + angles[i] + self.offset[i]))
        self.servo_write_raw(rel_angles)

    def servo_move(self, targets, speed=50, bpm=None):
        """
        Move servo to specific angles with speed or bpm

        :param targets: list of servo angles
        :type targets: list
        :param speed: speed of servo move
        :type speed: int or float
        :param bpm: beats per minute
        :type bpm: int or float
        """
        '''
            calculate the max delta angle, multiply by 2 to define a max_step
            loop max_step times, every servo add/minus 1 when step reaches its adder_flag
        '''
        speed = max(0, speed)
        speed = min(100, speed)
        step_time = 10  # ms 
        delta = []
        absdelta = []
        max_step = 0
        steps = []
        # print(f"targets: {targets}")
        # print(f"current:{self.servo_positions}")
        # st = time.time()
        # if self.name == "legs":
        #     print(f"move_interval: {time.time() - self.last_move_time}")
        #     self.last_move_time = time.time()

        for i in range(self.pin_num):
            value = targets[i] - self.servo_positions[i]
            delta.append(value)
            absdelta.append(abs(value))

        # Calculate max delta angle
        max_delta = int(max(absdelta))
        if max_delta == 0:
            time.sleep(step_time/1000)
            return

        # Calculate total servo move time
        if bpm: # bpm: beats per minute
            total_time = 60 / bpm * 1000 # time taken per beat, unit: ms
        else:
            total_time = -9.9 * speed + 1000 # time spent in one step, unit: ms
        # print(f"Total time: {total_time} ms")

        # Calculate max dps
        current_max_dps = max_delta / total_time * 1000 # dps, degrees per second

        # If current max dps is larger than max dps, then calculate a new total servo move time
        if current_max_dps > self.max_dps:
            # print(
            #     f"Current Max DPS {current_max_dps} is too high. Max DPS is {self.max_dps}")
            # print(f"Total time: {total_time} ms")
            # print(f"Max Delta: {max_delta}")
            total_time = max_delta / self.max_dps * 1000
            # print(f"New Total time: {total_time} ms")
        # calculate max step
        max_step = int(total_time / step_time)

        # Calculate all step-angles for each servo
        for i in range(self.pin_num):
            step = float(delta[i])/max_step
            steps.append(step)

        # print(f"usage1: {time.time() - st}")
        # st = time.time()

        # print(f"max_delta: {max_delta}, max_step: {max_step}")
        for _ in range(max_step):
            start_timer = time.time()
            delay = step_time/1000

            for j in range(self.pin_num):
                self.servo_positions[j] += steps[j]
            self.servo_write_all(self.servo_positions)

            servo_move_time = time.time() - start_timer
            # print(f"Servo move: {servo_move_time}")
            delay = delay - servo_move_time
            delay = max(0, delay)
            time.sleep(delay)
            # _dealy_start = time.time()
            # if delay > 0:
            #     while (time.time() - _dealy_start < delay):
            #         pass
        # print(f"usage2: {time.time() - st}, max_steps: {max_step}")

    def do_action(self, motion_name, step=1, speed=50):
        """
        Do prefix action with motion_name and step and speed

        :param motion_name: motion
        :type motion_name: str
        :param step: step of motion
        :type step: int
        :param speed: speed of motion
        :type speed: int or float
        """
        for _ in range(step):
            for motion in self.move_list[motion_name]:
                self.servo_move(motion, speed)

    def set_offset(self, offset_list):
        """
        Set offset of servo angles
#!/usr/bin/env python3
from .basic import _Basic_class
from .pwm import PWM
from .servo import Servo
import time
from .filedb import fileDB
import os

# user and User home directory
User = None
UserHome = None
config_file = None


class Robot(_Basic_class):
    """
    Robot class

    This class is for makeing a servo robot with Robot HAT

    There are servo initialization, all servo move in specific speed. servo offset and stuff. make it easy to make a robot.
    All Pi-series robot from SunFounder use this class. Check them out for more details.

    PiSloth: https://github.com/sunfounder/pisloth

    PiArm: https://github.com/sunfounder/piarm

    PiCrawler: https://github.com/sunfounder/picrawler
    """

    move_list = {}
    """Preset actions"""

    max_dps = 428  # dps, degrees per second, genally in 4.8V : 60des/0.14s, dps = 428
    # max_dps = 500
    """Servo max Degree Per Second"""

    def __init__(self, pin_list, db=config_file, name=None, init_angles=None, init_order=None, **kwargs):
        """
        Initialize the robot class

        :param pin_list: list of pin number[0-11]
        :type pin_list: list
        :param db: config file path
        :type db: str
        :param name: robot name
        :type name: str
        :param init_angles: list of initial angles
        :type init_angles: list
        :param init_order: list of initialization order(Servos will init one by one in case of sudden huge current, pulling down the power supply voltage. default order is the pin list. in some cases, you need different order, use this parameter to set it.)
        :type init_order: list
        :type init_angles: list
        """
        super().__init__(**kwargs)
        self.servo_list = []
        self.pin_num = len(pin_list)

        if name == None:
            self.name = 'other'
        else:
            self.name = name

        self.offset_value_name = f"{self.name}_servo_offset_list"
        # offset
        self.db = fileDB(db=db, mode='774', owner=User)
        temp = self.db.get(self.offset_value_name,
                           default_value=str(self.new_list(0)))
        temp = [float(i.strip()) for i in temp.strip("[]").split(",")]
        self.offset = temp

        # parameter init
        self.servo_positions = self.new_list(0)
        self.origin_positions = self.new_list(0)
        self.calibrate_position = self.new_list(0)
        self.direction = self.new_list(1)

        # servo init
        if None == init_angles:
            init_angles = [0]*self.pin_num
        elif len(init_angles) != self.pin_num:
            raise ValueError('init angels numbers do not match pin numbers ')

        if init_order == None:
            init_order = range(self.pin_num)

        for i, pin in enumerate(pin_list):
            self.servo_list.append(Servo(pin))
            self.servo_positions[i] = init_angles[i]
        for i in init_order:
            self.servo_list[i].angle(self.offset[i]+self.servo_positions[i])
            time.sleep(0.15)

        self.last_move_time = time.time()

    def new_list(self, default_value):
        """
        Create a list of servo angles with default value

        :param default_value: default value of servo angles
        :type default_value: int or float
        :return: list of servo angles
        :rtype: list
        """
        _ = [default_value] * self.pin_num
        return _

    def servo_write_raw(self, angle_list):
        """
        Set servo angles to specific raw angles

        :param angle_list: list of servo angles
        :type angle_list: list
        """
        for i in range(self.pin_num):
            self.servo_list[i].angle(angle_list[i])

    def servo_write_all(self, angles):
        """
        Set servo angles to specific angles with original angle and offset

        :param angles: list of servo angles
        :type angles: list
        """
        rel_angles = []  # ralative angle to home
        for i in range(self.pin_num):
            rel_angles.append(
                self.direction[i] * (self.origin_positions[i] + angles[i] + self.offset[i]))
        self.servo_write_raw(rel_angles)

    def servo_move(self, targets, speed=50, bpm=None):
        """
        Move servo to specific angles with speed or bpm

        :param targets: list of servo angles
        :type targets: list
        :param speed: speed of servo move
        :type speed: int or float
        :param bpm: beats per minute
        :type bpm: int or float
        """
        '''
            calculate the max delta angle, multiply by 2 to define a max_step
            loop max_step times, every servo add/minus 1 when step reaches its adder_flag
        '''
        speed = max(0, speed)
        speed = min(100, speed)
        step_time = 10  # ms 
        delta = []
        absdelta = []
        max_step = 0
        steps = []
        # print(f"targets: {targets}")
        # print(f"current:{self.servo_positions}")
        # st = time.time()
        # if self.name == "legs":
        #     print(f"move_interval: {time.time() - self.last_move_time}")
        #     self.last_move_time = time.time()

        for i in range(self.pin_num):
            value = targets[i] - self.servo_positions[i]
            delta.append(value)
            absdelta.append(abs(value))

        # Calculate max delta angle
        max_delta = int(max(absdelta))
        if max_delta == 0:
            time.sleep(step_time/1000)
            return

        # Calculate total servo move time
        if bpm: # bpm: beats per minute
            total_time = 60 / bpm * 1000 # time taken per beat, unit: ms
        else:
            total_time = -9.9 * speed + 1000 # time spent in one step, unit: ms
        # print(f"Total time: {total_time} ms")

        # Calculate max dps
        current_max_dps = max_delta / total_time * 1000 # dps, degrees per second

        # If current max dps is larger than max dps, then calculate a new total servo move time
        if current_max_dps > self.max_dps:
            # print(
            #     f"Current Max DPS {current_max_dps} is too high. Max DPS is {self.max_dps}")
            # print(f"Total time: {total_time} ms")
            # print(f"Max Delta: {max_delta}")
            total_time = max_delta / self.max_dps * 1000
            # print(f"New Total time: {total_time} ms")
        # calculate max step
        max_step = int(total_time / step_time)

        # Calculate all step-angles for each servo
        for i in range(self.pin_num):
            step = float(delta[i])/max_step
            steps.append(step)

        # print(f"usage1: {time.time() - st}")
        # st = time.time()

        # print(f"max_delta: {max_delta}, max_step: {max_step}")
        for _ in range(max_step):
            start_timer = time.time()
            delay = step_time/1000

            for j in range(self.pin_num):
                self.servo_positions[j] += steps[j]
            self.servo_write_all(self.servo_positions)

            servo_move_time = time.time() - start_timer
            # print(f"Servo move: {servo_move_time}")
            delay = delay - servo_move_time
            delay = max(0, delay)
            time.sleep(delay)
            # _dealy_start = time.time()
            # if delay > 0:
            #     while (time.time() - _dealy_start < delay):
            #         pass
        # print(f"usage2: {time.time() - st}, max_steps: {max_step}")

    def do_action(self, motion_name, step=1, speed=50):
        """
        Do prefix action with motion_name and step and speed

        :param motion_name: motion
        :type motion_name: str
        :param step: step of motion
        :type step: int
        :param speed: speed of motion
        :type speed: int or float
        """
        for _ in range(step):
            for motion in self.move_list[motion_name]:
                self.servo_move(motion, speed)

    def set_offset(self, offset_list):#!/usr/bin/env python3
from .basic import _Basic_class
from .pwm import PWM
from .servo import Servo
import time
from .filedb import fileDB
import os

# user and User home directory
User = None
UserHome = None
config_file = None


class Robot(_Basic_class):
    """
    Robot class

    This class is for makeing a servo robot with Robot HAT

    There are servo initialization, all servo move in specific speed. servo offset and stuff. make it easy to make a robot.
    All Pi-series robot from SunFounder use this class. Check them out for more details.

    PiSloth: https://github.com/sunfounder/pisloth

    PiArm: https://github.com/sunfounder/piarm

    PiCrawler: https://github.com/sunfounder/picrawler
    """

    move_list = {}
    """Preset actions"""

    max_dps = 428  # dps, degrees per second, genally in 4.8V : 60des/0.14s, dps = 428
    # max_dps = 500
    """Servo max Degree Per Second"""

    def __init__(self, pin_list, db=config_file, name=None, init_angles=None, init_order=None, **kwargs):
        """
        Initialize the robot class

        :param pin_list: list of pin number[0-11]
        :type pin_list: list
        :param db: config file path
        :type db: str
        :param name: robot name
        :type name: str
        :param init_angles: list of initial angles
        :type init_angles: list
        :param init_order: list of initialization order(Servos will init one by one in case of sudden huge current, pulling down the power supply voltage. default order is the pin list. in some cases, you need different order, use this parameter to set it.)
        :type init_order: list
        :type init_angles: list
        """
        super().__init__(**kwargs)
        self.servo_list = []
        self.pin_num = len(pin_list)

        if name == None:
            self.name = 'other'
        else:
            self.name = name

        self.offset_value_name = f"{self.name}_servo_offset_list"
        # offset
        self.db = fileDB(db=db, mode='774', owner=User)
        temp = self.db.get(self.offset_value_name,
                           default_value=str(self.new_list(0)))
        temp = [float(i.strip()) for i in temp.strip("[]").split(",")]
        self.offset = temp

        # parameter init
        self.servo_positions = self.new_list(0)
        self.origin_positions = self.new_list(0)
        self.calibrate_position = self.new_list(0)
        self.direction = self.new_list(1)

        # servo init
        if None == init_angles:
            init_angles = [0]*self.pin_num
        elif len(init_angles) != self.pin_num:
            raise ValueError('init angels numbers do not match pin numbers ')

        if init_order == None:
            init_order = range(self.pin_num)

        for i, pin in enumerate(pin_list):
            self.servo_list.append(Servo(pin))
            self.servo_positions[i] = init_angles[i]
        for i in init_order:
            self.servo_list[i].angle(self.offset[i]+self.servo_positions[i])
            time.sleep(0.15)

        self.last_move_time = time.time()

    def new_list(self, default_value):
        """
        Create a list of servo angles with default value

        :param default_value: default value of servo angles
        :type default_value: int or float
        :return: list of servo angles
        :rtype: list
        """
        _ = [default_value] * self.pin_num
        return _

    def servo_write_raw(self, angle_list):
        """
        Set servo angles to specific raw angles

        :param angle_list: list of servo angles
        :type angle_list: list
        """
        for i in range(self.pin_num):
            self.servo_list[i].angle(angle_list[i])

    def servo_write_all(self, angles):
        """
        Set servo angles to specific angles with original angle and offset

        :param angles: list of servo angles
        :type angles: list
        """
        rel_angles = []  # ralative angle to home
        for i in range(self.pin_num):
            rel_angles.append(
                self.direction[i] * (self.origin_positions[i] + angles[i] + self.offset[i]))
        self.servo_write_raw(rel_angles)

    def servo_move(self, targets, speed=50, bpm=None):
        """
        Move servo to specific angles with speed or bpm

        :param targets: list of servo angles
        :type targets: list
        :param speed: speed of servo move
        :type speed: int or float
        :param bpm: beats per minute
        :type bpm: int or float
        """
        '''
            calculate the max delta angle, multiply by 2 to define a max_step
            loop max_step times, every servo add/minus 1 when step reaches its adder_flag
        '''
        speed = max(0, speed)
        speed = min(100, speed)
        step_time = 10  # ms 
        delta = []
        absdelta = []
        max_step = 0
        steps = []
        # print(f"targets: {targets}")
        # print(f"current:{self.servo_positions}")
        # st = time.time()
        # if self.name == "legs":
        #     print(f"move_interval: {time.time() - self.last_move_time}")
        #     self.last_move_time = time.time()

        for i in range(self.pin_num):
            value = targets[i] - self.servo_positions[i]
            delta.append(value)
            absdelta.append(abs(value))

        # Calculate max delta angle
        max_delta = int(max(absdelta))
        if max_delta == 0:
            time.sleep(step_time/1000)
            return

        # Calculate total servo move time
        if bpm: # bpm: beats per minute
            total_time = 60 / bpm * 1000 # time taken per beat, unit: ms
        else:
            total_time = -9.9 * speed + 1000 # time spent in one step, unit: ms
        # print(f"Total time: {total_time} ms")

        # Calculate max dps
        current_max_dps = max_delta / total_time * 1000 # dps, degrees per second

        # If current max dps is larger than max dps, then calculate a new total servo move time
        if current_max_dps > self.max_dps:
            # print(
            #     f"Current Max DPS {current_max_dps} is too high. Max DPS is {self.max_dps}")
            # print(f"Total time: {total_time} ms")
            # print(f"Max Delta: {max_delta}")
            total_time = max_delta / self.max_dps * 1000
            # print(f"New Total time: {total_time} ms")
        # calculate max step
        max_step = int(total_time / step_time)

        # Calculate all step-angles for each servo
        for i in range(self.pin_num):
            step = float(delta[i])/max_step
            steps.append(step)

        # print(f"usage1: {time.time() - st}")
        # st = time.time()

        # print(f"max_delta: {max_delta}, max_step: {max_step}")
        for _ in range(max_step):
            start_timer = time.time()
            delay = step_time/1000

            for j in range(self.pin_num):
                self.servo_positions[j] += steps[j]
            self.servo_write_all(self.servo_positions)

            servo_move_time = time.time() - start_timer
            # print(f"Servo move: {servo_move_time}")
            delay = delay - servo_move_time
            delay = max(0, delay)
            time.sleep(delay)
            # _dealy_start = time.time()
            # if delay > 0:
            #     while (time.time() - _dealy_start < delay):
            #         pass
        # print(f"usage2: {time.time() - st}, max_steps: {max_step}")

    def do_action(self, motion_name, step=1, speed=50):
        """
        Do prefix action with motion_name and step and speed

        :param motion_name: motion
        :type motion_name: str
        :param step: step of motion
        :type step: int
        :param speed: speed of motion
        :type speed: int or float
        """
        for _ in range(step):
            for motion in self.move_list[motion_name]:
                self.servo_move(motion, speed)

    def set_offset(self, offset_list):
        """
        Set offset of servo angles
#!/usr/bin/env python3
from .basic import _Basic_class
from .pwm import PWM
from .servo import Servo
import time
from .filedb import fileDB
import os

# user and User home directory
User = None
UserHome = None
config_file = None


class Robot(_Basic_class):
    """
    Robot class

    This class is for makeing a servo robot with Robot HAT

    There are servo initialization, all servo move in specific speed. servo offset and stuff. make it easy to make a robot.
    All Pi-series robot from SunFounder use this class. Check them out for more details.

    PiSloth: https://github.com/sunfounder/pisloth

    PiArm: https://github.com/sunfounder/piarm

    PiCrawler: https://github.com/sunfounder/picrawler
    """

    move_list = {}
    """Preset actions"""

    max_dps = 428  # dps, degrees per second, genally in 4.8V : 60des/0.14s, dps = 428
    # max_dps = 500
    """Servo max Degree Per Second"""

    def __init__(self, pin_list, db=config_file, name=None, init_angles=None, init_order=None, **kwargs):
        """
        Initialize the robot class

        :param pin_list: list of pin number[0-11]
        :type pin_list: list
        :param db: config file path
        :type db: str
        :param name: robot name
        :type name: str
        :param init_angles: list of initial angles
        :type init_angles: list
        :param init_order: list of initialization order(Servos will init one by one in case of sudden huge current, pulling down the power supply voltage. default order is the pin list. in some cases, you need different order, use this parameter to set it.)
        :type init_order: list
        :type init_angles: list
        """
        super().__init__(**kwargs)
        self.servo_list = []
        self.pin_num = len(pin_list)

        if name == None:
            self.name = 'other'
        else:
            self.name = name

        self.offset_value_name = f"{self.name}_servo_offset_list"
        # offset
        self.db = fileDB(db=db, mode='774', owner=User)
        temp = self.db.get(self.offset_value_name,
                           default_value=str(self.new_list(0)))
        temp = [float(i.strip()) for i in temp.strip("[]").split(",")]
        self.offset = temp

        # parameter init
        self.servo_positions = self.new_list(0)
        self.origin_positions = self.new_list(0)
        self.calibrate_position = self.new_list(0)
        self.direction = self.new_list(1)

        # servo init
        if None == init_angles:
            init_angles = [0]*self.pin_num
        elif len(init_angles) != self.pin_num:
            raise ValueError('init angels numbers do not match pin numbers ')

        if init_order == None:
            init_order = range(self.pin_num)

        for i, pin in enumerate(pin_list):
            self.servo_list.append(Servo(pin))
            self.servo_positions[i] = init_angles[i]
        for i in init_order:
            self.servo_list[i].angle(self.offset[i]+self.servo_positions[i])
            time.sleep(0.15)

        self.last_move_time = time.time()

    def new_list(self, default_value):
        """
        Create a list of servo angles with default value

        :param default_value: default value of servo angles
        :type default_value: int or float
        :return: list of servo angles
        :rtype: list
        """
        _ = [default_value] * self.pin_num
        return _

    def servo_write_raw(self, angle_list):
        """
        Set servo angles to specific raw angles

        :param angle_list: list of servo angles
        :type angle_list: list
        """
        for i in range(self.pin_num):
            self.servo_list[i].angle(angle_list[i])

    def servo_write_all(self, angles):
        """
        Set servo angles to specific angles with original angle and offset

        :param angles: list of servo angles
        :type angles: list
        """
        rel_angles = []  # ralative angle to home
        for i in range(self.pin_num):
            rel_angles.append(
                self.direction[i] * (self.origin_positions[i] + angles[i] + self.offset[i]))
        self.servo_write_raw(rel_angles)

    def servo_move(self, targets, speed=50, bpm=None):
        """
        Move servo to specific angles with speed or bpm

        :param targets: list of servo angles
        :type targets: list
        :param speed: speed of servo move
        :type speed: int or float
        :param bpm: beats per minute
        :type bpm: int or float
        """
        '''
            calculate the max delta angle, multiply by 2 to define a max_step
            loop max_step times, every servo add/minus 1 when step reaches its adder_flag
        '''
        speed = max(0, speed)
        speed = min(100, speed)
        step_time = 10  # ms 
        delta = []
        absdelta = []
        max_step = 0
        steps = []
        # print(f"targets: {targets}")
        # print(f"current:{self.servo_positions}")
        # st = time.time()
        # if self.name == "legs":
        #     print(f"move_interval: {time.time() - self.last_move_time}")
        #     self.last_move_time = time.time()

        for i in range(self.pin_num):
            value = targets[i] - self.servo_positions[i]
            delta.append(value)
            absdelta.append(abs(value))

        # Calculate max delta angle
        max_delta = int(max(absdelta))
        if max_delta == 0:
            time.sleep(step_time/1000)
            return

        # Calculate total servo move time
        if bpm: # bpm: beats per minute
            total_time = 60 / bpm * 1000 # time taken per beat, unit: ms
        else:
            total_time = -9.9 * speed + 1000 # time spent in one step, unit: ms
        # print(f"Total time: {total_time} ms")

        # Calculate max dps
        current_max_dps = max_delta / total_time * 1000 # dps, degrees per second

        # If current max dps is larger than max dps, then calculate a new total servo move time
        if current_max_dps > self.max_dps:
            # print(
            #     f"Current Max DPS {current_max_dps} is too high. Max DPS is {self.max_dps}")
            # print(f"Total time: {total_time} ms")
            # print(f"Max Delta: {max_delta}")
            total_time = max_delta / self.max_dps * 1000
            # print(f"New Total time: {total_time} ms")
        # calculate max step
        max_step = int(total_time / step_time)

        # Calculate all step-angles for each servo
        for i in range(self.pin_num):
            step = float(delta[i])/max_step
            steps.append(step)

        # print(f"usage1: {time.time() - st}")
        # st = time.time()

        # print(f"max_delta: {max_delta}, max_step: {max_step}")
        for _ in range(max_step):
            start_timer = time.time()
            delay = step_time/1000

            for j in range(self.pin_num):
                self.servo_positions[j] += steps[j]
            self.servo_write_all(self.servo_positions)

            servo_move_time = time.time() - start_timer
            # print(f"Servo move: {servo_move_time}")
            delay = delay - servo_move_time
            delay = max(0, delay)
            time.sleep(delay)
            # _dealy_start = time.time()
            # if delay > 0:
            #     while (time.time() - _dealy_start < delay):
            #         pass
        # print(f"usage2: {time.time() - st}, max_steps: {max_step}")

    def do_action(self, motion_name, step=1, speed=50):
        """
        Do prefix action with motion_name and step and speed

        :param motion_name: motion
        :type motion_name: str
        :param step: step of motion
        :type step: int
        :param speed: speed of motion
        :type speed: int or float
        """
        for _ in range(step):
            for motion in self.move_list[motion_name]:
                self.servo_move(motion, speed)#!/usr/bin/env python3
from .basic import _Basic_class
from .pwm import PWM
from .servo import Servo
import time
from .filedb import fileDB
import os

# user and User home directory
User = None
UserHome = None
config_file = None


class Robot(_Basic_class):
    """
    Robot class

    This class is for makeing a servo robot with Robot HAT

    There are servo initialization, all servo move in specific speed. servo offset and stuff. make it easy to make a robot.
    All Pi-series robot from SunFounder use this class. Check them out for more details.

    PiSloth: https://github.com/sunfounder/pisloth

    PiArm: https://github.com/sunfounder/piarm

    PiCrawler: https://github.com/sunfounder/picrawler
    """

    move_list = {}
    """Preset actions"""

    max_dps = 428  # dps, degrees per second, genally in 4.8V : 60des/0.14s, dps = 428
    # max_dps = 500
    """Servo max Degree Per Second"""

    def __init__(self, pin_list, db=config_file, name=None, init_angles=None, init_order=None, **kwargs):
        """
        Initialize the robot class

        :param pin_list: list of pin number[0-11]
        :type pin_list: list
        :param db: config file path
        :type db: str
        :param name: robot name
        :type name: str
        :param init_angles: list of initial angles
        :type init_angles: list
        :param init_order: list of initialization order(Servos will init one by one in case of sudden huge current, pulling down the power supply voltage. default order is the pin list. in some cases, you need different order, use this parameter to set it.)
        :type init_order: list
        :type init_angles: list
        """
        super().__init__(**kwargs)
        self.servo_list = []
        self.pin_num = len(pin_list)

        if name == None:
            self.name = 'other'
        else:
            self.name = name

        self.offset_value_name = f"{self.name}_servo_offset_list"
        # offset
        self.db = fileDB(db=db, mode='774', owner=User)
        temp = self.db.get(self.offset_value_name,
                           default_value=str(self.new_list(0)))
        temp = [float(i.strip()) for i in temp.strip("[]").split(",")]
        self.offset = temp

        # parameter init
        self.servo_positions = self.new_list(0)
        self.origin_positions = self.new_list(0)
        self.calibrate_position = self.new_list(0)
        self.direction = self.new_list(1)

        # servo init
        if None == init_angles:
            init_angles = [0]*self.pin_num
        elif len(init_angles) != self.pin_num:
            raise ValueError('init angels numbers do not match pin numbers ')

        if init_order == None:
            init_order = range(self.pin_num)

        for i, pin in enumerate(pin_list):
            self.servo_list.append(Servo(pin))
            self.servo_positions[i] = init_angles[i]
        for i in init_order:
            self.servo_list[i].angle(self.offset[i]+self.servo_positions[i])
            time.sleep(0.15)

        self.last_move_time = time.time()

    def new_list(self, default_value):
        """
        Create a list of servo angles with default value

        :param default_value: default value of servo angles
        :type default_value: int or float
        :return: list of servo angles
        :rtype: list
        """
        _ = [default_value] * self.pin_num
        return _

    def servo_write_raw(self, angle_list):
        """
        Set servo angles to specific raw angles

        :param angle_list: list of servo angles
        :type angle_list: list
        """
        for i in range(self.pin_num):
            self.servo_list[i].angle(angle_list[i])

    def servo_write_all(self, angles):
        """
        Set servo angles to specific angles with original angle and offset

        :param angles: list of servo angles
        :type angles: list
        """
        rel_angles = []  # ralative angle to home
        for i in range(self.pin_num):
            rel_angles.append(
                self.direction[i] * (self.origin_positions[i] + angles[i] + self.offset[i]))
        self.servo_write_raw(rel_angles)

    def servo_move(self, targets, speed=50, bpm=None):
        """
        Move servo to specific angles with speed or bpm

        :param targets: list of servo angles
        :type targets: list
        :param speed: speed of servo move
        :type speed: int or float
        :param bpm: beats per minute
        :type bpm: int or float
        """
        '''
            calculate the max delta angle, multiply by 2 to define a max_step
            loop max_step times, every servo add/minus 1 when step reaches its adder_flag
        '''
        speed = max(0, speed)
        speed = min(100, speed)
        step_time = 10  # ms 
        delta = []
        absdelta = []
        max_step = 0
        steps = []
        # print(f"targets: {targets}")
        # print(f"current:{self.servo_positions}")
        # st = time.time()
        # if self.name == "legs":
        #     print(f"move_interval: {time.time() - self.last_move_time}")
        #     self.last_move_time = time.time()

        for i in range(self.pin_num):
            value = targets[i] - self.servo_positions[i]
            delta.append(value)
            absdelta.append(abs(value))

        # Calculate max delta angle
        max_delta = int(max(absdelta))
        if max_delta == 0:
            time.sleep(step_time/1000)
            return

        # Calculate total servo move time
        if bpm: # bpm: beats per minute
            total_time = 60 / bpm * 1000 # time taken per beat, unit: ms
        else:
            total_time = -9.9 * speed + 1000 # time spent in one step, unit: ms
        # print(f"Total time: {total_time} ms")

        # Calculate max dps
        current_max_dps = max_delta / total_time * 1000 # dps, degrees per second

        # If current max dps is larger than max dps, then calculate a new total servo move time
        if current_max_dps > self.max_dps:
            # print(
            #     f"Current Max DPS {current_max_dps} is too high. Max DPS is {self.max_dps}")
            # print(f"Total time: {total_time} ms")
            # print(f"Max Delta: {max_delta}")
            total_time = max_delta / self.max_dps * 1000
            # print(f"New Total time: {total_time} ms")
        # calculate max step
        max_step = int(total_time / step_time)

        # Calculate all step-angles for each servo
        for i in range(self.pin_num):
            step = float(delta[i])/max_step
            steps.append(step)

        # print(f"usage1: {time.time() - st}")
        # st = time.time()

        # print(f"max_delta: {max_delta}, max_step: {max_step}")
        for _ in range(max_step):
            start_timer = time.time()
            delay = step_time/1000

            for j in range(self.pin_num):
                self.servo_positions[j] += steps[j]
            self.servo_write_all(self.servo_positions)

            servo_move_time = time.time() - start_timer
            # print(f"Servo move: {servo_move_time}")
            delay = delay - servo_move_time
            delay = max(0, delay)
            time.sleep(delay)
            # _dealy_start = time.time()
            # if delay > 0:
            #     while (time.time() - _dealy_start < delay):
            #         pass
        # print(f"usage2: {time.time() - st}, max_steps: {max_step}")

    def do_action(self, motion_name, step=1, speed=50):
        """
        Do prefix action with motion_name and step and speed

        :param motion_name: motion
        :type motion_name: str
        :param step: step of motion
        :type step: int#!/usr/bin/env python3
from .basic import _Basic_class
from .pwm import PWM
from .servo import Servo
import time
from .filedb import fileDB
import os

# user and User home directory
User = None
UserHome = None
config_file = None


class Robot(_Basic_class):
    """
    Robot class

    This class is for makeing a servo robot with Robot HAT

    There are servo initialization, all servo move in specific speed. servo offset and stuff. make it easy to make a robot.
    All Pi-series robot from SunFounder use this class. Check them out for more details.

    PiSloth: https://github.com/sunfounder/pisloth

    PiArm: https://github.com/sunfounder/piarm

    PiCrawler: https://github.com/sunfounder/picrawler
    """

    move_list = {}
    """Preset actions"""

    max_dps = 428  # dps, degrees per second, genally in 4.8V : 60des/0.14s, dps = 428
    # max_dps = 500
    """Servo max Degree Per Second"""

    def __init__(self, pin_list, db=config_file, name=None, init_angles=None, init_order=None, **kwargs):
        """
        Initialize the robot class

        :param pin_list: list of pin number[0-11]
        :type pin_list: list
        :param db: config file path
        :type db: str
        :param name: robot name
        :type name: str
        :param init_angles: list of initial angles
        :type init_angles: list
        :param init_order: list of initialization order(Servos will init one by one in case of sudden huge current, pulling down the power supply voltage. default order is the pin list. in some cases, you need different order, use this parameter to set it.)
        :type init_order: list
        :type init_angles: list
        """
        super().__init__(**kwargs)
        self.servo_list = []
        self.pin_num = len(pin_list)

        if name == None:
            self.name = 'other'
        else:
            self.name = name

        self.offset_value_name = f"{self.name}_servo_offset_list"
        # offset
        self.db = fileDB(db=db, mode='774', owner=User)
        temp = self.db.get(self.offset_value_name,
                           default_value=str(self.new_list(0)))
        temp = [float(i.strip()) for i in temp.strip("[]").split(",")]
        self.offset = temp

        # parameter init
        self.servo_positions = self.new_list(0)
        self.origin_positions = self.new_list(0)
        self.calibrate_position = self.new_list(0)
        self.direction = self.new_list(1)

        # servo init
        if None == init_angles:
            init_angles = [0]*self.pin_num
        elif len(init_angles) != self.pin_num:
            raise ValueError('init angels numbers do not match pin numbers ')

        if init_order == None:
            init_order = range(self.pin_num)

        for i, pin in enumerate(pin_list):
            self.servo_list.append(Servo(pin))
            self.servo_positions[i] = init_angles[i]
        for i in init_order:
            self.servo_list[i].angle(self.offset[i]+self.servo_positions[i])
            time.sleep(0.15)

        self.last_move_time = time.time()

    def new_list(self, default_value):
        """
        Create a list of servo angles with default value

        :param default_value: default value of servo angles
        :type default_value: int or float
        :return: list of servo angles
        :rtype: list
        """
        _ = [default_value] * self.pin_num
        return _

    def servo_write_raw(self, angle_list):
        """
        Set servo angles to specific raw angles

        :param angle_list: list of servo angles
        :type angle_list: list
        """
        for i in range(self.pin_num):
            self.servo_list[i].angle(angle_list[i])

    def servo_write_all(self, angles):
        """
        Set servo angles to specific angles with original angle and offset

        :param angles: list of servo angles
        :type angles: list
        """
        rel_angles = []  # ralative angle to home
        for i in range(self.pin_num):
            rel_angles.append(
                self.direction[i] * (self.origin_positions[i] + angles[i] + self.offset[i]))
        self.servo_write_raw(rel_angles)

    def servo_move(self, targets, speed=50, bpm=None):
        """
        Move servo to specific angles with speed or bpm

        :param targets: list of servo angles
        :type targets: list
        :param speed: speed of servo move
        :type speed: int or float
        :param bpm: beats per minute
        :type bpm: int or float
        """
        '''
            calculate the max delta angle, multiply by 2 to define a max_step
            loop max_step times, every servo add/minus 1 when step reaches its adder_flag
        '''
        speed = max(0, speed)
        speed = min(100, speed)
        step_time = 10  # ms 
        delta = []
        absdelta = []
        max_step = 0
        steps = []
        # print(f"targets: {targets}")
        # print(f"current:{self.servo_positions}")
        # st = time.time()
        # if self.name == "legs":
        #     print(f"move_interval: {time.time() - self.last_move_time}")
        #     self.last_move_time = time.time()

        for i in range(self.pin_num):
            value = targets[i] - self.servo_positions[i]
            delta.append(value)
            absdelta.append(abs(value))

        # Calculate max delta angle
        max_delta = int(max(absdelta))
        if max_delta == 0:
            time.sleep(step_time/1000)
            return

        # Calculate total servo move time
        if bpm: # bpm: beats per minute
            total_time = 60 / bpm * 1000 # time taken per beat, unit: ms
        else:
            total_time = -9.9 * speed + 1000 # time spent in one step, unit: ms
        # print(f"Total time: {total_time} ms")

        # Calculate max dps
        current_max_dps = max_delta / total_time * 1000 # dps, degrees per second

        # If current max dps is larger than max dps, then calculate a new total servo move time
        if current_max_dps > self.max_dps:
            # print(
            #     f"Current Max DPS {current_max_dps} is too high. Max DPS is {self.max_dps}")
            # print(f"Total time: {total_time} ms")
            # print(f"Max Delta: {max_delta}")
            total_time = max_delta / self.max_dps * 1000
            # print(f"New Total time: {total_time} ms")
        # calculate max step
        max_step = int(total_time / step_time)

        # Calculate all step-angles for each servo
        for i in range(self.pin_num):
            step = float(delta[i])/max_step
            steps.append(step)

        # print(f"usage1: {time.time() - st}")
        # st = time.time()

        # print(f"max_delta: {max_delta}, max_step: {max_step}")
        for _ in range(max_step):
            start_timer = time.time()
            delay = step_time/1000

            for j in range(self.pin_num):
                self.servo_positions[j] += steps[j]
            self.servo_write_all(self.servo_positions)

            servo_move_time = time.time() - start_timer
            # print(f"Servo move: {servo_move_time}")
            delay = delay - servo_move_time
            delay = max(0, delay)
            time.sleep(delay)
            # _dealy_start = time.time()
            # if delay > 0:
            #     while (time.time() - _dealy_start < delay):
            #         pass
        # print(f"usage2: {time.time() - st}, max_steps: {max_step}")

    def do_action(self, motion_name, step=1, speed=50):
        """
        Do prefix action with motion_name and step and speed

        :param motion_name: motion
        :type motion_name: str
        :param step: step of motion
        :type step: int
        :param speed: speed of motion
        :type speed: int or float
        """
        for _ in range(step):
            for motion in self.move_list[motion_name]:
                self.servo_move(motion, speed)

    def set_offset(self, offset_list):
        """
        Set offset of servo angles
#!/usr/bin/env python3
from .basic import _Basic_class
from .pwm import PWM
from .servo import Servo
import time
from .filedb import fileDB
import os

# user and User home directory
User = None
UserHome = None
config_file = None


class Robot(_Basic_class):
    """
    Robot class

    This class is for makeing a servo robot with Robot HAT

    There are servo initialization, all servo move in specific speed. servo offset and stuff. make it easy to make a robot.
    All Pi-series robot from SunFounder use this class. Check them out for more details.

    PiSloth: https://github.com/sunfounder/pisloth

    PiArm: https://github.com/sunfounder/piarm

    PiCrawler: https://github.com/sunfounder/picrawler
    """

    move_list = {}
    """Preset actions"""

    max_dps = 428  # dps, degrees per second, genally in 4.8V : 60des/0.14s, dps = 428
    # max_dps = 500
    """Servo max Degree Per Second"""

    def __init__(self, pin_list, db=config_file, name=None, init_angles=None, init_order=None, **kwargs):
        """
        Initialize the robot class

        :param pin_list: list of pin number[0-11]
        :type pin_list: list
        :param db: config file path
        :type db: str
        :param name: robot name
        :type name: str
        :param init_angles: list of initial angles
        :type init_angles: list
        :param init_order: list of initialization order(Servos will init one by one in case of sudden huge current, pulling down the power supply voltage. default order is the pin list. in some cases, you need different order, use this parameter to set it.)
        :type init_order: list
        :type init_angles: list
        """
        super().__init__(**kwargs)
        self.servo_list = []
        self.pin_num = len(pin_list)

        if name == None:
            self.name = 'other'
        else:
            self.name = name

        self.offset_value_name = f"{self.name}_servo_offset_list"
        # offset
        self.db = fileDB(db=db, mode='774', owner=User)
        temp = self.db.get(self.offset_value_name,
                           default_value=str(self.new_list(0)))
        temp = [float(i.strip()) for i in temp.strip("[]").split(",")]
        self.offset = temp

        # parameter init
        self.servo_positions = self.new_list(0)
        self.origin_positions = self.new_list(0)
        self.calibrate_position = self.new_list(0)
        self.direction = self.new_list(1)

        # servo init
        if None == init_angles:
            init_angles = [0]*self.pin_num
        elif len(init_angles) != self.pin_num:
            raise ValueError('init angels numbers do not match pin numbers ')

        if init_order == None:
            init_order = range(self.pin_num)

        for i, pin in enumerate(pin_list):
            self.servo_list.append(Servo(pin))
            self.servo_positions[i] = init_angles[i]
        for i in init_order:
            self.servo_list[i].angle(self.offset[i]+self.servo_positions[i])
            time.sleep(0.15)

        self.last_move_time = time.time()

    def new_list(self, default_value):
        """
        Create a list of servo angles with default value

        :param default_value: default value of servo angles
        :type default_value: int or float
        :return: list of servo angles
        :rtype: list
        """
        _ = [default_value] * self.pin_num
        return _

    def servo_write_raw(self, angle_list):
        """
        Set servo angles to specific raw angles

        :param angle_list: list of servo angles
        :type angle_list: list
        """
        for i in range(self.pin_num):
            self.servo_list[i].angle(angle_list[i])

    def servo_write_all(self, angles):
        """
        Set servo angles to specific angles with original angle and offset

        :param angles: list of servo angles
        :type angles: list
        """
        rel_angles = []  # ralative angle to home
        for i in range(self.pin_num):
            rel_angles.append(
                self.direction[i] * (self.origin_positions[i] + angles[i] + self.offset[i]))
        self.servo_write_raw(rel_angles)

    def servo_move(self, targets, speed=50, bpm=None):
        """
        Move servo to specific angles with speed or bpm

        :param targets: list of servo angles
        :type targets: list
        :param speed: speed of servo move
        :type speed: int or float
        :param bpm: beats per minute
        :type bpm: int or float
        """
        '''
            calculate the max delta angle, multiply by 2 to define a max_step
            loop max_step times, every servo add/minus 1 when step reaches its adder_flag
        '''
        speed = max(0, speed)
        speed = min(100, speed)
        step_time = 10  # ms 
        delta = []
        absdelta = []
        max_step = 0
        steps = []
        # print(f"targets: {targets}")
        # print(f"current:{self.servo_positions}")
        # st = time.time()
        # if self.name == "legs":
        #     print(f"move_interval: {time.time() - self.last_move_time}")
        #     self.last_move_time = time.time()

        for i in range(self.pin_num):
            value = targets[i] - self.servo_positions[i]
            delta.append(value)
            absdelta.append(abs(value))

        # Calculate max delta angle
        max_delta = int(max(absdelta))
        if max_delta == 0:
            time.sleep(step_time/1000)
            return

        # Calculate total servo move time
        if bpm: # bpm: beats per minute
            total_time = 60 / bpm * 1000 # time taken per beat, unit: ms
        else:
            total_time = -9.9 * speed + 1000 # time spent in one step, unit: ms
        # print(f"Total time: {total_time} ms")

        # Calculate max dps
        current_max_dps = max_delta / total_time * 1000 # dps, degrees per second

        # If current max dps is larger than max dps, then calculate a new total servo move time
        if current_max_dps > self.max_dps:
            # print(
            #     f"Current Max DPS {current_max_dps} is too high. Max DPS is {self.max_dps}")
            # print(f"Total time: {total_time} ms")
            # print(f"Max Delta: {max_delta}")
            total_time = max_delta / self.max_dps * 1000
            # print(f"New Total time: {total_time} ms")
        # calculate max step
        max_step = int(total_time / step_time)

        # Calculate all step-angles for each servo
        for i in range(self.pin_num):
            step = float(delta[i])/max_step
            steps.append(step)

        # print(f"usage1: {time.time() - st}")
        # st = time.time()

        # print(f"max_delta: {max_delta}, max_step: {max_step}")
        for _ in range(max_step):
            start_timer = time.time()
            delay = step_time/1000

            for j in range(self.pin_num):
                self.servo_positions[j] += steps[j]
            self.servo_write_all(self.servo_positions)

            servo_move_time = time.time() - start_timer
            # print(f"Servo move: {servo_move_time}")
            delay = delay - servo_move_time
            delay = max(0, delay)
            time.sleep(delay)
            # _dealy_start = time.time()
            # if delay > 0:
            #     while (time.time() - _dealy_start < delay):
            #         pass
        # print(f"usage2: {time.time() - st}, max_steps: {max_step}")

    def do_action(self, motion_name, step=1, speed=50):
        """
        Do prefix action with motion_name and step and speed

        :param motion_name: motion
        :type motion_name: str
        :param step: step of motion
        :type step: int
        :param speed: speed of motion
        :type speed: int or float
        """
        for _ in range(step):
            for motion in self.move_list[motion_name]:
                self.servo_move(motion, speed)

    def set_offset(self, offset_list):#!/usr/bin/env python3
from .basic import _Basic_class
from .pwm import PWM
from .servo import Servo
import time
from .filedb import fileDB
import os

# user and User home directory
User = None
UserHome = None
config_file = None


class Robot(_Basic_class):
    """
    Robot class

    This class is for makeing a servo robot with Robot HAT

    There are servo initialization, all servo move in specific speed. servo offset and stuff. make it easy to make a robot.
    All Pi-series robot from SunFounder use this class. Check them out for more details.

    PiSloth: https://github.com/sunfounder/pisloth

    PiArm: https://github.com/sunfounder/piarm

    PiCrawler: https://github.com/sunfounder/picrawler
    """

    move_list = {}
    """Preset actions"""

    max_dps = 428  # dps, degrees per second, genally in 4.8V : 60des/0.14s, dps = 428
    # max_dps = 500
    """Servo max Degree Per Second"""

    def __init__(self, pin_list, db=config_file, name=None, init_angles=None, init_order=None, **kwargs):
        """
        Initialize the robot class

        :param pin_list: list of pin number[0-11]
        :type pin_list: list
        :param db: config file path
        :type db: str
        :param name: robot name
        :type name: str
        :param init_angles: list of initial angles
        :type init_angles: list
        :param init_order: list of initialization order(Servos will init one by one in case of sudden huge current, pulling down the power supply voltage. default order is the pin list. in some cases, you need different order, use this parameter to set it.)
        :type init_order: list
        :type init_angles: list
        """
        super().__init__(**kwargs)
        self.servo_list = []
        self.pin_num = len(pin_list)

        if name == None:
            self.name = 'other'
        else:
            self.name = name

        self.offset_value_name = f"{self.name}_servo_offset_list"
        # offset
        self.db = fileDB(db=db, mode='774', owner=User)
        temp = self.db.get(self.offset_value_name,
                           default_value=str(self.new_list(0)))
        temp = [float(i.strip()) for i in temp.strip("[]").split(",")]
        self.offset = temp

        # parameter init
        self.servo_positions = self.new_list(0)
        self.origin_positions = self.new_list(0)
        self.calibrate_position = self.new_list(0)
        self.direction = self.new_list(1)

        # servo init
        if None == init_angles:
            init_angles = [0]*self.pin_num
        elif len(init_angles) != self.pin_num:
            raise ValueError('init angels numbers do not match pin numbers ')

        if init_order == None:
            init_order = range(self.pin_num)

        for i, pin in enumerate(pin_list):
            self.servo_list.append(Servo(pin))
            self.servo_positions[i] = init_angles[i]
        for i in init_order:
            self.servo_list[i].angle(self.offset[i]+self.servo_positions[i])
            time.sleep(0.15)

        self.last_move_time = time.time()

    def new_list(self, default_value):
        """
        Create a list of servo angles with default value

        :param default_value: default value of servo angles
        :type default_value: int or float
        :return: list of servo angles
        :rtype: list
        """
        _ = [default_value] * self.pin_num
        return _

    def servo_write_raw(self, angle_list):
        """
        Set servo angles to specific raw angles

        :param angle_list: list of servo angles
        :type angle_list: list
        """
        for i in range(self.pin_num):
            self.servo_list[i].angle(angle_list[i])

    def servo_write_all(self, angles):
        """
        Set servo angles to specific angles with original angle and offset

        :param angles: list of servo angles
        :type angles: list
        """
        rel_angles = []  # ralative angle to home
        for i in range(self.pin_num):
            rel_angles.append(
                self.direction[i] * (self.origin_positions[i] + angles[i] + self.offset[i]))
        self.servo_write_raw(rel_angles)

    def servo_move(self, targets, speed=50, bpm=None):
        """
        Move servo to specific angles with speed or bpm

        :param targets: list of servo angles
        :type targets: list
        :param speed: speed of servo move
        :type speed: int or float
        :param bpm: beats per minute
        :type bpm: int or float
        """
        '''
            calculate the max delta angle, multiply by 2 to define a max_step
            loop max_step times, every servo add/minus 1 when step reaches its adder_flag
        '''
        speed = max(0, speed)
        speed = min(100, speed)
        step_time = 10  # ms 
        delta = []
        absdelta = []
        max_step = 0
        steps = []
        # print(f"targets: {targets}")
        # print(f"current:{self.servo_positions}")
        # st = time.time()
        # if self.name == "legs":
        #     print(f"move_interval: {time.time() - self.last_move_time}")
        #     self.last_move_time = time.time()

        for i in range(self.pin_num):
            value = targets[i] - self.servo_positions[i]
            delta.append(value)
            absdelta.append(abs(value))

        # Calculate max delta angle
        max_delta = int(max(absdelta))
        if max_delta == 0:
            time.sleep(step_time/1000)
            return

        # Calculate total servo move time
        if bpm: # bpm: beats per minute
            total_time = 60 / bpm * 1000 # time taken per beat, unit: ms
        else:
            total_time = -9.9 * speed + 1000 # time spent in one step, unit: ms
        # print(f"Total time: {total_time} ms")

        # Calculate max dps
        current_max_dps = max_delta / total_time * 1000 # dps, degrees per second

        # If current max dps is larger than max dps, then calculate a new total servo move time
        if current_max_dps > self.max_dps:
            # print(
            #     f"Current Max DPS {current_max_dps} is too high. Max DPS is {self.max_dps}")
            # print(f"Total time: {total_time} ms")
            # print(f"Max Delta: {max_delta}")
            total_time = max_delta / self.max_dps * 1000
            # print(f"New Total time: {total_time} ms")
        # calculate max step
        max_step = int(total_time / step_time)

        # Calculate all step-angles for each servo
        for i in range(self.pin_num):
            step = float(delta[i])/max_step
            steps.append(step)

        # print(f"usage1: {time.time() - st}")
        # st = time.time()

        # print(f"max_delta: {max_delta}, max_step: {max_step}")
        for _ in range(max_step):
            start_timer = time.time()
            delay = step_time/1000

            for j in range(self.pin_num):
                self.servo_positions[j] += steps[j]
            self.servo_write_all(self.servo_positions)

            servo_move_time = time.time() - start_timer
            # print(f"Servo move: {servo_move_time}")
            delay = delay - servo_move_time
            delay = max(0, delay)
            time.sleep(delay)
            # _dealy_start = time.time()
            # if delay > 0:
            #     while (time.time() - _dealy_start < delay):
            #         pass
        # print(f"usage2: {time.time() - st}, max_steps: {max_step}")

    def do_action(self, motion_name, step=1, speed=50):
        """
        Do prefix action with motion_name and step and speed

        :param motion_name: motion
        :type motion_name: str#!/usr/bin/env python3
from .basic import _Basic_class
from .pwm import PWM
from .servo import Servo
import time
from .filedb import fileDB
import os

# user and User home directory
User = None
UserHome = None
config_file = None


class Robot(_Basic_class):
    """
    Robot class

    This class is for makeing a servo robot with Robot HAT

    There are servo initialization, all servo move in specific speed. servo offset and stuff. make it easy to make a robot.
    All Pi-series robot from SunFounder use this class. Check them out for more details.

    PiSloth: https://github.com/sunfounder/pisloth

    PiArm: https://github.com/sunfounder/piarm

    PiCrawler: https://github.com/sunfounder/picrawler
    """

    move_list = {}
    """Preset actions"""

    max_dps = 428  # dps, degrees per second, genally in 4.8V : 60des/0.14s, dps = 428
    # max_dps = 500
    """Servo max Degree Per Second"""

    def __init__(self, pin_list, db=config_file, name=None, init_angles=None, init_order=None, **kwargs):
        """
        Initialize the robot class

        :param pin_list: list of pin number[0-11]
        :type pin_list: list
        :param db: config file path
        :type db: str
        :param name: robot name
        :type name: str
        :param init_angles: list of initial angles
        :type init_angles: list
        :param init_order: list of initialization order(Servos will init one by one in case of sudden huge current, pulling down the power supply voltage. default order is the pin list. in some cases, you need different order, use this parameter to set it.)
        :type init_order: list
        :type init_angles: list
        """
        super().__init__(**kwargs)
        self.servo_list = []
        self.pin_num = len(pin_list)

        if name == None:
            self.name = 'other'
        else:
            self.name = name

        self.offset_value_name = f"{self.name}_servo_offset_list"
        # offset
        self.db = fileDB(db=db, mode='774', owner=User)
        temp = self.db.get(self.offset_value_name,
                           default_value=str(self.new_list(0)))
        temp = [float(i.strip()) for i in temp.strip("[]").split(",")]
        self.offset = temp

        # parameter init
        self.servo_positions = self.new_list(0)
        self.origin_positions = self.new_list(0)
        self.calibrate_position = self.new_list(0)
        self.direction = self.new_list(1)

        # servo init
        if None == init_angles:
            init_angles = [0]*self.pin_num
        elif len(init_angles) != self.pin_num:
            raise ValueError('init angels numbers do not match pin numbers ')

        if init_order == None:
            init_order = range(self.pin_num)

        for i, pin in enumerate(pin_list):
            self.servo_list.append(Servo(pin))
            self.servo_positions[i] = init_angles[i]
        for i in init_order:
            self.servo_list[i].angle(self.offset[i]+self.servo_positions[i])
            time.sleep(0.15)

        self.last_move_time = time.time()

    def new_list(self, default_value):
        """
        Create a list of servo angles with default value

        :param default_value: default value of servo angles
        :type default_value: int or float
        :return: list of servo angles
        :rtype: list
        """
        _ = [default_value] * self.pin_num
        return _

    def servo_write_raw(self, angle_list):
        """
        Set servo angles to specific raw angles

        :param angle_list: list of servo angles
        :type angle_list: list
        """
        for i in range(self.pin_num):
            self.servo_list[i].angle(angle_list[i])

    def servo_write_all(self, angles):
        """
        Set servo angles to specific angles with original angle and offset

        :param angles: list of servo angles
        :type angles: list
        """
        rel_angles = []  # ralative angle to home
        for i in range(self.pin_num):
            rel_angles.append(
                self.direction[i] * (self.origin_positions[i] + angles[i] + self.offset[i]))
        self.servo_write_raw(rel_angles)

    def servo_move(self, targets, speed=50, bpm=None):
        """
        Move servo to specific angles with speed or bpm

        :param targets: list of servo angles
        :type targets: list
        :param speed: speed of servo move
        :type speed: int or float
        :param bpm: beats per minute
        :type bpm: int or float
        """
        '''
            calculate the max delta angle, multiply by 2 to define a max_step
            loop max_step times, every servo add/minus 1 when step reaches its adder_flag
        '''
        speed = max(0, speed)
        speed = min(100, speed)
        step_time = 10  # ms 
        delta = []
        absdelta = []
        max_step = 0
        steps = []
        # print(f"targets: {targets}")
        # print(f"current:{self.servo_positions}")
        # st = time.time()
        # if self.name == "legs":
        #     print(f"move_interval: {time.time() - self.last_move_time}")
        #     self.last_move_time = time.time()

        for i in range(self.pin_num):
            value = targets[i] - self.servo_positions[i]
            delta.append(value)
            absdelta.append(abs(value))

        # Calculate max delta angle
        max_delta = int(max(absdelta))
        if max_delta == 0:
            time.sleep(step_time/1000)
            return

        # Calculate total servo move time
        if bpm: # bpm: beats per minute
            total_time = 60 / bpm * 1000 # time taken per beat, unit: ms
        else:
            total_time = -9.9 * speed + 1000 # time spent in one step, unit: ms
        # print(f"Total time: {total_time} ms")

        # Calculate max dps
        current_max_dps = max_delta / total_time * 1000 # dps, degrees per second

        # If current max dps is larger than max dps, then calculate a new total servo move time
        if current_max_dps > self.max_dps:
            # print(
            #     f"Current Max DPS {current_max_dps} is too high. Max DPS is {self.max_dps}")
            # print(f"Total time: {total_time} ms")
            # print(f"Max Delta: {max_delta}")
            total_time = max_delta / self.max_dps * 1000
            # print(f"New Total time: {total_time} ms")
        # calculate max step
        max_step = int(total_time / step_time)

        # Calculate all step-angles for each servo
        for i in range(self.pin_num):
            step = float(delta[i])/max_step
            steps.append(step)

        # print(f"usage1: {time.time() - st}")
        # st = time.time()

        # print(f"max_delta: {max_delta}, max_step: {max_step}")
        for _ in range(max_step):
            start_timer = time.time()
            delay = step_time/1000

            for j in range(self.pin_num):
                self.servo_positions[j] += steps[j]
            self.servo_write_all(self.servo_positions)

            servo_move_time = time.time() - start_timer
            # print(f"Servo move: {servo_move_time}")
            delay = delay - servo_move_time
            delay = max(0, delay)
            time.sleep(delay)
            # _dealy_start = time.time()
            # if delay > 0:
            #     while (time.time() - _dealy_start < delay):
            #         pass
        # print(f"usage2: {time.time() - st}, max_steps: {max_step}")

    def do_action(self, motion_name, step=1, speed=50):
        """
        Do prefix action with motion_name and step and speed

        :param motion_name: motion
        :type motion_name: str
        :param step: step of motion
        :type step: int
        :param speed: speed of motion
        :type speed: int or float
        """
        for _ in range(step):
            for motion in self.move_list[motion_name]:
                self.servo_move(motion, speed)

    def set_offset(self, offset_list):
        """
        Set offset of servo angles

        :param offset_list: list of servo angles
        :type offset_list: list
        """
        offset_list = [min(max(offset, -20), 20) for offset in offset_list]
        temp = str(offset_list)
        self.db.set(self.offset_value_name, temp)
        self.offset = offset_list

    def calibration(self):
        """Move all servos to home position"""
        self.servo_positions = self.calibrate_position
        self.servo_write_all(self.servo_positions)

    def reset(self, list=None):
        """Reset servo to original position"""
        if list is None:
            self.servo_positions = self.new_list(0)
            self.servo_write_all(self.servo_positions)
        else:
            self.servo_positions = list
            self.servo_write_all(self.servo_positions)

    def soft_reset(self):
        temp_list = self.new_list(0)
        self.servo_write_all(temp_list)

        :param step: step of motion
        :type step: int
        :param speed: speed of motion
        :type speed: int or float
        """
        for _ in range(step):
            for motion in self.move_list[motion_name]:
                self.servo_move(motion, speed)

    def set_offset(self, offset_list):
        """
        Set offset of servo angles

        :param offset_list: list of servo angles
        :type offset_list: list
        """
        offset_list = [min(max(offset, -20), 20) for offset in offset_list]
        temp = str(offset_list)
        self.db.set(self.offset_value_name, temp)
        self.offset = offset_list

    def calibration(self):
        """Move all servos to home position"""
        self.servo_positions = self.calibrate_position
        self.servo_write_all(self.servo_positions)

    def reset(self, list=None):
        """Reset servo to original position"""
        if list is None:
            self.servo_positions = self.new_list(0)
            self.servo_write_all(self.servo_positions)
        else:
            self.servo_positions = list
            self.servo_write_all(self.servo_positions)

    def soft_reset(self):
        temp_list = self.new_list(0)
        self.servo_write_all(temp_list)

        """
        Set offset of servo angles

        :param offset_list: list of servo angles
        :type offset_list: list
        """
        offset_list = [min(max(offset, -20), 20) for offset in offset_list]
        temp = str(offset_list)
        self.db.set(self.offset_value_name, temp)
        self.offset = offset_list

    def calibration(self):
        """Move all servos to home position"""
        self.servo_positions = self.calibrate_position
        self.servo_write_all(self.servo_positions)

    def reset(self, list=None):
        """Reset servo to original position"""
        if list is None:
            self.servo_positions = self.new_list(0)
            self.servo_write_all(self.servo_positions)
        else:
            self.servo_positions = list
            self.servo_write_all(self.servo_positions)

    def soft_reset(self):
        temp_list = self.new_list(0)
        self.servo_write_all(temp_list)

        :param offset_list: list of servo angles
        :type offset_list: list
        """
        offset_list = [min(max(offset, -20), 20) for offset in offset_list]
        temp = str(offset_list)
        self.db.set(self.offset_value_name, temp)
        self.offset = offset_list

    def calibration(self):
        """Move all servos to home position"""
        self.servo_positions = self.calibrate_position
        self.servo_write_all(self.servo_positions)

    def reset(self, list=None):
        """Reset servo to original position"""
        if list is None:
            self.servo_positions = self.new_list(0)
            self.servo_write_all(self.servo_positions)
        else:
            self.servo_positions = list
            self.servo_write_all(self.servo_positions)

    def soft_reset(self):
        temp_list = self.new_list(0)
        self.servo_write_all(temp_list)

        :param speed: speed of motion
        :type speed: int or float
        """
        for _ in range(step):
            for motion in self.move_list[motion_name]:
                self.servo_move(motion, speed)

    def set_offset(self, offset_list):
        """
        Set offset of servo angles

        :param offset_list: list of servo angles
        :type offset_list: list
        """
        offset_list = [min(max(offset, -20), 20) for offset in offset_list]
        temp = str(offset_list)
        self.db.set(self.offset_value_name, temp)
        self.offset = offset_list

    def calibration(self):
        """Move all servos to home position"""
        self.servo_positions = self.calibrate_position
        self.servo_write_all(self.servo_positions)

    def reset(self, list=None):
        """Reset servo to original position"""
        if list is None:
            self.servo_positions = self.new_list(0)
            self.servo_write_all(self.servo_positions)
        else:
            self.servo_positions = list
            self.servo_write_all(self.servo_positions)

    def soft_reset(self):
        temp_list = self.new_list(0)
        self.servo_write_all(temp_list)


    def set_offset(self, offset_list):
        """
        Set offset of servo angles

        :param offset_list: list of servo angles
        :type offset_list: list
        """
        offset_list = [min(max(offset, -20), 20) for offset in offset_list]
        temp = str(offset_list)
        self.db.set(self.offset_value_name, temp)
        self.offset = offset_list

    def calibration(self):
        """Move all servos to home position"""
        self.servo_positions = self.calibrate_position
        self.servo_write_all(self.servo_positions)

    def reset(self, list=None):
        """Reset servo to original position"""
        if list is None:
            self.servo_positions = self.new_list(0)
            self.servo_write_all(self.servo_positions)
        else:
            self.servo_positions = list
            self.servo_write_all(self.servo_positions)

    def soft_reset(self):
        temp_list = self.new_list(0)
        self.servo_write_all(temp_list)

        :param offset_list: list of servo angles
        :type offset_list: list
        """
        offset_list = [min(max(offset, -20), 20) for offset in offset_list]
        temp = str(offset_list)
        self.db.set(self.offset_value_name, temp)
        self.offset = offset_list

    def calibration(self):
        """Move all servos to home position"""
        self.servo_positions = self.calibrate_position
        self.servo_write_all(self.servo_positions)

    def reset(self, list=None):
        """Reset servo to original position"""
        if list is None:
            self.servo_positions = self.new_list(0)
            self.servo_write_all(self.servo_positions)
        else:
            self.servo_positions = list
            self.servo_write_all(self.servo_positions)

    def soft_reset(self):
        temp_list = self.new_list(0)
        self.servo_write_all(temp_list)

        """
        Set offset of servo angles

        :param offset_list: list of servo angles
        :type offset_list: list
        """
        offset_list = [min(max(offset, -20), 20) for offset in offset_list]
        temp = str(offset_list)
        self.db.set(self.offset_value_name, temp)
        self.offset = offset_list

    def calibration(self):
        """Move all servos to home position"""
        self.servo_positions = self.calibrate_position
        self.servo_write_all(self.servo_positions)

    def reset(self, list=None):
        """Reset servo to original position"""
        if list is None:
            self.servo_positions = self.new_list(0)
            self.servo_write_all(self.servo_positions)
        else:
            self.servo_positions = list
            self.servo_write_all(self.servo_positions)

    def soft_reset(self):
        temp_list = self.new_list(0)
        self.servo_write_all(temp_list)

        :param offset_list: list of servo angles
        :type offset_list: list
        """
        offset_list = [min(max(offset, -20), 20) for offset in offset_list]
        temp = str(offset_list)
        self.db.set(self.offset_value_name, temp)
        self.offset = offset_list

    def calibration(self):
        """Move all servos to home position"""
        self.servo_positions = self.calibrate_position
        self.servo_write_all(self.servo_positions)

    def reset(self, list=None):
        """Reset servo to original position"""
        if list is None:
            self.servo_positions = self.new_list(0)
            self.servo_write_all(self.servo_positions)
        else:
            self.servo_positions = list
            self.servo_write_all(self.servo_positions)

    def soft_reset(self):
        temp_list = self.new_list(0)
        self.servo_write_all(temp_list)

        :param speed: speed of motion
        :type speed: int or float
        """
        for _ in range(step):
            for motion in self.move_list[motion_name]:
                self.servo_move(motion, speed)

    def set_offset(self, offset_list):
        """
        Set offset of servo angles

        :param offset_list: list of servo angles
        :type offset_list: list
        """
        offset_list = [min(max(offset, -20), 20) for offset in offset_list]
        temp = str(offset_list)
        self.db.set(self.offset_value_name, temp)
        self.offset = offset_list

    def calibration(self):
        """Move all servos to home position"""
        self.servo_positions = self.calibrate_position
        self.servo_write_all(self.servo_positions)

    def reset(self, list=None):
        """Reset servo to original position"""
        if list is None:
            self.servo_positions = self.new_list(0)
            self.servo_write_all(self.servo_positions)
        else:
            self.servo_positions = list
            self.servo_write_all(self.servo_positions)

    def soft_reset(self):
        temp_list = self.new_list(0)
        self.servo_write_all(temp_list)

            total_time = 60 / bpm * 1000 # time taken per beat, unit: ms
        else:
            total_time = -9.9 * speed + 1000 # time spent in one step, unit: ms
        # print(f"Total time: {total_time} ms")

        # Calculate max dps
        current_max_dps = max_delta / total_time * 1000 # dps, degrees per second

        # If current max dps is larger than max dps, then calculate a new total servo move time
        if current_max_dps > self.max_dps:
            # print(
            #     f"Current Max DPS {current_max_dps} is too high. Max DPS is {self.max_dps}")
            # print(f"Total time: {total_time} ms")
            # print(f"Max Delta: {max_delta}")
            total_time = max_delta / self.max_dps * 1000
            # print(f"New Total time: {total_time} ms")
        # calculate max step
        max_step = int(total_time / step_time)

        # Calculate all step-angles for each servo
        for i in range(self.pin_num):
            step = float(delta[i])/max_step
            steps.append(step)

        # print(f"usage1: {time.time() - st}")
        # st = time.time()

        # print(f"max_delta: {max_delta}, max_step: {max_step}")
        for _ in range(max_step):
            start_timer = time.time()
            delay = step_time/1000

            for j in range(self.pin_num):
                self.servo_positions[j] += steps[j]
            self.servo_write_all(self.servo_positions)

            servo_move_time = time.time() - start_timer
            # print(f"Servo move: {servo_move_time}")
            delay = delay - servo_move_time
            delay = max(0, delay)
            time.sleep(delay)
            # _dealy_start = time.time()
            # if delay > 0:
            #     while (time.time() - _dealy_start < delay):
            #         pass
        # print(f"usage2: {time.time() - st}, max_steps: {max_step}")

    def do_action(self, motion_name, step=1, speed=50):
        """
        Do prefix action with motion_name and step and speed

        :param motion_name: motion
        :type motion_name: str
        :param step: step of motion
        :type step: int
        :param speed: speed of motion
        :type speed: int or float
        """
        for _ in range(step):
            for motion in self.move_list[motion_name]:
                self.servo_move(motion, speed)

    def set_offset(self, offset_list):
        """
        Set offset of servo angles

        :param offset_list: list of servo angles
        :type offset_list: list
        """
        offset_list = [min(max(offset, -20), 20) for offset in offset_list]
        temp = str(offset_list)
        self.db.set(self.offset_value_name, temp)
        self.offset = offset_list

    def calibration(self):
        """Move all servos to home position"""
        self.servo_positions = self.calibrate_position
        self.servo_write_all(self.servo_positions)

    def reset(self, list=None):
        """Reset servo to original position"""
        if list is None:
            self.servo_positions = self.new_list(0)
            self.servo_write_all(self.servo_positions)
        else:
            self.servo_positions = list
            self.servo_write_all(self.servo_positions)

    def soft_reset(self):
        temp_list = self.new_list(0)
        self.servo_write_all(temp_list)

            # print(f"Servo move: {servo_move_time}")
            delay = delay - servo_move_time
            delay = max(0, delay)
            time.sleep(delay)
            # _dealy_start = time.time()
            # if delay > 0:
            #     while (time.time() - _dealy_start < delay):
            #         pass
        # print(f"usage2: {time.time() - st}, max_steps: {max_step}")

    def do_action(self, motion_name, step=1, speed=50):
        """
        Do prefix action with motion_name and step and speed

        :param motion_name: motion
        :type motion_name: str
        :param step: step of motion
        :type step: int
        :param speed: speed of motion
        :type speed: int or float
        """
        for _ in range(step):
            for motion in self.move_list[motion_name]:
                self.servo_move(motion, speed)

    def set_offset(self, offset_list):
        """
        Set offset of servo angles

        :param offset_list: list of servo angles
        :type offset_list: list
        """
        offset_list = [min(max(offset, -20), 20) for offset in offset_list]
        temp = str(offset_list)
        self.db.set(self.offset_value_name, temp)
        self.offset = offset_list

    def calibration(self):
        """Move all servos to home position"""
        self.servo_positions = self.calibrate_position
        self.servo_write_all(self.servo_positions)

    def reset(self, list=None):
        """Reset servo to original position"""
        if list is None:
            self.servo_positions = self.new_list(0)
            self.servo_write_all(self.servo_positions)
        else:
            self.servo_positions = list
            self.servo_write_all(self.servo_positions)

    def soft_reset(self):
        temp_list = self.new_list(0)
        self.servo_write_all(temp_list)

    DIR_MAX = 30
    CAM_PAN_MIN = -90
    CAM_PAN_MAX = 90
    CAM_TILT_MIN = -35
    CAM_TILT_MAX = 65

    PERIOD = 4095
    PRESCALER = 10
    TIMEOUT = 0.02

    # servo_pins: camera_pan_servo, camera_tilt_servo, direction_servo
    # motor_pins: left_swicth, right_swicth, left_pwm, right_pwm
    # grayscale_pins: 3 adc channels
    # ultrasonic_pins: trig, echo2
    # config: path of config file
    def __init__(self, 
                servo_pins:list=['P0', 'P1', 'P2'], 
                motor_pins:list=['D4', 'D5', 'P13', 'P12'],
                grayscale_pins:list=['A0', 'A1', 'A2'],
                ultrasonic_pins:list=['D2','D3'],
                config:str=CONFIG,
                ):

        # reset robot_hat
        utils.reset_mcu()
        time.sleep(0.2)

        # --------- config_flie ---------
        self.config_flie = fileDB(config, 777, os.getlogin())

        # --------- servos init ---------
        self.cam_pan = Servo(servo_pins[0])
        self.cam_tilt = Servo(servo_pins[1])   
        self.dir_servo_pin = Servo(servo_pins[2])
        # get calibration values
        self.dir_cali_val = float(self.config_flie.get("picarx_dir_servo", default_value=0))
        self.cam_pan_cali_val = float(self.config_flie.get("picarx_cam_pan_servo", default_value=0))
        self.cam_tilt_cali_val = float(self.config_flie.get("picarx_cam_tilt_servo", default_value=0))
        # set servos to init angle
        self.dir_servo_pin.angle(self.dir_cali_val)
        self.cam_pan.angle(self.cam_pan_cali_val)
        self.cam_tilt.angle(self.cam_tilt_cali_val)

        # --------- motors init ---------
        self.left_rear_dir_pin = Pin(motor_pins[0])
        self.right_rear_dir_pin = Pin(motor_pins[1])
        self.left_rear_pwm_pin = PWM(motor_pins[2])
        self.right_rear_pwm_pin = PWM(motor_pins[3])
        self.motor_direction_pins = [self.left_rear_dir_pin, self.right_rear_dir_pin]
        self.motor_speed_pins = [self.left_rear_pwm_pin, self.right_rear_pwm_pin]
        # get calibration values
        self.cali_dir_value = self.config_flie.get("picarx_dir_motor", default_value="[1, 1]")
        self.cali_dir_value = [int(i.strip()) for i in self.cali_dir_value.strip().strip("[]").split(",")]
        self.cali_speed_value = [0, 0]
        self.dir_current_angle = 0
        # init pwm
        for pin in self.motor_speed_pins:
            pin.period(self.PERIOD)
            pin.prescaler(self.PRESCALER)

        # --------- grayscale module init ---------
        adc0, adc1, adc2 = [ADC(pin) for pin in grayscale_pins]
        self.grayscale = Grayscale_Module(adc0, adc1, adc2, reference=None)
        # get reference
        self.line_reference = self.config_flie.get("line_reference", default_value=str(self.DEFAULT_LINE_REF))
        self.line_reference = [float(i) for i in self.line_reference.strip().strip('[]').split(',')]
        self.cliff_reference = self.config_flie.get("cliff_reference", default_value=str(self.DEFAULT_CLIFF_REF))
        self.cliff_reference = [float(i) for i in self.cliff_reference.strip().strip('[]').split(',')]
        # transfer reference
        self.grayscale.reference(self.line_reference)

        # --------- ultrasonic init ---------
        trig, echo= ultrasonic_pins
        self.ultrasonic = Ultrasonic(Pin(trig), Pin(echo, mode=Pin.IN, pull=Pin.PULL_DOWN))
        
    def set_motor_speed(self, motor, speed):
        ''' set motor speed
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param speed: speed
        type speed: int      
        '''
        speed = constrain(speed, -100, 100)
        motor -= 1
        if speed >= 0:
            direction = 1 * self.cali_dir_value[motor]
        elif speed < 0:
            direction = -1 * self.cali_dir_value[motor]
        speed = abs(speed)
        # print(f"direction: {direction}, speed: {speed}")
        if speed != 0:
            speed = int(speed /2 ) + 50
        speed = speed - self.cali_speed_value[motor]
        if direction < 0:
            self.motor_direction_pins[motor].high()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
        else:
            self.motor_direction_pins[motor].low()
            self.motor_speed_pins[motor].pulse_width_percent(speed)

    def motor_speed_calibration(self, value):
        self.cali_speed_value = value
        if value < 0:
            self.cali_speed_value[0] = 0
            self.cali_speed_value[1] = abs(self.cali_speed_value)
        else:
            self.cali_speed_value[0] = abs(self.cali_speed_value)
            self.cali_speed_value[1] = 0

    def motor_direction_calibrate(self, motor, value):
        ''' set motor direction calibration value
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param value: speed
        type value: int
        '''      
        motor -= 1
        if value == 1:
            self.cali_dir_value[motor] = 1
        elif value == -1:
            self.cali_dir_value[motor] = -1
        self.config_flie.set("picarx_dir_motor", self.cali_dir_value)

    def dir_servo_calibrate(self, value):
        self.dir_cali_val = value
        self.config_flie.set("picarx_dir_servo", "%s"%value)
        self.dir_servo_pin.angle(value)

    def set_dir_servo_angle(self, value):
        self.dir_current_angle = constrain(value, self.DIR_MIN, self.DIR_MAX)
        angle_value  = self.dir_current_angle + self.dir_cali_val
        self.dir_servo_pin.angle(angle_value)

    def cam_pan_servo_calibrate(self, value):
        self.cam_pan_cali_val = value
        self.config_flie.set("picarx_cam_pan_servo", "%s"%value)
        self.cam_pan.angle(value)

    def cam_tilt_servo_calibrate(self, value):
        self.cam_tilt_cali_val = value
        self.config_flie.set("picarx_cam_tilt_servo", "%s"%value)
        self.cam_tilt.angle(value)

    def set_cam_pan_angle(self, value):
        value = constrain(value, self.CAM_PAN_MIN, self.CAM_PAN_MAX)
        self.cam_pan.angle(-1*(value + -1*self.cam_pan_cali_val))

    def set_cam_tilt_angle(self,value):
        value = constrain(value, self.CAM_TILT_MIN, self.CAM_TILT_MAX)
        self.cam_tilt.angle(-1*(value + -1*self.cam_tilt_cali_val))

    def set_power(self, speed):
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)

    def backward(self, speed):
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            if abs_current_angle > self.DIR_MAX:
                abs_current_angle = self.DIR_MAX
            power_scale = (100 - abs_current_angle) / 100.0 
            if (current_angle / abs_current_angle) > 0:
                self.set_motor_speed(1, -1*speed)
                self.set_motor_speed(2, speed * power_scale)
            else:
                self.set_motor_speed(1, -1*speed * power_scale)
                self.set_motor_speed(2, speed )
        else:
            self.set_motor_speed(1, -1*speed)
            self.set_motor_speed(2, speed)  

    def forward(self, speed):
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            if abs_current_angle > self.DIR_MAX:
                abs_current_angle = self.DIR_MAX
            power_scale = (100 - abs_current_angle) / 100.0
            if (current_angle / abs_current_angle) > 0:
                self.set_motor_speed(1, 1*speed * power_scale)
                self.set_motor_speed(2, -speed) 
            else:
                self.set_motor_speed(1, speed)
                self.set_motor_speed(2, -1*speed * power_scale)
        else:
            self.set_motor_speed(1, speed)
            self.set_motor_speed(2, -1*speed)                  

    def stop(self):
        '''
        Execute twice to make sure it stops
        '''
        for _ in range(2):
            self.motor_speed_pins[0].pulse_width_percent(0)
            self.motor_speed_pins[1].pulse_width_percent(0)
            time.sleep(0.002)

    def get_distance(self):
        return self.ultrasonic.read()

    def set_grayscale_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.line_reference = value
            self.grayscale.reference(self.line_reference)
            self.config_flie.set("line_reference", self.line_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")

    def get_grayscale_data(self):
        return list.copy(self.grayscale.read())

    def get_line_status(self,gm_val_list):
        return self.grayscale.read_status(gm_val_list)

    def set_line_reference(self, value):
        self.set_grayscale_reference(value)

    def get_cliff_status(self,gm_val_list):
        for i in range(0,3):
            if gm_val_list[i]<=self.cliff_reference[i]:
                return True
        return False

    def set_cliff_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.cliff_reference = value
            self.config_flie.set("cliff_reference", self.cliff_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")

    def reset(self):
        self.stop()
        self.set_dir_servo_angle(0)
        self.set_cam_tilt_angle(0)
        self.set_cam_pan_angle(0)

    def close(self):
        self.reset()
        self.ultrasonic.close()

if __name__ == "__main__":
    px = Picarx()
    px.forward(50)
    time.sleep(1)
    px.stop()
