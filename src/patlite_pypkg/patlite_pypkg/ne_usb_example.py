"""
@author: PATLITE Corporation
パトライト制御
"""
import struct
import sys

import usb.core
from usb.core import Device


VENDOR_ID = 0x191A
"""Vendor ID"""
DEVICE_ID = 0x6001
"""Device ID"""
COMMAND_VERSION = 0x0
"""Command version"""
COMMAND_ID_CONTROL = 0x0
COMMAND_ID_SETTING = 0x1
COMMAND_ID_GETSTATE = 0x80
"""Command ID"""
ENDPOINT_ADDRESS = 0x01
"""Endpoint address for sending to host -> USB controlled multicolor indicator"""
ENDPOINT_ADDRESS_GET = 0x81
"""Endpoint address for sending to USB -> host controlled multicolor indicator"""
SEND_TIMEOUT = 1000
"""Time-out time when sending a command"""

# LED color
LED_COLOR_OFF = 0
"""Off"""
LED_COLOR_RED = 1
"""Red"""
LED_COLOR_GREEN = 2
"""green"""
LED_COLOR_YELLOW = 3
"""yellow"""
LED_COLOR_BLUE = 4
"""Blue"""
LED_COLOR_PURPLE = 5
"""purple"""
LED_COLOR_LIGHTBLUE = 6
"""Sky blue"""
LED_COLOR_WHITE = 7
"""White"""
LED_COLOR_KEEP = 0xF
"""Keep the current settings"""

# LED pattern
LED_OFF = 0x0
"""Off"""
LED_ON = 0x1
"""Lit"""
LED_PATTERN1 = 0x2
"""LED pattern1"""
LED_PATTERN2 = 0x3
"""LED pattern2"""
LED_PATTERN3 = 0x4
"""LED pattern3"""
LED_PATTERN4 = 0x5
"""LED pattern4"""
LED_PATTERN5 = 0x6
"""LED pattern5"""
LED_PATTERN6 = 0x7
"""LED pattern6"""
LED_PATTERN_KEEP = 0xF
"""Keep the current settings"""

# Number of buzzers
BUZZER_COUNT_CONTINUE = 0x0
"""Continuous operation"""
BUZZER_COUNT_KEEP = 0xF
"""Keep the current settings"""

# Buzzer pattern
BUZZER_OFF = 0x0
"""Stop"""
BUZZER_ON = 0x1
"""Blow (continuous)"""
BUZZER_SWEEP = 0x2
"""Sweep sound"""
BUZZER_INTERMITTENT = 0x3
"""Intermittent sound"""
BUZZER_WEEK_ATTENTION = 0x4
"""Weak caution sound"""
BUZZER_STRONG_ATTENTION = 0x5
"""Strong attention sound"""
BUZZER_SHINING_STAR = 0x6
"""shining star"""
BUZZER_LONDON_BRIDGE = 0x7
"""London bridge"""
BUZZER_KEEP = 0xF
"""Keep the current settings"""

# Buzzer volume
BUZZER_VOLUME_OFF = 0x0
"""Mute"""
BUZZER_VOLUME_MAX = 0xA
"""Maximum volume"""
BUZZER_VOLUME_KEEP = 0xF
"""Keep the current settings"""

# Setting
SETTING_OFF = 0x0
"""OFF"""
SETTING_ON = 0x1
"""ON"""

# others
BLANK = 0x0
"""openings"""

def main():
    args = sys.argv
    argc = len(sys.argv)

    # Connect to USB control multi-color indicator via USB communication
    dev = usb_open()

    # Get the command identifier specified by the command line argument
    commandId = 0;
    if argc > 1:
        commandId = args[1];

    if commandId == '1':
        # Specify the LED color and LED pattern to turn on and turn on the pattern
        if argc >= 4:
            set_light(dev, int(args[2]), int(args[3]))

    elif commandId == '2':
        # Specify the buzzer pattern and make the buzzer sound
        if argc >= 4:
            set_buz(dev, int(args[2]), int(args[3]))

    elif commandId == '3':
        # Change the buzzer volume by specifying the volume
        if argc >= 3:
            set_vol(dev, int(args[2]))

    elif commandId == '4':
        # Sound the buzzer by specifying the buzzer pattern, number of times, and volume.
        if argc >= 5:
            set_buz_ex(dev, int(args[2]), int(args[3]), int(args[4]))

    elif commandId == '5':
        # Change the connection display settings
        if argc >= 3:
            set_setting(dev, int(args[2]))

    elif commandId == '6':
        # Get touch sensor input status
        state = GetTouchSensorState(dev)
        if state == 1:
            print("touch sensor input = ON")
        elif state == 0:
            print("touch sensor input = OFF")
        else:
            print("USB communication failed")

    elif commandId == '7':
        # Turn off the LED and stop the buzzer
        reset(dev)

    # Device reset
    dev.reset()

    # End USB communication with USB control multi-color indicator
    usb_close()


def usb_open() -> Device:
    """
    Connect to USB control multi-color indicator via USB communication

    Returns
    -------
    dev : Device
        Device instance of USB controlled multicolor indicator
    """

    # Search for devices
    dev = usb.core.find(idVendor=VENDOR_ID, idProduct=DEVICE_ID)
    if dev is None:
        raise ValueError('device not found')

    if sys.platform == 'linux':
        # Kernel driver detachment
        # * Executed only in Linux environment
        if dev.is_kernel_driver_active(0):
            dev.detach_kernel_driver(0)

    # Device configuration settings
    dev.set_configuration()

    return dev


def usb_close():
    """
    End USB communication with USB control multi-color indicator
    * Since it is automatically released, there is no mounting
    """
    pass


def send_command(dev: Device, data: bytes) -> bool:
    """
    Send command

    Parameters
    ----------
    dev: Device
        Destination Device instance
    data: bytes
        Transmission data

    Returns
    -------
    bool
        Transmission result (success: True, failure: False)
    """

    try:
        # send
        write_length = dev.write(ENDPOINT_ADDRESS, data, SEND_TIMEOUT)
        if sys.platform == 'win32':
            # In the Windows environment, the write size is returned one byte more.
            # Reduce by 1 byte
            write_length -= 1

        if write_length != len(data):
            print('falsed to send')
            return False

        return True

    finally:
        pass


def set_light(dev: Device, color: int, state: int) -> bool:
    """
    Specify the LED color and LED pattern to turn on the USB control multi-color indicator and turn on the pattern.

    Buzzer pattern and buzzer volume maintain their current state

    Parameters
    ----------
    dev: Device
        Destination Device instance
    color: int
        LED color to control (off: 0, red: 1, green: 2, yellow: 3, blue: 4, purple: 5, sky blue: 6, white: 7, keep the current settings: 0x8-0xF)
    state: int
        LED pattern (off: 0x0, on: 0x1, LED pattern 1: 0x2, LED pattern 2: 0x3, LED pattern 3: 0x4, LED pattern 4: 0x5, LED pattern 5: 0x6, LED pattern 6: 0x7, current settings Maintain: 0x8 ~ 0xF)

    Returns
    -------
    bool
        Transmission result (success: True, failure: False)
    """

    # Argument range check
    if 0xF < color or 0xF < state:
        return False

    # Buzzer control (maintain the status quo)
    buzzer_control = BUZZER_COUNT_KEEP << 4
    buzzer_control |= BUZZER_KEEP

    # LED control
    led = color << 4
    led |= state
    
    # Convert to communication data
    data = struct.pack(
        'BBBBBxxx',         # format
        COMMAND_VERSION,    # Command version (0x00: fixed)
        COMMAND_ID_CONTROL, # Command ID
        buzzer_control,     # Buzzer control
        BUZZER_VOLUME_KEEP, # Buzzer volume (maintain the status quo)
        led,                # LED control
    )

    # Send command
    ret = send_command(dev, data)
    if ret is False:
        print('failed to send data')
        return False

    return True


def set_buz(dev: Device, buz_state: int, limit: int) -> bool:
    """
    Specify the buzzer pattern and make the buzzer sound

    LED and buzzer volume maintain current state

    Parameters
    ----------
    dev: Device
        Destination Device instance
    buz_state: int
        Buzzer pattern (stop: 0x0, continuous sound: 0x1, sweep sound: 0x2, intermittent sound: 0x3, weak caution sound: 0x4, strong caution sound: 0x5, glitter star: 0x6, London Bridge: 0x7, maintain the current settings: 0x8-0xF)
    limit: int
        Continuous operation: 0, Number of operations: 1 to 14, Maintain current settings: 0xF

    Returns
    -------
    bool
        Transmission result (success: True, failure: False)
    """

    # Argument range check
    if 0xF < buz_state or 0xF < limit:
        return False

    # Buzzer control
    buzzer_control = limit << 4
    buzzer_control |= buz_state
    
    # LED control (maintaining the status quo)
    led = LED_COLOR_KEEP << 4
    led |= LED_PATTERN_KEEP

    # Convert to communication data
    data = struct.pack(
        'BBBBBxxx',         # format
        COMMAND_VERSION,    # Command version (0x00: fixed)
        COMMAND_ID_CONTROL, # Command ID
        buzzer_control,     # Buzzer control
        BUZZER_VOLUME_KEEP, # Buzzer volume (maintain the status quo)
        led,                # LED control
    )

    # Send command
    ret = send_command(dev, data)
    if ret is False:
        print('failed to send data')
        return False

    return True


def set_vol(dev: Device, Volume: int) -> bool:
    """
    Change the buzzer volume by specifying the volume

    LED and buzzer patterns maintain their current state

    Parameters
    ----------
    dev: Device
        Destination Device instance
    Volume: int
        Volume (silence: 0x0, step volume: 0x1 to 0x9, maximum volume: 0xA, maintain current settings: 0xB to 0xF)

    Returns
    -------
    bool
        Transmission result (success: True, failure: False)
    """

    # Argument range check
    if 0xF < Volume:
        return False

    # Buzzer control (maintain the status quo)
    buzzer_control = BUZZER_COUNT_KEEP << 4
    buzzer_control |= BUZZER_KEEP

    # LED control (maintaining the status quo)
    led = LED_COLOR_KEEP << 4
    led |= LED_PATTERN_KEEP

    # Convert to communication data
    data = struct.pack(
        'BBBBBxxx',         # format
        COMMAND_VERSION,    # Command version (0x00: fixed)
        COMMAND_ID_CONTROL, # Command ID
        buzzer_control,     # Buzzer control
        Volume,             # Buzzer volume (maintain the status quo)
        led,                # LED control
    )

    # Send command
    ret = send_command(dev, data)
    if ret is False:
        print('failed to send data')
        return False

    return True


def set_buz_ex(dev: Device, buz_state: int, limit: int, volume: int) -> bool:
    """
    Sound the buzzer by specifying the buzzer pattern, number of times, and volume.

    Parameters
    ----------
    dev: Device
        Destination Device instance
    buz_state: int
        Buzzer pattern (stop: 0x0, continuous sound: 0x1, sweep sound: 0x2, intermittent sound: 0x3, weak caution sound: 0x4, strong caution sound: 0x5, glitter star: 0x6, London Bridge: 0x7, maintain the current settings: 0x8-0xF)
    limit: int
        Continuous operation: 0, Number of operations: 1 to 14, Maintain current settings: 0xF
    volume: int
        Volume (silence: 0x0, step volume: 0x1 to 0x9, maximum volume: 0xA, maintain current settings: 0xB to 0xF)

    Returns
    -------
    bool
        Transmission result (success: True, failure: False)
    """

    # Argument range check
    if 0xF < buz_state or 0xF < limit or 0xF < volume:
        return False

    # Buzzer control
    buzzer_control = limit << 4
    buzzer_control |= buz_state

    # LED control (maintaining the status quo)
    led = LED_COLOR_KEEP << 4
    led |= LED_PATTERN_KEEP

    # Convert to communication data
    data = struct.pack(
        'BBBBBxxx',         # format
        COMMAND_VERSION,    # Command version (0x00: fixed)
        COMMAND_ID_CONTROL, # Command ID
        buzzer_control,     # Buzzer control
        volume,             # Buzzer volume
        led,                # LED control
    )

    # Send command
    ret = send_command(dev, data)
    if ret is False:
        print('failed to send data')
        return False

    return True


def set_setting(dev: Device, setting: int) -> bool:
    """
    Change the connection display settings

    Parameters
    ----------
    dev: Device
        Destination Device instance
    setting: int
        Setting (OFF: 0x0, ON: 0x1)

    Returns
    -------
    bool
        Transmission result (success: True, failure: False)
    """

    # Argument range check
    if SETTING_OFF != setting and SETTING_ON != setting:
        return False

    # Convert to communication data
    data = struct.pack(
        'BBBxxxxx',         # format
        COMMAND_VERSION,    # Command version (0x00: fixed)
        COMMAND_ID_SETTING, # Command ID
        setting,            # Setting
    )

    # Send command
    ret = send_command(dev, data)
    if ret is False:
        print('failed to send data')
        return False

    return True


def GetTouchSensorState(dev: Device):
    """
    Get touch sensor input status

    Parameters
    ----------
    dev: Device
        Destination Device instance

    Returns
    -------
    int
        Acquisition result(Acquisition failure:-1, Touch sensor input OFF:0、Touch sensor input ON:1)
    """

    # Convert to communication data
    data = struct.pack(
        'BBxxxxxx',          # format
        COMMAND_VERSION,     # Command version (0x00: fixed)
        COMMAND_ID_GETSTATE, # Command ID
    )

    # Send command
    ret = send_command(dev, data)
    if ret is False:
        print('failed to send data')
        return -1
        
    # Receive response
    file_read = [2]
    file_read = dev.read(ENDPOINT_ADDRESS_GET, 2)
    if (file_read[1] & 1) == 1:
        return 1
    return 0


def reset(dev: Device) -> bool:
    """
    Turn off the LED and stop the buzzer

    Parameters
    ----------
    dev: Device
        Destination Device instance

    Returns
    -------
    bool
        Transmission result (success: True, failure: False)
    """

    # Buzzer control (the number of times is maintained as it is)
    buzzer_control = BUZZER_COUNT_KEEP << 4
    buzzer_control |= BUZZER_OFF

    # LED control
    led = LED_COLOR_OFF << 4
    led |= LED_OFF

    # Convert to communication data
    data = struct.pack(
        'BBBBBxxx',         # format
        COMMAND_VERSION,    # Command version (0x00: fixed)
        COMMAND_ID_CONTROL, # Command ID
        buzzer_control,     # Buzzer control
        BUZZER_VOLUME_KEEP, # Buzzer volume (maintain the status quo)
        led,                # LED control
    )

    # Send command
    ret = send_command(dev, data)
    if ret is False:
        print('failed to send data')
        return False

    return True


if __name__ == '__main__':
    main()