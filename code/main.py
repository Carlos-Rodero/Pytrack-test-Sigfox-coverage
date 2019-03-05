import machine
import pycom
import time
import gc
import socket
import os
from micropyGPS import MicropyGPS
from pytrack import Pytrack
from machine import I2C, SD, RTC
from network import Sigfox


""" SET UP """

# Heartbeat LED flashes in blue colour once every 4s to signal that the system
# is alive. Turn off firmware blinking
pycom.heartbeat(False)

# ID of device. It can be any ASCII character
device_id = "B"

# Deployment sequence. Initialize to None
deployment_seq = None

# Time values in seconds of different variables
time_searching_GPS = 30
time_searching_Sigfox = 60
time_to_deep_sleep = 300

# Boolean to save if data have been sent to Sigfox
data_sent = False

# Dictionary to save GPS values
data_gps = {}
data_gps['latitude'] = None
data_gps['longitude'] = None
data_gps['altitude'] = None
data_gps['hdop'] = None

# Variables to save path values
sd_mount_dir = "/sd"
deployment_filename = "/sd/deployment.txt"
data_filename = "/sd/data.txt"

# color leds
led_red = 0x7f0000
led_green = 0x007f00
led_yellow = 0x7f7f00
led_blue = 0x00FFFF
led_orange = 0xFF9900
led_pink = 0xFF00FF

# Enable garbage collector
gc.enable()

# SD
sd = None

# RTC
rtc = RTC()


""" FUNCTIONS """


def blink_led(times, ms, color):
    """ Function to uses the RGB LED to make a blink light

    Parameters
    ----------
        times: int
            number of times to blink the light.
        ms: int
            time in ms that will be blink the light
        color: string
            type of color
    """
    for cycles in range(times):
        pycom.rgbled(color)
        time.sleep_ms(int(ms/2))
        pycom.heartbeat(False)
        time.sleep_ms(int(ms/2))


def sd_access():
    """ Function to access to SD.

    Returns
    -------
        sd: obj
            initialized sd object
    """
    try:
        sd = SD()

    except OSError as e:
        print("Error: {}. SD card not found".format(e))
        while True:
            blink_led(1, 500, led_red)

    os.mount(sd, sd_mount_dir)
    return sd


def _write_deployment():
    """
    Function to write the deployment file into the SD
    """
    f_write = open(deployment_filename, 'w')
    f_write.write("{}".format("a"))
    f_write.close()


def _write_data():
    """
    Function to write the data file into the SD
    """
    f_write = open(data_filename, 'w')
    f_write.close()


def get_deployment_seq():
    """
    Function to get the sequence deployment from deployment file.
    It updates the character sequence from deployment file adding 1 to the
    sequence. I.e.: a + 1 => b

    Returns
    -------
        deployment: string
            character sequence from deployment.txt
    """
    deployment = None

    sd = sd_access()
    f_read = False

    try:
        f_read = open(deployment_filename, 'r')
        deployment = f_read.readall()
        f_read.close()
    except OSError as e:
        print("Error: {}. Deployment file not found".format(e))

    if f_read is False:
        deployment = "a"
        _write_deployment()

    if deployment is None or deployment is '' or (ord(deployment) < 97) or \
       (ord(deployment) > 122):
        deployment = "a"
        _write_deployment()

    f_write = open(deployment_filename, 'w')
    next_deployment = chr(ord(deployment) + 1)
    f_write.write("{}".format(next_deployment))
    f_write.close()

    sd.deinit()
    os.unmount(sd_mount_dir)

    return deployment


def get_lat_lon_datetime_gps(time_searching_GPS):
    """
    Function to get latitude, longitude, altitude and hdop from GPS.

    Parameters
    ----------
        time_searching_GPS : int
            seconds for searching the GPS

    Returns
    -------
        last_data : dict
            dictionary that contains datetime, latitude, longitude,
            altitude, and hdop
    """
    GPS_TIMEOUT_SECS = time_searching_GPS
    # init I2C to P21/P22
    i2c = machine.I2C(0, mode=I2C.MASTER, pins=('P22', 'P21'))
    # write to address of GPS
    GPS_I2CADDR = const(0x10)
    raw = bytearray(1)
    i2c.writeto(GPS_I2CADDR, raw)
    # create MicropyGPS instance. location formatting with commas
    gps = MicropyGPS(location_formatting='dd')
    # start a timer
    chrono = machine.Timer.Chrono()
    chrono.start()
    # store results here
    last_data = {}
    # return from validate coordinates
    res = False

    def check_for_valid_coordinates(gps):
        '''
        Given a MicropyGPS object, this function checks if valid coordinate
        data has been parsed successfully. If so, copies it over to global
        last_data.

        Parameters
        ----------
            gps : obj
                MicropyGPS object

        Returns
        -------
            bool
                Returns True if values from GPS are not 0

        '''
        if gps.satellite_data_updated() and gps.valid:
            # blink_led(1, 500, led_yellow)

            timestamp_list = gps.timestamp
            time = (timestamp_list[0], timestamp_list[1],
                    int(timestamp_list[2]), 0, 0)

            date_list = gps.date
            date = (2000 + date_list[2], date_list[1], (date_list[0]))

            datetime = tuple(date) + tuple(time)

            lat_list = gps.latitude_string().split("°")
            lat = float(lat_list[0])

            lon_list = gps.longitude_string().split("°")
            lon = float(lon_list[0])

            alt = gps.altitude

            hdop = gps.hdop

            last_data['datetime'] = datetime
            last_data['latitude'] = lat
            last_data['longitude'] = lon
            last_data['altitude'] = alt
            last_data['hdop'] = hdop

            if last_data['datetime'] is not 0 and last_data['latitude'] \
               is not 0 and last_data['longitude'] is not 0:
                return True
            else:
                return False

        else:
            return False

    check_for_valid_coordinates(gps)

    while True:
        # read some data from module via I2C
        raw = i2c.readfrom(GPS_I2CADDR, 16)
        # feed into gps object
        for b in raw:
            sentence = gps.update(chr(b))
            if sentence is not None:
                # gps successfully parsed a message from module
                # see if we have valid coordinates
                res = check_for_valid_coordinates(gps)
        elapsed = chrono.read()

        if elapsed > GPS_TIMEOUT_SECS:
            break

    if 'latitude' in last_data and 'longitude' in last_data and \
       'datetime' in last_data:
        i2c.deinit()
        return last_data
    else:
        last_data['datetime'] = 0
        last_data['latitude'] = 0
        last_data['longitude'] = 0
        last_data['altitude'] = 0
        last_data['hdop'] = 0
        i2c.deinit()
        return last_data


def set_datetime():
    """
    Function to initialize real time clock from GPS in Lopy/Sipy device
    """
    if data_gps['datetime'] is not 0:
        rtc.init(data_gps['datetime'])
    else:
        rtc.init()


def save_header():
    """
    Function to write header to data file in SD. Header contains the following
    values:
    device_id, deployment, datetime, latitude, longitude, altitude, hdop,
    voltage and data_sent.
    """
    sd = sd_access()
    f_test = False

    try:
        f_test = open(data_filename, 'r')
    except OSError as e:
        print("Error: {}. Data file not found".format(e))

    if f_test is False:
        _write_data()
        f = open(data_filename, 'a')
        header = "{},{},{},{},{},{},{},{},{},\n".format("#device_id",
                                                        "deployment",
                                                        "datetime",
                                                        "latitude",
                                                        "longitude",
                                                        "altitude", "hdop",
                                                        "voltage", "data_sent")
        f.write(header)
        f.close()

    sd.deinit()
    os.unmount(sd_mount_dir)


def save_data(device_id=None, deployment=None, datetime=None, lat=None,
              lon=None, alt=None, hdop=None, volt=None, data_sent=None):
    """
    Function to save data file in SD. Data contains the following
    values:
    device_id, deployment, datetime, lat, lon, alt, hdop, volt and data_sent.

    Parameters
    ----------
        device_id: string
            the id of the prototype
        deployment: string
            the deployment sequence of the measurement
        datetime: tuple
            tuple that contains the real time clock
        lat: float
            latitude from gps
        lon: float
            longitude from gps
        alt: int
            altitude from gps
        hdop: int
            hdop (quality data) from gps
        volt: int
            voltage from battery
        data_sent: bool
            True if data has been sent to Sigfox

    """

    sd = sd_access()
    f = open(data_filename, 'a')

    date_std = "{}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}.{}+00:00".format(
                datetime[0], datetime[1], datetime[2],
                datetime[3], datetime[4], datetime[5],
                datetime[6])

    data = "{},{},{},{},{},{},{},{},{},\n".format(device_id, deployment,
                                                  date_std, lat, lon, alt,
                                                  hdop, volt, data_sent)

    f.write(data)

    f.close()
    sd.deinit()
    os.unmount(sd_mount_dir)


def convert_data_to_payload_gps(device_id=None, deployment=None, lat=None,
                                lon=None, alt=None, hdop=None):
    """
    Function to convert data to a bytes payload

    Parameters
    ----------
        device_id: string
            the id of the device
        deployment: string
            the deployment sequence of the measurement
        lat: float
            latitude from gps
        lon: float
            longitude from gps
        alt: int
            altitude from gps
        hdop: int
            hdop (quality data) from gps

    Returns
    -------
        payload: list
            list of data values codified in bytes

    """
    payload = []
    latb = int(((lat + 90) / 180) * 0xFFFFFF)
    lonb = int(((lon + 180) / 360) * 0xFFFFFF)
    altb = int(round(float(alt), 0))
    hdopb = int(float(hdop) * 10)
    data_type = 0

    # payload gps: id(1) deploy(1) data_type(1) lat(3) lon(3) alt(2) hdop(1)
    # --> 12 bytes
    payload.append(ord(device_id))
    payload.append(ord(deployment))
    payload.append(data_type)
    payload.append(((latb >> 16) & 0xFF))
    payload.append(((latb >> 8) & 0xFF))
    payload.append((latb & 0xFF))
    payload.append(((lonb >> 16) & 0xFF))
    payload.append(((lonb >> 8) & 0xFF))
    payload.append((lonb & 0xFF))
    payload.append(((altb >> 8) & 0xFF))
    payload.append((altb & 0xFF))
    payload.append(hdopb & 0xFF)

    return payload


def send_data_Sigfox(data):
    """
    Function to send data to Sigfox backend.

    Parameters
    ----------
        data: bytes
            list of bytes to send

    Returns
    -------
        bool
            True if data has been sent to Sigfox backend. Otherwise returns
            False
    """
    # init Sigfox for RCZ1 (Europe)
    sigfox = Sigfox(mode=Sigfox.SIGFOX, rcz=Sigfox.RCZ1)

    # create a Sigfox socket
    s = socket.socket(socket.AF_SIGFOX, socket.SOCK_RAW)

    # make the socket blocking
    s.setblocking(True)

    # configure it as uplink only
    s.setsockopt(socket.SOL_SIGFOX, socket.SO_RX, False)

    bytes_data = len(data)
    data_sent = 0

    # wait until the module has joined the network
    start = time.ticks_ms()
    while data_sent < bytes_data:
        time.sleep(1)
        # print('Not yet sent to Sigfox...')

        # send bytes
        data_sent = s.send(data)
        # print(time.ticks_diff(start, time.ticks_ms()))

        # counter to send data to Sigfox. time_searching_Sigfox in seconds
        if time.ticks_diff(start, time.ticks_ms()) > \
           (time_searching_Sigfox * 1000):
            print("possible timeout")
            return False

    return True


def get_battery_status():
    """
    Function to get the battery voltage

    Returns
    -------
        volt: int
            voltage from battery
    """
    i2c = machine.I2C(0, mode=I2C.MASTER, pins=('P22', 'P21'))
    py = Pytrack(i2c=i2c)
    volt = py.read_battery_voltage()
    i2c.deinit()

    return volt


def deep_sleep(time_to_deep_sleep):
    """
    Function to set Pytrack in ultra low power (deep sleep) during x time.

    Returns
    -------
        time_to_deep_sleep: int
            seconds to stay in deep sleep
    """
    i2c = machine.I2C(0, mode=I2C.MASTER, pins=('P22', 'P21'))
    py = Pytrack(i2c=i2c)
    py.setup_sleep(time_to_deep_sleep)
    py.go_to_sleep(gps=True)
    i2c.deinit()

""" CODE """

# Start. LED green
blink_led(1, 500, led_green)

# Get voltage and deployment sequence. LED pink
time.sleep_ms(1000)
volt = get_battery_status()
deployment_seq = get_deployment_seq()
# blink_led(1, 1000, led_pink)

# Get GPS and set datetime. LED yellow
time.sleep_ms(1000)
data_gps = get_lat_lon_datetime_gps(time_searching_GPS)
set_datetime()
blink_led(1, 1000, led_yellow)

# Send data to Sigfox. LED orange
time.sleep_ms(1000)
payload_gps = convert_data_to_payload_gps(device_id=device_id,
                                          deployment=deployment_seq,
                                          lat=data_gps['latitude'],
                                          lon=data_gps['longitude'],
                                          alt=data_gps['altitude'],
                                          hdop=data_gps['hdop'])
data_sent = send_data_Sigfox(bytes(payload_gps))
blink_led(1, 1000, led_orange)

# Save data to SD. LED blue
time.sleep_ms(1000)
save_header()
save_data(device_id=device_id, deployment=deployment_seq,
          datetime=rtc.now(), lat=data_gps['latitude'],
          lon=data_gps['longitude'], alt=data_gps['altitude'],
          hdop=data_gps['hdop'], volt=volt, data_sent=data_sent)
blink_led(1, 1000, led_blue)

# Enter to deep sleep. LED red
time.sleep_ms(1000)
blink_led(1, 1000, led_red)
deep_sleep(time_to_deep_sleep)
