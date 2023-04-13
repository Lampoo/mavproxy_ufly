#!/usr/bin/python3

import paho.mqtt.client as mqtt
import sys, os, signal, queue, select, time, json, math
import pprint
import configparser

from optparse import OptionParser
from pymavlink import mavutil

""" configuration variables """


def parse_bool(OPTION):
    if OPTION == '0':
        OPTION = False
    else:
        OPTION = True
    return OPTION


class Configuration(object):
    def __init__(self, _conf_file_path=None):
        self.success = False
        _conf = configparser.ConfigParser(interpolation=configparser.ExtendedInterpolation())
        _conf_file_paths = []
        if _conf_file_path:
            _conf_file_paths.append(_conf_file_path)
        if 'HOME' in os.environ:
            _conf_file_path = os.path.join(os.environ['HOME'], ".mavufly.ini")
            _conf_file_paths.append(_conf_file_path)
        if 'LOCALAPPDATA' in os.environ:
            _conf_file_path = os.path.join(os.environ['LOCALAPPDATA'], "MAVProxy", "mavufly.ini")
            _conf_file_paths.append(_conf_file_path)

        try:  # try to use pkg_resources to allow for zipped python eggs
            import pkg_resources
            _cur_dir = pkg_resources.resource_filename('MAVProxy.modules.mavproxy_ufly', 'app')
        except:  # otherwise fall back to the standard file system
            _cur_dir = os.path.dirname(os.path.abspath(__file__))

        _conf_file_paths.append(os.path.join(_cur_dir, 'mavufly_default.ini'))

        for _conf_file_path in _conf_file_paths:
            if os.path.exists(_conf_file_path):
                try:
                    # load the config
                    _conf.read_file(open(_conf_file_path))

                    self.VERSION = str(_conf.get('general', 'version'))
                    self.CLIENT_ID = str(_conf.get('general', 'clientid'))
                    self.HOST = str(_conf.get('general', 'host'))
                    self.PORT = _conf.get('general', 'port')
                    self.TRANSPORT = str(_conf.get('general', 'transport'))
                    self.USERNAME = str(_conf.get('general', 'username'))
                    self.PASSWORD = str(_conf.get('general', 'password'))
                    self.KEEPALIVE = str(_conf.get('general', 'keepalive'))

                    self.VENDOR_ID = str(_conf.get('device', 'vendorid'))
                    self.VENDOR_NAME = str(_conf.get('device', 'vendorname'))
                    self.SERIAL_NUM = str(_conf.get('device', 'serialnum'))
                    self.FLIGHT_TYPE = str(_conf.get('device', 'flighttype'))

                    self.PUBLISH_TOPIC = str(_conf.get('publish', 'topic'))
                    self.PUBLISH_QOS = _conf.get('publish', 'qos')
                    #self.SUBSCRIBE_TOPIC = str(_conf.get('subscribe', 'topic'))
                    #self.SUBSCRIBE_QOS = _conf.get('subscribe', 'qos')

                    self.APP_DEBUG = parse_bool(_conf.get('debug', 'app_debug'))
                    self.MODULE_DEBUG = parse_bool(_conf.get('debug', 'module_debug'))

                    if self.MODULE_DEBUG:
                        print('Using config file at {}'.format(_conf_file_path))

                    self.success = True
                    break  # use first working config

                except Exception as e:
                    print('Failed to use config file at {} : {}'.format(_conf_file_path, e))
                    self.success = False


class MqttClientHelper(object):
    def __init__(self, config, callback):
        self.config = config
        self.callback = callback
        self.connected = False

        if self.config.VERSION == '5':
            self.client = mqtt.Client(client_id=self.config.CLIENT_ID,
                                      transport=self.config.TRANSPORT,
                                      protocol=mqtt.MQTTv5)
        else:
            self.client = mqtt.Client(client_id=self.config.CLIENT_ID,
                                      transport=self.config.TRANSPORT,
                                      protocol=mqtt.MQTTv311,
                                      clean_session=True)

        if self.config.USERNAME != '' and self.config.PASSWORD != '':
            self.client.username_pw_set(username=self.config.USERNAME, password=self.config.PASSWORD)

        if self.config.VERSION == '5':
            self.client.on_connect = self.on_connect_v5
        else:
            self.client.on_connect = self.on_connect

        self.client.on_message = self.on_message
        # self.client.on_disconnect = self.on_disconnect
        # self.client.on_publish = self.on_publish
        # self.client.on_subscribe = self.on_subscribe
        if self.config.MODULE_DEBUG:
            self.client.on_log = self.on_log

    def connect(self):
        if self.connected:
            return

        try:
            if self.config.VERSION == '5':
                self.client.connect(self.config.HOST,
                                    port=int(self.config.PORT),
                                    clean_start=mqtt.MQTT_CLEAN_START_FIRST_ONLY,
                                    keepalive=int(self.config.KEEPALIVE))
            else:
                self.client.connect(self.config.HOST,
                                    port=int(self.config.PORT),
                                    keepalive=int(self.config.KEEPALIVE))
        except Exception as e:
            print(f'mqtt: could not establish connection: {e}')
            return

        self.connected = True
        self.client.loop_start()

    def stop(self):
        self.client.loop_stop()

    # The MQTTv5 callback takes the additional 'props' parameter.
    def on_connect_v5(self, client, userdata, flags, rc, props):
        print("Connected: '" + str(flags) + "', '" + str(rc) + "', '" + str(props))
        if rc == 0:
            #self.client.subscribe(self.config.SUBSCRIBE_TOPIC, int(self.config.SUBSCRIBE_QOS))
            pass

    def on_connect(self, client, userdata, flags, rc):
        print("Connected: '" + str(flags) + "', '" + str(rc))
        if rc == 0:
            if self.config.SUBSCRIBE_TOPIC is not None:
                self.client.subscribe(self.config.SUBSCRIBE_TOPIC, int(self.config.SUBSCRIBE_QOS))

    def on_message(self, client, userdata, msg):
        print(msg.topic + " " + str(msg.qos) + str(msg.payload))

    def on_log(self, client, userdata, level, message):
        print(message)

    def publish(self, data):
        print(json.dumps(data))
        self.client.publish(self.config.PUBLISH_TOPIC, json.dumps(data), int(self.config.PUBLISH_QOS))


#
# FlightMode comes from mavutil.py with customization
#

# Custom mode definitions from PX4
PX4_CUSTOM_MAIN_MODE_MANUAL            = 1
PX4_CUSTOM_MAIN_MODE_ALTCTL            = 2
PX4_CUSTOM_MAIN_MODE_POSCTL            = 3
PX4_CUSTOM_MAIN_MODE_AUTO              = 4
PX4_CUSTOM_MAIN_MODE_ACRO              = 5
PX4_CUSTOM_MAIN_MODE_OFFBOARD          = 6
PX4_CUSTOM_MAIN_MODE_STABILIZED        = 7
PX4_CUSTOM_MAIN_MODE_RATTITUDE         = 8

PX4_CUSTOM_SUB_MODE_OFFBOARD           = 0
PX4_CUSTOM_SUB_MODE_AUTO_READY         = 1
PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF       = 2
PX4_CUSTOM_SUB_MODE_AUTO_LOITER        = 3
PX4_CUSTOM_SUB_MODE_AUTO_MISSION       = 4
PX4_CUSTOM_SUB_MODE_AUTO_RTL           = 5
PX4_CUSTOM_SUB_MODE_AUTO_LAND          = 6
PX4_CUSTOM_SUB_MODE_AUTO_RTGS          = 7
PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET = 8


auto_mode_flags  = mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED \
                 | mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED \
                 | mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED


def interpret_px4_mode(base_mode, custom_mode):
    custom_main_mode = (custom_mode & 0xFF0000)   >> 16
    custom_sub_mode  = (custom_mode & 0xFF000000) >> 24

    if base_mode & mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED != 0: #manual modes
        if custom_main_mode == PX4_CUSTOM_MAIN_MODE_MANUAL:
            return "MANUAL"
        elif custom_main_mode == PX4_CUSTOM_MAIN_MODE_ACRO:
            return "ACRO"
        elif custom_main_mode == PX4_CUSTOM_MAIN_MODE_RATTITUDE:
            return "RATTITUDE"
        elif custom_main_mode == PX4_CUSTOM_MAIN_MODE_STABILIZED:
            return "STABILIZED"
        elif custom_main_mode == PX4_CUSTOM_MAIN_MODE_ALTCTL:
            return "ALTCTL"
        elif custom_main_mode == PX4_CUSTOM_MAIN_MODE_POSCTL:
            return "POSCTL"
    elif (base_mode & auto_mode_flags) == auto_mode_flags: #auto modes
        if custom_main_mode & PX4_CUSTOM_MAIN_MODE_AUTO != 0:
            if custom_sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_MISSION:
                return "MISSION"
            elif custom_sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF:
                return "TAKEOFF"
            elif custom_sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_LOITER:
                return "LOITER"
            elif custom_sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET:
                return "FOLLOWME"
            elif custom_sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_RTL:
                return "RTL"
            elif custom_sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_LAND:
                return "LAND"
            elif custom_sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_RTGS:
                return "RTGS"
            elif custom_sub_mode == PX4_CUSTOM_SUB_MODE_OFFBOARD:
                return "OFFBOARD"
    return "UNKNOWN"


class FactGroup(object):
    def __init__(self, name):
        self.name = name 
        self.dict = dict()

    def set(self, key, value):
        self.dict[key] = value

    def get(self, key):
        return self.dict.get(key)

    def set_default(self, key, value=None):
        if key in self.dict.keys():
            self.dict.setdefault(key, value)

    def is_valid(self):
        if bool(self.dict):
            return True
        else:
            return False

    def value(self):
        if self.is_valid():
            if self.name is not None:
                return dict({self.name : self.dict})
            else:
                return self.dict

        return dict()


    def handle_message(self, conn, msg):
        pass

    def load_from_json(self, file):
        # TODO
        pass


class HomePositionFactGroup(FactGroup):
    LatFactName = 'homeLatitude'
    LonFactName = 'homeLongitude'
    AltFactName = 'homeAltitude'

    def __init__(self):
        super(HomePositionFactGroup, self).__init__(None)
        self.set_default(HomePositionFactGroup.LatFactName)
        self.set_default(HomePositionFactGroup.LonFactName)
        self.set_default(HomePositionFactGroup.AltFactName)

    def is_valid(self):
        if self.get(HomePositionFactGroup.LatFactName) is not None:
            return True
        else:
            return False

    def handle_message(self, conn, msg):
        type = msg.get_type()

        if type == 'HOME_POSITION':
            self.handle_home_position(conn, msg)
        else:
            return False

        return True

    def handle_home_position(self, conn, msg):
        self.set(HomePositionFactGroup.LatFactName, msg.latitude / 1.0e7)
        self.set(HomePositionFactGroup.LonFactName, msg.longitude / 1.0e7)
        self.set(HomePositionFactGroup.AltFactName, msg.altitude / 1000.0)


class GPSFactGroup(FactGroup):
    LatFactName = 'Latitude'
    LonFactName = 'Longitude'
    AltFactName = 'Altitude'
    SatFactName = 'Satellite'

    def __init__(self):
        super(GPSFactGroup, self).__init__('GPSData')
        self.set_default(GPSFactGroup.LatFactName)
        self.set_default(GPSFactGroup.LonFactName)
        self.set_default(GPSFactGroup.AltFactName)
        self.set_default(GPSFactGroup.SatFactName)

    def is_valid(self):
        if self.get(GPSFactGroup.LatFactName) is not None:
            return True
        else:
            return False

    def handle_message(self, conn, msg):
        type = msg.get_type()

        if type == 'GPS_RAW_INT':
            self.handle_gps_raw_int(conn, msg)
        elif type == 'HIGH_LATENCY':
            self.handle_high_latency(conn, msg)
        elif type == 'HIGH_LATENCY2':
            self.handle_high_latency2(conn, msg)
        else:
            return False

        return True

    def handle_gps_raw_int(self, conn, msg):
        self.set(GPSFactGroup.LatFactName, msg.lat / 1.0e7)
        self.set(GPSFactGroup.LonFactName, msg.lon / 1.0e7)
        self.set(GPSFactGroup.AltFactName, msg.alt / 1000.0)
        self.set(GPSFactGroup.SatFactName, msg.satellites_visible)

    def handle_high_latency(self, conn, msg):
        pass

    def handle_high_latency2(self, conn, msg):
        pass


class RTKFactGroup(FactGroup):
    def __init__(self):
        super(RTKFactGroup, self).__init__('RTKData')

    def handle_message(self, conn, msg):
        return False


class AttitudeFactGroup(FactGroup):
    PitchFactName = 'Pitch'
    RollFactName  = 'Roll'
    YawFactName   = 'Yaw'

    def __init__(self):
        super(AttitudeFactGroup, self).__init__('AttitudeAngle')
        self.set_default(AttitudeFactGroup.PitchFactName)
        self.set_default(AttitudeFactGroup.RollFactName)
        self.set_default(AttitudeFactGroup.YawFactName)

    def is_valid(self):
        if self.get(AttitudeFactGroup.PitchFactName) is not None:
            return True
        else:
            return False

    def handle_message(self, conn, msg):
        type = msg.get_type()

        if type == 'ATTITUDE':
            self.handle_attitude(conn, msg)
        else:
            return False

        return True

    def handle_attitude(self, conn, msg):
        self.set(AttitudeFactGroup.PitchFactName, msg.pitch)
        self.set(AttitudeFactGroup.RollFactName, msg.roll)
        self.set(AttitudeFactGroup.YawFactName, msg.yaw)


class BatteryFactGroup(FactGroup):
    PercentRemainingFactName = 'BatteryPercent'
    VoltageFactName = 'BatteryVoltage'

    def __init__(self):
        super(BatteryFactGroup, self).__init__(None)
        self.set_default(BatteryFactGroup.PercentRemainingFactName)
        self.set_default(BatteryFactGroup.VoltageFactName)

    def is_valid(self):
        if self.get(BatteryFactGroup.PercentRemainingFactName) is not None:
            return True
        else:
            return False

    def handle_message(self, conn, msg):
        type = msg.get_type()

        if type == 'SYS_STATUS':
            self.handle_sys_status(conn, msg)

        return False

    def handle_sys_status(self, conn, msg):
        self.set(BatteryFactGroup.PercentRemainingFactName, msg.battery_remaining)
        self.set(BatteryFactGroup.VoltageFactName, msg.voltage_battery / 1000.0)

    def handle_battery_status(self, conn, msg):
        pass


class VehicleFactGroup(FactGroup):
    LatFactName = 'Latitude'
    LonFactName = 'Longitude'
    AltFactName = 'Altitude'
    GPSFactName = 'GPSLevel'
    SPXFactName = 'SpeedX'
    SPYFactName = 'SpeedY'
    SPZFactName = 'SpeedZ'
    TimeFactName = 'Timestamp'
    LandFactName = 'LandMode'
    StatusFactName = 'DroneStatus'
    SNFactName = 'DroneSerialNo'
    FlightTypeFactName = 'FlightType'

    def __init__(self):
        super(VehicleFactGroup, self).__init__(None)
        self.set_default(VehicleFactGroup.LatFactName)
        self.set_default(VehicleFactGroup.LonFactName)
        self.set_default(VehicleFactGroup.AltFactName)
        self.set_default(VehicleFactGroup.GPSFactName)
        self.set_default(VehicleFactGroup.SPXFactName)
        self.set_default(VehicleFactGroup.SPYFactName)
        self.set_default(VehicleFactGroup.SPZFactName)
        self.set_default(VehicleFactGroup.LandFactName)
        self.set_default(VehicleFactGroup.StatusFactName)
        self.set_default(VehicleFactGroup.TimeFactName)
        self.set_default(VehicleFactGroup.SNFactName)
        self.set_default(VehicleFactGroup.FlightTypeFactName)

    def set_status(self, status):
        self.set(VehicleFactGroup.StatusFactName, status)

    def set_land_mode(self, mode):
        self.set(VehicleFactGroup.LandFactName, mode)

    def set_flight_type(self, flightType):
        self.set(VehicleFactGroup.FlightTypeFactName, flightType)

    def set_serial_num(self, serialNum):
        self.set(VehicleFactGroup.SNFactName, serialNum)

    def is_valid(self):
        if self.get(VehicleFactGroup.TimeFactName) is not None:
            return True
        else:
            return False

    def handle_message(self, conn, msg):
        type = msg.get_type()

        self.set(VehicleFactGroup.TimeFactName, int(conn.timestamp))

        if type == 'GLOBAL_POSITION_INT':
            self.handle_global_position_int(conn, msg)
        else:
            return False

        return True

    def handle_global_position_int(self, conn, msg):
        self.set(VehicleFactGroup.LatFactName, msg.lat / 1.0e7)
        self.set(VehicleFactGroup.LonFactName, msg.lon / 1.0e7)
        self.set(VehicleFactGroup.AltFactName, msg.alt / 1000.0)
        self.set(VehicleFactGroup.SPXFactName, msg.vx)
        self.set(VehicleFactGroup.SPYFactName, msg.vy)
        self.set(VehicleFactGroup.SPZFactName, msg.vz)


class Vehicle(object):
    def __init__(self, sysid, compid):
        self.system = sysid
        self.component = compid

        self.ready = False
        self.armed = False
        self.landed_state = -1
        self.vehicleFactGroup = VehicleFactGroup()

        self.fact_groups = []
        self.fact_groups.append(GPSFactGroup())
        self.fact_groups.append(HomePositionFactGroup())
        self.fact_groups.append(RTKFactGroup())
        self.fact_groups.append(AttitudeFactGroup())
        self.fact_groups.append(BatteryFactGroup())
        self.fact_groups.append(self.vehicleFactGroup)

    def set_flight_type(self, flightType):
        self.vehicleFactGroup.set_flight_type(flightType)

    def set_serial_num(self, serialNum):
        self.vehicleFactGroup.set_serial_num(serialNum)

    def handle_msg(self, conn, msg):
        type = msg.get_type()
        sysid = msg.get_srcSystem()
        compid = msg.get_srcComponent()

        if self.system != 0 and self.system != sysid:
            return False

        if self.component != 0 and self.component != compid:
            return False

        for group in self.fact_groups:
            if group.handle_message(conn, msg):
                return True

        if type == 'HEARTBEAT':
            self.handle_heartbeat(conn, msg)
        elif type == 'EXTENDED_SYS_STATE':
            self.handle_extended_sys_state(conn, msg)
        elif type == 'SYS_STATUS':
            self.handle_sys_status(conn, msg)
        else:
            return False

        return True

    def handle_heartbeat(self, conn, msg):
        self.armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY == mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY
        if self.armed:
            if self.landed_state == mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF:
                status = 'Takeoff'
            elif self.landed_state == mavutil.mavlink.MAV_LANDED_STATE_LANDING:
                status = 'Landing'
            elif self.landed_state == mavutil.mavlink.MAV_LANDED_STATE_IN_AIR:
                if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED == mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
                    # TODO: handle PX4 custom mode
                    status = interpret_px4_mode(msg.base_mode, msg.custom_mode) #'Custom:0x%x' % msg.custom_mode
                elif msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED == mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED:
                    status = 'Manual'
                elif msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED == mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED:
                    status = 'Stablize'
                elif msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED == mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED:
                    status = 'Guide'
                elif msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED == mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED:
                    status = 'Auto'
                elif msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_TEST_ENABLED == mavutil.mavlink.MAV_MODE_FLAG_TEST_ENABLED:
                    status = 'Test'
                else:
                    status = 'Unknown'
            else:
                status = 'Ready'
        elif self.ready:
            status = 'Ready'
        else:
            status = 'Unknown'

        self.vehicleFactGroup.set_status(status)

    def handle_sys_status(self, conn, msg):
        if msg.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK == mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK:
            self.ready = True
        else:
            self.ready = False

    def handle_extended_sys_state(self, conn, msg):
        self.landed_state = msg.landed_state

        if self.landed_state == mavutil.mavlink.MAV_LANDED_STATE_UNDEFINED:
            landMode = -1
        elif self.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
            landMode = 0
        else:
            # IN_AIR
            landMode = 1

        self.vehicleFactGroup.set_land_mode(landMode)

    def telemetry(self):
        data = dict({'Topic': 'TRACK'})
        for group in self.fact_groups:
            data = data | group.value()

        return data

    def update(self):
        pass


class Application(object):
    def __init__(self, optsargs):
        (self.opts, self.args) = optsargs

        self.config = Configuration(self.opts.configuration)
        pprint.pprint(vars(self.config))

        self.vehicle = Vehicle(self.opts.target_system, self.opts.target_component)
        self.vehicle.set_flight_type(self.config.FLIGHT_TYPE)
        self.vehicle.set_serial_num(self.config.SERIAL_NUM)
        self.stream_rate = self.opts.stream_rate
        self.last_timestamp = time.monotonic()

        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

        self.message_queue = queue.Queue(maxsize=10)

        # create MAVLink link
        try:
            self.mavlink = mavutil.mavlink_connection(self.opts.connection)
            print("MAVLINK %s" % (self.opts.connection))
        except Exception as err:
            print("Failed to connect to %s: %s" % (self.opts.connection, err))
            sys.exit(1)

        self.mqtt_handler = MqttClientHelper(self.config, self.callback)

        self.exit = False
        self.main_loop()

        self.mqtt_handler.stop()

    def callback(self, data):
        '''callback for data coming in from MQTT subscription'''
        try:
            self.message_queue.put_nowait(data)
        except queue.Full:
            print('Queue full, client data is unable to be enqueued')

    def drain_message_queue(self):
        '''unload data that has been placed on the message queue by the client'''
        while not self.message_queue.empty():
            try:
                data = self.message_queue.get_nowait()
            except queue.Empty:
                return
            else:
                # TODO: handle the user feedback
                pass

    def process_mavlink(self, timeout=0.01):
        '''receive MAVLink messages'''
        try:
            inputready, outputready, exceptready = select.select([self.mavlink.port], [], [], timeout)
            # block for 0.1 sec if there is nothing on the connection
            # otherwise we just dive right in...
            for s in inputready:
                self.mavlink.recv_msg()

            # mavlink buffer is never getting cleared
            # force clear the buffer to avoid memory leak
            if self.mavlink.mav.buf_len() == 0 and self.mavlink.mav.buf_index != 0:
                self.mavlink.mav.buf = bytearray()
                self.mavlink.mav.buf_index = 0

        except select.error:
            pass

    def handle_msg(self, conn, msg):
        '''callback for received MAVLink messages'''
        type = msg.get_type()
        sysid = msg.get_srcSystem()
        compid = msg.get_srcComponent()

        if sysid == 0 and compid == 0:
            return

        self.vehicle.handle_msg(conn, msg)

    def update(self):
        now = time.monotonic()
        if now - self.last_timestamp > 1.0 / self.stream_rate:
            self.vehicle.update()
            self.mqtt_handler.publish(self.vehicle.telemetry())
            self.last_timestamp = now

        self.mqtt_handler.connect()

    def main_loop(self):
        self.mavlink.message_hooks.append(self.handle_msg)

        while not self.exit:
            self.process_mavlink(timeout=0.01)
            self.drain_message_queue()
            self.update()

    def exit_gracefully(self, signum, frame):
        self.exit = True


def main():
    parser = OptionParser('ufly_app.py [options]')
    parser.add_option("--connection", dest="connection", type="str",
                      help="MAVLink communication link", default="udp:0.0.0.0:14550")
    parser.add_option("--dialect", dest="dialect", type="str",
                      help="MAVLink dialect", default="ardupilotmega")
    parser.add_option("--stream-rate", dest="stream_rate", type="float",
                      help="Stream Rate in HZ", default=1.0)
    parser.add_option("--configuration", dest="configuration", type="str",
                      help="path to configuration file", default=None)
    parser.add_option("--target-system", dest="target_system", type="int",
                      help="MAVLink target system for autopilot", default=1)
    parser.add_option("--target-component", dest="target_component", type="int",
                      help="MAVLink target component for autopilot", default=1)
    optsargs = parser.parse_args()
    (opts, args) = optsargs

    os.environ['MAVLINK20'] = '1'  # force MAVLink v2
    mavutil.set_dialect(opts.dialect)

    Application(optsargs)


if __name__ == '__main__':
    main()
