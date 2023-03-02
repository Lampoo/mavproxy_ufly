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
                    self.CLIENT_ID = str(_conf.get('general', 'client_id'))
                    self.HOST = str(_conf.get('general', 'host'))
                    self.PORT = _conf.get('general', 'port')
                    self.TRANSPORT = str(_conf.get('general', 'transport'))
                    self.USERNAME = str(_conf.get('general', 'username'))
                    self.PASSWORD = str(_conf.get('general', 'password'))
                    self.KEEPALIVE = str(_conf.get('general', 'keepalive'))

                    self.VID = str(_conf.get('device', 'vid'))
                    self.PID = str(_conf.get('device', 'pid'))

                    self.PUBLISH_TOPIC = str(_conf.get('publish', 'topic'))
                    self.PUBLISH_QOS = _conf.get('publish', 'qos')
                    self.SUBSCRIBE_TOPIC = str(_conf.get('subscribe', 'topic'))
                    self.SUBSCRIBE_QOS = _conf.get('subscribe', 'qos')

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

        self.client.loop_start()

    def stop(self):
        self.client.loop_stop()

    # The MQTTv5 callback takes the additional 'props' parameter.
    def on_connect_v5(self, client, userdata, flags, rc, props):
        print("Connected: '" + str(flags) + "', '" + str(rc) + "', '" + str(props))
        self.connected = True
        self.client.subscribe(self.config.SUBSCRIBE_TOPIC, 2)

    def on_connect(self, client, userdata, flags, rc):
        print("Connected: '" + str(flags) + "', '" + str(rc))
        self.connected = True
        self.client.subscribe(self.config.SUBSCRIBE_TOPIC, 2)

    def on_message(self, client, userdata, msg):
        print(msg.topic + " " + str(msg.qos) + str(msg.payload))

    def on_log(self, client, userdata, level, message):
        print(message)

    def send_message(self, data):
        print(json.dumps(data))
        self.client.publish(self.config.PUBLISH_TOPIC, json.dumps(data), int(self.config.PUBLISH_QOS))

    def publish(self, data):
        self.send_message({'MessageType': 'message', 'Cmd': 'drone_realdata', 'DroneSerialNo': self.config.PID, 'Data': data})


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


class Vehicle(object):
    def __init__(self, sysid, compid):
        self.system = sysid
        self.component = compid

        self.ready = False
        self.armed = False
        self.landed_state = -1

        self.data = {}
        self.data['Timestamp'] = 0
        self.data['DroneMode'] = 'GPS' #
        self.data['LandMode'] = -1
        self.data['DroneStatus'] = 'Unknown'
        self.data['BatteryVoltage'] = 0.0
        self.data['BatteryPercent'] = 0
        self.data['GPSLevel'] = 0
        self.data['Longitude'] = 0.0
        self.data['Latitude'] = 0.0
        self.data['Altitude'] = 0.0
        self.data['SpeedX'] = 0
        self.data['SpeedY'] = 0
        self.data['SpeedZ'] = 0
        self.data['GPSData'] = {'Longitude': 0.0, 'Latitude': 0.0, 'Altitude': 0.0, 'Satellite': 0}
        self.data['RTKData'] = {'Longitude': 0.0, 'Latitude': 0.0, 'Altitude': 0.0, 'Satellite': 0, 'Status': 0}
        self.data['AttitudeAngle'] = {'Roll': 0.0, 'Pitch': 0.0, 'Yaw': 0.0 }

    def handle_heartbeat(self, conn, msg):
        self.armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY == mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY
        if self.armed:
            if self.landed_state == mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF:
                self.data['DroneStatus'] = 'Takeoff'
            elif self.landed_state == mavutil.mavlink.MAV_LANDED_STATE_LANDING:
                self.data['DroneStatus'] = 'Landing'
            elif self.landed_state == mavutil.mavlink.MAV_LANDED_STATE_IN_AIR:
                if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED == mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
                    # TODO: handle PX4 custom mode
                    self.data['DroneStatus'] = interpret_px4_mode(msg.base_mode, msg.custom_mode) #'Custom:0x%x' % msg.custom_mode
                elif msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED == mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED:
                    self.data['DroneStatus'] = 'Manual'
                elif msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED == mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED:
                    self.data['DroneStatus'] = 'Stabilize'
                elif msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED == mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED:
                    self.data['DroneStatus'] = 'Guide'
                elif msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED == mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED:
                    self.data['DroneStatus'] = 'Auto'
                elif msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_TEST_ENABLED == mavutil.mavlink.MAV_MODE_FLAG_TEST_ENABLED:
                    self.data['DroneStatus'] = 'Test'
                else:
                    self.data['DroneStatus'] = 'Unknown'
            else:
                self.data['DroneStatus'] = 'Ready'
        elif self.ready:
            self.data['DroneStatus'] = 'Ready'
        else:
            self.data['DroneStatus'] = 'Unknown'

    def handle_sys_status(self, conn, msg):
        if msg.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK == mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK:
            self.ready = True
        else:
            self.ready = False

        self.data['BatteryVoltage'] = msg.voltage_battery
        self.data['BatteryPercent'] = msg.battery_remaining

    def handle_extended_sys_state(self, conn, msg):
        self.landed_state = msg.landed_state

        if self.landed_state == mavutil.mavlink.MAV_LANDED_STATE_UNDEFINED:
            self.data['LandMode'] = -1
        elif self.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
            self.data['LandMode'] = 0
        else:
            # IN_AIR
            self.data['LandMode'] = 1

    def handle_global_position_int(self, conn, msg):
        self.data['Longitude'] = msg.lon / 1.0e7
        self.data['Latitude'] = msg.lat / 1.0e7
        self.data['Altitude'] = msg.alt / 1000.0
        self.data['SpeedX'] = msg.vx
        self.data['SpeedY'] = msg.vy
        self.data['SpeedZ'] = msg.vz

    def handle_attitude(self, conn, msg):
        self.data['AttitudeAngle']['roll'] = msg.roll
        self.data['AttitudeAngle']['yaw'] = msg.yaw
        self.data['AttitudeAngle']['pitch'] = msg.pitch

    def handle_battery_status(self, conn, msg):
        pass

    def handle_gps_raw_int(self, conn, msg):
        self.data['GPSLevel'] = msg.satellites_visible
        self.data['GPSData']['Longitude'] = msg.lon / 1.0e7
        self.data['GPSData']['Latitude'] = msg.lat / 1.0e7
        self.data['GPSData']['Altitude'] = msg.alt / 1000.0
        self.data['GPSData']['Satellite'] = msg.satellites_visible
        self.data['RTKData']['Longitude'] = msg.lon / 1.0e7
        self.data['RTKData']['Latitude'] = msg.lat / 1.0e7
        self.data['RTKData']['Altitude'] = msg.alt / 1000.0
        self.data['RTKData']['Satellite'] = msg.satellites_visible
        self.data['RTKData']['Status'] = msg.fix_type

    def handle_msg(self, conn, msg):
        type = msg.get_type()
        sysid = msg.get_srcSystem()
        compid = msg.get_srcComponent()

        if self.system != 0 and self.system != sysid:
            return False

        if self.component != 0 and self.component != compid:
            return False

        self.data['Timestamp'] = int(conn.timestamp)

        if type == 'HEARTBEAT':
            self.handle_heartbeat(conn, msg)
        elif type == 'ATTITUDE':
            self.handle_attitude(conn, msg)
        elif type == 'EXTENDED_SYS_STATE':
            self.handle_extended_sys_state(conn, msg)
        elif type == 'SYS_STATUS':
            self.handle_sys_status(conn, msg)
        elif type == 'BATTERY_STATUS':
            self.handle_battery_status(conn, msg)
        elif type == 'GLOBAL_POSITION_INT':
            self.handle_global_position_int(conn, msg)
        elif type == 'GPS_RAW_INT':
            self.handle_gps_raw_int(conn, msg)
        else:
            # message type not handleded 
            pass

        return True

    def update(self):
        pass

class Application(object):
    def __init__(self, optsargs):
        (self.opts, self.args) = optsargs

        self.config = Configuration(self.opts.configuration)
        pprint.pprint(vars(self.config))

        self.vehicle = Vehicle(self.opts.target_system, self.opts.target_component)
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
            self.mqtt_handler.publish(self.vehicle.data)
            self.last_timestamp = now

        if not self.mqtt_handler.connected:
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
