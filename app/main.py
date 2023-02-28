#!/usr/bin/python3

import paho.mqtt.client as mqtt
import sys, os, signal, queue, select, time, json
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

        # self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        # self.client.on_publish = self.on_publish
        # self.client.on_subscribe = self.on_subscribe
        if self.config.MODULE_DEBUG:
            self.client.on_log = self.on_log

    def start(self):
        if self.config.VERSION == '5':
            self.client.connect(self.config.HOST,
                                port=int(self.config.PORT),
                                clean_start=mqtt.MQTT_CLEAN_START_FIRST_ONLY,
                                keepalive=int(self.config.KEEPALIVE))
        else:
            self.client.connect(self.config.HOST,
                                port=int(self.config.PORT),
                                keepalive=int(self.config.KEEPALIVE))

        self.client.loop_start()

    def stop(self):
        self.client.loop_stop()

    # The MQTTv5 callback takes the additional 'props' parameter.
    def on_connect_v5(self, client, userdata, flags, rc, props):
        print("Connected: '" + str(flags) + "', '" + str(rc) + "', '" + str(props))
        self.client.subscribe(self.config.SUBSCRIBE, 2)

    def on_connect(self, client, userdata, flags, rc):
        print("Connected: '" + str(flags) + "', '" + str(rc))
        self.client.subscribe(self.config.SUBSCRIBE, 2)

    def on_message(self, client, userdata, msg):
        print(msg.topic + " " + str(msg.qos) + str(msg.payload))

    def on_log(self, client, userdata, level, message):
        print(message)

    def send_message(self, data):
        print(json.dumps(data))
        self.client.publish(self.config.PUBLISH_TOPIC, json.dumps(data), self.config.PUBLISH_QOS)

    def publish(self, data):
        self.send_message({'MessageType': 'message', 'Cmd': 'drone_realdata', 'Data': data})


class Application(object):
    def __init__(self, optsargs):
        (self.opts, self.args) = optsargs

        self.config = Configuration(self.opts.configuration)
        pprint.pprint(vars(self.config))

        self.target_system = self.opts.target_system
        self.target_component = self.opts.target_component

        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

        self.message_queue = queue.Queue(maxsize=10)

        self.last_timestamp = 0.0
        self.data = {
            'DroneSerialNo': self.config.PID,
            'Timestamp': 0,
            'DroneMode': 'Unknown',
            'LandMode': -1,
            'DroneStatus': 'Unknown'
        }

        # create MAVLink link
        try:
            self.mavlink = mavutil.mavlink_connection(self.opts.connection)
            print("MAVLINK %s" % (self.opts.connection))
        except Exception as err:
            print("Failed to connect to %s: %s" % (self.opts.connection, err))
            sys.exit(1)

        self.mqttc = MqttClientHelper(self.config, self.callback)
        self.mqttc.start()

        self.exit = False
        self.main_loop()

        self.mqttc.stop()

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

        if self.target_system != 0 and self.target_system != sysid:
            return

        if self.target_component != 0 and self.target_component != compid:
            return

        self.data['Timestamp'] = int(conn.timestamp)

        if type == 'HEARTBEAT':
            pass
        elif type == 'HIGH_LATENCY':
            pass
        elif type == 'SYS_STATUS':
            self.data['BatteryVoltage'] = msg.voltage_battery
            self.data['BatteryPercent'] = msg.battery_remaining
        elif type == 'GLOBAL_POSITION_INT':
            # self.data['GPSLevel'] = 4 # FIXME?
            self.data['Longtitude'] = msg.lat / 1.0e7
            self.data['Latitude'] = msg.lon / 1.0e7
            self.data['Altitude'] = msg.alt / 1000.0
            self.data['SpeedX'] = msg.vx
            self.data['SpeedY'] = msg.vy
            self.data['SpeedZ'] = msg.vz
            pass
        elif type == 'GPS_RAW_INT':
            if 'GPSData' not in self.data:
                self.data['GPSData'] = {}
            self.data['GPSData']['Longtitude'] = msg.lat / 1.0e7
            self.data['GPSData']['Latitude'] = msg.lon / 1.0e7
            self.data['GPSData']['Altitude'] = msg.alt / 1000.0
            self.data['GPSData']['Satellite'] = msg.satellites_visible

        else:
            # message type not handleded 
            pass

    def update(self):
        now = time.time()
        if now - self.last_timestamp > 1.0:
            self.mqttc.publish(self.data)
            self.last_timestamp = now

    def main_loop(self):
        self.mavlink.message_hooks.append(self.handle_msg)
        # self.mavlink.mav.request_data_stream_send(1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, self.opts.stream_rate, 1)
        while not self.exit:
            self.process_mavlink(timeout=0.01)
            self.drain_message_queue()
            self.update()

    def exit_gracefully(self, signum, frame):
        self.exit = True


def main():
    parser = OptionParser('ufly_app.py [options]')
    parser.add_option("--connection", dest="connection", type="str",
                      help="MAVLink communication link", default="udp:127.0.0.1:14550")
    parser.add_option("--dialect", dest="dialect", type="str",
                      help="MAVLink dialect", default="ardupilotmega")
    parser.add_option("--stream-rate", dest="stream_rate", type="int",
                      help="Stream Rate in HZ", default=10)
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
