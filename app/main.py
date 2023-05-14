#!/usr/bin/python3

import sys, os, signal, queue, select, time, json, math, requests
import pprint

from vehicle import Vehicle
from config import Configuration
from mqtt import MqttHandler
from rtmp import GSTPipeline

from optparse import OptionParser
from pymavlink import mavutil

class Application(object):
    def __init__(self, optsargs):
        (self.opts, self.args) = optsargs

        self.config = Configuration(self.opts.configuration)
        if self.config.APP_DEBUG:
            pprint.pprint(vars(self.config))

        self.vehicle = Vehicle(self.opts.target_system, self.opts.target_component, self.config.SERIAL_NUM, self.config.FLIGHT_TYPE, self.config.CAMERA_TYPE)
        self.vehicle.on_image_downloaded = self.on_image_downloaded
        self.stream_rate = self.opts.stream_rate
        self.last_timestamp = time.monotonic()

        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

        self.message_queue = queue.Queue(maxsize=10)
        self.stream_thread = None
        self.exit = False

        if self.config.MAVLINK_CONNECTION is not None:
            connection = self.config.MAVLINK_CONNECTION
        else:
            connection = self.opts.connection

        # create MAVLink link
        while not self.exit:
            try:
                self.mavlink = mavutil.mavlink_connection(connection)
                if self.config.APP_DEBUG:
                    print("MAVLINK %s" % (connection))
                break
            except Exception as err:
                print("Failed to connect to %s: %s" % (connection, err))
                time.sleep(0.1)

        self.mqtt_handler = MqttHandler(self.config, self.callback)

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
                message = self.message_queue.get_nowait()
            except queue.Empty:
                return

            uav_serial_num = message.get('UAVSerialNum')
            if uav_serial_num == self.config.SERIAL_NUM:
                self.handle_ufly_msg(message)

    def handle_ufly_msg(self, message):
        type = message.get('Type')
        streamUrl = message.get('StreamUrl')
        if type is not None and streamUrl is not None:
            if type == 1:
                if self.stream_thread is None:
                    self.stream_thread = GSTPipeline.from_uri(self.config.STREAM_SRC)
                    if self.stream_thread is not None:
                        self.stream_thread.set_location(streamUrl)
                        self.stream_thread.start()
            else:
                if self.stream_thread is not None:
                    self.stream_thread.stop()
                    self.stream_thread = None

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

    def handle_mavlink_msg(self, conn, msg):
        '''callback for received MAVLink messages'''
        type = msg.get_type()
        sysid = msg.get_srcSystem()
        compid = msg.get_srcComponent()

        if sysid == 0 and compid == 0:
            return

        self.vehicle.handle_msg(conn, msg)

    def on_image_downloaded(self, filename, content):
        url = 'http://58.210.75.102:17002/common/File/UploadFileWithoutSecret'
        data = { 'file': filename, 'fileCategoryID': 376 }
        r = requests.post(url, data=data, files=BytesIO(content))
        if r.status_code == requests.status_codes.codes.ok:
            reply = r.json()
            if 'code' in reply and reply['code'] == 200 and 'data' in reply:
                data = { "requestData": { "data": { "FileInfoldList": [reply['data']], "PhotoVariety": 1 } }, "sign": "" }
                requests.post('http://58.210.75.102:17002/api/SNDefect/uploadSNFileinfo', data=data)

    def update(self):
        now = time.monotonic()
        if now - self.last_timestamp > 1.0 / self.stream_rate:
            self.vehicle.update()
            self.mqtt_handler.publish(self.vehicle.telemetry())
            self.last_timestamp = now

        self.mqtt_handler.connect()

    def main_loop(self):
        self.mavlink.message_hooks.append(self.handle_mavlink_msg)

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
