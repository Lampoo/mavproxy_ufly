#!/usr/bin/python3

import paho.mqtt.client as mqtt
import sys, os, signal, queue, select, time, json, math

class MqttHandler(object):
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
                self.client.connect_async(self.config.HOST,
                                    port=int(self.config.PORT),
                                    clean_start=mqtt.MQTT_CLEAN_START_FIRST_ONLY,
                                    keepalive=int(self.config.KEEPALIVE))
            else:
                self.client.connect_async(self.config.HOST,
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
        if self.config.APP_DEBUG:
            print("Connected: '" + str(flags) + "', '" + str(rc) + "', '" + str(props))
        if rc == 0:
                self.client.subscribe(self.config.SUBSCRIBE_TOPIC, int(self.config.SUBSCRIBE_QOS))

    def on_connect(self, client, userdata, flags, rc):
        if self.config.APP_DEBUG:
            print("Connected: '" + str(flags) + "', '" + str(rc))
        if rc == 0:
                self.client.subscribe(self.config.SUBSCRIBE_TOPIC, int(self.config.SUBSCRIBE_QOS))

    def on_message(self, client, userdata, msg):
        if self.config.APP_DEBUG:
            print(msg.topic + " " + str(msg.qos) + str(msg.payload))
        message = json.loads(msg.payload)
        uav_serial_num = message.get('UAVSerialNum')
        if uav_serial_num == self.config.SERIAL_NUM:
            handle_ufly_message(message)

    def handle_ufly_message(self, message):
        type = message.get('Type')
        streamUrl = message.get('StreamUrl')
        if type is not None and streamUrl is not None:
            if type == 1:
                pass
            else:
                pass

    def on_log(self, client, userdata, level, message):
        print(message)

    def publish(self, data):
        if self.config.APP_DEBUG:
            print(json.dumps(data))
        self.client.publish(self.config.PUBLISH_TOPIC, json.dumps(data), int(self.config.PUBLISH_QOS))