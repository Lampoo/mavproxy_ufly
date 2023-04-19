#!/usr/bin/python3

import sys, os, signal, queue, select, time, json, math
import configparser


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