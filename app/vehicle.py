import sys, os, signal, queue, select, time, json, math

from concurrent.futures import ThreadPoolExecutor, as_completed
from pymavlink import mavutil

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
        self.dict = {}

    def set(self, key, value):
        if key in self.dict.keys():
            self.dict[key] = value

    def get(self, key):
        return self.dict.get(key)

    def set_default(self, key, value=None):
        self.dict.setdefault(key, value)

    def is_valid(self):
        if bool(self.dict):
            return True
        else:
            return False

    def value(self):
        if self.is_valid():
            res = { key : val for key, val in self.dict.items() if val is not None }

            if self.name is not None:
                return {self.name : res}
            else:
                return res

        return {}


    def handle_message(self, conn, msg):
        return False

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


class TrackInfoFactGroup(FactGroup):
    TopicFactName = 'Topic'
    DroneSerialNoFactName = 'DroneSerialNo'
    FlightTypeFactName = 'FlightType'

    def __init__(self, serialNum, flightType):
        super(TrackInfoFactGroup, self).__init__(None)
        self.set_default(TrackInfoFactGroup.TopicFactName, 'TRACK')
        self.set_default(TrackInfoFactGroup.DroneSerialNoFactName, serialNum)
        self.set_default(TrackInfoFactGroup.FlightTypeFactName, flightType)

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

    def set_status(self, status):
        self.set(VehicleFactGroup.StatusFactName, status)

    def set_land_mode(self, mode):
        self.set(VehicleFactGroup.LandFactName, mode)

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
    def __init__(self, sysid, compid, serialNum, flightType):
        self.system = sysid
        self.component = compid

        self.ready = False
        self.armed = False
        self.landed_state = -1

        self.executor = ThreadPoolExecutor(max_workers=5)

        self.vehicleFactGroup = VehicleFactGroup()
        self.fact_groups = []
        self.fact_groups.append(TrackInfoFactGroup(serialNum, flightType))
        self.fact_groups.append(GPSFactGroup())
        self.fact_groups.append(HomePositionFactGroup())
        self.fact_groups.append(RTKFactGroup())
        self.fact_groups.append(AttitudeFactGroup())
        self.fact_groups.append(BatteryFactGroup())
        self.fact_groups.append(self.vehicleFactGroup)

    def handle_msg(self, conn, msg):
        type = msg.get_type()
        sysid = msg.get_srcSystem()
        compid = msg.get_srcComponent()

        if self.system == 0 or self.system == sysid:
            if self.component == 0 or self.component == compid:
                if self.handle_message_in_groups(conn, msg):
                    return True

                if type == 'HEARTBEAT':
                    self.handle_heartbeat(conn, msg)
                elif type == 'EXTENDED_SYS_STATE':
                    self.handle_extended_sys_state(conn, msg)
                elif type == 'SYS_STATUS':
                    self.handle_sys_status(conn, msg)
                else:
                    return False
            elif self.component >= mavutil.mavlink.MAV_COMP_ID_CAMERA and self.component <= mavutil.mavlink.MAV_COMP_ID_CAMERA6:
                # FIXME: Should be done in the CameraManager class
                if type == 'CAMERA_IMAGE_CAPTURED':
                    self.handle_camera_image_captured(conn, msg)
                else:
                    return False

        return True

    def handle_message_in_groups(self, conn, msg):
        for grp in self.fact_groups:
            if grp.handle_message(conn, msg):
                return True
        return False

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

    def download(self, file_url, fn, *args, **kwargs):
        if file_url.startswith('http'):
            r = requests.get(file_url)
            if r.status_code == requests.codes.ok:
                fn(0, r.content, args, kwargs)
            else:
                fn(1, r.content, args, kwargs)


    def handle_camera_image_captured(self, conn, msg):
        file_url = msg.file_url
        capture_result = msg.capture_result
        if capture_result == 1:
            #lat = msg.lat / 1.0e7
            #lon = msg.lon / 1.0e7
            #alt = msg.alt / 1000.0
            if file_url is not None and file_url.startswith("http"):
                self.execute(self.download, file_url)


    def execute(self, fn, *args, **kwargs):
        self.executor.submit(fn, args, kwargs)

    def telemetry(self):
        data = {}
        for group in self.fact_groups:
            data |= group.value()

        return data

    def update(self):
        pass