"""
monitoring of waypoint glide slope achievment
"""

import math

from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_module

FULL_DEFLECTION = 20.0

class WPSlopeModule(mp_module.MPModule):

    def __init__(self, mpstate):
        super(WPSlopeModule, self).__init__(mpstate, "WPSlope", "WPSlope", public=False)
        self.last_wp_pos = None
        self.next_wp_pos = None
        self.wp_current = None
        self.console.set_status('WPAltError', 'WPAltError --', row=5)
        self.console.set_status('CAlt', 'Alt --', row=5)
        self.console.set_status('PrevAlt', 'NextAlt --', row=5)
        self.console.set_status('NextAlt', 'NextAlt --', row=5)
        self.console.set_status('P', 'P --', row=5)
        self.console.set_status('Airspeed2', 'Airspeed2 --', row=5)
        self.console.set_status('RearLeft', 'RearLeft -', row=6)
        self.console.set_status('RearRight', 'RearRight -', row=6)
        self.console.set_status('FrontLeft', 'FrontLeft -', row=6)
        self.console.set_status('FrontRight', 'FrontRight -', row=6)

    def ServoAngle(self, SERVO_OUTPUT_RAW, snum, angle, revmul=1.0):
        '''return surface defelection in degrees'''
        smin = self.get_mav_param("SERVO%u_MIN" % snum)
        smax = self.get_mav_param("SERVO%u_MAX" % snum)
        strim = self.get_mav_param("SERVO%u_TRIM" % snum)
        srev = self.get_mav_param("SERVO%u_REVERSED" % snum)
        v = getattr(SERVO_OUTPUT_RAW, 'servo%u_raw' % snum)
        if v > strim:
            return revmul * angle * (v - strim) / float(smax - strim)
        return revmul * angle * (v - strim) / float(strim - smin)

        
    def wp_lookup(self, seq):
        '''lookup a wp number for position'''
        wpmod = self.module('wp')
        wploader = wpmod.wploader
        w = wploader.wp(seq)
        if w is None:
            return None
        if not wploader.is_location_command(w.command):
            return None
        return (w.x, w.y, w.z)

    def get_distance_NE(self, point1, point2):
        dist = mp_util.gps_distance(point1[0], point1[1], point2[0], point2[1])
        bearing = mp_util.gps_bearing(point1[0], point1[1], point2[0], point2[1])
        dN = dist * math.cos(math.radians(bearing))
        dE = dist * math.sin(math.radians(bearing))
        return (dN, dE)

    def line_path_proportion(self, pos, point1, point2):
        '''return proportion that pos is along path from point1 to point2'''
        (vec1_x, vec1_y) = self.get_distance_NE(point1, point2)
        (vec2_x, vec2_y) = self.get_distance_NE(point1, pos)
        dsquared = vec1_x**2 + vec1_y**2
        if dsquared < 0.001:
            # the two points are very close together
            return 1.0
        dot_product = vec1_x*vec2_x + vec1_y*vec2_y
        return dot_product / dsquared

    def update_alt_error(self, pos):
        '''update displayed alt error'''
        home_alt = 0
        if 'HOME_POSITION' in self.master.messages:
            home_position = self.master.messages['HOME_POSITION']
            home_alt = home_position.altitude*0.001
        if 'NAMED_VALUE_FLOAT' in self.master.messages:
            nvf = self.master.messages['NAMED_VALUE_FLOAT']
            asp2 = nvf.value
            self.console.set_status('Airspeed2', 'Airspeed2 %s' % self.speed_string(asp2), row=5)

        proportion = self.line_path_proportion(pos, self.last_wp_pos, self.next_wp_pos)
        target_alt = self.next_wp_pos[2]*proportion + self.last_wp_pos[2]*(1.0-proportion)
        alterr = pos[2] - target_alt
        err_str = self.height_string(abs(alterr))
        if alterr < 0:
            err_str += "(low)"
        elif alterr > 0:
            err_str += "(high)"
        self.console.set_status('WPAltError', 'WPAltError %s' % err_str, row=5)
        self.console.set_status('CAlt', 'Alt %s' % self.height_string(pos[2]-home_alt), row=5)
        self.console.set_status('PrevAlt', 'PrevAlt %s' % self.height_string(self.last_wp_pos[2]-home_alt), row=5)
        self.console.set_status('NextAlt', 'NextAlt %s' % self.height_string(self.next_wp_pos[2]-home_alt), row=5)
        self.console.set_status('P', 'P %.2f' % proportion, row=5)

    def mavlink_packet(self, msg):
        '''handle an incoming mavlink packet'''
        if msg.get_type() == 'SERVO_OUTPUT_RAW':
            self.console.set_status('RearLeft', 'RearLeft %.1f' % self.ServoAngle(msg,2,FULL_DEFLECTION), row=6)
            self.console.set_status('RearRight', 'RearRight %.1f' % self.ServoAngle(msg,4,FULL_DEFLECTION,-1), row=6)
            self.console.set_status('FrontLeft', 'FrontLeft %.1f' % self.ServoAngle(msg,3,FULL_DEFLECTION), row=6)
            self.console.set_status('FrontRight', 'FrontRight %.1f' % self.ServoAngle(msg,1,FULL_DEFLECTION,-1), row=6)

        if self.status.flightmode != 'AUTO':
            self.wp_current = None
            self.last_wp_pos = None
            self.next_wp_pos = None
            return

        type = msg.get_type()

        if type == 'MISSION_CURRENT':
            if self.wp_current is None:
                # first waypoint
                self.wp_current = msg.seq
                self.next_wp_pos = self.wp_lookup(msg.seq)
                self.last_wp_pos = self.pos
            elif self.wp_current != msg.seq:
                # moved to new wp
                self.wp_current = msg.seq
                self.last_wp_pos = self.next_wp_pos
                self.next_wp_pos = self.wp_lookup(msg.seq)

        if type == 'GLOBAL_POSITION_INT':
            self.pos = (msg.lat*1.0e-7, msg.lon*1.0e-7, msg.alt*1.0e-3)
            if self.last_wp_pos is None:
                self.last_wp_pos = self.pos
            if self.last_wp_pos is not None and self.next_wp_pos is not None:
                self.update_alt_error(self.pos)


def init(mpstate):
    '''initialise module'''
    return WPSlopeModule(mpstate)
