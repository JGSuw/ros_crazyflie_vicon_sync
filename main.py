import sys
import struct
import logging
import rospy

# import crazyflie-lib-python packages
import cflib.crazyflie
import cflib.crtp
from cflib.crtp import crtpstack
from cflib.crazyflie.log import LogConfig

from cf_vicon_sync.srv import *

logging.basicConfig()

GCS_SYNC_PORT = 8
TIMESTAMP_REQUEST_CHANNEL = 0
LED_TOGGLE_CHANNEL = 1
"""
"""
class Node :

    def __init__ ( self ) :

        rospy.init_node( 'cf_vicon_sync' )

        self.crazyflie = cflib.crazyflie.Crazyflie()

        # Advertize services
        self.get_interfaces_srv = rsopy.Service ( 'get_interfaces', GetInterfaces, self.get_interfaces )

        self.connect_srv = rospy.Service ( 'connect', Connect, self.connect )

        self.disconnect_srv = rospy.Service ('disconnect', Disconnect, self.disconnect )

        self.ir_led_toggle_srv = rospy.Service ( 'toggle_ir_led', ToggleIrLed, self.toggle_ir_led )

        self.flash_ir_led = rospy.Service ( 'flash_ir_led', FlashIrLed, self.flash_ir_led )

        self.crazyflie.add_port_callback ( GCS_SYNC_PORT, self.handle_packet )

        # Subscribe to topics
        self.marker_sub = rospy.Subscriber( 'vicon/markers', Markers, self.marker_callback )


    """
    Opens an indexed link interface to a crazyflie
    """
    def connect ( self, request ) :

        if self.cf.connected_ts is not None:
            return "already connected to a crazyflie"

        else :
            interfaces = cflib.crtp.scan_interfaces( None )
            try:
                interface = interfaces[request.index].pop(0)
                self.cf.connect(interface)
            except IndexError as e :
                exc = rospy.ServiceException("Service could not process request: " + str(e) )
                raise(exc)

    """
    Closes an open link to a crazyflie
    """
    def disconnect ( self ) :

        if self.cf.connected_ts is not None :
            self.cf.close_link()
            return "ok, disconnected"
        else : return "nothing to disconnect"

    """
    Sets the state of the active marker on the crazyflie to on or off
    """
    def toggle_ir_led ( self, request ) :

        if self.cf.connected_ts is not None :

            packet = crtpstack.CRTPPacket()
            packet._set_port ( GCS_SYNC_PORT )
            packet._set_channel ( LED_TOGGLE_CHANNEL )
            packet._set_data ( struct.pack('b', request.led_state ) )
            self.crazyflie.send_packet ( packet )
            return "ok"

        else : return "not connected"


    """
    Pulses the active marker on the crazyflie for 100ms every 1s, flash_count
    number of times.
    """
    def flash_ir_led ( self, flash_count ) :

        if self.cf.connected_ts is not None :

            for i in range ( request.flash_count ):

                packet = crtpstack.CRTPPacket()
                packet._set_port ( GCS_SYNC_PORT )
                packet._set_channel ( TIMESTAMP_REQUEST_CHANNEL )
                packet.set_channel ( LED_TOGGLE_CHANNEL )

                timestamp = rospy.get_time()

                packet._set_data ( struct.pack('ii', timestamp.secs, timestamp.nsecs) )

                self.crazyflie.send_packet ( packet )

                rospy.sleep(1.)

            return "ok"

        else : return "not connected"

if __name__ == "__main__" :

    cflib.crtp.init_drivers ( enable_debug_driver = False )
    # Connect to the crazyflie
    interfaces = cflib.crtp.scan_interfaces(None)

    if len(interfaces) > 0 :
        link = interfaces[0][0]
        crazyflie = cflib.crazyflie.Crazyflie()
        crazyflie.open_link ( interfaces[0][0] )

        node = Node ( crazyflie )

        rospy.spin()
