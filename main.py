import sys
import os
import struct
import logging
import time
import datetime
import rospy

# import crazyflie-lib-python packages
import cflib.crazyflie
import cflib.crtp
from cflib.crtp import crtpstack
from cflib.crazyflie.log import LogConfig

from cf_vicon_sync.srv import *
from vicon_bridge.msg import Marker

logging.basicConfig()

GCS_SYNC_PORT = 8
TIMESTAMP_REQUEST_CHANNEL = 0
LED_TOGGLE_CHANNEL = 1
"""
"""
class Node :

    def __init__ ( self ) :

        rospy.init_node( 'cf_vicon_sync' )

        self.output_directory = None

        self.output_files = {}


        self.crazyflie = cflib.crazyflie.Crazyflie()

        # Advertize services

        self.begin_trial_srv = rospy.Service ( 'begin_trial', BeginTrial, self.begin_trial )

        self.end_trial_srv = rospy.Service ( 'end_trial', EndTrial, self.end_trial )

        self.get_interfaces_srv = rospy.Service ( 'get_interfaces', GetInterfaces, self.get_interfaces )

        self.connect_srv = rospy.Service ( 'connect', Connect, self.connect )

        self.disconnect_srv = rospy.Service ('disconnect', Disconnect, self.disconnect )

        self.ir_led_toggle_srv = rospy.Service ( 'toggle_ir_led', ToggleIrLed, self.toggle_ir_led )

        self.flash_ir_led = rospy.Service ( 'flash_ir_led', FlashIrLed, self.flash_ir_led )

        # Subscribe to topics
        self.marker_sub = rospy.Subscriber( 'vicon/marker', Marker, self.marker_callback )

        self.crazyflie.add_port_callback ( GCS_SYNC_PORT, self.crtp_callback )



    """
    Begins a new trial (records incoming data to files), at the specified directory.
    """

    def begin_trial ( self, request ) :

        if self.output_directory is not None :
            return "trial is already in progress."

        else :
            self.output_directory = request.directory
            now = datetime.date.fromtimestamp( time.time() )
            self.output_directory += '_' + str(now)

            trial_count = 1

            if os.path.exists(self.output_directory) :
                for name in os.listdir(self.output_directory) :
                    if name.rfind('trial') >= 0 :
                        trial_count += 1

            self.output_directory += '/trial_{}'.format(trial_count)

            try :
                os.makedirs ( self.output_directory )
                os.chdir ( self.output_directory )
                return "ok, trial started at {}".format( self.output_directory )

            except OSError as e:
                error_string = str(e)
                raise ( rospy.ServiceException(error_string) )



    """
    Closes all open data files to end this trial
    """

    def end_trial ( self, request ) :

        if self.output_directory is None :
            return "no trial in progress."

        else :
            for filename in self.output_files.keys() :
                self.output_files[filename].close()
                self.output_files.pop(filename)
            self.output_files = None
            return "ok, trial finished"


    """
    Returns a stringified list of available CRTP interfaces
    """
    def get_interfaces ( self, request ) :
        interfaces = cflib.crtp.scan_interfaces( None )
        return str(interfaces)

    """
    Opens an indexed link interface to a crazyflie
    """
    def connect ( self, request ) :

        if self.crazyflie.connected_ts is not None:
            return "already connected to a crazyflie"

        else :
            interfaces = cflib.crtp.scan_interfaces( None )
            try:
                interface = interfaces[request.index].pop(0)
                self.crazyflie.open_link(interface)
                return "ok"
            except IndexError as e :
                exc = rospy.ServiceException( str(e) )
                raise(exc)



    """
    Closes an open link to a crazyflie
    """
    def disconnect ( self, request ) :

        if self.crazyflie.connected_ts is not None :
            self.crazyflie.close_link()
            return "ok, disconnected"
        else : return "nothing to disconnect"



    """
    Sets the state of the active marker on the crazyflie to on or off
    """
    def toggle_ir_led ( self, request ) :

        if self.crazyflie.connected_ts is not None :

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
    def flash_ir_led ( self, request ) :

        if self.crazyflie.connected_ts is not None :

            for i in range ( request.flash_count ):

                packet = crtpstack.CRTPPacket()
                packet._set_port ( GCS_SYNC_PORT )
                packet._set_channel ( TIMESTAMP_REQUEST_CHANNEL )
                timestamp = rospy.get_rostime()

                packet._set_data ( struct.pack('ii', timestamp.secs, timestamp.nsecs) )

                self.crazyflie.send_packet ( packet )

                rospy.sleep(1.)

            return "ok"

        else : return "not connected"



    """
    Handles marker messages, writing the data for each marker to its own file
    """
    def marker_callback ( self, data ) :
        filename = "{}_{}.csv".format( data.subject_name, data.marker_name )


        seconds = data.header.stamp.seconds + data.header.stamp.nanoseconds / 1e9

        line = "{},{},{},{}\n".format( data.x,
                                     data.y,
                                     data.z,
                                     seconds )

        if self.output_directory is not None :
            if filename not in self.output_files.keys() :
                self.output_files[filename] = open ( filename, 'w')

            self.output_files[filename].write( line )



    """
    Handles packets from the crazyflie, writing timestamps to a file
    """
    def crtp_callback ( self, packet ) :

        print("incoming packet!")

        ( seconds, nseconds, cfstamp ) = struct.unpack ( 'iii',
                                                   packet._get_data() )

        posixstamp = seconds + nseconds / 1e9

        line = "{},{}\n".format( posixstamp, cfstamp )

        if self.output_files is not None :
            if 'cf_timestamps.csv' not in self.output_files.keys() :
                self.output_files['cf_timestamps.csv'] = open ( 'cf_timestamps.csv', 'w' )
            self.output_files['cf_timestamps.csv'].write( line )



if __name__ == "__main__" :

    cflib.crtp.init_drivers ( enable_debug_driver = False )
    node = Node ()
    rospy.spin()
