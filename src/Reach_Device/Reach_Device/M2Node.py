import rclpy
import serial
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Int8
from sensor_msgs.msg import NavSatFix

ReachSerial = serial.Serial('/dev/ReachM2', baudrate=115200, timeout=0.1)

class M2GPS(Node):
    def __init__(self):
        super().__init__('M2_GPS') #namespace="R1")   #node name, namespace
        self.pubGPS = self.create_publisher(NavSatFix, 'ReachM2/GPS', 10)   #msg type, topic name, queue size
        self.pubGPS_status = self.create_publisher(Int8, 'ReachM2/status', 10)
        self.pubGPS_ns = self.create_publisher(Int8, 'ReachM2/ns', 10)
        self.timer = self.create_timer(0.1, self.dataPub)  #period time in sec, function name

    def dataPub(self):
        if ReachSerial.in_waiting:
            status = Int8()
            ns = Int8()

            str = ReachSerial.readline()
            data = str.decode().split()     #GPS date, GPS time, latitude (deg), longitude (deg), height (m), status, number of satellites, sigma north (m), sigma east (m), sigma up (m)
            latitude = float(data[2])
            longitude = float(data[3])
            height = float(data[4])
            status.data = int(data[5])       #1:fix, 2:float, 3:sbas, 4:dgps, 5:single, 6:ppp
            ns.data = int(data[6])
            sigmaN = float(data[7])
            sigmaE = float(data[8])
            sigmaU = float(data[9])
            

            GPSmsg = NavSatFix()
            GPSmsg.header.frame_id = "M2gps_frame"
            GPSmsg.header.stamp = self.get_clock().now().to_msg()
            GPSmsg.latitude = latitude
            GPSmsg.longitude = longitude
            GPSmsg.altitude = height                
            GPSmsg.status.status = 0
            GPSmsg.position_covariance = [sigmaE*sigmaE, 0.0, 0.0, 0.0, sigmaN*sigmaN, 0.0, 0.0, 0.0, sigmaU*sigmaU]
            self.pubGPS.publish(GPSmsg)
            self.pubGPS_status.publish(status)
            self.pubGPS_ns.publish(ns)


def main():
    rclpy.init()
    my_pub = M2GPS()

    try:
        rclpy.spin(my_pub)
    except KeyboardInterrupt:
        my_pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

