import rclpy
import serial
import struct
import numpy as np
import math
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float64, Int8, UInt8MultiArray
from sensor_msgs.msg import NavSatFix, Imu

teensySerial = serial.Serial('/dev/TeensyCOM', baudrate=115200, timeout=0.1)
print_imu_diagnosis = 0
compass_incorrect_cov = math.pi/3
compass_correct_cov = 3.0*math.pi/180.0
gyro_incorrect_cov = math.pi/3
gyro_correct_cov = 5.0238538949e-4
accel_incorrect_cov = 10.0
accel_correct_cov = 6.0952

class IMUGPS(Node):
    def __init__(self):
        super().__init__('IMU_GPS') #namespace="R1")   #node name, namespace
        ns = self.get_namespace()
        if ns == "/":
            nsf = ""
        self.pubGPS = self.create_publisher(NavSatFix, 'Teensy/GPS', 10)   #msg type, topic name, queue size
        self.pubIMU = self.create_publisher(Imu, 'Teensy/IMU', 10)
        self.pubCompass = self.create_publisher(Float64, 'Teensy/Compass', 10)
        self.pubCompass_status = self.create_publisher(Int8, 'Teensy/Compass_status', 10)
        self.pubGyro_status = self.create_publisher(Int8, 'Teensy/Gyro_status', 10)
        self.pubAccel_status = self.create_publisher(Int8, 'Teensy/Accel_status', 10)
        self.pubXbeeRX = self.create_publisher(UInt8MultiArray, 'Teensy/XbeeRX', 10)

        #self.pubtemp = self.create_publisher(UInt8MultiArray, 'Teensy/XbeeTX', 10)

        self.timer = self.create_timer(0.005, self.dataPub)  #period time in sec, function name

        self.subscription_xbee = self.create_subscription(UInt8MultiArray, 'Teensy/XbeeTX', self.xbee_callback, 10)

    def dataPub(self):
        Compass = Float64()
        CompassCal = Int8()
        GyroCal = Int8()
        AccelCal = Int8()
        teensySerial.write(b'\x01')
        if teensySerial.in_waiting:
            teensy_cmd, = struct.unpack('B', teensySerial.read(1))
            if teensy_cmd == 1:
                compassCal, =  struct.unpack('B', teensySerial.read(1))
                gyroCal, = struct.unpack('B', teensySerial.read(1))
                accelCal, = struct.unpack('B', teensySerial.read(1))
                imuMsg = Imu()
                imuMsg.header.frame_id = "imu_frame"
                imuMsg.header.stamp = self.get_clock().now().to_msg()
                imuMsg.orientation.x, = struct.unpack('f', teensySerial.read(4))
                imuMsg.orientation.y, = struct.unpack('f', teensySerial.read(4))
                imuMsg.orientation.z, = struct.unpack('f', teensySerial.read(4))
                imuMsg.orientation.w, = struct.unpack('f', teensySerial.read(4))
                imuMsg.angular_velocity.x, = struct.unpack('f', teensySerial.read(4))
                imuMsg.angular_velocity.y, = struct.unpack('f', teensySerial.read(4))
                imuMsg.angular_velocity.z, = struct.unpack('f', teensySerial.read(4))
                imuMsg.linear_acceleration.x, = struct.unpack('f', teensySerial.read(4))
                imuMsg.linear_acceleration.y, = struct.unpack('f', teensySerial.read(4))
                imuMsg.linear_acceleration.z, = struct.unpack('f', teensySerial.read(4))
                Compass.data, = struct.unpack('f', teensySerial.read(4))
                if imuMsg.orientation.x + imuMsg.orientation.y + imuMsg.orientation.z + imuMsg.orientation.w == 0:
                    self.get_logger().info('BNO055 is dead.')
                else:
                    Compass.data += 90
                    if Compass.data > 360:
                        Compass.data -= 360
                    self.pubCompass.publish(Compass)
                    CompassCal.data = compassCal
                    GyroCal.data = gyroCal
                    AccelCal.data = accelCal
                    self.pubCompass_status.publish(CompassCal)
                    self.pubGyro_status.publish(GyroCal)
                    self.pubAccel_status.publish(AccelCal)

                    if compassCal == 0:
                        cov_compass = compass_incorrect_cov
                    else:
                        cov_compass = compass_correct_cov
                    if gyroCal == 0:
                        cov_gyro = gyro_incorrect_cov
                    else:
                        cov_gyro = gyro_correct_cov
                    if accelCal == 0:
                        cov_accel = accel_incorrect_cov
                    else:
                        cov_accel = accel_correct_cov
                    imuMsg.orientation_covariance = [cov_compass, 0., 0.,
                                                     0., cov_compass, 0.,
                                                     0., 0., cov_compass]
                    imuMsg.angular_velocity_covariance = [cov_gyro, 0., 0.,
                                                          0., cov_gyro, 0.,
                                                          0., 0., cov_gyro]
                    imuMsg.linear_acceleration_covariance = [cov_accel, 0., 0.,
                                                             0., cov_accel, 0.,
                                                             0., 0., cov_accel]  
                    self.pubIMU.publish(imuMsg)
                    if print_imu_diagnosis:
                        if compassCal == 0:
                            self.get_logger().info('Compass is not fixed.')
                        if gyroCal == 0:
                            self.get_logger().info('Gyro is not fixed.')
                        if accelCal == 0:
                            self.get_logger().info('Accel is not fixed.')

            elif teensy_cmd == 2:
                GPSfix, = struct.unpack('B', teensySerial.read(1))
                latitudeData, = struct.unpack('d', teensySerial.read(8))
                longitudeData, = struct.unpack('d', teensySerial.read(8))
                GPSmsg = NavSatFix()
                GPSmsg.header.frame_id = "gps_frame"
                GPSmsg.header.stamp = self.get_clock().now().to_msg()
                GPSmsg.latitude = latitudeData
                GPSmsg.longitude = longitudeData
                GPSmsg.position_covariance = [6.25, 0., 0.,
                                              0., 6.25, 0.,
                                              0., 0., 0.]  
                if GPSfix == 0:
                    self.get_logger().info('GPS is not fixed.')
                    GPSmsg.status.status = -1
                else:
                    GPSmsg.status.status = 0
                self.pubGPS.publish(GPSmsg)

            elif teensy_cmd == 3:
                xbeeRXmsg = UInt8MultiArray()
                XbeeRXLength, = struct.unpack('B', teensySerial.read(1))
                XbeeRXData = teensySerial.read(XbeeRXLength+1)
                xbeeRXmsg.data = XbeeRXData
                self.pubXbeeRX.publish(xbeeRXmsg)
            
                
    def xbee_callback(self, msg):
        teensySerial.write(b'\x03')
        teensySerial.write(len(msg.data).to_bytes(1, 'big'))
        teensySerial.write(msg.data)


def main():
    rclpy.init()
    my_pub = IMUGPS()

    try:
        rclpy.spin(my_pub)
    except KeyboardInterrupt:
        my_pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
