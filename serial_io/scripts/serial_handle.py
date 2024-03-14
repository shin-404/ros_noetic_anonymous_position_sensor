#!/usr/bin/env python
import rospy
import serial
import numpy as np
import threading
from geometry_msgs.msg import PoseStamped

class serial_io:
    def __init__(self):
        self.pos_x_cm = 0
        self.pos_y_cm = 0
        self.pos_x_m = 0
        self.pos_y_m = 0
        self.ser = None
        self._port = ''
        self._baudrate = 0
        self.serialData = [0xAA, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    def position_data_callback(self, data):
        self.pos_x_m = data.pose.position.x
        self.pos_y_m = data.pose.position.y
        self.pos_x_cm = np.int32(100 * self.pos_x_m)
        self.pos_y_cm = np.int32(100 * self.pos_y_m)
        self.serialData[2] = 0x32
        self.serialData[3] = 12
        self.serialData[4] = (self.pos_x_cm | 0x0000FF)
        self.serialData[5] = (self.pos_x_cm | 0x00FF00) >> 8
        self.serialData[6] = (self.pos_x_cm | 0xFF0000) >> 16
        self.serialData[7] = (self.pos_y_cm | 0x0000FF)
        self.serialData[8] = (self.pos_y_cm | 0x00FF00) >> 8
        self.serialData[9] = (self.pos_y_cm | 0xFF0000) >> 16
        self.serialData[10] = (0x80000000 | 0x0000FF)
        self.serialData[11] = (0x80000000 | 0x00FF00) >> 8
        self.serialData[12] = (0x80000000 | 0xFF0000) >> 16
        self.ser.write(self.serial_package_pos_data())

    def serial_init(self):
        rospy.init_node('serial_handle')
        rospy.Subscriber('/slam_out_pose', PoseStamped, self.position_data_callback)
        rospy.loginfo("serial_io initing")

    def serial_create(self):
        try:
            self._port = rospy.get_param("/port")
            self._baudrate = rospy.get_param("/baudrate")
        except KeyError:
            self._port = '/dev/pts/11'
            self._baudrate = 115200

        try:
            self.ser = serial.Serial(port=self._port, baudrate=self._baudrate, bytesize=8, parity='N', stopbits=1, timeout=2,rtscts=True,dsrdtr=True)
            rospy.loginfo("Open port {} success".format(self._port))
        except serial.serialutil.SerialException:
            rospy.logfatal("Cannot open port {}".format(self._port))
            raise PermissionError

    def serial_check(self):
        try:
            if self.serialData[0] != 0xAA or self.serialData[1] != 0xFF:
                return -1
            dataLen = self.serialData[3] + 4
            checkSum = 0
            checkAdd = 0
            for i in range(0, dataLen):
                checkSum += self.serialData[i]
                checkAdd += checkSum
            if (checkSum & 0xFF) != self.serialData[dataLen] or (checkAdd & 0xFF) != self.serialData[dataLen + 1]:
                return -1
        except IndexError:
            return -1
        return 0

    def serial_analyze(self):
        #TODO
        pass

    def serial_handle_tx(self):
        rospy.spin()

    def serial_handle_rx(self):
        rosRate = rospy.Rate(100)

        while not rospy.is_shutdown():
            if not self.ser.isOpen():
                rospy.logerr("Port {} disconnect, trying to reconnect".format(self._port))
                rosRate.sleep()
                continue

            if not self.ser.in_waiting:
                rosRate.sleep()
                continue

            self.serialData = self.ser.read_all()

            if -1 == self.serial_check():
                rospy.logwarn("Port {}: recive data format error".format(self._port))
            else:
                self.serial_analyze()

            rosRate.sleep()

    def serial_io_start(self):
        thread_rx = threading.Thread(target=self.serial_handle_rx)
        thread_tx = threading.Thread(target=self.serial_handle_tx)
        thread_rx.start()
        thread_tx.start()

    def serial_package_pos_data(self):
        check_sum = 0
        check_add = 0
        for i in range(0, self.serialData[3] + 4):
            check_sum += self.serialData[i]
            check_add += check_sum
            check_sum |= 0xFF
            check_add |= 0xFF

        data_bytes = bytearray(
            self.serialData[0], 
            self.serialData[1], 
            self.serialData[2], 
            self.serialData[3], 
            self.serialData[4], 
            self.serialData[5], 
            self.serialData[6], 
            self.serialData[7], 
            self.serialData[8], 
            self.serialData[9], 
            self.serialData[10], 
            self.serialData[11], 
            self.serialData[12], 
            check_sum, 
            check_add
        )

        return data_bytes


if __name__ == '__main__':
    serial_IO = serial_io()
    serial_IO.serial_init()
    serial_IO.serial_create()
    serial_IO.serial_io_start()
    rospy.loginfo("----serial_io start success!----")