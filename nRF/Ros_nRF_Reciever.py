import rospy
from std_msgs.msg import String

import RPi.GPIO as GPIO
from lib_nrf24 import NRF24
import time
import spidev


pub = rospy.Publisher('nRF24_msg', String, queue_size=10)
rospy.init_node('nRF24', anonymous=True)

rate = rospy.Rate(30) # 10hz






GPIO.setmode(GPIO.BCM)
pips = [[0xE8, 0xE8, 0xF0, 0xF0, 0xE1], [0xF0, 0xF0, 0xF0, 0xF0, 0xE1]]
radio = NRF24(GPIO, spidev.SpiDev())
radio.begin(0, 17)

#0xF0, 0xF0, 0xF0, 0xF0, 0xE1

radio.setPayloadSize(32)
#radio.setChannel(0x76)
radio.setDataRate(NRF24.BR_1MBPS)
radio.setPALevel(NRF24.PA_MIN)


radio.setAutoAck(True)
radio.enableDynamicPayloads()
#radio.enableAckPayload()

radio.openReadingPipe(1,pips[0])
#radio.printDetails()
radio.startListening()



while True:
    while not radio.available(0):
        time.sleep(1/100)
    receivedMessage = []
    radio.read(receivedMessage, radio.getDynamicPayloadSize())

    receivedMessage = receivedMessage.decode('utf-8')

    print("Received:{}".format(receivedMessage.decode('utf-8')))
    pub.publish(msg)
