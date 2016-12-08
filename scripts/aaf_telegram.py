#!/usr/bin/env python
# -*- coding: utf-8 -*-
#

import rospy
from telegram_bridge.srv import TelegramTextUpdate
from telegram_bridge.srv import TelegramCommand
from sensor_msgs.msg import Image



class AAFService:
    def __init__(self):
        self.text_service = rospy.Service('~look',
                                          TelegramCommand,
                                          self.look)
        self.text_service = rospy.Service('/telegram_bridge/text_srv',
                                          TelegramTextUpdate,
                                          self.echo)

    def look(self, question):
        json = ''
        try:
            image = rospy.wait_for_message('/head_xtion/rgb/image_color',
                                           Image,
                                           timeout=5)
        except:
            image = None
        response = "you asked %s" % question.message
        return [response, json, image]

    def echo(self, question):
        return "echoing your message: %s" % question.message

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('aaf_telegram_service', anonymous=True)
    es = AAFService()
    es.spin()


