#!/usr/bin/env python
# -*- coding: utf-8 -*-
#

import rospy
from telegram_bridge.srv import TelegramTextUpdate


class EchoService:
    def echo(self, question):
        return "echoing your message: %s" % question.message

    def __init__(self):
        self.text_service = rospy.Service('/telegram_bridge/text_srv',
                                          TelegramTextUpdate,
                                          self.echo)

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('telegram_echo_service', anonymous=True)
    es = EchoService()
    es.spin()


