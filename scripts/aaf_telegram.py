#!/usr/bin/env python
# -*- coding: utf-8 -*-
#

import rospy
from telegram_bridge.srv import TelegramTextUpdate
from telegram_bridge.srv import TelegramCommand
from scitos_msgs.msg import BatteryState
from sensor_msgs.msg import Image
from std_msgs.msg import String
from pprint import pformat
from rosgraph_msgs.msg import Log




class AAFService:
    def __init__(self):
        self.text_service = rospy.Service('~look',
                                          TelegramCommand,
                                          self.look)
        self.text_service = rospy.Service('~battery',
                                          TelegramCommand,
                                          self.battery)
        self.text_service = rospy.Service('~where',
                                          TelegramCommand,
                                          self.closest_node)

        self.text_service = rospy.Service('/telegram_bridge/text_srv',
                                          TelegramTextUpdate,
                                          self.echo)
        self.notification = rospy.Publisher('/notification', String,
                                            queue_size=10)

        self.rosfilter = rospy.Subscriber('/rosout_filtered', Log,
                                          self.notify_rosout, queue_size=10)

        self.last_rosout = 0

        self.max_rosout_freqency = 300

    def notify_rosout(self, msg):
        if msg.header.stamp.secs - self.last_rosout < self.max_rosout_freqency:
            rospy.logwarn('too frequent updates on rosout, skipping '
                          'for %d seconds.', self.max_rosout_freqency)
            return
        else:
            log = pformat(msg)
            rospy.loginfo('update on rosout: %s' % log)
            self.notification.publish('New relevant information on rosout:'
                                      '\n%s' % log)

    def look(self, question):
        json = ''
        try:
            image = rospy.wait_for_message('/head_xtion/rgb/image_color',
                                           Image,
                                           timeout=5)
            response = "This is what the world looks right now"
        except:
            image = None
            response = "Sorry, I waited 5 seconds and still didn't get a live image."
        return [response, json, image]

    def battery(self, question):
        image = None
        try:
            state = rospy.wait_for_message('/battery_state',
                                           BatteryState,
                                           timeout=5)
            response = "current battery status"
            json = pformat(state)
        except Exception as e:
            json = pformat(e)
            response = "Sorry, I waited 5 seconds and still didn't get a status update."
        return [response, json, image]

    def closest_node(self, question):
        image = None
        json = ''
        try:
            state = rospy.wait_for_message('/closest_node',
                                           String,
                                           timeout=5)
            response = 'My current closest node is "%s"' % state.data
        except Exception as e:
            response = "Sorry, I waited 5 seconds and still don't know where I am "
            json = pformat(e)
        return [response, json, image]

    def echo(self, question):
        return "echoing your message: %s" % question.message

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('aaf_telegram_service', anonymous=False)
    es = AAFService()
    es.spin()


