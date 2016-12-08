#!/usr/bin/env python
# -*- coding: utf-8 -*-
#

from telegram.ext import Updater, CommandHandler, MessageHandler, Filters
from telegram import ParseMode
import rospy
from std_msgs.msg import String
from telegram_bridge.srv import TelegramTextUpdate
from telegram_bridge.srv import TelegramCommand
from rosservice import rosservice_find
from cv_bridge import CvBridge
import cv2
from io import BytesIO

class TelegramBridge:
    def __init__(self):
        # Create the EventHandler and pass it your bot's token.
        self.updater = Updater("274757559:AAFXxuy1jQxgd8PQi3_QngVKV6RCb-0kR6g")
        self.service_map = {}
        self.bridge = CvBridge()

        # Get the dispatcher to register handlers
        dp = self.updater.dispatcher

        # on different commands - answer in Telegram
        dp.add_handler(CommandHandler("start", self.start))
        dp.add_handler(CommandHandler("help", self.help))

        # on noncommand i.e message - echo the message on Telegram
        dp.add_handler(MessageHandler(Filters.text, self.echo))

        # log all errors
        dp.add_error_handler(self.error)

        self.text_put = rospy.Publisher('~text_msg', String)
        self.text_service_proxy = rospy.ServiceProxy('~text_srv',
                                                     TelegramTextUpdate)

    def start(self, bot, update):
        update.message.reply_text('Hi!')
        self.help(bot, update)

    def dispatch_command(self, bot, update, user_data, cmd, service):
        rospy.loginfo("commands %s requested, calling service %s" %
                      (cmd, service))
        rospy.loginfo("user_data: %s" %
                      user_data)

        proxy = rospy.ServiceProxy(service, TelegramCommand)
        try:
            answer = proxy.call(update.message.text, '')
            rospy.loginfo('%s => %s' % (update.message.text, answer.response))
            if answer.response is not '':
                update.message.reply_text(answer.response)
            if answer.image.width > 0:
                img = self.bridge.imgmsg_to_cv2(answer.image,
                                                desired_encoding='passthrough')
                f = BytesIO(cv2.imencode('.png', img)[1].tostring())
                update.message.reply_photo(photo=f)
            if answer.json is not '':
                update.message.reply_text('<pre>'+answer.json+'</pre>', parse_mode=ParseMode.HTML)

            user_data['last_answer'] = answer.response
        except Exception as e:
            rospy.logerr(e)
            update.message.reply_text('error occured when you said "%s": %s' %
                                      (update.message.text, str(e)))

    def help(self, bot, update):
        self.discover_services()

        sv = ['/'+s for s in self.service_map.keys()]

        srv_str = '\n'.join(sv)
        update.message.reply_text('Known commands are: \n%s' % srv_str)

    def discover_services(self):
        services = rosservice_find('telegram_bridge/TelegramCommand')
        rospy.loginfo('services found: %s' % str(services))
        dp = self.updater.dispatcher
        for s in services:
            cmd_name = s.split('/')[-1]
            if cmd_name not in self.service_map:
                dp.add_handler(CommandHandler(cmd_name,
                                              lambda bot,
                                              update,
                                              user_data,
                                              cmd=cmd_name,
                                              service=s:
                                              self.dispatch_command(bot,
                                                                    update,
                                                                    user_data,
                                                                    cmd,
                                                                    service),
                                              pass_user_data=True))
                rospy.loginfo('add handler for "%s" command: %s' %
                              (cmd_name, s))

            self.service_map[cmd_name] = s

    def echo(self, bot, update):
        self.text_put.publish(update.message.text)

        try:
            answer = self.text_service_proxy.call(update.message.text)
            rospy.loginfo('%s => %s' % (update.message.text, answer.response))
            update.message.reply_text(answer.response)
        except Exception as e:
            update.message.reply_text('error occured when you said "%s": %s' %
                                      (update.message.text, str(e)))

    def error(self, bot, update, error):
        rospy.logwarn('Update "%s" caused error "%s"' % (update, error))

    def spin(self):
        # Start the Bot
        self.updater.start_polling()

        rospy.spin()
        self.updater.stop()


if __name__ == '__main__':
    rospy.init_node('telegram_bridge')
    bridge = TelegramBridge()
    bridge.spin()

