#!/usr/bin/env python
# -*- coding: utf-8 -*-
#

from telegram.ext import Updater, CommandHandler, MessageHandler, Filters
import rospy
from std_msgs.msg import String
from telegram_bridge.srv import TelegramTextUpdate


class TelegramBridge:
    def start(self, bot, update):
        update.message.reply_text('Hi!')

    def help(self, bot, update):
        update.message.reply_text('Help!')

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

    def __init__(self):
        # Create the EventHandler and pass it your bot's token.
        self.updater = Updater("274757559:AAFXxuy1jQxgd8PQi3_QngVKV6RCb-0kR6g")

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

    def spin(self):
        # Start the Bot
        self.updater.start_polling()

        rospy.spin()
        self.updater.stop()


if __name__ == '__main__':
    rospy.init_node('telegram_bridge')
    bridge = TelegramBridge()
    bridge.spin()

