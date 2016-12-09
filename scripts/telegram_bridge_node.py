#!/usr/bin/env python
# -*- coding: utf-8 -*-
#

from telegram.ext import Updater, CommandHandler, MessageHandler, Filters
from telegram import ParseMode, Chat
import rospy
from std_msgs.msg import String
from telegram_bridge.srv import TelegramTextUpdate
from telegram_bridge.srv import TelegramCommand
from rosservice import rosservice_find
from cv_bridge import CvBridge
import cv2
from io import BytesIO
from mongodb_store.message_store import MessageStoreProxy
from rostopic import get_topic_class
from pprint import pformat

class TelegramBridge:
    def __init__(self):
        # Create the EventHandler and pass it your bot's token.
        self.updater = Updater(
            rospy.get_param('~token',
                            '283082068:AAG_68rJlZAX9ny6lZoO4YlIbN0mKBpUSiY')
            )
        self.service_map = {}

        # self.chats_ms = MessageStoreProxy(database="ros_telegram",
        #                                   collection="chats")

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

        self.text_put = rospy.Publisher('~text_msg', String, queue_size=10)
        self.text_service_proxy = rospy.ServiceProxy('~text_srv',
                                                     TelegramTextUpdate)
        self.discover_services()
        self.users = [int(u)
                      for u in rospy.get_param('~users',
                                               '325388792 209662680'
                                               ).split(' ')]
        self.subscriptions = {}
        self.chats = set([])

    def check_allowed(self, bot, update):
        if bot.get_chat(update.message.chat_id).type is Chat.GROUP:
            admins = self.get_admin_ids(bot, update.message.chat_id)
            if update.message.from_user.id in admins:
                return True
        else:
            if update.message.from_user.id in self.users:
                return True
        # error handling:
        update.message.reply_text("Sorry, I won't talk to "
                                  "you as I don't know you personally.")
        rospy.logwarn('unknown user %s tried to log in: %s %s %d' %
                      (
                        update.message.from_user.username,
                        update.message.from_user.first_name,
                        update.message.from_user.last_name,
                        update.message.from_user.id
                      )
                      )
        return False

    def get_admin_ids(self, bot, chat_id):
        """Returns a list of admin IDs for a given chat."""
        return [admin.user.id for admin in bot.getChatAdministrators(chat_id)]

    def start(self, bot, update):
        update.message.reply_text('Hi %s!'
                                  ' You are talking to an actual robot.' %
                                  update.message.from_user.first_name)
        if self.check_allowed(bot, update):
            rospy.loginfo('new user %s logged in: %s %s %d' %
                          (
                            update.message.from_user.username,
                            update.message.from_user.first_name,
                            update.message.from_user.last_name,
                            update.message.from_user.id
                          )
                          )
            self.register_topics(bot, update.message.chat_id)
            # update.message.reply_text('registered topics')
            self.help(bot, update)
            self.chats.add(update.message.chat_id)
            rospy.loginfo('currently active chats: %s' % pformat(self.chats))

    def topic_callback(self, msg, topic, bot, chat_id):
        rospy.loginfo('received update on topic %s (type: %s)' %
                      (topic, msg._type))
        if msg._type == "sensor_msgs/Image":
            img = self.bridge.imgmsg_to_cv2(msg,
                                            desired_encoding='passthrough')
            f = BytesIO(cv2.imencode('.png', img)[1].tostring())
            bot.send_message(chat_id,
                             'I have this image for you from topic '
                             '"<code>%s</code>":' % topic,
                             parse_mode=ParseMode.HTML)
            bot.send_photo(chat_id, photo=f)
        else:
            rospy.loginfo(pformat(msg))
            bot.send_message(chat_id,
                             'I have some update for you from topic '
                             '"<code>%s</code>":'
                             '<pre>%s</pre>' % (topic, pformat(msg)),
                             parse_mode=ParseMode.HTML)

    def register_topics(self, bot, chat_id):
        default_topics = ' '.join([
            '/notification',
            '/notification_image'
            ])
        topics = rospy.get_param('~subscribed_topics',
                                 default_topics).split(' ')
        for t in topics:
            try:
                msg_class, _, _ = get_topic_class(t)
                if msg_class is not None:
                    if chat_id not in self.subscriptions:
                        self.subscriptions[chat_id] = {}
                    if t not in self.subscriptions[chat_id]:
                        s = rospy.Subscriber(t, msg_class,
                                             lambda msg, topic=t, bot=bot,
                                             chat_id=chat_id:
                                             self.topic_callback(msg,
                                                                 topic, bot,
                                                                 chat_id),
                                             queue_size=1)
                        self.subscriptions[chat_id][t] = s
                        rospy.loginfo('register subscriber for topic'
                                      ' %s with type %s' %
                                      (t, str(msg_class)))
                else:
                    rospy.logwarn("couldn't determine type of topic '%s'."
                                  " So not subscribing to it." % t)

            except:
                rospy.logwarn("couldn't subscribe to topic %s" % t)

    def dispatch_command(self, bot, update, user_data, cmd, service):
        if not self.check_allowed(bot, update):
            return

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
                update.message.reply_text('<pre>'+answer.json+'</pre>',
                                          parse_mode=ParseMode.HTML)

            user_data['last_answer'] = answer.response
        except Exception as e:
            rospy.logerr(e)
            update.message.reply_text('error occured when you said "%s": %s' %
                                      (update.message.text, str(e)))

    def help(self, bot, update):
        # self.discover_services()
        if not self.check_allowed(bot, update):
            return

        sv = ['/'+s for s in self.service_map.keys()]

        srv_str = '\n'.join(sv)
        update.message.reply_text('Known commands are: \n%s' % srv_str)

    def discover_services(self):
        rospy.loginfo('looking for services')
        # services = rosservice_find('telegram_bridge/TelegramCommand')
        # TEMP HACK:
        default_services = ' '.join([
            '/aaf_telegram_service/battery',
            '/aaf_telegram_service/look',
            '/aaf_telegram_service/schedule',
            '/aaf_telegram_service/where'
            ])
        services = rospy.get_param('~services', default_services).split(' ')
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
        if not self.check_allowed(bot, update):
            return
        self.text_put.publish(update.message.text)

        # try:
        #     answer = self.text_service_proxy.call(update.message.text)
        #     rospy.loginfo('%s => %s' % (update.message.text, answer.response))
        #     update.message.reply_text(answer.response)
        # except Exception as e:
        #     update.message.reply_text('error occured when you said "%s": %s' %
        #                               (update.message.text, str(e)))

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

