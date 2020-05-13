import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding import QtCore
from python_qt_binding.QtGui import QPalette

RESP_RATE_TOPIC = 'full_resp_rate'
HEART_RATE_TOPIC = 'full_heart_rate'
SKIN_TEMP_TOPIC = 'full_skin_temp'
SPO2_TOPIC = 'full_spo2'

VALID_DURATION_SEC = 5.0

OLD_PALETTE = QPalette(QtCore.Qt.black)
OFF_NOMINAL_PALETTE = QPalette(QtCore.Qt.red)
NOMINAL_PALETTE = QPalette(QtCore.Qt.green)

from std_msgs.msg import Float32

class VitalsReadout(Plugin):

    def __init__(self, context):
        super(VitalsReadout, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('VitalsReadout')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resources" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('drspot'), 'resources', 'VitalsReadout.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('VitalsReadoutUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.resp_rate_sub = rospy.Subscriber(RESP_RATE_TOPIC, Float32, self.resp_rate_callback,
                                              queue_size=1)
        self.heart_rate_sub = rospy.Subscriber(HEART_RATE_TOPIC, Float32, self.heart_rate_callback,
                                               queue_size=1)
        self.skin_temp_sub = rospy.Subscriber(SKIN_TEMP_TOPIC, Float32, self.skin_temp_callback,
                                              queue_size=1)
        self.spo2_sub = rospy.Subscriber(SPO2_TOPIC, Float32, self.spo2_callback,
                                         queue_size=1)

        self.resp_rate_timeout_callback(None)
        self.heart_rate_timeout_callback(None)
        self.skin_temp_timeout_callback(None)
        self.spo2_timeout_callback(None)

        self.resp_rate_timeout = None
        self.heart_rate_timeout = None
        self.skin_temp_timeout = None
        self.spo2_timeout = None

    def resp_rate_callback(self, data):
        self._widget.resp_rate.display(int(data.data))
        rospy.logdebug('resp_rate_callback')
        if self.resp_rate_timeout is not None:
            self.resp_rate_timeout.shutdown()
        self.resp_rate_timeout = rospy.Timer(rospy.Duration(VALID_DURATION_SEC),
                                             self.resp_rate_timeout_callback, oneshot=True)
        self._widget.resp_rate.setPalette(NOMINAL_PALETTE)

    def resp_rate_timeout_callback(self, event):
        rospy.loginfo('resp_rate_timeout_callback')
        self._widget.resp_rate.setPalette(OLD_PALETTE)

    def heart_rate_callback(self, data):
        self._widget.heart_rate.display(int(data.data))
        rospy.logdebug('heart_rate_callback')
        if self.heart_rate_timeout is not None:
            self.heart_rate_timeout.shutdown()
        self.heart_rate_timeout = rospy.Timer(rospy.Duration(VALID_DURATION_SEC),
                                              self.heart_rate_timeout_callback, oneshot=True)
        self._widget.heart_rate.setPalette(NOMINAL_PALETTE)

    def heart_rate_timeout_callback(self, event):
        rospy.loginfo('heart_rate_timeout_callback')
        self._widget.heart_rate.setPalette(OLD_PALETTE)

    def spo2_callback(self, data):
        self._widget.spo2.display(int(data.data))
        rospy.logdebug('spo2_callback')
        if self.spo2_timeout is not None:
            self.spo2_timeout.shutdown()
        self.spo2_timeout = rospy.Timer(rospy.Duration(VALID_DURATION_SEC),
                                        self.spo2_timeout_callback, oneshot=True)
        self._widget.spo2.setPalette(NOMINAL_PALETTE)

    def spo2_timeout_callback(self, event):
        rospy.loginfo('spo2_timeout_callback')
        self._widget.spo2.setPalette(OLD_PALETTE)

    def skin_temp_callback(self, data):
        rospy.logdebug('skin_temp_callback')
        self._widget.skin_temp.display('{:.1f}'.format(data.data))
        if self.skin_temp_timeout is not None:
            self.skin_temp_timeout.shutdown()
        self.skin_temp_timeout = rospy.Timer(rospy.Duration(VALID_DURATION_SEC),
                                             self.skin_temp_timeout_callback, oneshot=True)
        self._widget.skin_temp.setPalette(NOMINAL_PALETTE)

    def skin_temp_timeout_callback(self, event):
        rospy.loginfo('skin_temp_timeout_callback')
        self._widget.skin_temp.setPalette(OLD_PALETTE)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
