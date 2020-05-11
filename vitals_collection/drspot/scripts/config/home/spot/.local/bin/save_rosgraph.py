#!/usr/bin/env python

import sys
from python_qt_binding.QtWidgets import QApplication, QWidget, QVBoxLayout
from python_qt_binding.QtCore import QFile, QIODevice, QObject
from rqt_graph.ros_graph import RosGraph

class FakePluginContext(QObject):
    def __init__(self):
        super(FakePluginContext, self).__init__()
        self.setObjectName('FakePluginContext')

    def serial_number(self):
        return 0

    def add_widget(self, widget):
        pass

if __name__ == "__main__":
    app = QApplication(sys.argv)
    fpc = FakePluginContext()
    r = RosGraph(fpc)

    handle = QFile(sys.argv[1])
    if not handle.open(QIODevice.WriteOnly | QIODevice.Text):
        exit(1)

    handle.write(r._generate_dotcode())
    handle.close()
