#!/usr/bin/env python

# Copyright 2020 Boston Dynamics Inc.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from rospy import Publisher
from rospy.msg import args_kwds_to_message

class CachingPublisher(Publisher):
    def publish(self, *args, **kwargs):
        self.payload = args_kwds_to_message(self.data_class, args, kwargs)
        super(CachingPublisher, self).publish(*args, **kwargs)

    def get_payload(self):
        payload = self.payload
        self.payload = None
        return payload

    def __init__(self, *args, **kwargs):
        self.payload = None

        Publisher.__init__(self, *args, **kwargs)
