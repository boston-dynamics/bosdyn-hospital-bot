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

import cv2
import numpy as np

class ROITracker(object):
    def __init__(self, tracker_type):
        self.ok = False
        self.bbox = np.array([0, 0, 0, 0])

        try:
            self.tracker = cv2.Tracker_create(tracker_type)
        except AttributeError:
            if tracker_type == 'BOOSTING':
                self.tracker = cv2.TrackerBoosting_create()
            if tracker_type == 'MIL':
                self.tracker = cv2.TrackerMIL_create()
            if tracker_type == 'KCF':
                self.tracker = cv2.TrackerKCF_create()
            if tracker_type == 'TLD':
                self.tracker = cv2.TrackerTLD_create()
            if tracker_type == 'MEDIANFLOW':
                self.tracker = cv2.TrackerMedianFlow_create()
            if tracker_type == 'GOTURN':
                self.tracker = cv2.TrackerGOTURN_create()
            if tracker_type == 'MOSSE':
                self.tracker = cv2.TrackerMOSSE_create()
            if tracker_type == 'CSRT':
                self.tracker = cv2.TrackerCSRT_create()

    def get_center(self):
        return np.array([(self.bbox[0] + self.bbox[2]) * 0.5,
                         (self.bbox[1] + self.bbox[3]) * 0.5])

    def start_track(self, frame, bbox):
        temp_bbox = (int(bbox[0]), int(bbox[1]), int(bbox[2] - bbox[0]), int(bbox[3] - bbox[1]))
        self.tracker.init(frame, temp_bbox)
        self.ok = True
        self.bbox = np.array(bbox)

    def update(self, frame):
        if not self.ok:
            return

        self.ok, temp_bbox = self.tracker.update(frame)
        self.bbox = np.array([temp_bbox[0], temp_bbox[1],
                              temp_bbox[0] + temp_bbox[2], temp_bbox[1] + temp_bbox[3]])
