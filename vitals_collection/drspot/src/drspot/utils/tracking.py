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

CV2_SUPPORTED_TRACKERS = ['CSRT']
CV2_SUPPORTED_TEMPLATE_DICT = dict(TM_CCORR_NORMED=cv2.TM_CCORR_NORMED)

class ROITracker(object):
    def __init__(self, tracker_type):
        self.ok = False
        self.bbox = np.array([0, 0, 0, 0])

        self.update_metadata = None

        self.tracker_type = tracker_type

        if tracker_type in CV2_SUPPORTED_TRACKERS:
            try:
                self.tracker = cv2.Tracker_create(tracker_type)
            except AttributeError:
                if tracker_type == 'CSRT':
                    self.tracker = cv2.TrackerCSRT_create()
        elif tracker_type in CV2_SUPPORTED_TEMPLATE_DICT.keys():
            pass

    def get_center(self):
        return np.array([(self.bbox[0] + self.bbox[2]) * 0.5,
                         (self.bbox[1] + self.bbox[3]) * 0.5])

    def start_track(self, frame, bbox):
        self.bbox = np.array(bbox)
        if self.tracker_type in CV2_SUPPORTED_TRACKERS:
            temp_bbox = (int(bbox[0]), int(bbox[1]),
                         int(bbox[2] - bbox[0]), int(bbox[3] - bbox[1]))
            self.tracker.init(frame, temp_bbox)
        elif self.tracker_type in CV2_SUPPORTED_TEMPLATE_DICT.keys():
            ibx = self.bbox.astype(int)
            self.template = frame[ibx[1]:ibx[3], ibx[0]:ibx[2]]
        self.ok = True

    def update(self, frame, expected_location=None):
        if not self.ok:
            return

        if self.tracker_type in CV2_SUPPORTED_TRACKERS:
            self.ok, temp_bbox = self.tracker.update(frame)
            self.bbox = np.array([temp_bbox[0], temp_bbox[1],
                                  temp_bbox[0] + temp_bbox[2],
                                  temp_bbox[1] + temp_bbox[3]])
        elif self.tracker_type in CV2_SUPPORTED_TEMPLATE_DICT.keys():
            # TODO - update the tracking status properly.
            self.ok = True
            res = cv2.matchTemplate(frame, self.template,
                                    CV2_SUPPORTED_TEMPLATE_DICT[self.tracker_type])
            if expected_location is not None:
                y, x = np.ogrid[0.0:res.shape[0], 0.0:res.shape[1]]
                x -= expected_location[0]; y -= expected_location[1]
                dist_sq_with_min = x**2 + y**2
                # TODO - make the distance cost layer configurable.
                dist_sq_with_min[dist_sq_with_min < 200] = 200
                tshape = self.template.shape
                dist_sq_with_min *= tshape[0] * tshape[1] / 100.0
                if self.tracker_type in ['TM_SQDIFF', 'TM_SQDIFF_NORMED']:
                    # Minimum value is best, so add distance from expected location.
                    res += dist_sq_with_min
                else:
                    # Maximum value is best, so subtract distance from expected location.
                    res -= dist_sq_with_min
            self.update_metadata = res
            valn, valx, locn, locx = cv2.minMaxLoc(res)
            if self.tracker_type in ['TM_SQDIFF', 'TM_SQDIFF_NORMED']:
                loc = locn # Minimum value is best.
            else:
                loc = locx # Maximum value is best.
            self.bbox[0] = loc[0]; self.bbox[1] = loc[1]
            self.bbox[2] = loc[0] + self.template.shape[1]
            self.bbox[3] = loc[1] + self.template.shape[0]
