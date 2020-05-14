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

import threading
import numpy as np
import cv2
import insightface

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PolygonStamped, Point32
from std_msgs.msg import Bool
from cv_bridge import CvBridge

from drspot.utils.tracking import ROITracker

ENABLE_TOPIC = 'multichrome_face_tracker_enable'

SCALED_RED_IMAGE_TOPIC = 'debug_mono_red_tracking_rescale'
DEBUG_RED_IMAGE_TOPIC = 'debug_mono_red_tracking'

RED_IMAGE_TOPIC = 'camera_array/mono_red/image_raw'
RED_TRACKING_STATUS_TOPIC = 'mono_red_tracking_status'
RED_CROPPED_IMAGE_TOPIC = 'mono_red_cropped'
RED_REGION_IN_IMAGE_TOPIC = 'mono_red_region'

RED_FACE_IN_REGION_TOPIC = 'mono_red_roi'

NIR_IMAGE_TOPIC = 'camera_array/mono_nir/image_raw'
NIR_TRACKING_STATUS_TOPIC = 'mono_nir_tracking_status'
NIR_CROPPED_IMAGE_TOPIC = 'mono_nir_cropped'
NIR_REGION_IN_IMAGE_TOPIC = 'mono_nir_region'

NARROW_NIR_IMAGE_TOPIC = 'camera_array/mono_narrow_nir/image_raw'
NARROW_NIR_TRACKING_STATUS_TOPIC = 'mono_narrow_nir_tracking_status'
NARROW_NIR_CROPPED_IMAGE_TOPIC = 'mono_narrow_nir_cropped'
NARROW_NIR_REGION_IN_IMAGE_TOPIC = 'mono_narrow_nir_region'

# Buffer about 1 second of full-size frames.
IMG_QUEUE_SIZE = 35
IMG_BUF_SIZE = IMG_QUEUE_SIZE * 2448 * 2048

MAX_DROPOUT_BEFORE_RESET_SEC = 1.0
MAX_TIME_SINCE_DETECTION_SEC = 2.5

RATE_CUT = False
RATE_DIV = 2

DETECT_TRACK_SCALE_FACTOR = 0.125

FACE_DETECTION_LIKELIHOOD_THRESHOLD = 0.7
MIN_FACE_DIM_PIXELS = 10
FACE_DETECTION_SCALE = 0.5
# Use a computationally cheap tracker and overbound the ROI. This tracking
# needs to operate close to frame rate. The ROI tracking is refined in the
# heart_rate_spo2 node.
CV_TRACKER_TYPE = 'CSRT'

FACE_DETECTION_INTENSITY_RANGE = 150.0
FACE_DETECTION_INTENSITY_OFFSET = (255.0 - FACE_DETECTION_INTENSITY_RANGE) / 2.0

# Shift the ROI down from the detected landmarks.
ROI_HEIGHT_FRAC_OF_FACE_DOWN = 0.1
# Overbound the region
ROI_HALF_HEIGHT_FRAC_OF_FACE = 1.3
ROI_HALF_WIDTH_FRAC_OF_FACE = 1.25

# The face height is better related to scene depth than the width is.
# Therefore, we use it as a proxy for the depth-dependent ROI offset
# we need to apply because the camera centers are offset.
NIR_OFFSET_FRAC_OF_FACE_HEIGHT = -0.35
NARROW_NIR_X_OFFSET_FRAC_OF_FACE_HEIGHT = -0.35
NARROW_NIR_Y_OFFSET_FRAC_OF_FACE_HEIGHT = -0.6

class FaceDetectorTracker(object):
    def image_callback(self, data):
        if not self.enabled:
            return

        if RATE_CUT:
            if self.RATE_CUT is None: self.RATE_CUT = 0
            self.RATE_CUT += 1
            if self.RATE_CUT == RATE_DIV:
                self.RATE_CUT = 0
            else:
                return
        det_msg = Bool()
        det_msg.data = False
        t = data.header.stamp.to_sec()
        if self.tlast is None:
            self.tlast = t
            self.tracking_status_pub.publish(det_msg)
            return

        # Detect backward jumps in time in replay.
        if self.tlast > t:
            self.clear_msmt_state()
            self.tlast = t
            rospy.logwarn_throttle(1, '{}: Backward jump in time'.format(self.name))
            self.tracking_status_pub.publish(det_msg)
            return

        # Detect skips in data.
        if t - self.tlast > MAX_DROPOUT_BEFORE_RESET_SEC:
            rospy.logwarn_throttle(1, '{}: Dropped samples for {} sec'.format(self.name, t - self.tlast))
            self.clear_msmt_state()
            self.tlast = t
            self.tracking_status_pub.publish(det_msg)
            return

        self.tlast = t

        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        (rows, cols) = cv_image.shape

        # Use our member function to make sure behavior is consistent.
        roi_unscaled = self.get_roi()

        do_detect_track = self.detect_track_thread is None or not self.detect_track_thread.is_alive()
        if do_detect_track:
            # Spawn a thread to do the detection and tracking.
            self.detect_track_thread = threading.Thread(target=self.detect_track,
                                                        name='detect_track_' + self.name,
                                                        args=(cv_image, t, data.header))
            self.detect_track_thread.start()
            # Maybe eventually keep the thread around all the time, but make it block
            # on popping from a size 1 queue. And use a nonblocking push (fail if full).

        if roi_unscaled is not None:
            xmin = int(roi_unscaled[0][0])
            ymin = int(roi_unscaled[0][1])
            xmax = int(roi_unscaled[1][0])
            ymax = int(roi_unscaled[1][1])

            if xmin < 0 or ymin < 0 or xmax >= cols or ymax >= rows:
                rospy.logwarn_throttle(1, '{}: edge of frame at {}, seq {}. {} {} {} {} {} {}'.format(
                    self.name, t, data.header.seq, xmin, xmax, cols, ymin, ymax, rows))
                if xmin < 0: xmin = 0
                if ymin < 0: ymin = 0
                if xmax >= cols: xmax = cols-1
                if ymax >= cols: ymax = cols-1

            if cv_image[ymin:ymax, xmin:xmax].size == 0:
                rospy.logwarn_throttle(1, '{}: empty image at {}, seq {}'.format(
                    self.name, t, data.header.seq))
                self.clear_msmt_state()
                return

            cv_roi = cv_image[ymin:ymax, xmin:xmax]
            msg = self.bridge.cv2_to_imgmsg(cv_roi, encoding='passthrough')
            msg.header = data.header
            self.cropped_image_pub.publish(msg)

            region = PolygonStamped()
            region.header = data.header
            region.polygon.points = [Point32(x=xmin, y=ymin, z=0),
                                     Point32(x=xmax, y=ymax, z=0)]
            self.region_pub.publish(region)

    def detect_track(self, cv_image, t, header):
        det_msg = Bool()
        det_msg.data = False

        (rows, cols) = cv_image.shape
        r = int(rows * DETECT_TRACK_SCALE_FACTOR)
        c = int(cols * DETECT_TRACK_SCALE_FACTOR)
        cv_image = cv2.resize(cv_image, (c, r), interpolation=cv2.INTER_AREA)

        imin = np.amin(cv_image)
        imax = np.amax(cv_image)
        irange = imax - imin
        if irange == 0:
            rospy.logwarn_throttle(1, '{}: Blank mono image'.format(self.name))
            self.tracking_status_pub.publish(det_msg)
            return
        #cv_image = ((cv_image - imin) * FACE_DETECTION_INTENSITY_RANGE / irange
        #            + FACE_DETECTION_INTENSITY_OFFSET).astype('uint8')

        msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='passthrough')
        msg.header = header
        self.scaled_image_pub.publish(msg)

        chan3image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)

        # If tracking is OK, track. Else, detect and configure tracker.
        bboxes = np.array([])
        landmarks = np.array([])
        fidx = 0
        self.roi_tracker.update(cv_image)
        last_detection_ok = self.tlast_detection is not None and t - self.tlast_detection < MAX_TIME_SINCE_DETECTION_SEC
        if self.roi_tracker.ok and last_detection_ok:
            bboxes = np.array([self.roi_tracker.bbox])
        else:
            if not self.roi_tracker.ok:
                rospy.logwarn_throttle(1, '{}: Lost tracking; running face detector'.format(self.name))
            else:
                rospy.loginfo_throttle(1, '{}: Time-triggered face detection'.format(self.name))
            self.roi_tracker = ROITracker(CV_TRACKER_TYPE)
            bboxes, landmarks = self.detector.detect(chan3image,
                                                     threshold=FACE_DETECTION_LIKELIHOOD_THRESHOLD,
                                                     scale=FACE_DETECTION_SCALE)

            if len(bboxes) == 0:
                self.tracking_status_pub.publish(det_msg)
                rospy.logwarn_throttle(1, '{}: no faces detected'.format(self.name))
                self.clear_msmt_state()
                return

            # TODO - If we select a specific face, re-order so that face is first (index 0).
            if len(bboxes) > 1:
                rospy.logwarn_throttle(1, '{}: {} faces detected; picking the first'.format(
                    self.name, len(bboxes)))

            self.roi_tracker.start_track(cv_image, bboxes[fidx][0:4].reshape((4)))
            self.tlast_detection = t

        wface = bboxes[fidx][2] - bboxes[fidx][0]
        hface = bboxes[fidx][3] - bboxes[fidx][1]
        if (wface < MIN_FACE_DIM_PIXELS) or (hface < MIN_FACE_DIM_PIXELS):
            self.tracking_status_pub.publish(det_msg)
            rospy.logwarn_throttle(1, '{}: face too small'.format(self.name))
            self.clear_msmt_state()
            return

        det_msg.data = True
        self.tracking_status_pub.publish(det_msg)

        midroi = self.roi_tracker.get_center()
        offset = np.array([0.0, hface * ROI_HEIGHT_FRAC_OF_FACE_DOWN])
        half_wroi = wface * ROI_HALF_WIDTH_FRAC_OF_FACE
        half_hroi = hface * ROI_HALF_HEIGHT_FRAC_OF_FACE
        roi = midroi + offset + np.array([[-half_wroi, -half_hroi],
                                          [half_wroi, half_hroi]])

        roi_unscaled = roi / DETECT_TRACK_SCALE_FACTOR
        with self.lock:
            self.roi = roi_unscaled

        region = PolygonStamped()
        region.header = header
        face_in_region = bboxes[fidx][0:4].reshape((2, 2)) / DETECT_TRACK_SCALE_FACTOR - np.tile(roi_unscaled[:][0], (2, 1))
        region.polygon.points = [Point32(x=face_in_region[0][0], y=face_in_region[0][1], z=0),
                                 Point32(x=face_in_region[1][0], y=face_in_region[1][1], z=0)]
        self.face_in_region_pub.publish(region)

        # All overlays have colors below in case we use an RGB image in the future.
        for box in bboxes.astype(int):
            x, y, x2, y2 = box[0:4]
            # Draw a rectangle for each face bounding box.
            cv2.rectangle(cv_image, (x, y), (x2, y2), (255,255,255), 2)
        for face in landmarks.astype(int):
            for point in face:
                x, y = point
                # Draw a circle at each landmark.
                cv2.circle(cv_image, (x, y), 5, (255,255,255), 1)

        roi = roi.astype(int)
        cv2.rectangle(cv_image, tuple(roi[0].tolist()), tuple(roi[1].tolist()), (255,255,255), 2)

        msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='passthrough')
        msg.header = header
        self.debug_image_pub.publish(msg)

    def clear_msmt_state(self):
        rospy.logwarn_throttle(1, '{}: Resetting'.format(self.name))
        with self.lock:
            self.roi_tracker.ok = False
            self.tlast = None
            self.tlast_detection = None
            # Share bounding box between detection/tracking thread and ROI grab.
            self.roi = None

    def get_roi(self):
        with self.lock:
            return self.roi

    def __init__(self, name,
                 tracking_status_topic,
                 cropped_image_pub, region_in_image_pub):
        if RATE_CUT:
            self.RATE_CUT = None

        self.name = name

        self.enabled = True

        self.lock = threading.Lock()
        self.detect_track_thread = None

        self.bridge = CvBridge()

        self.detector = insightface.model_zoo.get_model('retinaface_r50_v1')
        self.detector.prepare(ctx_id=-1, nms=0.4)

        self.roi_tracker = ROITracker(CV_TRACKER_TYPE)

        self.clear_msmt_state()

        self.cropped_image_pub = cropped_image_pub
        self.region_pub = region_in_image_pub
        self.tracking_status_pub = rospy.Publisher(tracking_status_topic,
                                                   Bool, queue_size=10)

        self.face_in_region_pub = rospy.Publisher(RED_FACE_IN_REGION_TOPIC,
                                                  PolygonStamped,
                                                  queue_size=10)

        self.debug_image_pub = rospy.Publisher(DEBUG_RED_IMAGE_TOPIC, Image, queue_size=10)
        self.scaled_image_pub = rospy.Publisher(SCALED_RED_IMAGE_TOPIC, Image, queue_size=10)

class CalibratedCameraTrack(object):
    def image_callback(self, data):
        if not self.enabled:
            return

        if RATE_CUT:
            if self.RATE_CUT is None: self.RATE_CUT = 0
            self.RATE_CUT += 1
            if self.RATE_CUT == RATE_DIV:
                self.RATE_CUT = 0
            else:
                return
        det_msg = Bool()
        det_msg.data = False
        t = data.header.stamp.to_sec()
        if self.tlast is None:
            self.tlast = t
            self.tracking_status_pub.publish(det_msg)
            return

        # Detect backward jumps in time in replay.
        if self.tlast > t:
            self.clear_msmt_state()
            self.tlast = t
            rospy.logwarn_throttle(1, '{}: Backward jump in time'.format(self.name))
            self.tracking_status_pub.publish(det_msg)
            return

        # Detect skips in data.
        if t - self.tlast > MAX_DROPOUT_BEFORE_RESET_SEC:
            rospy.logwarn_throttle(1, '{}: Dropped samples for {} sec'.format(self.name, t - self.tlast))
            self.clear_msmt_state()
            self.tlast = t
            self.tracking_status_pub.publish(det_msg)
            return

        self.tlast = t

        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        (rows, cols) = cv_image.shape

        if self.rot180:
            cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)

        roi_unscaled = self.face_tracker.get_roi()

        if roi_unscaled is None:
            rospy.logdebug_throttle(1, '{}: Not tracking'.format(self.name))
            self.clear_msmt_state()
            self.tlast = t
            self.tracking_status_pub.publish(det_msg)
            return

        hroi = roi_unscaled[1][1] - roi_unscaled[0][1]
        face_height = 0.5 * hroi / ROI_HALF_HEIGHT_FRAC_OF_FACE
        xoff = self.x_pix_offset * face_height
        yoff = self.y_pix_offset * face_height
        xmin = int(roi_unscaled[0][0] + xoff)
        ymin = int(roi_unscaled[0][1] + yoff)
        xmax = int(roi_unscaled[1][0] + xoff)
        ymax = int(roi_unscaled[1][1] + yoff)

        if xmin < 0 or ymin < 0 or xmax >= cols or ymax >= rows:
            rospy.logwarn_throttle(1, '{}: edge of frame at {}, seq {}. {} {} {} {} {} {}'.format(
                self.name, t, data.header.seq, xmin, xmax, cols, ymin, ymax, rows))
            if xmin < 0: xmin = 0
            if ymin < 0: ymin = 0
            if xmax >= cols: xmax = cols-1
            if ymax >= cols: ymax = cols-1

        if cv_image[ymin:ymax, xmin:xmax].size == 0:
            rospy.logwarn_throttle(1, '{}: empty image at {}, seq {}'.format(
                self.name, t, data.header.seq))
            self.clear_msmt_state()
            return

        det_msg.data = True
        self.tracking_status_pub.publish(det_msg)

        cv_roi = cv_image[ymin:ymax, xmin:xmax]
        msg = self.bridge.cv2_to_imgmsg(cv_roi, encoding='passthrough')
        msg.header = data.header
        self.cropped_image_pub.publish(msg)

        region = PolygonStamped()
        region.header = data.header
        region.polygon.points = [Point32(x=xmin, y=ymin, z=0),
                                 Point32(x=xmax, y=ymax, z=0)]
        self.region_pub.publish(region)

    def clear_msmt_state(self):
        rospy.logwarn_throttle(1, '{}: Resetting'.format(self.name))
        self.tlast = None

    def __init__(self, name, face_tracker,
                 tracking_status_topic,
                 cropped_image_pub, region_in_image_pub,
                 # Moving the ROI down and right is positive.
                 # Expressed as a fraction of the ROI height.
                 x_pix_offset, y_pix_offset,
                 rot180=False):
        if RATE_CUT:
            self.RATE_CUT = None

        self.name = name

        self.enabled = True

        self.face_tracker = face_tracker

        self.x_pix_offset = x_pix_offset
        self.y_pix_offset = y_pix_offset
        self.rot180 = rot180

        self.bridge = CvBridge()

        self.clear_msmt_state()

        self.cropped_image_pub = cropped_image_pub
        self.region_pub = region_in_image_pub
        self.tracking_status_pub = rospy.Publisher(tracking_status_topic,
                                                   Bool, queue_size=10)
