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

# The algorithm implemented here is based on:
# https://www.nature.com/articles/srep38609

DEBUG_SAVE_IMAGES = False
DEBUG_SAVE_DATA = False
DEBUG_MKDIR = DEBUG_SAVE_IMAGES or DEBUG_SAVE_DATA
if DEBUG_MKDIR:
    LOG_DIR = '/log'
    import os

import numpy as np
from scipy import signal
import threading
import cv2
import math
import insightface

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PolygonStamped
from std_msgs.msg import Float32MultiArray, Float32, Bool, Empty
from cv_bridge import CvBridge

from drspot.utils.tracking import ROITracker

# Inputs.
RED_IMAGE_TOPIC = 'mono_red_cropped'
RED_REGION_IN_IMAGE_TOPIC = 'mono_red_region'
NIR_IMAGE_TOPIC = 'mono_nir_cropped'
NIR_REGION_IN_IMAGE_TOPIC = 'mono_nir_region'
NARROW_NIR_IMAGE_TOPIC = 'mono_narrow_nir_cropped'
NARROW_NIR_REGION_IN_IMAGE_TOPIC = 'mono_narrow_nir_region'

TRACKING_STATUS_TOPIC = 'mono_red_tracking_status'

# Outputs.
HEART_RATE_MSMT_TOPIC = 'full_heart_rate'
SPO2_MSMT_TOPIC = 'full_spo2'

START_COMPUTE_TOPIC = 'start_compute_heart_rate_spo2'
ABORT_COMPUTE_TOPIC = 'abort_compute_heart_rate_spo2'

# Carotid
# SUBROI_EXTRA_HALF_WIDTH_FRAC_OF_FACE_WIDTH = 0.2
# SUBROI_HEIGHT_FRAC_OF_FACE_HEIGHT_DOWN = 1.2
# SUBROI_HEIGHT_FRAC_OF_FACE_HEIGHT = 0.3
# NUM_SUBREGIONS_X = 12
# NUM_SUBREGIONS_Y = 5

# Forehead
SUBROI_EXTRA_HALF_WIDTH_FRAC_OF_FACE_WIDTH = 0.1
SUBROI_HEIGHT_FRAC_OF_FACE_HEIGHT_DOWN = 0.0
SUBROI_HEIGHT_FRAC_OF_FACE_HEIGHT = 0.25
NUM_SUBREGIONS_X = 10
NUM_SUBREGIONS_Y = 3

HELPER_TIMER_PERIOD_SEC = 1
FULL_MSMT_PERIOD_SEC = 10
MAX_DELTA_DIFF_FRAC_OF_DELTA = 1.0
NUM_DELTA_DIFF_FRAC_OF_DELTA = 3
MAX_DROPOUT_BEFORE_RESET_SEC = 0.5
# Buffer about 3 seconds of cropped frames.
IMG_QUEUE_SIZE = 3 * 35
IMG_BUF_SIZE = IMG_QUEUE_SIZE * 1000 * 1000

RATE_CUT = False
RATE_DIV = 2

MAX_DELTA_T = 0.1 # seconds.

T_EPS = 1e-4

DETECT_TRACK_SCALE_FACTOR = 0.125

FACE_DETECTION_LIKELIHOOD_THRESHOLD = 0.9
FACE_DETECTION_SCALE = 1.0
# Potentially use a more accurate tracker than the one in the multichrome_face_tracker node,
# and/or run on every frame, at the expense of more computation time.
CV_COARSE_TRACKER_TYPE = 'CSRT'
CV_FINE_TRACKER_TYPE = 'TM_CCORR_NORMED'

NUM_PLOT_X = NUM_SUBREGIONS_X
NUM_PLOT_Y = NUM_SUBREGIONS_Y

# Minimum of 45 beats per minute.
MIN_HEART_HZ = 45.0 / 60.0
# Maximum of 220 beats per minute.
MAX_HEART_HZ = 220.0 / 60.0
# Minimum of 30 beats per minute in stored FFT.
MIN_HZ = 30.0 / 60.0
# Enough to look for the 2nd harmonic of heart rate.
MAX_HZ = 2 * MAX_HEART_HZ

QUALITY_PRUNE_FACTOR = 1.0

CHAN_IDX = (0, 1, 2)
CHAN_TXT = ('red', 'nir', 'narrow_nir')
CC = len(CHAN_IDX)

RUN_CONTINUOUS = False

# IMX264 monochrome with BN660, BP880, BN810
## halogen illumination
PBV_STATIC = np.array([ 0.68,  1.0,    0.56])
PBV_UPDATE = np.array([-0.014, 0.0024, -0.0003])

## sunlight illumination
#PBV_STATIC = np.array([ 1.0,   0.56,    0.42])
#PBV_UPDATE = np.array([-0.021, 0.0013, -0.0003])

PBV_TO_CHECK = [100, 95, 90, 85, 80, 75, 70]

def to_uint8_scale(img):
    gmin = np.amin(img); grange = np.amax(img) - gmin
    if grange > 1e-3:
        img = (img - gmin) / grange * 255.0
    img = img.astype(np.uint8)
    return img

def grad(im) :
    # Calculate the x and y gradients using Sobel operator
    grad_x = cv2.Sobel(im,cv2.CV_32F,1,0,ksize=3)
    grad_y = cv2.Sobel(im,cv2.CV_32F,0,1,ksize=3)

    # Combine the two gradients
    grad = cv2.addWeighted(np.absolute(grad_x), 0.5, np.absolute(grad_y), 0.5, 0)
    return to_uint8_scale(grad)

class HeartRate(object):
    def synced_callback(self, red_image_data, nir_image_data, narrow_nir_image_data,
                        red_region, nir_region, narrow_nir_region):
        if RATE_CUT:
            if self.RATE_CUT is None: self.RATE_CUT = 0
            self.RATE_CUT += 1
            if self.RATE_CUT == RATE_DIV:
                self.RATE_CUT = 0
            else:
                return
        with self.lock:
            chans = zip((red_image_data, nir_image_data, narrow_nir_image_data),
                        (red_region, nir_region, narrow_nir_region),
                        CHAN_TXT)

            for image_data, region, txt in chans:
                t = image_data.header.stamp.to_sec()

                if self.tlast[txt] is None:
                    self.tlast[txt] = t
                    return

                # Detect backward jumps in time in replay.
                if self.tlast[txt] > t + T_EPS:
                    self.clear_msmt_state()
                    self.tlast[txt] = t
                    rospy.logwarn_throttle(1, '{} {}: Backward jump in time {:0.6f} vs {:0.6f}'.format(
                        self.name, txt, self.tlast[txt], t))
                    return

                # Detect duplicate timestamps due to message synchronization.
                if self.tlast[txt] >= t:
                    rospy.logwarn('{} {}: Duplicate timestamp {:0.6f} vs {:0.6f}'.format(
                        self.name, txt, self.tlast[txt], t))

                # Detect skips in data.
                if t - self.tlast[txt] > MAX_DROPOUT_BEFORE_RESET_SEC:
                    rospy.logwarn_throttle(1, '{} {}: Dropped samples for {} sec'.format(
                        self.name, txt, t - self.tlast[txt]))
                    self.clear_msmt_state()
                    self.tlast[txt] = t
                    return

                # Detect region message header mismatch with image header.
                region_t = region.header.stamp.to_sec()
                if abs(t - region_t) > T_EPS:
                    self.clear_msmt_state()
                    self.tlast[txt] = t
                    rospy.logwarn_throttle(1, '{} {}: Region out of sync with image {:0.6f} vs {:0.6f}'.format(
                        self.name, txt, region_t, t))
                    return

                if self.delta[txt] is None:
                    self.delta[txt] = t - self.tlast[txt]
                    return

                self.delta[txt] = self.delta_alpha * (t - self.tlast[txt]) + (1 - self.delta_alpha) * self.delta[txt]
                self.tlast[txt] = t
        # End locked region.

        red_image = self.bridge.imgmsg_to_cv2(red_image_data,
                                              desired_encoding='passthrough')
        nir_image = self.bridge.imgmsg_to_cv2(nir_image_data,
                                              desired_encoding='passthrough')
        narrow_nir_image = self.bridge.imgmsg_to_cv2(narrow_nir_image_data,
                                               desired_encoding='passthrough')

        t = red_image_data.header.stamp.to_sec()
        with self.lock:
            if self.start_msmt: self.ti = t
            self.start_msmt = False
            self.t_buffer.append(t)
            self.red_msmt_buffer.append(red_image)
            self.nir_msmt_buffer.append(nir_image)
            self.narrow_nir_msmt_buffer.append(narrow_nir_image)
            self.red_offset_buffer.append(np.array([red_region.polygon.points[0].x,
                                                    red_region.polygon.points[0].y]))
            self.nir_offset_buffer.append(np.array([nir_region.polygon.points[0].x,
                                                    nir_region.polygon.points[0].y]))
            self.narrow_nir_offset_buffer.append(np.array([narrow_nir_region.polygon.points[0].x,
                                                           narrow_nir_region.polygon.points[0].y]))
            self.tf = t
        # End locked region.

    def tracking_status_callback(self, data):
        with self.lock:
            if not self.tracking_status or not data.data:
                # Clear the measurement buffer unless we are valid, staying valid.
                self.clear_msmt_state()
            self.tracking_status = data.data

    def msmt_helper_callback(self, event):
        with self.lock:
            if not self.tracking_status:
                rospy.logwarn_throttle(1, '{}: Invalid tracking status.'.format(self.name))
                self.clear_msmt_state()
                return

            ti = self.ti
            tf = self.tf
            t_data = np.array(self.t_buffer)
            red_data = list(self.red_msmt_buffer)
            nir_data = list(self.nir_msmt_buffer)
            narrow_nir_data = list(self.narrow_nir_msmt_buffer)
            red_offset = list(self.red_offset_buffer)
            nir_offset = list(self.nir_offset_buffer)
            narrow_nir_offset = list(self.narrow_nir_offset_buffer)
            do_it = ti is not None and tf is not None and tf - ti >= FULL_MSMT_PERIOD_SEC
            if do_it:
                self.clear_buffers()
        # End locked region.

        if do_it:
            msmt_n = len(red_data)
            delta = (tf - ti) / msmt_n
            rospy.loginfo('{}: starting computation; nsamp: {}; ti: {:.1f}; tf: {:.1f}; delta: {:.4f}'.format(
                self.name, msmt_n, ti, tf, delta))
            # Spawn a thread to do the measurement.
            if RUN_CONTINUOUS:
                do_msmt = True
            else:
                do_msmt = self.msmt_thread is None or not self.msmt_thread.is_alive()
            if do_msmt:
                t = threading.Thread(target=self.msmt_callback, name='full_' + self.name,
                                     args=(event, ti, tf,
                                           red_data, nir_data, narrow_nir_data,
                                           red_offset, nir_offset, narrow_nir_offset,
                                           t_data, delta))
                if not RUN_CONTINUOUS:
                    self.msmt_thread = t
                t.start()

    def msmt_callback(self, event, ti, tf,
                      red_data, nir_data, narrow_nir_data,
                      red_offset, nir_offset, narrow_nir_offset,
                      t_data, delta):
        msmt_n = len(red_data)

        delta_compare = np.abs(np.diff(t_data) - delta)
        bad_sample_idx = (delta_compare >= MAX_DELTA_DIFF_FRAC_OF_DELTA * delta)
        if bad_sample_idx.sum() > NUM_DELTA_DIFF_FRAC_OF_DELTA:
            rospy.loginfo_throttle(1, '{}: invalid sample set; {} in {}:{} with delta {:.4f}; bad delta on samples {}'.format(
                                   self.name, msmt_n, ti, tf, delta, np.where(bad_sample_idx)))
            self.abort_compute_pub.publish(Empty())
            return

        if delta > MAX_DELTA_T:
            rospy.logwarn_throttle(1, '{}: Bad delta {} sec in {}:{}'.format(
                self.name, delta, ti, tf))
            self.abort_compute_pub.publish(Empty())
            return

        global DEBUG_SAVE_IMAGES
        global DEBUG_SAVE_DATA
        global DEBUG_MKDIR
        if DEBUG_MKDIR:
            statvfs = os.statvfs(LOG_DIR)
            # Number of free bytes available to non-priveleged users.
            if statvfs.f_frsize * statvfs.f_bavail < 1000 * 1000 * 1000:
                DEBUG_MKDIR = False
                DEBUG_SAVE_IMAGES = False
                DEBUG_SAVE_DATA = False
            else:
                output_dir = LOG_DIR + os.path.sep + str(int(ti)) + os.path.sep
                os.mkdir(output_dir)
                if DEBUG_SAVE_IMAGES:
                    output_img_dir = output_dir + 'img' + os.path.sep
                    os.mkdir(output_img_dir)

        self.start_compute_pub.publish(Empty())

        ri = 0 # ROI index over time.
        patch_avg = np.zeros((msmt_n, NUM_SUBREGIONS_X * NUM_SUBREGIONS_Y, 3))
        ii = 0 # Index into synchronized channels.
        # Initialize a tracker on the sub-roi. Use a more accurate tracker than the one in
        # the multichrome_face_tracker node and/or track on every frame at the expense of more
        # computation time.
        red_coarse = ROITracker(CV_COARSE_TRACKER_TYPE)
        nir_coarse = ROITracker(CV_COARSE_TRACKER_TYPE)
        narrow_nir_coarse = ROITracker(CV_COARSE_TRACKER_TYPE)
        red_fine = ROITracker(CV_FINE_TRACKER_TYPE)
        nir_fine = ROITracker(CV_FINE_TRACKER_TYPE)
        narrow_nir_fine = ROITracker(CV_FINE_TRACKER_TYPE)
        grad_coarse = ROITracker(CV_COARSE_TRACKER_TYPE)
        detector = insightface.model_zoo.get_model('retinaface_r50_v1')
        detector.prepare(ctx_id=-1, nms=0.4)
        for red, nir, narrow_nir, red_off, nir_off, narrow_nir_off, t in zip(red_data,
                                                                             nir_data,
                                                                             narrow_nir_data,
                                                                             red_offset,
                                                                             nir_offset,
                                                                             narrow_nir_offset,
                                                                             t_data):
            for chan, off, coarse, fine, cc, txt in zip((red, nir, narrow_nir),
                                                        (red_off,
                                                         nir_off,
                                                         narrow_nir_off),
                                                        (red_coarse,
                                                         nir_coarse,
                                                         narrow_nir_coarse),
                                                        (red_fine,
                                                         nir_fine,
                                                         narrow_nir_fine),
                                                        CHAN_IDX, CHAN_TXT):
                (rows, cols) = chan.shape
                r = int(rows * DETECT_TRACK_SCALE_FACTOR)
                c = int(cols * DETECT_TRACK_SCALE_FACTOR)
                resize = cv2.resize(chan, (c, r), interpolation=cv2.INTER_AREA)
                if ii == 0 and cc == 0:
                    chan3 = cv2.cvtColor(resize, cv2.COLOR_GRAY2RGB)
                    bboxes, landmarks = detector.detect(chan3,
                                                        threshold=FACE_DETECTION_LIKELIHOOD_THRESHOLD,
                                                        scale=FACE_DETECTION_SCALE)
                    if DEBUG_SAVE_IMAGES:
                        copy = resize.copy()
                        # All overlays have colors below in case we use an RGB image in the future.
                        for box in bboxes.astype(int):
                            x, y, x2, y2 = box[0:4]
                            # Draw a rectangle for each face bounding box.
                            cv2.rectangle(copy, (x, y), (x2, y2), (255,255,255), 2)
                        for face in landmarks.astype(int):
                            for point in face:
                                x, y = point
                                # Draw a circle at each landmark.
                                cv2.circle(copy, (x, y), 5, (255,255,255), 1)
                        cv2.imwrite(output_dir + txt + '_debug_det.png', copy)
                    if len(bboxes) != 1:
                        rospy.logwarn_throttle(1, '{} {}: {} faces detected in {}:{}; erroring'.format(
                            self.name, txt, len(bboxes), ti, tf))
                        self.abort_compute_pub.publish(Empty())
                        return
                    wface = bboxes[0][2] - bboxes[0][0]
                    hface = bboxes[0][3] - bboxes[0][1]
                    bboxes[0][0] -= SUBROI_EXTRA_HALF_WIDTH_FRAC_OF_FACE_WIDTH * wface
                    bboxes[0][1] += SUBROI_HEIGHT_FRAC_OF_FACE_HEIGHT_DOWN * hface
                    bboxes[0][2] += SUBROI_EXTRA_HALF_WIDTH_FRAC_OF_FACE_WIDTH * wface
                    bboxes[0][3] = bboxes[0][1] + SUBROI_HEIGHT_FRAC_OF_FACE_HEIGHT * hface
                    gim = grad(resize)
                    try:
                        grad_coarse.start_track(gim, bboxes[0])
                    except Exception as e:
                        rospy.logwarn_throttle(1, '{} {}: failed gradient matcher init in {}:{}; {} = {}'.format(
                            self.name, txt, ti, tf, bboxes[0], e))
                        self.abort_compute_pub.publish(Empty())
                        return
                else:
                    if ii == 0:
                        gim = grad(resize)
                        grad_coarse.update(gim)
                        bboxes = np.array([grad_coarse.bbox])
                    else:
                        coarse.update(resize)
                        bboxes = np.array([coarse.bbox])

                if ii == 0:
                    if DEBUG_SAVE_IMAGES:
                        cv2.imwrite(output_dir + txt + '_grad.png', gim)
                    try:
                        coarse.start_track(resize, bboxes[0])
                    except Exception as e:
                        rospy.logwarn_throttle(1, '{} {}: failed track init in {}:{}; {} = {}'.format(
                            self.name, txt, ti, tf, bboxes[0], e))
                        self.abort_compute_pub.publish(Empty())
                        return

                resized_bbox = tuple((bboxes[0][0:4] / DETECT_TRACK_SCALE_FACTOR).astype(int))
                xmin, ymin, xmax, ymax = resized_bbox

                if ii == 0 and cc == 0:
                    try:
                        fine.start_track(chan, resized_bbox)
                    except Exception as e:
                        rospy.logwarn_throttle(1, '{} {}: failed track init in {}:{}; {} = {}'.format(
                            self.name, txt, ti, tf, resized_bbox, e))
                        self.abort_compute_pub.publish(Empty())
                        return
                else:
                    # Refine tracked location.
                    # TODO - base this on region size or something smarter than hardcoding.
                    # Pixel amount that bounds the slop in the coarse tracker.
                    if ii == 0:
                        slop = 50
                        tmpl_rows = red_fine.template.shape[0]
                        tmpl_cols = red_fine.template.shape[1]
                    else:
                        slop = 30
                        tmpl_rows = fine.template.shape[0]
                        tmpl_cols = fine.template.shape[1]
                    # Compensate for the tracked region sloppily changing size.
                    adj_xmax = xmin + tmpl_cols
                    adj_ymax = ymin + tmpl_rows
                    full_size_crop_plus_slop = chan[ymin-slop:adj_ymax+slop, xmin-slop:adj_xmax+slop]
                    try:
                        if ii == 0:
                            red_fine.update(full_size_crop_plus_slop)
                            if DEBUG_SAVE_IMAGES:
                                cv2.imwrite(output_dir + txt + '_match_red_fine.png', to_uint8_scale(red_fine.update_metadata))
                        else:
                            fine.update(full_size_crop_plus_slop)
                            if DEBUG_SAVE_IMAGES:
                                cv2.imwrite(output_img_dir + txt + '_match_prev_fine_{:05d}.png'.format(ii), to_uint8_scale(fine.update_metadata))
                    except Exception as e:
                        rospy.logwarn_throttle(1, '{} {}: failed/skipping fine update in {}:{} idx {}; {} = {}'.format(
                            self.name, txt, ti, tf, ii, resized_bbox, e))
                        if ii == 0:
                            self.abort_compute_pub.publish(Empty())
                            return
                    else:
                        if ii == 0:
                            xmin_s, ymin_s, xmax_s, ymax_s = red_fine.bbox
                        else:
                            xmin_s, ymin_s, xmax_s, ymax_s = fine.bbox

                        xmin += xmin_s - slop; ymin += ymin_s - slop
                        xmax = adj_xmax + xmin_s - slop
                        ymax = adj_ymax + ymin_s - slop

                        try:
                            # TODO - update fine template size if coarse tracked region has really changed size over time?
                            fine.start_track(chan, [xmin, ymin, xmax, ymax])
                        except Exception as e:
                            rospy.logwarn_throttle(1, '{} {}: failed track reinit in {}:{}; {} = {}'.format(
                                self.name, txt, ti, tf, resized_bbox, e))

                if xmin < 0: xmin = 0
                if ymin < 0: ymin = 0
                if xmax >= cols: xmax = cols-1
                if ymax >= rows: ymax = rows-1

                xd = int(math.floor((xmax - xmin) / float(NUM_SUBREGIONS_X)))
                yd = int(math.floor((ymax - ymin) / float(NUM_SUBREGIONS_Y)))

                crop = chan[ymin:ymin + yd * NUM_SUBREGIONS_Y,
                            xmin:xmin + xd * NUM_SUBREGIONS_X]

                if DEBUG_SAVE_IMAGES:
                    try:
                        cv_image = chan.copy()
                        # Draw vertical lines (constant x coordinate).
                        for xline in range(xmin, xmax, xd):
                            cv2.line(cv_image, (xline, ymin), (xline, ymax), (255,255,255), 3)
                        # Draw vertical lines (constant y coordinate).
                        for yline in range(ymin, ymax, yd):
                            cv2.line(cv_image, (xmin, yline), (xmax, yline), (255,255,255), 3)
                        # Recover values from before refinement.
                        xmin, ymin, xmax, ymax = resized_bbox
                        if not (ii == 0 and cc == 0):
                            cv2.rectangle(cv_image, (xmin-slop, ymin-slop), (xmax+slop, ymax+slop), (255,255,255), 2)
                        cv2.imwrite(output_img_dir + txt + '_{:05d}.png'.format(ii), cv_image)
                        cv2.imwrite(output_img_dir + txt + '_crop_{:05d}.png'.format(ii), crop)
                    except Exception as e:
                        print(e)
                        pass

                # Spatial average over each subregion at this point in time.
                try:
                    patch_avg[ii, :, cc] = np.mean(crop.reshape((xd, yd, -1)),
                                                   axis=(0,1))
                except Exception as e:
                    rospy.logwarn_throttle(1, ('{} {}: failed spatial average in '
                                           + '{}:{} idx {}; {} {} {} {} {} {} {} {} = {}').format(
                        self.name, txt, ti, tf, ii, crop.shape, chan.shape, xd, yd, xmin, ymin, xmax, ymax, e))
                    self.abort_compute_pub.publish(Empty())
                    return
            # End loop over colors.
            ii += 1
        # End loop through data series.

        for rr in range(patch_avg.shape[1]):
            if DEBUG_SAVE_DATA:
                np.savetxt(output_dir + 'raw_{:03d}.csv'.format(rr),
                           np.hstack((t_data.reshape((-1, 1)) - t_data[0],
                                      patch_avg[:, rr, :].reshape((-1,CC)))),
                           delimiter=',', fmt='%.3e')
        patch_avg /= np.mean(patch_avg, axis=0)
        patch_avg -= 1.0
        patch_pulse = np.zeros((msmt_n, patch_avg.shape[1], len(PBV_TO_CHECK)))

        # Find the range of real frequency bins that we are interested in.
        fminind = int(MIN_HZ * msmt_n * delta)
        fmaxind = int(MAX_HZ * msmt_n * delta)
        hr_fminind = int(MIN_HEART_HZ * msmt_n * delta) - fminind
        hr_fmaxind = int(MAX_HEART_HZ * msmt_n * delta) - fminind
        fminind += 1
        hr_fminind += 1
        patch_freq = np.zeros((fmaxind-fminind+1, patch_avg.shape[1], len(PBV_TO_CHECK)))
        # Will set up binary mask later.
        fft_mask = np.zeros((fmaxind-fminind+1))
        peak_freq_count = np.zeros(patch_freq.shape[0])
        freq = None
        # Compute this for all subregions and candidate SpO2 values.
        peak_freq = np.zeros((patch_avg.shape[1], len(PBV_TO_CHECK)))
        # Compute these for all subregion combinations and candidate SpO2 values.
        snr = np.zeros((patch_avg.shape[1], patch_avg.shape[1], len(PBV_TO_CHECK)))
        peak_corr = np.zeros(snr.shape)

        for rr in range(patch_avg.shape[1]):
            c_n = patch_avg[:, rr, :].reshape((-1,3)).T
            q_inv = np.linalg.inv(np.matmul(c_n, c_n.T))
            if DEBUG_SAVE_DATA:
                np.savetxt(output_dir + 'qinv_{:03d}.csv'.format(rr), q_inv,
                           delimiter=',', fmt='%.3e')
            for ss in range(len(PBV_TO_CHECK)):
                spo2 = PBV_TO_CHECK[ss]
                w_pbv = np.matmul(PBV_STATIC + (100.0 - spo2) * PBV_UPDATE,
                                  q_inv)
                w_pbv /= np.linalg.norm(w_pbv)
                patch_pulse[:, rr, ss] = np.matmul(w_pbv, c_n)

                sp = np.fft.fft(patch_pulse[:, rr, ss])
                if freq is None:
                    freq = np.fft.fftfreq(sp.shape[-1], d=delta)[fminind:fmaxind+1]
                patch_freq[:, rr, ss] = np.abs(sp)[fminind:fmaxind+1]
                freq_idx = patch_freq[hr_fminind:hr_fmaxind, rr, ss].argmax() + hr_fminind
                peak_freq[rr, ss] = freq[freq_idx]
                peak_freq_count[freq_idx] += 1

            if DEBUG_SAVE_DATA:
                np.savetxt(output_dir + 'normalized_{:03d}.csv'.format(rr),
                           np.hstack((t_data.reshape((-1, 1)) - t_data[0],
                                      patch_avg[:, rr, :].reshape((-1,CC)))),
                           delimiter=',', fmt='%.3e')
                np.savetxt(output_dir + 'pulse_{:03d}.csv'.format(rr),
                           np.hstack((t_data.reshape((-1, 1)) - t_data[0],
                                      patch_pulse[:, rr, :].reshape((-1, patch_pulse.shape[2])))),
                           delimiter=',', fmt='%.3e')
                np.savetxt(output_dir + 'freq_{:03d}.csv'.format(rr),
                           np.hstack((freq.reshape((-1, 1)) * 60.0,
                                      patch_freq[:, rr, :].reshape((-1, patch_freq.shape[2])))),
                           delimiter=',', fmt='%.3e')
        # End loop over regions.
        if DEBUG_SAVE_DATA:
            np.savetxt(output_dir + 'peak_freq.csv', peak_freq * 60.0,
                       delimiter=',', fmt='%.1f')

        peak_freq_idx = peak_freq_count.argmax()
        beats = freq[peak_freq_idx]
        # Take into account our truncated spectrum when finding the 2nd harmonic bin.
        second_harmonic = (peak_freq_idx + fminind) * 2 - fminind

        msg = Float32()
        msg.data = beats * 60.0
        self.heart_rate_msmt_pub.publish(msg)

        fft_mask[peak_freq_idx-1:peak_freq_idx+2] = 1.0
        # Make the mask wider at the second harmonic.
        fft_mask[second_harmonic-2:second_harmonic+3] = 1.0
        for rr0 in range(patch_avg.shape[1]):
            # Only compute the upper triangular entries - these are symmetric.
            for rr1 in range(rr0, patch_avg.shape[1]):
                for ss in range(len(PBV_TO_CHECK)):
                    cross_region_energy = patch_freq[:, rr0, ss] * patch_freq[:, rr1, ss]
                    masked = fft_mask * cross_region_energy
                    snr[rr0, rr1, ss] = np.sum(masked) / np.sum(1.0 - masked)
                    # If we want to compute the peak over the whole FFT range,
                    # not the restricted heart-rate range we use to get peak_freq.
                    #freq_idx = patch_freq[:, rr, ss].argmax()
                    #freq[freq_idx]
                    peak_corr[rr0, rr1, ss] = np.abs(peak_freq[rr0, ss] - peak_freq[rr1, ss])

        lower = np.tril_indices(patch_avg.shape[1], -1)
        # Make the matrices symmetric.
        for ss in range(len(PBV_TO_CHECK)):
            snr[:, :, ss][lower] = snr[:, :, ss].T[lower]
            peak_corr[:, :, ss][lower] = peak_corr[:, :, ss].T[lower]
        # Normalize snr and peak_corr.
        snr /= np.amax(snr)
        if np.amax(peak_corr) < 1e-3:
            peak_corr = np.ones(peak_corr.shape)
        else:
            peak_corr = 1.0 - peak_corr / np.amax(peak_corr)
        qual = snr * peak_corr

        if DEBUG_SAVE_DATA:
            for rr in range(patch_avg.shape[1]):
                np.savetxt(output_dir + 'snr_{:03d}.csv'.format(rr),
                           snr[rr, :, :],
                           delimiter=',', fmt='%.2f')
                np.savetxt(output_dir + 'peak_corr_{:03d}.csv'.format(rr),
                           peak_corr[rr, :, :],
                           delimiter=',', fmt='%.2f')
                np.savetxt(output_dir + 'qual_{:03d}.csv'.format(rr),
                           qual[rr, :, :],
                           delimiter=',', fmt='%.2f')

        # Sum quality measure for each subregion across other subregions
        # and across candidate SpO2 values.
        qual_prune = np.sum(qual, axis=(1,2)) / (patch_avg.shape[1] * len(PBV_TO_CHECK))
        mean_qual = np.mean(qual_prune)
        use_region_idx = np.nonzero(qual_prune > mean_qual * QUALITY_PRUNE_FACTOR)[0]

        fft_per_spo2 = np.zeros((fmaxind-fminind+1, len(PBV_TO_CHECK)))
        snr_per_spo2 = np.zeros((len(PBV_TO_CHECK)))
        pulse_per_spo2 = np.sum(patch_pulse, axis=1, keepdims=False) / len(use_region_idx)
        if DEBUG_SAVE_DATA:
            np.savetxt(output_dir + 'avg_pulse.csv',
                       np.hstack((t_data.reshape((-1, 1)) - t_data[0],
                                  pulse_per_spo2)),
                       delimiter=',', fmt='%.3e')
        for ss in range(len(PBV_TO_CHECK)):
            sp = np.fft.fft(pulse_per_spo2[:, ss])
            fft_per_spo2[:, ss] = np.abs(sp)[fminind:fmaxind+1]
            energy = fft_per_spo2[:, ss] ** 2
            masked = fft_mask * energy
            snr_per_spo2[ss] = np.sum(masked) / np.sum(1.0 - masked)

        if DEBUG_SAVE_DATA:
            np.savetxt(output_dir + 'avg_pulse_fft.csv',
                       np.hstack((freq.reshape((-1, 1)) * 60.0,
                                  fft_per_spo2)),
                       delimiter=',', fmt='%.3e')
            np.savetxt(output_dir + 'avg_pulse_snr.csv', snr_per_spo2,
                       delimiter=',', fmt='%.3e')

        spo2 = PBV_TO_CHECK[snr_per_spo2.argmax()]

        msg.data = spo2
        self.spo2_msmt_pub.publish(msg)

        rospy.loginfo('{}: {:.1f} beats / min; SpO2 {}%; nsamp: {}; ti: {:.1f}; tf: {:.1f}; delta: {:.4f}'.format(
            self.name, beats * 60.0, spo2, msmt_n, ti, tf, (tf - ti) / msmt_n))

    def clear_buffers(self):
        # Measurement buffers
        self.t_buffer = []
        self.red_msmt_buffer = []
        self.nir_msmt_buffer = []
        self.narrow_nir_msmt_buffer = []
        self.red_offset_buffer = []
        self.nir_offset_buffer = []
        self.narrow_nir_offset_buffer = []

        self.start_msmt = True

    def clear_msmt_state(self):
        rospy.logwarn_throttle(1, '{}: Resetting'.format(self.name))

        # Tracking status
        self.tracking_status = False

        self.ti = None
        self.tf = None

        self.tlast = dict(red=None, nir=None, narrow_nir=None)
        self.delta = dict(red=None, nir=None, narrow_nir=None)

        self.clear_buffers()

    def __init__(self, name):
        if RATE_CUT:
            self.RATE_CUT = None

        self.name = name

        if not RUN_CONTINUOUS:
            self.msmt_thread = None

        self.lock = threading.Lock()

        self.clear_msmt_state()

        # Software time delta calculation
        self.delta_alpha = 0.8

        self.bridge = CvBridge()

        self.start_compute_pub = rospy.Publisher(START_COMPUTE_TOPIC,
                                                 Empty,
                                                 queue_size=1)

        self.abort_compute_pub = rospy.Publisher(ABORT_COMPUTE_TOPIC,
                                                 Empty,
                                                 queue_size=1)

        self.heart_rate_msmt_pub = rospy.Publisher(HEART_RATE_MSMT_TOPIC,
                                                   Float32,
                                                   queue_size=10)
        self.spo2_msmt_pub = rospy.Publisher(SPO2_MSMT_TOPIC,
                                             Float32,
                                             queue_size=10)

        self.tracking_status_sub = rospy.Subscriber(TRACKING_STATUS_TOPIC, Bool,
                                                    self.tracking_status_callback, queue_size=1)

        self.msmt_helper_timer = rospy.Timer(rospy.Duration(HELPER_TIMER_PERIOD_SEC),
                                             self.msmt_helper_callback)
