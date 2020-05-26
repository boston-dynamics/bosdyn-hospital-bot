# Buffer about 1 second of full-size frames.
IMG_QUEUE_SIZE = 35
IMG_BUF_SIZE = IMG_QUEUE_SIZE * 2448 * 2048

ENABLE_TOPIC = 'multichrome_face_tracker_enable'

CHAN_TXT = ('rgb',)
CHAN_NUM_CHANS = (3,)

# IMX265
## halogen illumination
PBV_STATIC = [ 0.92,    1.0,    0.45]
PBV_UPDATE = [-0.0047, -0.0016, 0.0003]

## sunlight illumination
#PBV_STATIC = [ 0.42,    1.0,    0.49]
#PBV_UPDATE = [-0.0034, -0.0011, 0.0004]

RGB_IMAGE_TOPIC = 'camera_array/rgb/image_raw'
RGB_TRACKING_STATUS_TOPIC = 'rgb_tracking_status'
RGB_CROPPED_IMAGE_TOPIC = 'rgb_cropped'
RGB_REGION_IN_IMAGE_TOPIC = 'rgb_region'
