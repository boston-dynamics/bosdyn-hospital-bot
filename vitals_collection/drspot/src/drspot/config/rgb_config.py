# Buffer about 1 second of full-size frames.
IMG_QUEUE_SIZE = 35
IMG_BUF_SIZE = IMG_QUEUE_SIZE * 2448 * 2048

ENABLE_TOPIC = 'multichrome_face_tracker_enable'

CHAN_TXT = ('rgb')
CHAN_NUM_CHANS = (3)

# IMX265
## halogen illumination
PBV_STATIC = [ 0.68,  1.0,     0.56]
PBV_UPDATE = [-0.014, 0.0024, -0.0003]

## sunlight illumination
#PBV_STATIC = [ 1.0,   0.56,    0.42]
#PBV_UPDATE = [-0.021, 0.0013, -0.0003]

RGB_IMAGE_TOPIC = 'camera_array/mono_rgb/image_raw'
RGB_TRACKING_STATUS_TOPIC = 'mono_rgb_tracking_status'
RGB_CROPPED_IMAGE_TOPIC = 'mono_rgb_cropped'
RGB_REGION_IN_IMAGE_TOPIC = 'mono_rgb_region'
