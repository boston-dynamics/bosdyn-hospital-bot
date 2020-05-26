# Buffer about 1 second of full-size frames.
IMG_QUEUE_SIZE = 35
IMG_BUF_SIZE = IMG_QUEUE_SIZE * 2448 * 2048

ENABLE_TOPIC = 'multichrome_face_tracker_enable'

CHAN_TXT = ('red', 'nir', 'narrow_nir')

# IMX264 monochrome with BN660, BP880, BN810
## halogen illumination
PBV_STATIC = [ 0.68,  1.0,     0.56]
PBV_UPDATE = [-0.014, 0.0024, -0.0003]

## sunlight illumination
#PBV_STATIC = [ 1.0,   0.56,    0.42]
#PBV_UPDATE = [-0.021, 0.0013, -0.0003]

RED_IMAGE_TOPIC = 'camera_array/mono_red/image_raw'
RED_TRACKING_STATUS_TOPIC = 'mono_red_tracking_status'
RED_CROPPED_IMAGE_TOPIC = 'mono_red_cropped'
RED_REGION_IN_IMAGE_TOPIC = 'mono_red_region'

NIR_IMAGE_TOPIC = 'camera_array/mono_nir/image_raw'
NIR_TRACKING_STATUS_TOPIC = 'mono_nir_tracking_status'
NIR_CROPPED_IMAGE_TOPIC = 'mono_nir_cropped'
NIR_REGION_IN_IMAGE_TOPIC = 'mono_nir_region'

NARROW_NIR_IMAGE_TOPIC = 'camera_array/mono_narrow_nir/image_raw'
NARROW_NIR_TRACKING_STATUS_TOPIC = 'mono_narrow_nir_tracking_status'
NARROW_NIR_CROPPED_IMAGE_TOPIC = 'mono_narrow_nir_cropped'
NARROW_NIR_REGION_IN_IMAGE_TOPIC = 'mono_narrow_nir_region'

# The face height is better related to scene depth than the width is.
# Therefore, we use it as a proxy for the depth-dependent ROI offset
# we need to apply because the camera centers are offset.
NIR_OFFSET_FRAC_OF_FACE_HEIGHT = -0.35
NARROW_NIR_X_OFFSET_FRAC_OF_FACE_HEIGHT = -0.35
NARROW_NIR_Y_OFFSET_FRAC_OF_FACE_HEIGHT = -0.6
