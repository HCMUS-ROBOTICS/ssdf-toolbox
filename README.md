# ssdf-toolbox
Some utility packages for SSDF

## image_pub package

This package is used for publishing images located in a folder to a topic. It might useful in case you have extracted or recorded image files from external sources and want to use them for debugging.

**Arguments**

- `image_path`: The image location. It could be a path to an image file or a directory containing images.
- `pub_topic` (optional): The publising topic. Default is `/camera/rgb/image/compressed`
- `rate` (optional): The publishing rate. Default is `15` Hz
- `loop` (optional): Re-run from the beginning instead of exiting. Default is `False`
- `imshow` (optional): Whether to show image using `cv2` or not. Default is `False`
- `msg_format` (optional): ROS Image message format. Only support `jpeg` or `png`. Default is `jpeg`

When `image_path` is a directory, you might want to set some optional arguments:
- `image_ext` (optional): image extension to glob images. Default is `.png`
- `disable_sort` (optional): disable sorting images by file names. We sort images by default. Defaul is `False`.

**Publishing topics**
- pub_topic (`CompressedImage`): Default is `/camera/rgb/image/compressed`.

**Subscribing topics**
- None

**Running examples**

```bash
rosrun image_pub image_pub.py _pub_topic:=/lane_seg _image_path:=<image_dir> _msg_format:='jpeg' _loop:=true
```
