# ssdf-toolbox
Some utility packages for SSDF

## image_pub package

This package is used for publishing images located in a folder to a topic. It might be useful in case you have extracted or recorded image files from external sources and want to use them for debugging.

**Arguments**
- `image_path`: The image location. It could be a path to an image file or a directory containing images.
- `pub_topic` (optional): The publising topic. Default is `/camera/rgb/image`.
- `rate` (optional): The publishing rate. Default is `15` Hz
- `loop` (optional): Re-run from the beginning instead of exiting. Default is `False`
- `imshow` (optional): Whether to show image using OpenCV or not. Default is `False`

When `image_path` is a directory, you might want to set some optional arguments:
- `disable_sort` (optional): disable sorting images by file names. We sort images by default. Use at your own risk. Defaul is `False`.

**Publishing topics**
- `pub_topic` (`Image`): Default is `/camera/rgb/image`.

**Subscribing topics**
- None

**Running examples**

```bash
rosrun image_pub_sub image_pub_node _pub_topic:=/image _image_path:=<image_dir> _loop:=false _imshow:=true
```
