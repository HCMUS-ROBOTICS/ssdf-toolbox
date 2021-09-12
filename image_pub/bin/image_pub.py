#!/bin/python
import sys
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage


def glob_images(image_path: Path, image_ext: str, disable_sort: bool) -> Iterable:
    if not image_path.exists():
        raise ValueError(f'image_path="{image_path}" does not exists')

    if image_path.is_file():
        return [image_path]

    if disable_sort:
        return list(image_path.glob(f'*{image_ext}'))

    return sorted(list(image_path.glob(f'*{image_ext}')))


def main():
    rospy.init_node('image_pub_node', argv=sys.argv)

    image_path = Path(rospy.get_param('~image_path'))
    pub_topic = rospy.get_param('~pub_topic', default='/camera/rgb/image/compressed')
    rate = rospy.get_param('~rate', default=15)
    image_ext = rospy.get_param('~image_ext', default='.png')
    disable_sort = rospy.get_param('~disable_sort', default=False)
    is_loop = rospy.get_param('~loop', default=False)
    imshow = rospy.get_param('~imshow', default=False)

    msg_format = rospy.get_param('~msg_format', default='jpeg')
    if msg_format not in ['jpeg', 'png']:
        raise ValueError(f'msg_format only support "jpeg", "png".'
                         ' msg_format = "{msg_format}"')

    publisher = rospy.Publisher(pub_topic, CompressedImage, queue_size=10)

    msg = CompressedImage()
    msg.format = msg_format

    rospy.loginfo('Start image_pub_node')

    index = 0
    image_paths = glob_images(image_path, image_ext, disable_sort)
    num_images = len(image_paths)

    assert num_images > 0, f"Images should not be empty. The number of images is {num_images}"

    rospy.loginfo('Publishing %d image(s) at rate = %d', num_images, rate)

    rate = rospy.Rate(rate)

    while True:

        if index == num_images:
            if is_loop:
                index = 0
                rospy.loginfo('Reach the last file. Start from zero')
            else:
                rospy.loginfo('Reach the last file. End')
                break

        msg.header.stamp = rospy.Time.now()
        image_path = image_paths[index]

        rospy.logdebug('image_path = %s', str(image_path))

        image = cv2.imread(str(image_path), cv2.IMREAD_ANYCOLOR)
        msg.data = np.array(cv2.imencode(f'.{msg_format}', image)[1]).tobytes()
        publisher.publish(msg)

        if rospy.is_shutdown():
            break

        if imshow:
            cv2.imshow('Frame', image)
            cv2.waitKey(1)

        rate.sleep()
        index += 1

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
