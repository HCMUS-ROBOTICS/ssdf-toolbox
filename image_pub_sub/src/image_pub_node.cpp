#include <filesystem>
#include <algorithm>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include "image_pub_sub/factory.hpp"

namespace fs = std::filesystem;

int main(int argc, char **argv)
{
    constexpr const char *NODE_NAME = "image_pub_node";
    ros::init(argc, argv, NODE_NAME);

    size_t subCounter = 0;
    ros::NodeHandle nh;
    std::string pubTopic;
    ROS_ASSERT(ros::param::get("~pub_topic", pubTopic));
    image_transport::ImageTransport imageTransport{nh};
    image_transport::Publisher publisher = imageTransport.advertise(
        pubTopic,
        10,
        [&subCounter](const image_transport::SingleSubscriberPublisher &subscriber)
        {
            subCounter++;
            ROS_INFO_STREAM("New subscriber from " << subscriber.getSubscriberName());
        },
        [&subCounter](const image_transport::SingleSubscriberPublisher &subscriber)
        {
            subCounter--;
            ROS_INFO_STREAM("Remove subscriber from " << subscriber.getSubscriberName());
        });

    std::string imagePath;
    ROS_ASSERT(ros::param::get("~image_path", imagePath));

    fs::path fsImagePath{imagePath};
    std::shared_ptr<ImageReader> imageReader = ImageReaderFactory::open(fsImagePath);
    if (imageReader == nullptr)
    {
        return EXIT_FAILURE;
    }

    bool isLoop = false;
    ros::param::get("~loop", isLoop);
    imageReader->setLoop(isLoop);

    bool isImShow = false;
    ros::param::get("~imshow", isImShow);

    float rate = 15;
    ros::param::get("~rate", rate);
    ROS_INFO_STREAM("Publishing " << imageReader->size() << " image(s) at rate = " << rate);
    ros::Rate pubRate{rate};
    while (ros::ok())
    {
        ros::spinOnce();

        std::optional<cv::Mat> image = imageReader->get();
        if (!image.has_value())
            break;

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image.value()).toImageMsg();
        if (subCounter > 0)
        {
            publisher.publish(msg);
        }

        if (isImShow)
        {
            cv::imshow("Frame", image.value());
            cv::waitKey(1);
        }

        pubRate.sleep();
    }

    cv::destroyAllWindows();

    return 0;
}