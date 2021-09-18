#include <filesystem>
#include <algorithm>
#include <vector>
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
    constexpr const char *NODE_NAME = "image_sub_node";
    ros::init(argc, argv, NODE_NAME);

    ros::NodeHandle nh;
    std::string subTopic;
    ROS_ASSERT(ros::param::get("~sub_topic", subTopic));
    image_transport::ImageTransport imageTransport{nh};

    std::string outputDir;
    ROS_ASSERT(ros::param::get("~output_dir", outputDir));

    bool isOutVideo = false;
    ros::param::get("~is_out_video", isOutVideo);

    fs::path fsOutputDir{outputDir};
    std::shared_ptr<ImageWriter> writer = ImageWriterFactory::open(fsOutputDir, isOutVideo);
    if (writer == nullptr)
    {
        return EXIT_FAILURE;
    }

    writer->open();

    int num_split = 0;
    ros::param::get("~num_split", num_split);
    writer->setNumSplit(num_split);

    auto callback = [&](const sensor_msgs::ImageConstPtr &msg)
    {
        try
        {
            auto cv_ptr = cv_bridge::toCvShare(msg);
            if (!cv_ptr->image.empty())
            {
                writer->write(cv_ptr->image);
            }
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR_STREAM("CvBridge error:" << e.what());
        }
    };

    auto subscriber = imageTransport.subscribe(subTopic, 20, callback);

    ROS_INFO_STREAM("Listening on \"" << subTopic << '\"');
    ros::spin();

    writer->close();

    return 0;
}