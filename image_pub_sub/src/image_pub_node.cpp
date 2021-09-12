#include <filesystem>
#include <algorithm>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

namespace fs = std::filesystem;

class ImageReader
{
public:
    virtual std::optional<cv::Mat> get() = 0;
    void setLoop(bool isLoop)
    {
        this->_isLoop = isLoop;
    }

    virtual size_t size() = 0;

    bool isLoop()
    {
        return this->_isLoop;
    }

private:
    bool _isLoop = false;
};

class FolderImageReader
    : public ImageReader
{
public:
    FolderImageReader(fs::path imagePath, bool isDisableSort = false)
        : _curIndex{0}
    {
        if (!fs::is_directory(imagePath))
            throw std::runtime_error("imagepath is not a directory");

        auto dirIter = fs::directory_iterator(imagePath);
        for (const auto &path : dirIter)
        {
            if (path.is_regular_file())
            {
                _imagePaths.push_back(path.path());
            }
        }

        if (_imagePaths.empty())
            throw std::runtime_error("image folder is empty");

        if (!isDisableSort)
            std::sort(_imagePaths.begin(), _imagePaths.end());
    }

    virtual std::optional<cv::Mat> get() override
    {
        if (_curIndex >= _imagePaths.size())
        {
            if (isLoop())
            {
                _curIndex = 0;
            }
            else
            {
                return std::nullopt;
            }
        }
        const fs::path &imagePath = _imagePaths[_curIndex++];
        ROS_DEBUG_STREAM("Reading from " << imagePath);
        return cv::imread(imagePath);
    }

    virtual size_t size() override
    {
        return this->_imagePaths.size();
    }

private:
    size_t _curIndex;
    std::vector<fs::path> _imagePaths;
};

class FileImageReader
    : public ImageReader
{
public:
    FileImageReader(fs::path imagePath)
        : _isReadOnce{false}
    {
        if (!fs::is_regular_file(imagePath))
            throw std::runtime_error("image path is not a regular file");
        _image = cv::imread(imagePath);
        if (_image.empty())
            throw std::runtime_error("image path is not an image");
    }

    virtual std::optional<cv::Mat> get() override
    {
        if (_isReadOnce)
        {
            if (isLoop())
                return _image;
            return std::nullopt;
        }

        _isReadOnce = true;
        return _image;
    }

    virtual size_t size() override
    {
        return 1l;
    }

private:
    bool _isReadOnce;
    cv::Mat _image;
};

class ImageReaderFactory
{
public:
    static std::shared_ptr<ImageReader> open(const fs::path &imagePath)
    {
        try
        {
            if (!fs::exists(imagePath))
            {
                throw std::runtime_error("image path does not exist");
            }
            if (fs::is_directory(imagePath))
            {
                bool isDisableSort = false;
                ros::param::get("~disable_sort", isDisableSort);
                return std::make_shared<FolderImageReader>(imagePath, isDisableSort);
            }
            else if (fs::is_regular_file(imagePath))
            {
                return std::make_shared<FileImageReader>(imagePath);
            }
            else
            {
                throw std::runtime_error("image path is not supported.");
            }
        }
        catch (const std::exception &e)
        {
            ROS_FATAL_STREAM(e.what());
            return nullptr;
        }
        return nullptr;
    }
};

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