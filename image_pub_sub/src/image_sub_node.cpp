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

static fs::path createSplitDirName(const fs::path &parent)
{
    using namespace std::chrono;
    auto sessionTime = system_clock::to_time_t(system_clock::now());
    auto gmtTime = std::gmtime(&sessionTime);
    std::ostringstream out;
    out << std::put_time(gmtTime, "%Y%m%d_%H%M%S");
    fs::path splitDir = parent / out.str();
    return splitDir;
}

class ImageWriter
{
public:
    ImageWriter(const fs::path &outputDir)
        : _outputDir{outputDir}, _frameCounter{0}, _splitCounter{0}, _numSplit{UINT64_MAX}
    {
    }

    virtual void open() = 0;
    virtual void write(const cv::Mat &image)
    {
        _frameCounter++;
        if (_numSplit > 0 && _frameCounter % _numSplit == 0)
        {
            _frameCounter = 0;
            _splitCounter++;
            ROS_DEBUG_STREAM("Reach " << _numSplit << " frame(s). Split writer at frame = " << _frameCounter);
            this->split(_splitCounter);
        }
    }
    virtual void close() = 0;
    virtual void split(size_t splitCount) = 0;

    int getFrameCount() const { return _frameCounter; }

    size_t getSplitCount() const { return _splitCounter; }

    void setNumSplit(size_t numSplit) { _numSplit = numSplit; }

    fs::path getOutputDir() const { return _outputDir; }

private:
    int _frameCounter;
    int _splitCounter;
    size_t _numSplit;
    fs::path _outputDir;
};

class FolderImageWriter
    : public ImageWriter
{
public:
    FolderImageWriter(fs::path outputDir, const std::string &imageExt)
        : ImageWriter{outputDir}, _imageExt{imageExt}
    {
    }

    virtual void open() override
    {
        _splitDir = createSplitDirName(this->getOutputDir());
        fs::create_directories(_splitDir);
    }

    virtual void write(const cv::Mat &image) override
    {
        fs::path writePath = getFilePath();
        ROS_INFO_STREAM("Write to " << writePath);
        cv::imwrite(writePath, image);
        ImageWriter::write(image);
    }

    virtual void split(size_t splitCount) override
    {
        this->open();
    }

    virtual void close() override {}

private:
    fs::path getFilePath() const
    {
        std::ostringstream out{"frame_"};
        out.fill('0');
        out.width(10);
        out << this->getFrameCount() << _imageExt;
        return _splitDir / out.str();
    }

private:
    std::string _imageExt;
    fs::path _splitDir;
};

class VideoFileWriter
    : public ImageWriter
{
public:
    VideoFileWriter(const fs::path &outputDir, int fourcc, double fps = 24)
        : ImageWriter(outputDir), _fourcc{fourcc}, _fps{fps}
    {
    }

    VideoFileWriter(const fs::path &outputDir)
        : VideoFileWriter(outputDir, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 24)
    {
    }

    VideoFileWriter() : VideoFileWriter(fs::current_path()) {}

    virtual void open() override
    {
        this->split(0);
    }

    virtual void write(const cv::Mat &image) override
    {
        cv::Size imageSize{image.cols, image.rows};
        if (_frameSize.empty() || _frameSize != imageSize)
        {
            _frameSize = imageSize;
            this->split(this->getFrameCount());
        }
        _impl.write(image);
        ImageWriter::write(image);
    }

    virtual void split(size_t splitCount) override
    {
        if (_impl.isOpened())
            _impl.release();
        auto filePath = getFilePath(splitCount);
        if (!_impl.open(filePath, _fourcc, _fps, _frameSize))
        {
            if (!_frameSize.empty())
            {
                ROS_ERROR_STREAM("Error open video path: " << filePath);
                throw std::runtime_error("Unable to open video file to write");
            }
        }
        else
        {
            ROS_INFO_STREAM("Create video path: " << filePath);
        }
    }

    virtual void close() override { _impl.release(); }

private:
    fs::path getFilePath(size_t splitCount) const
    {
        std::ostringstream out{"video_"};
        out.fill('0');
        out.width(10);
        out << splitCount << ".mp4";
        return this->getOutputDir() / out.str();
    }

private:
    int _fourcc;
    double _fps;
    cv::Size _frameSize;
    cv::VideoWriter _impl;
};

class ImageWriterFactory
{
public:
    static std::shared_ptr<ImageWriter> open(const fs::path &outputDir, bool outVideo)
    {
        try
        {
            fs::create_directories(outputDir);

            if (outVideo)
            {
                return std::make_shared<VideoFileWriter>(outputDir);
            }
            else
            {
                std::string imageExt = ".png";
                ros::param::get("~image_ext", imageExt);
                return std::make_shared<FolderImageWriter>(outputDir, imageExt);
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