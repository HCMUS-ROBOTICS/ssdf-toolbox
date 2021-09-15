#include "image_pub_sub/image_writer.hpp"
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

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

ImageWriter::ImageWriter(const fs::path &outputDir)
    : _outputDir{outputDir}, _frameCounter{0}, _splitCounter{0}, _numSplit{UINT64_MAX}
{
}

void ImageWriter::write(const cv::Mat &image)
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

////////////////////////////////////////////////////

FolderImageWriter::FolderImageWriter(fs::path outputDir, const std::string &imageExt)
    : ImageWriter{outputDir}, _imageExt{imageExt}
{
}
void FolderImageWriter::open()

{
    _splitDir = createSplitDirName(this->getOutputDir());
    fs::create_directories(_splitDir);
}

void FolderImageWriter::write(const cv::Mat &image)
{
    fs::path writePath = getFilePath();
    ROS_INFO_STREAM("Write to " << writePath);
    cv::imwrite(writePath, image);
    ImageWriter::write(image);
}

void FolderImageWriter::split(size_t splitCount)
{
    this->open();
}

fs::path FolderImageWriter::getFilePath() const
{
    std::ostringstream out{"frame_"};
    out.fill('0');
    out.width(10);
    out << this->getFrameCount() << _imageExt;
    return _splitDir / out.str();
}

//////////////////////////////////////////////////////

VideoFileWriter::VideoFileWriter(const fs::path &outputDir, int fourcc, double fps)
    : ImageWriter(outputDir), _fourcc{fourcc}, _fps{fps}
{
}

VideoFileWriter::VideoFileWriter(const fs::path &outputDir)
    : VideoFileWriter(outputDir, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 24)
{
}

VideoFileWriter::VideoFileWriter() : VideoFileWriter(fs::current_path()) {}

void VideoFileWriter::open()
{
    this->split(0);
}

void VideoFileWriter::write(const cv::Mat &image)
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

void VideoFileWriter::split(size_t splitCount)
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

fs::path VideoFileWriter::getFilePath(size_t splitCount) const
{
    std::ostringstream out{"video_"};
    out.fill('0');
    out.width(10);
    out << splitCount << ".mp4";
    return this->getOutputDir() / out.str();
}
