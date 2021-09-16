#include "image_pub_sub/image_reader.hpp"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

namespace fs = std::filesystem;

FolderImageReader::FolderImageReader(fs::path imagePath, bool isDisableSort)
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

std::optional<cv::Mat> FolderImageReader::get()
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

//////////////////////////////////////////////////////

FileImageReader::FileImageReader(fs::path imagePath)
    : _isReadOnce{false}
{
    _image = cv::imread(imagePath);
    if (_image.empty())
        throw std::runtime_error("image path is not an image");
}

std::optional<cv::Mat> FileImageReader::get()
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

bool FileImageReader::isImageFile(const std::filesystem::path &path)
{
    auto image = cv::imread(path);
    return !image.empty();
}


///////////////////////////////////////////////////////

VideoFileReader::VideoFileReader(const fs::path &videoPath)
    : _video{cv::VideoCapture(videoPath)}
{
    if (!_video.isOpened())
    {
        throw std::runtime_error("Cannot read video path or the extension is not supported");
    }
}

std::optional<cv::Mat> VideoFileReader::get()
{
    cv::Mat frame;
    if (_video.read(frame))
    {
        return frame;
    }

    _video.release();
    if (isLoop())
    {
        _video.open(_videoPath);
        if (_video.read(frame))
        {
            return frame;
        }
    }
    return std::nullopt;
}

size_t VideoFileReader::size() const
{
    double frameCount = _video.get(cv::CAP_PROP_FRAME_COUNT);
    return static_cast<size_t>(frameCount);
}

bool VideoFileReader::isVideoFile(const std::filesystem::path &path)
{
    cv::Mat frame;
    auto video = cv::VideoCapture(path);
    if (video.read(frame))
    {
        return true;
    }
    return false;
}
