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
    if (!fs::is_regular_file(imagePath))
        throw std::runtime_error("image path is not a regular file");
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