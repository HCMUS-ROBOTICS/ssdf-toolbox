#pragma once
#include <optional>
#include <filesystem>
#include <opencv2/core.hpp>

/**
 * @brief A base class provides some common methods to read image from a source
 */
class ImageReader
{
public:
    /**
     * @brief Get an image from a source
     * @return an image if the source is readable, otherwise an std::nullopt
     */
    virtual std::optional<cv::Mat> get() = 0;

    /**
     * @brief Set looping so that when the source reaches its end. It can go back
     * to the begining
     * @param isLoop whether to loop or not
     */
    void setLoop(bool isLoop) { this->_isLoop = isLoop; }

    bool isLoop() const { return this->_isLoop; }

    /**
     * @brief Get the number of images in this reader
     * @return the number of images in this reader
     */
    virtual size_t size() const = 0;

private:
    bool _isLoop = false;
};

/**
 * @brief A class to read images from a folder containing ONLY images.
 * Since we only list all the file in the given directory, it might not
 * work if this folder contains a strange file, i.e. a text file.
 */
class FolderImageReader
    : public ImageReader
{
public:
    /**
     * @brief A default constructor
     * @param imagePath a directory path
     * @param isDisableSort whether to sort the paths or not. Use with your caution.
     */
    FolderImageReader(std::filesystem::path imagePath, bool isDisableSort = false);

    virtual std::optional<cv::Mat> get() override;

    virtual size_t size() const override { return this->_imagePaths.size(); }

private:
    size_t _curIndex;
    std::vector<std::filesystem::path> _imagePaths;
};

/**
 * @brief A class to read image from a singular file.
 * It is convenient for debugging when you just want
 * this node to publish a single image for the whole life time.
 */
class FileImageReader
    : public ImageReader
{
public:
    /**
     * @brief A default constructor
     * @param imagePath a path to an image file. Only supports types that
     * OpenCV supports.
     */
    FileImageReader(std::filesystem::path imagePath);

    virtual std::optional<cv::Mat> get() override;

    virtual size_t size() const override { return 1l; }

private:
    bool _isReadOnce;
    cv::Mat _image;
};