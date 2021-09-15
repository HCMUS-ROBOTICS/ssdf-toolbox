#pragma once
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <string>

class ImageWriter
{
public:
    ImageWriter(const std::filesystem::path &outputDir);
    virtual void open() = 0;
    virtual void write(const cv::Mat &image);
    virtual void close() = 0;
    virtual void split(size_t splitCount) = 0;

    int getFrameCount() const { return _frameCounter; }

    size_t getSplitCount() const { return _splitCounter; }

    void setNumSplit(size_t numSplit) { _numSplit = numSplit; }

    std::filesystem::path getOutputDir() const { return _outputDir; }

private:
    int _frameCounter;
    int _splitCounter;
    size_t _numSplit;
    std::filesystem::path _outputDir;
};

class FolderImageWriter
    : public ImageWriter
{
public:
    FolderImageWriter(std::filesystem::path outputDir, const std::string &imageExt);

    virtual void open() override;

    virtual void write(const cv::Mat &image) override;

    virtual void split(size_t splitCount) override;

    virtual void close() override {}

private:
    std::filesystem::path getFilePath() const;

private:
    std::string _imageExt;
    std::filesystem::path _splitDir;
};

class VideoFileWriter
    : public ImageWriter
{
public:
    VideoFileWriter(const std::filesystem::path &outputDir, int fourcc, double fps = 24);

    VideoFileWriter(const std::filesystem::path &outputDir);

    VideoFileWriter();

    virtual void open() override;

    virtual void write(const cv::Mat &image) override;

    virtual void split(size_t splitCount) override;

    virtual void close() override { _impl.release(); }

private:
    std::filesystem::path getFilePath(size_t splitCount) const;

private:
    int _fourcc;
    double _fps;
    cv::Size _frameSize;
    cv::VideoWriter _impl;
};