#pragma once
#include <filesystem>
#include <memory>
#include <ros/ros.h>
#include "image_pub_sub/image_reader.hpp"
#include "image_pub_sub/image_writer.hpp"

/**
 * @brief A default ImageReader factory
 * This class provide an easy to use that hiding if/else from
 * the main function.
 */
class ImageReaderFactory
{
public:
    /**
     * @brief Open an ImageReader given an image path
     * @param imagePath the image path. It could be a single image file
     * or a directory containing images.
     * @return a shared pointer to an ImageReader instance
     */
    static std::shared_ptr<ImageReader> open(const std::filesystem::path &imagePath)
    {
        namespace fs = std::filesystem;
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

/**
 * @brief A default ImageWriter factory
 * This class provide an easy to use that hiding if/else from
 * the main function.
 */
class ImageWriterFactory
{
public:

    /**
     * @brief Create ImageWriter instance
     * This method will create outputDir if it does not exist.
     * @param outputDir the output location to store the output images/videos
     * @param outVideo whether to write down a video or image frames
     * @return a shared pointer to an ImageWriter instance
     */
    static std::shared_ptr<ImageWriter> open(const std::filesystem::path &outputDir, bool outVideo)
    {
        namespace fs = std::filesystem;
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
