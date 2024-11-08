// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include "../cv-helpers.hpp"    // Helper functions for conversions between RealSense and OpenCV
#include <wiringSerial.h>

const float CAMERA_OFFSET = -0.25f;

struct dfs_result {
    double count;
    double x_accum;
    double y_accum;
};

void add(dfs_result* r1, dfs_result r2) {
    r1->count += r2.count;
    r1->x_accum += r2.x_accum;
    r1->y_accum += r2.y_accum;
}

// thanks google gpt?

// Function to perform DFS and count the size of the component
dfs_result dfs(cv::Mat matrix, int i, int j, std::vector<std::vector<bool>>& visited) {
    if (i < 0 || i >= matrix.rows || j < 0 || j >= matrix.cols || visited[i][j] || matrix.at<unsigned char>(i, j) != 255) {
        return dfs_result {
            count: 0,
            x_accum: 0,
            y_accum: 0,
        };
    }

    visited[i][j] = true;
    dfs_result out = dfs_result {
        count: 1,
        x_accum: j,
        y_accum: i,
    };

    // Explore adjacent cells
    add(&out, dfs(matrix, i + 1, j, visited));

    add(&out, dfs(matrix, i - 1, j, visited));

    add(&out, dfs(matrix, i, j - 1, visited));

    add(&out, dfs(matrix, i, j + 1, visited));

    return out;
}

dfs_result largestConnectedComponent(cv::Mat matrix) {
    int rows = matrix.rows;
    int cols = matrix.cols;
    // std::cout << rows << ", " << cols << std::endl;
    double maxComponentSize = 0.0f;
    double x_centroid = 0.0f;
    double y_centroid = 0.0f;

    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (matrix.at<unsigned char>(i, j) == 255 && !visited[i][j]) {
                dfs_result result = dfs(matrix, i, j, visited);
                if (result.count > maxComponentSize) {
                    x_centroid = result.x_accum / result.count;
                    y_centroid = result.y_accum / result.count;
                }
                maxComponentSize = std::max(maxComponentSize, result.count);
            }
        }
    }

    dfs_result retval;
    retval.count = maxComponentSize;
    retval.x_accum = x_centroid;
    retval.y_accum = y_centroid;
    return retval;
}

int main(int argc, char * argv[]) try
{
    using namespace cv;
    using namespace rs2;

    bool write_to_serial = argc > 1;

    // Define colorizer and align processing-blocks
    colorizer colorize;
    align align_to(RS2_STREAM_COLOR);

    // Start the camera
    pipeline pipe;
    pipe.start();

    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    // We are using StructuringElement for erode / dilate operations
    auto gen_element = [](int erosion_size)
    {
        return getStructuringElement(MORPH_RECT,
            Size(erosion_size + 1, erosion_size + 1),
            Point(erosion_size, erosion_size));
    };

    int fd ;

    if (write_to_serial) {
        if((fd=serialOpen("/dev/ttyACM0",9600))<0){
            fprintf(stderr,"Unable to open serial device: %s\n",strerror(errno));
            return 1;
        }
    }

    const int erosion_size = 3;
    auto erode_less = gen_element(erosion_size);
    auto erode_more = gen_element(erosion_size * 2);

    // The following operation is taking grayscale image,
    // performs threashold on it, closes small holes and erodes the white area
    auto create_mask_from_depth = [&](Mat& depth, int thresh, ThresholdTypes type)
    {
        threshold(depth, depth, thresh, 255, type);
        dilate(depth, depth, erode_less);
        erode(depth, depth, erode_more);
    };

    int ctr = 0;
    // Skips some frames to allow for auto-exposure stabilization
    for (int i = 0; i < 10; i++) pipe.wait_for_frames();

    rs2_intrinsics intrinsics = pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();

    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        ctr += 1;
        frameset data = pipe.wait_for_frames();
        // Make sure the frameset is spatialy aligned 
        // (each pixel in depth image corresponds to the same pixel in the color image)
        frameset aligned_set = align_to.process(data);
        depth_frame depth = aligned_set.get_depth_frame();
        auto color_mat = frame_to_mat(aligned_set.get_color_frame());

        // Colorize depth image with white being near and black being far
        // This will take advantage of histogram equalization done by the colorizer
        // colorize.set_option(RS2_OPTION_COLOR_SCHEME, 2);
        frame bw_depth = depth.apply_filter(colorize);

        Mat hsv_frame;
        Mat red_mask;
        cvtColor(color_mat, hsv_frame, COLOR_RGB2HSV);
        inRange(hsv_frame, Scalar(110,160,50), Scalar(130,255,255), red_mask);

        auto color_depth = frame_to_mat(bw_depth);

        // Extract foreground pixels based on refined mask from the algorithm
        Mat3b foreground = Mat3b::zeros(color_mat.rows, color_mat.cols);
        color_depth.copyTo(foreground, (red_mask == 255));
        dfs_result cc_info = largestConnectedComponent(red_mask);
        // std::cout << cc_info.count << " = size. Pixel: " << cc_info.y_accum << ", " << + cc_info.x_accum << "\n";

        float point[3];
        // Get the depth frame's dimensions
        float width = depth.get_width();
        float height = depth.get_height();
        float pixel[2] = {(float) cc_info.x_accum, (float) cc_info.y_accum};
        rs2_deproject_pixel_to_point(point, &intrinsics, pixel, depth.get_distance(pixel[0], pixel[1]));
        // std::cout << point[0] << ", " << point[1] << ", " << point[2] << "\n";
        if (ctr == 60) {
            ctr = 0;
            if (write_to_serial) {
                // write depth to serial
                float out = point[2] + CAMERA_OFFSET;
                char buffer[64];

                int ret = sprintf(buffer, "%f", out);
                std::cout << "Writing " << buffer << " to serial..." << std::endl;
                serialPuts(fd, buffer);
            }
        }

        imshow(window_name, foreground);
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}



