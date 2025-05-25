#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ColorFilter : public rclcpp::Node {
public:
    ColorFilter() : Node("color_filter") {
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/webcam/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&ColorFilter::image_callback, this, std::placeholders::_1)
        );
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::Mat hsv, mask1, mask2, mask, result;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Detect red color range (lower and upper)
        cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask1);
        cv::inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255), mask2);
        cv::bitwise_or(mask1, mask2, mask);

        result = frame.clone();
        result.setTo(cv::Scalar(255, 0, 0), mask);                 // Change red to blue

        // Convert mask to red-colored visualization
        cv::Mat red_mask_vis;
        cv::cvtColor(mask, red_mask_vis, cv::COLOR_GRAY2BGR);
        red_mask_vis.setTo(cv::Scalar(0, 0, 255), mask);           // show red in red color

        cv::imshow("Original Image", frame);
        cv::imshow("Detected red region", red_mask_vis);
        cv::imshow("Red to Blue changed part", result);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColorFilter>());
    rclcpp::shutdown();
    return 0;
}

