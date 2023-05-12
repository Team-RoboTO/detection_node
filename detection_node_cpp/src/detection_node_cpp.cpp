#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <image_transport/image_transport.hpp>

class PixelPosition : public rclcpp::Node
{
public:
    PixelPosition() : Node("pixel_position")
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("/zedm/zed_node/rgb/image_rect_color", qos, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
    this->imageCallback(msg);
});
        pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/zedm/zed_node/point_cloud/cloud_registered", qos, std::bind(&PixelPosition::pointcloudCallback, this, std::placeholders::_1));
        bbox_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/yolo/bboxs", qos, std::bind(&PixelPosition::bboxCallback, this, std::placeholders::_1));
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        if (!bboxs_.empty())
        {
            for (const auto& bbox : bboxs_)
            {
                int x1 = bbox[0], y1 = bbox[1], x2 = bbox[2], y2 = bbox[3];
                int xc = (x2 + x1) / 2;
                int yc = (y2 + y1) / 2;
                cv::Vec3f xyz = getPointcloudXYZ(xc, yc);
                cv::rectangle(cv_ptr->image, cv::Point(x1, y1), cv::Point(x2, y2), CV_RGB(255,0,0), 2);
                std::ostringstream text;
                text << "(" << xyz[0] << ", " << xyz[1] << ", " << xyz[2] << ")";
                cv::putText(cv_ptr->image, text.str(), cv::Point(x1, y2 - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255,0,0), 2);
            }
            cv::imshow("Pixel Position", cv_ptr->image);
            cv::waitKey(1);
        }
    }

    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::fromROSMsg(*msg, pointcloud_);
    }

    cv::Vec3f getPointcloudXYZ(int x, int y)
    {
        if (!pointcloud_.empty())
        {
            pcl::PointXYZ point = pointcloud_.at(x, y);
            return cv::Vec3f(point.x, point.y, point.z);
        }
        else
        {
            return cv::Vec3f(0, 0, 0);
        }
    }

    void bboxCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->layout.dim[0].label == "full")
        {
            bboxs_.clear();
            int size = msg->layout.dim[0].size;
            int stride = msg->layout.dim[0].stride;
            for (int i = 0; i < size; ++i)
            {
                std::vector<float> bbox;
                for (int j = 0; j < stride; ++j)
                {
                    bbox.push_back(msg->data[i * stride + j]);
                }
                bboxs_.push_back(bbox);
            }
        }
        else if (msg->layout.dim[0].label == "empty")
        {
            bboxs_.clear();
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr bbox_subscriber_;
    pcl::PointCloud<pcl::PointXYZ> pointcloud_;
    std::vector<std::vector<float>> bboxs_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PixelPosition>());
    rclcpp::shutdown();
    return 0;
}
