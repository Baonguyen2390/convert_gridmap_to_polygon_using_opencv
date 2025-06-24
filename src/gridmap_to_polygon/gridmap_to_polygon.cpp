#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
#include <string>
#include <memory>
#include <algorithm>

#include <gridmap_to_polygon/gridmap_to_polygon.h>

GridMapToPolygonConverter::GridMapToPolygonConverter()
    : Node("gridmap_to_polygon")
    {
        // Khai báo các tham số
        declare_parameter("map_topic", "/map");
        declare_parameter("use_file_input", false);
        declare_parameter("map_yaml_path", "");
        declare_parameter("map_pgm_path", "");
        declare_parameter("occupancy_threshold", 50);
        declare_parameter("min_polygon_area", 0.5);
        declare_parameter("simplify_tolerance", 2.0);
        declare_parameter("min_area", 100);
        declare_parameter("publish_frame_id", "map");

        // Lấy giá trị tham số
        get_parameter("map_topic", map_topic_);
        get_parameter("use_file_input", use_file_input_);
        get_parameter("map_yaml_path", map_yaml_path_);
        get_parameter("map_pgm_path", map_pgm_path_);
        get_parameter("occupancy_threshold", occupancy_threshold_);
        get_parameter("min_polygon_area", min_polygon_area_);
        get_parameter("simplify_tolerance", simplify_tolerance_);
        get_parameter("min_area", min_area_);
        get_parameter("publish_frame_id", publish_frame_id_);

        // In giá trị tham số để kiểm tra
        RCLCPP_INFO(this->get_logger(), "Parameter use_file_input: %s", use_file_input_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "Parameter map_yaml_path: %s", map_yaml_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Parameter map_pgm_path: %s", map_pgm_path_.c_str());

        // Khởi tạo publishers
        outer_polygon_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>("outer_polygons", 10);
        inner_polygon_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>("inner_polygons", 10);

        // Nếu sử dụng file input, đọc map từ file
        if (use_file_input_) {
            RCLCPP_INFO(this->get_logger(), "Using file input mode.");
            loadMapFromFile();
        } else {
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
            qos.reliable();
            qos.transient_local();
            map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
                map_topic_, qos,
                std::bind(&GridMapToPolygonConverter::mapCallback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s with QoS: reliable, transient_local, depth 10", map_topic_.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "GridMap to Polygon Converter initialized.");
    }

void GridMapToPolygonConverter::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received map data with width: %d, height: %d, data size: %zu",
                msg->info.width, msg->info.height, msg->data.size());
    if (msg->data.size() != static_cast<size_t>(msg->info.width * msg->info.height)) {
        RCLCPP_ERROR(this->get_logger(), "Data size mismatch: expected %d, got %zu",
                        msg->info.width * msg->info.height, msg->data.size());
        return;
    }
    processGridMap(*msg);
}

void GridMapToPolygonConverter::loadMapFromFile()
{
    try {
        RCLCPP_INFO(this->get_logger(), "Loading map from YAML: %s", map_yaml_path_.c_str());
        YAML::Node config = YAML::LoadFile(map_yaml_path_);
        if (!config["image"]) {
            RCLCPP_ERROR(this->get_logger(), "YAML file missing 'image' field");
            return;
        }
        if (!config["resolution"]) {
            RCLCPP_ERROR(this->get_logger(), "YAML file missing 'resolution' field");
            return;
        }
        if (!config["origin"]) {
            RCLCPP_ERROR(this->get_logger(), "YAML file missing 'origin' field");
            return;
        }
        std::string image_path = config["image"].as<std::string>();
        float resolution = config["resolution"].as<float>();
        std::vector<double> origin = config["origin"].as<std::vector<double>>();
        bool negate = config["negate"].as<bool>(false);

        cv::Mat pgm_image = cv::imread(map_pgm_path_, cv::IMREAD_GRAYSCALE);
        if (pgm_image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load PGM file: %s", map_pgm_path_.c_str());
            return;
        }

        // Chuyển ảnh thành nhị phân như trong image.py
        cv::Mat binary_image = cv::Mat::zeros(pgm_image.size(), CV_8UC1);
        if (negate) {
            pgm_image = 255 - pgm_image;
        }
        cv::threshold(pgm_image, binary_image, 253, 255, cv::THRESH_BINARY); // Giá trị >= 254 -> 255

        // Lưu ảnh nhị phân để debug
        cv::imwrite("/home/nguyen/binary_output.png", binary_image);

        // Tạo OccupancyGrid
        nav_msgs::msg::OccupancyGrid grid_map;
        grid_map.info.resolution = resolution;
        grid_map.info.width = pgm_image.cols;
        grid_map.info.height = pgm_image.rows;
        grid_map.info.origin.position.x = origin[0];
        grid_map.info.origin.position.y = origin[1];
        grid_map.data.resize(pgm_image.cols * pgm_image.rows);

        for (int y = 0; y < pgm_image.rows; ++y) {
            for (int x = 0; x < pgm_image.cols; ++x) {
                int value = binary_image.at<uchar>(pgm_image.rows - 1 - y, x);
                grid_map.data[y * pgm_image.cols + x] = value == 255 ? 100 : 0;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Loaded map with width: %d, height: %d", pgm_image.cols, pgm_image.rows);
        processGridMap(grid_map);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error loading map file: %s", e.what());
    }
}

cv::Mat GridMapToPolygonConverter::removeNoise(const cv::Mat &binary_image)
{
    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(binary_image, labels, stats, centroids, 8);
    cv::Mat output = cv::Mat::zeros(binary_image.size(), CV_8UC1);

    for (int i = 1; i < num_labels; ++i) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area >= min_area_) {
            output.setTo(255, labels == i);
        }
    }
    return output;
}

void GridMapToPolygonConverter::processGridMap(const nav_msgs::msg::OccupancyGrid &grid_map)
{
    RCLCPP_INFO(this->get_logger(), "Processing map with %zu data points", grid_map.data.size());

    // Chuyển grid map thành ảnh nhị phân
    cv::Mat binary_image(grid_map.info.height, grid_map.info.width, CV_8UC1);
    try {
        for (size_t i = 0; i < grid_map.data.size(); ++i) {
            int x = i % grid_map.info.width;
            int y = grid_map.info.height - 1 - (i / grid_map.info.width);
            binary_image.at<uchar>(y, x) = (grid_map.data[i] >= occupancy_threshold_) ? 255 : 0;
        }
        RCLCPP_INFO(this->get_logger(), "Converted to binary image");
    } catch (const cv::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "OpenCV error in binary conversion: %s", e.what());
        return;
    }

    // Lưu ảnh nhị phân
    cv::imwrite("/home/nguyen/binary_output.png", binary_image);

    // Lọc nhiễu
    binary_image = removeNoise(binary_image);
    RCLCPP_INFO(this->get_logger(), "Applied noise removal with min_area %d", min_area_);

    // Lưu ảnh sau lọc nhiễu
    cv::imwrite("/home/nguyen/binary_filtered_output.png", binary_image);

    // Tìm contours với RETR_CCOMP để lấy cả outer và inner
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    try {
        cv::findContours(binary_image, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
        RCLCPP_INFO(this->get_logger(), "Found %zu contours", contours.size());
        if (contours.empty()) {
            RCLCPP_WARN(this->get_logger(), "No contours found in the binary image");
        }
    } catch (const cv::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "OpenCV error in findContours: %s", e.what());
        return;
    }

    // Tạo ảnh để vẽ contours (tương tự polygon.py)
    cv::Mat vis = cv::Mat::zeros(binary_image.size(), CV_8UC3);

    // Xử lý từng contour
    for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]) * grid_map.info.resolution * grid_map.info.resolution;
        // Log diện tích của tất cả contours
        bool is_outer = hierarchy[i][3] == -1;
        std::string kind = is_outer ? "OUTER" : "INNER";
        RCLCPP_INFO(this->get_logger(), "[%s] Contour %zu: area = %.1f", kind.c_str(), i, area);

        if (area < min_polygon_area_) {
            RCLCPP_INFO(this->get_logger(), "Skipping contour %zu due to area (%.1f) < min_polygon_area (%.1f)", i, area, min_polygon_area_);
            continue;
        }

        // Đơn giản hóa contour
        std::vector<cv::Point> simplified_contour;
        try {
            cv::approxPolyDP(contours[i], simplified_contour, simplify_tolerance_, true);
            RCLCPP_INFO(this->get_logger(), "Simplified contour %zu to %zu points", i, simplified_contour.size());
        } catch (const cv::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV error in approxPolyDP: %s", e.what());
            continue;
        }

        // Vẽ contour lên ảnh (outer: xanh lá, inner: đỏ)
        cv::Scalar color = is_outer ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        cv::polylines(vis, simplified_contour, true, color, 1);

        // Chuyển thành PolygonStamped
        geometry_msgs::msg::PolygonStamped polygon_msg;
        polygon_msg.header.frame_id = publish_frame_id_;
        polygon_msg.header.stamp = this->now();

        for (const auto &point : simplified_contour) {
            geometry_msgs::msg::Point32 polygon_point;
            polygon_point.x = grid_map.info.origin.position.x + point.x * grid_map.info.resolution;
            polygon_point.y = grid_map.info.origin.position.y + (grid_map.info.height - point.y) * grid_map.info.resolution;
            polygon_msg.polygon.points.push_back(polygon_point);
        }

        // Xuất bản polygon trên topic tương ứng
        if (is_outer) {
            outer_polygon_pub_->publish(polygon_msg);
            RCLCPP_INFO(this->get_logger(), "Published OUTER polygon with %zu points on topic /outer_polygons", simplified_contour.size());
        } else {
            inner_polygon_pub_->publish(polygon_msg);
            RCLCPP_INFO(this->get_logger(), "Published INNER polygon with %zu points on topic /inner_polygons", simplified_contour.size());
        }
    }

    // Lưu ảnh với contours
    cv::imwrite("/home/nguyen/polygon_contours_output.png", vis);
}
