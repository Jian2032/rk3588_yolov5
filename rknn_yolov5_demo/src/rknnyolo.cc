#include "rknnyolo.hpp"
#include "postprocess.h"

// 创建静态指针后期用于保存对象
RknnYoloNode* RknnYoloNode::instance_ = nullptr;

// RknnYoloNode构造函数
RknnYoloNode::RknnYoloNode(const std::string &model_path)
    : Node("rknn_yolov5_node"), frames_(0), last_time_(0.0)
{
    RCLCPP_INFO(this->get_logger(), "Initializing RKNN YOLOv5 Node...");
    // 保存当前类的指针
    instance_ = this;
    // 创建线程数
    int threadNum = 3;
    // 创建线程池
    pool_ = std::make_shared<rknnPool<rkYolov5s, cv::Mat, cv::Mat>>(model_path, threadNum);
    int ret = pool_->init();
    if (ret != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize rknnPool! Error code: %d", ret);
        rclcpp::shutdown();
        return; // 安全退出
    }

    // 订阅相机图像
    try {
        sub_image_raw_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 100,
            std::bind(&RknnYoloNode::imageCallback, this, std::placeholders::_1));
        sub_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/aligned_depth_to_color/image_raw",100,
            std::bind(&RknnYoloNode::depthCallback, this, std::placeholders::_1));
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create subscription: %s", e.what());
        rclcpp::shutdown();
        return;
    }

    // 发布检测后的图像和目标点
    pub_img_ = this->create_publisher<sensor_msgs::msg::Image>("/yolov5/detected_image", 30);

    pub_targets_ = this->create_publisher<robot_msgs::msg::TargetArray>("target_info", 30);

    // 开启处理线程
    running_ = true;
    worker_ = std::thread(&RknnYoloNode::processThread, this);

    RCLCPP_INFO(this->get_logger(), "RKNN YOLOv5 Node started successfully!");
}

RknnYoloNode::~RknnYoloNode()
{
    running_ = false;
    if (worker_.joinable()) worker_.join();

    robot_msgs::msg::TargetArray msg{};
    pub_targets_->publish(msg);
}

void RknnYoloNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (!pool_) return;

    cv::Mat img;
    try {
        img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // 入队（只管放图像，不做推理）
    if (pool_->put(img) != 0) {
        RCLCPP_WARN(this->get_logger(), "ThreadPool is full, dropping frame");
    }
}

void RknnYoloNode::processThread()
{
    while (rclcpp::ok() && running_)
    {
        cv::Mat img;
        if (pool_->get(img) != 0) continue;

        // FPS 统计
        frames_++;
        if (frames_ % 60 == 0) {
            auto now = this->now();
            double fps = 60.0 / (now.seconds() - last_time_);
            RCLCPP_INFO(this->get_logger(), "FPS: %.2f", fps);
            last_time_ = now.seconds();
        }

        // 发布结果
        std_msgs::msg::Header header;
        header.stamp = this->now();
        auto out_msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
        pub_img_->publish(*out_msg);
    }
}

void RknnYoloNode::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // ROS2 图像消息转 OpenCV 格式
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // 获取深度图
    depth_image = cv_ptr->image;

}

int RknnYoloNode::getDistance(int u, int v)
{
    // 防止越界
    if (v >= depth_image.rows || u >= depth_image.cols)
    {
        RCLCPP_WARN(this->get_logger(), "Target pixel out of image range!");
        return -1;
    }

    // 深度值（单位：毫米）
    uint16_t depth_value = depth_image.at<uint16_t>(v, u);

    // if (depth_value == 0)
    // {
    //     RCLCPP_WARN(this->get_logger(), "No depth at pixel (%d, %d)", u, v);
    // }
    // else
    // {
    //     RCLCPP_INFO(this->get_logger(), "Pixel (%d, %d) depth: %d mm",
    //                 u, v, depth_value);
    // }
    return depth_value;
}

void RknnYoloNode::addTarget(detect_result_group_t* group)
{
    detect_result_t EMPTY_RESULT{};

    auto makeTarget = [](int idx, const detect_result_t& d) -> TargetInfo {
        return TargetInfo{idx, d.box.center_u, d.box.center_v, d.box.distance};
    };

    target_.total_count = group->count;

    if(target_.total_count == 4)
    {
        // 临时数组
        int copy_count = std::min(group->count, MAX_RESULTS);
        detect_result_t temp[MAX_RESULTS];
        std::copy_n(group->results, copy_count, temp);

        // 按 distance 从远到近排序
        std::sort(temp, temp + copy_count, [](const detect_result_t& a, const detect_result_t& b) {
            return a.box.distance > b.box.distance;
        });

        // 远的两个
        const detect_result_t& far1 = temp[0];
        const detect_result_t& far2 = temp[1];
        if (far1.box.center_u < far2.box.center_u) {
            target_.targets[0] = makeTarget(1, far1);
            target_.targets[1] = makeTarget(2, far2);
        } else {
            target_.targets[0] = makeTarget(1, far2);
            target_.targets[1] = makeTarget(2, far1);
        }

        // 近的两个
        const detect_result_t& near1 = temp[copy_count - 2];
        const detect_result_t& near2 = temp[copy_count - 1];
        if (near1.box.center_u < near2.box.center_u) {
            target_.targets[2] = makeTarget(3, near1);
            target_.targets[3] = makeTarget(4, near2);
        } else {
            target_.targets[2] = makeTarget(3, near2);
            target_.targets[3] = makeTarget(4, near1);
        }
    }
    else if(target_.total_count == 3)
    {
        // 临时数组
        int copy_count = std::min(group->count, MAX_RESULTS);
        detect_result_t temp[MAX_RESULTS];
        std::copy_n(group->results, copy_count, temp);

        // 按 distance 从远到近排序
        std::sort(temp, temp + copy_count, [](const detect_result_t& a, const detect_result_t& b) {
            return a.box.distance > b.box.distance;
        });

        // 远的两个
        const detect_result_t& far2 = temp[0];

        target_.targets[0] = makeTarget(1, EMPTY_RESULT);
        target_.targets[1] = makeTarget(2, far2);

        // 近的两个
        const detect_result_t& near1 = temp[copy_count - 2];
        const detect_result_t& near2 = temp[copy_count - 1];
        if (near1.box.center_u < near2.box.center_u) {
            target_.targets[2] = makeTarget(3, near1);
            target_.targets[3] = makeTarget(4, near2);
        } else {
            target_.targets[2] = makeTarget(3, near2);
            target_.targets[3] = makeTarget(4, near1);
        }
    }
    else if(target_.total_count == 2)
    {
        // 临时数组
        int copy_count = std::min(group->count, MAX_RESULTS);
        detect_result_t temp[MAX_RESULTS];
        std::copy_n(group->results, copy_count, temp);

        // 按 distance 从远到近排序
        std::sort(temp, temp + copy_count, [](const detect_result_t& a, const detect_result_t& b) {
            return a.box.distance > b.box.distance;
        });

        // 远的两个
        target_.targets[0] = makeTarget(1, EMPTY_RESULT);
        target_.targets[1] = makeTarget(2, EMPTY_RESULT);

        // 近的两个
        const detect_result_t& near1 = temp[copy_count - 2];
        const detect_result_t& near2 = temp[copy_count - 1];
        if (near1.box.center_u < near2.box.center_u) {
            target_.targets[2] = makeTarget(3, near1);
            target_.targets[3] = makeTarget(4, near2);
        } else {
            target_.targets[2] = makeTarget(3, near2);
            target_.targets[3] = makeTarget(4, near1);
        }
    }
    else if(target_.total_count == 1)
    {
        // 临时数组
        int copy_count = std::min(group->count, MAX_RESULTS);
        detect_result_t temp[MAX_RESULTS];
        std::copy_n(group->results, copy_count, temp);

        // 按 distance 从远到近排序
        std::sort(temp, temp + copy_count, [](const detect_result_t& a, const detect_result_t& b) {
            return a.box.distance > b.box.distance;
        });

        // 远的两个
        target_.targets[0] = makeTarget(1, EMPTY_RESULT);
        target_.targets[1] = makeTarget(2, EMPTY_RESULT);

        // 近的两个
        const detect_result_t& near2 = temp[copy_count - 1];

        target_.targets[2] = makeTarget(3, EMPTY_RESULT);
        target_.targets[3] = makeTarget(4, near2);
    }
    else
    {
        target_.targets[0] = makeTarget(1, EMPTY_RESULT);
        target_.targets[1] = makeTarget(2, EMPTY_RESULT);
        target_.targets[2] = makeTarget(3, EMPTY_RESULT);
        target_.targets[3] = makeTarget(4, EMPTY_RESULT);
    }
    // RCLCPP_INFO(this->get_logger(), "/////////////////////////////////////////////");
    for (int i = 0; i < 4; i++)
    {
        const TargetInfo& t = target_.targets[i];
        // RCLCPP_INFO(this->get_logger(),
        //     "目标 %d | 编号 index=%d | center_u=%d | center_v=%d | distance=%d",
        //     i + 1,
        //     t.index,
        //     t.center_u,
        //     t.center_v,
        //     t.distance);
    }

    robot_msgs::msg::TargetArray msg;
    msg.total_count = target_.total_count;
    for (int i = 0; i < 4; i++)
    {
        robot_msgs::msg::TargetInfo t_msg;
        t_msg.index = target_.targets[i].index;
        t_msg.center_u = target_.targets[i].center_u;
        t_msg.center_v = target_.targets[i].center_v;
        t_msg.distance = target_.targets[i].distance;
        msg.targets.push_back(t_msg);
    }
    pub_targets_->publish(msg);
}