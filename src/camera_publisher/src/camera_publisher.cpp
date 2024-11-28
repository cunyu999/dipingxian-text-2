#include <rclcpp/rclcpp.hpp>
#include <my_camera_msgs/msg/camera_image.hpp>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <semaphore.h>
#include <unistd.h>
#include <cstring>

class CameraPublisher : public rclcpp::Node {
public:
    CameraPublisher() : Node("camera_publisher") {
        publisher_ = this->create_publisher<my_camera_msgs::msg::CameraImage>("camera_image", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&CameraPublisher::publish_image, this));
        
        // 打开摄像头设备
        fd_ = open("/dev/video0", O_RDWR);
        if (fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video device");
            return;
        }

        // 设置摄像头格式
        struct v4l2_format format;
        memset(&format, 0, sizeof(format));
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.width = 2560;
        format.fmt.pix.height = 1600;
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
        format.fmt.pix.field = V4L2_FIELD_INTERLACED;
        if (ioctl(fd_, VIDIOC_S_FMT, &format) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set video format");
            return;
        }

        // 请求缓冲区
        struct v4l2_requestbuffers req;
        memset(&req, 0, sizeof(req));
        req.count = 1;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd_, VIDIOC_REQBUFS, &req) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to request buffers");
            return;
        }

        // 查询缓冲区
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = 0;
        if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to query buffer");
            return;
        }

        // 映射缓冲区
        buffer_ = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
        if (buffer_ == MAP_FAILED) {
            RCLCPP_ERROR(this->get_logger(), "Failed to map buffer");
            return;
        }

        // 启动视频捕获
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(fd_, VIDIOC_STREAMON, &type) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start video capture");
            return;
        }

        // 创建共享内存
        shm_fd_ = shm_open("/camera_shm", O_CREAT | O_RDWR, 0666);
        if (shm_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create shared memory");
            return;
        }
        ftruncate(shm_fd_, buf.length);
        shm_buffer_ = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
        if (shm_buffer_ == MAP_FAILED) {
            RCLCPP_ERROR(this->get_logger(), "Failed to map shared memory");
            return;
        }

        // 初始化信号量
        sem_init(&sem_, 1, 1);
    }

    ~CameraPublisher() {
        // 停止视频捕获
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd_, VIDIOC_STREAMOFF, &type);

        close(fd_);
        shm_unlink("/camera_shm");
        sem_destroy(&sem_);
    }

private:
    void publish_image() {
        // 采集图像
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = 0;
        if (ioctl(fd_, VIDIOC_QBUF, &buf) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to queue buffer");
            return;
        }
        if (ioctl(fd_, VIDIOC_DQBUF, &buf) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to dequeue buffer");
            return;
        }

        // 复制图像数据到共享内存
        sem_wait(&sem_);
        memcpy(shm_buffer_, buffer_, buf.bytesused);
        sem_post(&sem_);

        // 发布自定义消息
        auto message = my_camera_msgs::msg::CameraImage();
        message.shm_id = "/camera_shm";
        message.width = 2560;
        message.height = 1600;
        message.encoding = "nv12";
        message.timestamp = this->now();
        publisher_->publish(message);
    }

    rclcpp::Publisher<my_camera_msgs::msg::CameraImage>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int fd_;
    void* buffer_;
    int shm_fd_;
    void* shm_buffer_;
    sem_t sem_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
    return 0;
}
