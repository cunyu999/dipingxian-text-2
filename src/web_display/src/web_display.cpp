#include <rclcpp/rclcpp.hpp>
#include <my_camera_msgs/msg/camera_image.hpp>
#include <fcntl.h>
#include <sys/mman.h>
#include <semaphore.h>
#include <unistd.h>
#include <cstring>
#include <httplib.h>
#include <jpeglib.h>

class WebDisplay : public rclcpp::Node {
public:
    WebDisplay() : Node("web_display"), shm_buffer_(nullptr), width_(0), height_(0) {
        subscription_ = this->create_subscription<my_camera_msgs::msg::CameraImage>(
            "camera_image", 10, std::bind(&WebDisplay::on_image_received, this, std::placeholders::_1));
        
        // 初始化HTTP服务器
        server_.Get("/video", [this](const httplib::Request&, httplib::Response& res) {
            res.set_content_provider("multipart/x-mixed-replace; boundary=frame", [this](size_t /*offset*/, httplib::DataSink& sink) {
                while (rclcpp::ok()) {
                    sem_wait(&sem_);
                    if (shm_buffer_ != nullptr) {
                        std::vector<unsigned char> jpeg_data = nv12_to_jpeg(shm_buffer_, width_, height_);
                        std::string header = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + std::to_string(jpeg_data.size()) + "\r\n\r\n";
                        sink.write(header.data(), header.size());
                        sink.write(reinterpret_cast<const char*>(jpeg_data.data()), jpeg_data.size());
                        sink.write("\r\n", 2);
                    }
                    sem_post(&sem_);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                return true;
            });
        });

        // 启动HTTP服务器
        server_thread_ = std::thread([this]() {
            server_.listen("0.0.0.0", 8000);
        });
    }

    ~WebDisplay() {
        server_.stop();
        server_thread_.join();
        if (shm_buffer_ != nullptr) {
            munmap(shm_buffer_, width_ * height_ * 3 / 2);
        }
        shm_unlink("/camera_shm");
        sem_destroy(&sem_);
    }

private:
    void on_image_received(const my_camera_msgs::msg::CameraImage::SharedPtr msg) {
        width_ = msg->width;
        height_ = msg->height;

        // 打开共享内存
        int shm_fd = shm_open(msg->shm_id.c_str(), O_RDONLY, 0666);
        if (shm_fd == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open shared memory");
            return;
        }

        // 映射共享内存
        if (shm_buffer_ != nullptr) {
            munmap(shm_buffer_, width_ * height_ * 3 / 2);
        }
        shm_buffer_ = mmap(NULL, width_ * height_ * 3 / 2, PROT_READ, MAP_SHARED, shm_fd, 0);
        if (shm_buffer_ == MAP_FAILED) {
            RCLCPP_ERROR(this->get_logger(), "Failed to map shared memory");
            shm_buffer_ = nullptr;
            close(shm_fd);
            return;
        }

        close(shm_fd);

        // 初���化信号量
        sem_init(&sem_, 1, 1);
    }

    std::vector<unsigned char> nv12_to_jpeg(void* nv12_data, int width, int height) {
        std::vector<unsigned char> jpeg_data;
        jpeg_compress_struct cinfo;
        jpeg_error_mgr jerr;

        cinfo.err = jpeg_std_error(&jerr);
        jpeg_create_compress(&cinfo);

        unsigned char* outbuffer = nullptr;
        unsigned long outsize = 0;
        jpeg_mem_dest(&cinfo, &outbuffer, &outsize);

        cinfo.image_width = width;
        cinfo.image_height = height;
        cinfo.input_components = 3;
        cinfo.in_color_space = JCS_YCbCr;

        jpeg_set_defaults(&cinfo);
        jpeg_set_quality(&cinfo, 85, TRUE);
        jpeg_start_compress(&cinfo, TRUE);

        JSAMPROW row_pointer[1];
        int row_stride = width * 3;
        unsigned char* yuv_data = static_cast<unsigned char*>(nv12_data);

        while (cinfo.next_scanline < cinfo.image_height) {
            row_pointer[0] = &yuv_data[cinfo.next_scanline * row_stride];
            jpeg_write_scanlines(&cinfo, row_pointer, 1);
        }

        jpeg_finish_compress(&cinfo);
        jpeg_destroy_compress(&cinfo);

        jpeg_data.assign(outbuffer, outbuffer + outsize);
        free(outbuffer);

        return jpeg_data;
    }

    rclcpp::Subscription<my_camera_msgs::msg::CameraImage>::SharedPtr subscription_;
    httplib::Server server_;
    std::thread server_thread_;
    void* shm_buffer_;
    int width_;
    int height_;
    sem_t sem_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WebDisplay>());
    rclcpp::shutdown();
    return 0;
}