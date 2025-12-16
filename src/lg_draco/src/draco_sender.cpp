#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>

// Draco Compression Headers
#include <draco/compression/encode.h>
#include <draco/compression/point_cloud/point_cloud_encoder.h>
#include <draco/core/encoder_buffer.h>
#include <draco/point_cloud/point_cloud.h>
#include <draco/point_cloud/point_cloud_builder.h>

// WebSocket / Network Headers
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>

// Standard Library
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string_view>
#include <thread>
#include <unordered_set>
#include <vector>

// TF2 Headers
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nlohmann/json.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

using json = nlohmann::json;

// 복셀 인덱싱 구조체
struct VoxelIndex {
    int x, y, z;
    bool operator==(const VoxelIndex& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

// 복셀 해시 함수
struct VoxelHash {
    size_t operator()(const VoxelIndex& k) const {
        size_t res = 17;
        res = res * 31 + std::hash<int>()(k.x);
        res = res * 31 + std::hash<int>()(k.y);
        res = res * 31 + std::hash<int>()(k.z);
        return res;
    }
};

class DracoSenderNode : public rclcpp::Node {
   public:
    DracoSenderNode() : Node("draco_sender_node") {
        // 파라미터 선언
        this->declare_parameter("host", "");
        this->declare_parameter("port", "");
        this->declare_parameter("endpoint", "");
        this->declare_parameter("input_topic", "/camera/camera/depth/color/points");

        // 필터링 파라미터
        this->declare_parameter("voxel_size", 0.001);  // 1mm 단위
        this->declare_parameter("skip_length", 1.0f);  // 1m 이상 거리는 자름 (카메라 기준)

        // TF 타겟 프레임 (예: 로봇의 바닥)
        this->declare_parameter("target_frame", "base_link");

        // TF 리스너 초기화
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 웹소켓 초기화 (무한 재시도 포함)
        if (!reconnect_with_retry(-1, 1000)) {
            RCLCPP_ERROR(this->get_logger(), "Initial WebSocket connection not established yet. Will keep retrying in background.");
        }

        // ping 수신 시각 초기화 및 수신 루프 시작
        last_ping_ms_.store(now_ms());
        start_ws_reader();

        fps_time = this->now();
        std::string topic_name = this->get_parameter("input_topic").as_string();
        subscription_ =
            this->create_subscription<sensor_msgs::msg::PointCloud2>(topic_name, rclcpp::SensorDataQoS(), std::bind(&DracoSenderNode::topic_callback, this, std::placeholders::_1));
        network_publisher_ = this->create_publisher<std_msgs::msg::String>("/network_status", 10);

        RCLCPP_INFO(this->get_logger(), "Draco Sender Node Started.");
        RCLCPP_INFO(this->get_logger(), "Target Frame: %s, Skip Length: %.2fm", this->get_parameter("target_frame").as_string().c_str(),
                    this->get_parameter("skip_length").as_double());
    }

   private:
    net::io_context io_context_;
    std::shared_ptr<websocket::stream<tcp::socket>> ws_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr network_publisher_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Time fps_time;
    int frame_count = 0;
    double fps = 0.0;
    std::atomic<int64_t> last_ping_ms_{0};
    std::atomic<bool> stop_reader_{false};
    std::thread ws_reader_;
    std::mutex ws_mutex_;
    std::atomic<bool> connected_{false};
    std::atomic<bool> reconnecting_{false};

    int64_t now_ms() const {
        return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    void start_ws_reader() {
        ws_reader_ = std::thread([this]() {
            beast::flat_buffer buffer;
            while (!stop_reader_.load()) {
                if (!ensure_connection()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    continue;
                }

                std::shared_ptr<websocket::stream<tcp::socket>> ws_copy;
                {
                    std::lock_guard<std::mutex> lock(ws_mutex_);
                    ws_copy = ws_;
                }
                if (!ws_copy) {
                    connected_.store(false);
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    continue;
                }

                beast::error_code ec;
                ws_copy->read(buffer, ec);
                if (ec) {
                    RCLCPP_WARN(this->get_logger(), "WebSocket read warning: %s", ec.message().c_str());
                    close_current_ws();
                    reconnect_with_retry(-1, 500);
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    continue;
                }
                auto data = beast::buffers_to_string(buffer.data());
                buffer.consume(buffer.size());
                if (data == "ping") {
                    last_ping_ms_.store(now_ms());
                }
            }
        });
        ws_reader_.detach();
    }

    void close_current_ws() {
        std::lock_guard<std::mutex> lock(ws_mutex_);
        if (ws_) {
            beast::error_code ec;
            if (ws_->is_open()) {
                ws_->close(websocket::close_code::normal, ec);
            }
            ws_.reset();
        }
        connected_.store(false);
    }

    bool connect_once() {
        try {
            std::string host = this->get_parameter("host").as_string();
            std::string port = this->get_parameter("port").as_string();
            std::string endpoint = this->get_parameter("endpoint").as_string();

            io_context_.restart();  // 이전 실패로 stop 상태가 되었을 수 있으니 재시작
            tcp::resolver resolver(io_context_);
            auto const results = resolver.resolve(host, port);

            auto new_ws = std::make_shared<websocket::stream<tcp::socket>>(io_context_);
            auto ep = net::connect(new_ws->next_layer(), results);

            std::string host_port = host + ":" + std::to_string(ep.port());

            new_ws->handshake(host_port, endpoint);
            new_ws->binary(false);
            new_ws->write(net::buffer("lidar/draco"));
            new_ws->binary(true);

            {
                std::lock_guard<std::mutex> lock(ws_mutex_);
                ws_ = new_ws;
            }
            connected_.store(true);
            last_ping_ms_.store(now_ms());
            RCLCPP_INFO(this->get_logger(), "Connected to WebSocket: %s", host_port.c_str());
            return true;
        } catch (std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "WebSocket Init Error: %s", e.what());
            return false;
        }
    }

    bool reconnect_with_retry(int attempts = -1, int delay_ms = 500) {
        bool expected = false;
        if (!reconnecting_.compare_exchange_strong(expected, true)) {
            RCLCPP_WARN(this->get_logger(), "WebSocket reconnect already in progress");
            return false;  // 이미 재연결 중
        }
        auto on_exit = std::unique_ptr<void, std::function<void(void*)>>(nullptr, [this](void*) { reconnecting_.store(false); });

        int tries = 0;
        RCLCPP_WARN(this->get_logger(), "WebSocket reconnect loop start (attempts=%s, delay_ms=%d)", (attempts < 0 ? "inf" : std::to_string(attempts).c_str()), delay_ms);
        while (attempts < 0 || tries < attempts) {
            close_current_ws();
            RCLCPP_WARN(this->get_logger(), "WebSocket reconnect attempt #%d", tries + 1);
            if (connect_once()) {
                RCLCPP_INFO(this->get_logger(), "WebSocket reconnect success");
                return true;
            }
            ++tries;
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        }
        RCLCPP_ERROR(this->get_logger(), "WebSocket reconnect failed after %d attempts", tries);
        return false;
    }

    bool ensure_connection() {
        if (connected_.load()) {
            return true;
        }
        return reconnect_with_retry();
    }

    draco::EncoderBuffer compress_msg(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const geometry_msgs::msg::TransformStamped& transform) {
        // 1. TF 메시지를 고속 연산용 행렬로 변환
        tf2::Transform tf_mat;
        tf2::fromMsg(transform.transform, tf_mat);

        bool need_transform = (msg->header.frame_id != transform.header.frame_id);

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        bool has_rgb = false;
        for (const auto& field : msg->fields) {
            if (field.name == "rgb" || field.name == "rgba")
                has_rgb = true;
        }

        std::vector<float> point_data;
        std::vector<uint8_t> color_data;

        size_t estimated_points = msg->width * msg->height;
        point_data.reserve(estimated_points * 3);
        color_data.reserve(estimated_points * 3);

        double voxel_size = this->get_parameter("voxel_size").as_double();
        bool use_sampling = (voxel_size > 0.001);

        double skip_length = this->get_parameter("skip_length").as_double();
        double skip_length_sq = skip_length * skip_length;  // 제곱 거리로 비교 (성능 최적화)

        std::unordered_set<VoxelIndex, VoxelHash> visited_voxels;
        if (use_sampling)
            visited_voxels.reserve(estimated_points / 2);

        if (has_rgb) {
            sensor_msgs::PointCloud2ConstIterator<float> iter_rgb(*msg, "rgb");
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_rgb) {
                // [A] 원본 좌표 읽기 (Camera Frame)
                float raw_x = *iter_x;
                float raw_y = *iter_y;
                float raw_z = *iter_z;

                if (std::isnan(raw_x) || std::isnan(raw_y) || std::isnan(raw_z))
                    continue;

                // [B] 거리 필터링 (원본 좌표 기준)
                // 카메라로부터의 직선 거리가 설정값보다 멀면 제거
                if ((raw_x * raw_x + raw_y * raw_y + raw_z * raw_z) > skip_length_sq)
                    continue;

                // [C] 좌표 변환 (Camera -> Target Frame)
                float x, y, z;
                if (need_transform) {
                    tf2::Vector3 p_in(raw_x, raw_y, raw_z);
                    tf2::Vector3 p_out = tf_mat * p_in;
                    x = p_out.x();
                    y = p_out.y();
                    z = p_out.z();
                } else {
                    x = raw_x;
                    y = raw_y;
                    z = raw_z;
                }

                // [D] 복셀 샘플링 (변환된 World 좌표 기준)
                // 로봇 기준 공간에서 겹치는 점들을 하나로 합침
                if (use_sampling) {
                    VoxelIndex idx;
                    idx.x = static_cast<int>(std::floor(x / voxel_size));
                    idx.y = static_cast<int>(std::floor(y / voxel_size));
                    idx.z = static_cast<int>(std::floor(z / voxel_size));

                    if (visited_voxels.find(idx) != visited_voxels.end()) {
                        continue;  // 이미 등록된 복셀이면 스킵
                    }
                    visited_voxels.insert(idx);
                }

                const uint32_t rgb = *reinterpret_cast<const uint32_t*>(&(*iter_rgb));
                uint8_t r = (rgb >> 16) & 0xFF;
                uint8_t g = (rgb >> 8) & 0xFF;
                uint8_t b = (rgb) & 0xFF;

                // 검은색(0,0,0) 포인트 제거 옵션 (필요시 사용)
                if (r == 0 && g == 0 && b == 0)
                    continue;

                point_data.push_back(x);
                point_data.push_back(y);
                point_data.push_back(z);

                color_data.push_back(r);
                color_data.push_back(g);
                color_data.push_back(b);
            }
        }

        if (point_data.empty())
            return draco::EncoderBuffer();

        draco::PointCloudBuilder builder;
        builder.Start(point_data.size() / 3);

        const int pos_att_id = builder.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);
        const int color_att_id = builder.AddAttribute(draco::GeometryAttribute::COLOR, 3, draco::DT_UINT8);

        builder.SetAttributeValuesForAllPoints(pos_att_id, point_data.data(), 3 * sizeof(float));
        builder.SetAttributeValuesForAllPoints(color_att_id, color_data.data(), 3 * sizeof(uint8_t));

        std::unique_ptr<draco::PointCloud> draco_cloud = builder.Finalize(false);

        draco::Encoder encoder;
        encoder.SetSpeedOptions(10, 10);                                           // 0(느림/고압축) ~ 10(빠름/저압축)
        encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, 12);  // 8-16비트 권장
        encoder.SetAttributeQuantization(draco::GeometryAttribute::COLOR, 3);

        draco::EncoderBuffer buffer;
        draco::Status status = encoder.EncodePointCloudToBuffer(*draco_cloud, &buffer);

        if (!status.ok()) {
            RCLCPP_ERROR(this->get_logger(), "Draco Encoding Error: %s", status.error_msg());
            return draco::EncoderBuffer();
        }

        return buffer;
    }

    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 최근 10초간 ping 미수신 시 인코딩/전송 스킵
        if (now_ms() - last_ping_ms_.load() > 10000) {
            return;
        }

        auto start_time = this->now();
        std::string target_frame = this->get_parameter("target_frame").as_string();

        // TF 조회
        geometry_msgs::msg::TransformStamped t_stamped;
        try {
            if (msg->header.frame_id != target_frame) {
                // 가장 최신 TF 가져오기
                t_stamped = tf_buffer_->lookupTransform(target_frame, msg->header.frame_id, tf2::TimePointZero);
            } else {
                t_stamped.header.frame_id = target_frame;
                t_stamped.child_frame_id = target_frame;
                t_stamped.transform.rotation.w = 1.0;
            }
        } catch (tf2::TransformException& ex) {
            // TF가 아직 준비되지 않았으면 경고 출력 후 이번 프레임 스킵
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF Lookup Failed: %s", ex.what());
            return;
        }
        auto tf_time = this->now() - start_time;

        draco::EncoderBuffer buffer;
        try {
            // 압축 실행 (내부에서 필터링 및 변환 수행)
            buffer = compress_msg(msg, t_stamped);
            if (buffer.size() == 0)
                return;
        } catch (std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Compression Exception: %s", e.what());
            return;
        }
        auto compress_time = this->now() - start_time - tf_time;

        frame_count++;
        rclcpp::Duration elapsed = this->now() - fps_time;
        if (elapsed.seconds() >= 1.0) {
            fps = frame_count;
            fps_time = this->now();
            frame_count = 0;
            json delay_info = {{"name", "point_cloud"}, {"processingTimes_ms", {{"tf", tf_time.seconds() * 1000.0}, {"compress", compress_time.seconds() * 1000.0}}}, {"fps", fps}};
            std_msgs::msg::String delay_msg;
            delay_msg.data = delay_info.dump();
            network_publisher_->publish(delay_msg);
        }

        try {
            // 웹소켓 전송
            if (!ensure_connection()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "WebSocket not connected. Skipping frame.");
                return;
            }

            std::shared_ptr<websocket::stream<tcp::socket>> ws_copy;
            {
                std::lock_guard<std::mutex> lock(ws_mutex_);
                ws_copy = ws_;
            }
            if (!ws_copy) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "WebSocket handle missing. Skipping frame.");
                connected_.store(false);
                return;
            }

            ws_copy->write(net::buffer(buffer.data(), buffer.size()));

            // [디버그용 로그] 전송 성공 시 1초에 한 번만 출력 (필요시 주석 해제)
            // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Sent %zu bytes", buffer.size());
        } catch (std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "WebSocket Write Error: %s", e.what());
            connected_.store(false);
            reconnect_with_retry(-1, 500);
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DracoSenderNode>());
    rclcpp::shutdown();
    return 0;
}