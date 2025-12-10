#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <memory>
#include <string>

class CameraTFBroadcaster : public rclcpp::Node {
public:
    CameraTFBroadcaster() : Node("camera_tf_broadcaster") {
        // [파라미터 설정]
        // 부모 프레임 (예: 로봇의 중심)
        this->declare_parameter("parent_frame", "base_link");
        // 자식 프레임 (예: 포인트 클라우드의 frame_id와 일치해야 함)
        // 실제 카메라 토픽의 헤더에 있는 frame_id를 확인해서 적어야 합니다.
        // 예: camera_link, camera_depth_optical_frame 등
        this->declare_parameter("child_frame", "camera_depth_optical_frame");

        // 위치 (미터 단위)
        this->declare_parameter("x", 0.2);  // 앞쪽으로 20cm
        this->declare_parameter("y", 0.0);
        this->declare_parameter("z", 0.5);  // 바닥에서 50cm 위

        // 회전 (라디안 단위, RPY)
        this->declare_parameter("roll", 0.0);
        this->declare_parameter("pitch", 0.0);
        this->declare_parameter("yaw", 0.0);

        // TF 브로드캐스터 초기화
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 100ms(10Hz)마다 TF 송출
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&CameraTFBroadcaster::broadcast_timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "TF Broadcaster Started");
    }

private:
    void broadcast_timer_callback() {
        geometry_msgs::msg::TransformStamped t;

        // 1. 타임스탬프 설정 (현재 시간)
        t.header.stamp = this->get_clock()->now();
        
        // 2. 프레임 ID 설정
        t.header.frame_id = this->get_parameter("parent_frame").as_string();
        t.child_frame_id = this->get_parameter("child_frame").as_string();

        // 3. 위치(Translation) 설정
        t.transform.translation.x = this->get_parameter("x").as_double();
        t.transform.translation.y = this->get_parameter("y").as_double();
        t.transform.translation.z = this->get_parameter("z").as_double();

        // 4. 회전(Rotation) 설정 - 오일러(RPY) -> 쿼터니언 변환
        double roll = this->get_parameter("roll").as_double();
        double pitch = this->get_parameter("pitch").as_double();
        double yaw = this->get_parameter("yaw").as_double();

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw); // Roll, Pitch, Yaw 순서

        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // 5. TF 송출
        tf_broadcaster_->sendTransform(t);
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraTFBroadcaster>());
    rclcpp::shutdown();
    return 0;
}