#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <cartographer_ros_msgs/msg/status_code.hpp>
#include <cartographer_ros_msgs/msg/trajectory_states.hpp>
#include <cartographer_ros_msgs/srv/start_trajectory_with_pose.hpp>
#include <cartographer_ros_msgs/srv/finish_trajectory.hpp>
#include <cartographer_ros_msgs/srv/get_trajectory_states.hpp>

using StartTrajectoryWithPose = cartographer_ros_msgs::srv::StartTrajectoryWithPose;
using FinishTrajectory = cartographer_ros_msgs::srv::FinishTrajectory;
using GetTrajectoryStates = cartographer_ros_msgs::srv::GetTrajectoryStates;
using namespace std::chrono_literals;

class TrajectoryManager : public rclcpp::Node {
public:
  // TrajectoryManager：构造函数
  TrajectoryManager() : Node("trajectory_manager") {
    // 声明参数并设置默认值
    this->declare_parameter<std::string>("configuration_directory", 
      "/home/zqyhia/cartographer_ws/src/cartographer_ros/cartographer_ros/configuration_files");
    this->declare_parameter<std::string>("configuration_basename", "fishbot_2d_location.lua");
    config_dir_ = this->get_parameter("configuration_directory").as_string();
    config_basename_ = this->get_parameter("configuration_basename").as_string();

    // 初始化服务客户端
    start_client_ = this->create_client<StartTrajectoryWithPose>(
      "/start_trajectory_with_pose");
    finish_client_ = this->create_client<FinishTrajectory>(
      "/finish_trajectory");
    states_client_ = this->create_client<GetTrajectoryStates>(
      "/get_trajectory_states");

    // 订阅 /initialpose 话题
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        handle_new_pose(msg);
      });

    RCLCPP_INFO(get_logger(), "Trajectory Manager 已启动，请进行初始位置估计");
  }

private:
  // handle_new_pose：订阅并处理rviz的位置信息
  void handle_new_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose) { // 这里的pose是订阅的 /initialpose 话题
    // 步骤0：打印相对原点的定位信息（确保成功订阅rviz的位姿话题）
    const auto& position = pose->pose.pose.position;
    const auto& orientation = pose->pose.pose.orientation;
    RCLCPP_INFO(this->get_logger(), "设置位姿\n位置: x=%.2f, y=%.2f, z=%.2f\n朝向: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                position.x, position.y, position.z, 
                orientation.x, orientation.y, orientation.z, orientation.w);

    // 步骤1：获取当前所有轨迹状态
    while (!this->states_client_->wait_for_service(1s)) // 非运算之后，1s中服务没有上线为True，上线则为False
    {
      if (!rclcpp::ok()) // 非运算之后，如果rclcpp不在线为True，退出程序
      {
        RCLCPP_ERROR(this->get_logger(), "等待轨迹状态服务上线过程中，rclcpp离线，退出客户端");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "等待服务上线中...");
    }
    this->states_client_->async_send_request(std::make_shared<GetTrajectoryStates::Request>(),
                                              [this, pose](rclcpp::Client<GetTrajectoryStates>::SharedFuture states_future)->void{
      auto states_response = states_future.get(); // 调用get()方法以阻塞方式获取服务的响应
      if (states_response->status.code != cartographer_ros_msgs::msg::StatusCode::OK) {
        RCLCPP_ERROR(get_logger(), "获取轨迹状态失败: %s", states_response->status.message.c_str());
        return;
      }

      // 步骤2：结束所有ACTIVE状态的轨迹
      for (size_t i = 0; i < states_response->trajectory_states.trajectory_id.size(); ++i) {
        if (states_response->trajectory_states.trajectory_state[i] == 
            cartographer_ros_msgs::msg::TrajectoryStates::ACTIVE) {
          auto finish_request = std::make_shared<FinishTrajectory::Request>();
          finish_request->trajectory_id = states_response->trajectory_states.trajectory_id[i];
          while (!this->finish_client_->wait_for_service(1s)) // 非运算之后，1s中服务没有上线为True，上线则为False
          {
            if (!rclcpp::ok()) // 非运算之后，如果rclcpp不在线为True，退出程序
            {
              RCLCPP_ERROR(this->get_logger(), "等待结束轨迹服务上线过程中，rclcpp离线，退出客户端");
              return;
            }
            RCLCPP_INFO(this->get_logger(), "等待服务上线中...");
          }
          this->finish_client_->async_send_request(finish_request,
                [this, id = finish_request->trajectory_id, pose](rclcpp::Client<FinishTrajectory>::SharedFuture finish_future) {
            auto finish_response = finish_future.get();
            if (finish_response->status.code == cartographer_ros_msgs::msg::StatusCode::OK) {
              RCLCPP_INFO(get_logger(), "已成功结束轨迹 %d", id);
            } else {
              RCLCPP_WARN(get_logger(), "结束轨迹 %d 失败: %s", id, finish_response->status.message.c_str());
            }
            // 步骤3：准备新轨迹请求
            auto request = std::make_shared<StartTrajectoryWithPose::Request>();
            request->initial_pose = *pose;
            request->configuration_directory = config_dir_; // 这里暂时先用绝对路径
            request->configuration_basename = config_basename_; // 这里暂时先直接给出lua文件名
            request->relative_to_trajectory_id = 0; // 参考轨迹ID，应该为0，对应pbstreaem文件的轨迹ID

            // 步骤4：启动新轨迹
            start_client_->async_send_request(request,
              [this](rclcpp::Client<StartTrajectoryWithPose>::SharedFuture future) {
                auto response = future.get();
                if (response->status.code == cartographer_ros_msgs::msg::StatusCode::OK) {
                  RCLCPP_INFO(get_logger(), "新轨迹启动成功! ID: %d", response->trajectory_id);
                } else {
                  RCLCPP_ERROR(get_logger(), "轨迹启动失败: %s", response->status.message.c_str());
                }
              });
          });
        }
      }
    });
  }

  std::string config_dir_;
  std::string config_basename_;
  rclcpp::Client<StartTrajectoryWithPose>::SharedPtr start_client_;
  rclcpp::Client<FinishTrajectory>::SharedPtr finish_client_;
  rclcpp::Client<GetTrajectoryStates>::SharedPtr states_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}