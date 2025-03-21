#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"

using namespace std::chrono_literals;

// copied from quadruped.h

constexpr int FR_0 = 0;      // joint index
constexpr int FR_1 = 1;      
constexpr int FR_2 = 2;

constexpr int FL_0 = 3;
constexpr int FL_1 = 4;
constexpr int FL_2 = 5;

constexpr int RR_0 = 6;
constexpr int RR_1 = 7;
constexpr int RR_2 = 8;

constexpr int RL_0 = 9;
constexpr int RL_1 = 10;
constexpr int RL_2 = 11;


// TODO: should put these functions into a library, like I did for turtlelib
long factorial(long n){
  if ((n == 1) || (n == 0)){
    return 1;
  } else {
    return n * factorial(n - 1);
  }
}

long choose(long n, long k){
  return factorial(n)/(factorial(k)*factorial(n-k));
}

double p_i(long i, long order, double t, double point){
  return choose(order,i) * pow((1-t),(order-i)) * pow(t,i)*point;
  // PYTHON VERSION: return choose(order,i)*((1-t)**(order-i))*(t**i)*point;
}

std::vector<double> bezier(std::vector<double> points, double step){
  const auto order = points.size();
  std::vector<double> curve;
  for (double t = 0.0; t<=1; t+=step){
    double current = 0;
    for (int i=0; i<order; i++){
      current += p_i(i,order-1,t,points.at(i));
    }
    curve.push_back(current);
  }
  return curve;
}



class CustomGait : public rclcpp::Node
{
  public:
    CustomGait()
    : Node("custom_gait")
    {
      rate = (std::chrono::milliseconds) ((int) rate_ms);
      cmd_pub_ = create_publisher<ros2_unitree_legged_msgs::msg::LowCmd>("low_cmd", 10);
      state_sub_ = create_subscription<ros2_unitree_legged_msgs::msg::LowState>("low_state", 10,
        std::bind(&CustomGait::state_cb, this, std::placeholders::_1));
      timer_ = create_wall_timer(
        rate,
        std::bind(&CustomGait::timer_callback, this));
      // initialize low_cmd fields
      low_cmd_ros.head[0] = 0xFE;
      low_cmd_ros.head[1] = 0xEF;
      low_cmd_ros.level_flag = 0xFF; // LOWLEVEL;
      for (int i = 0; i < 12; i++)
      {
        low_cmd_ros.motor_cmd[i].mode = 0x0A;  // motor switch to servo (PMSM) mode
        low_cmd_ros.motor_cmd[i].q = (2.146E+9f); // PosStopF; // 禁止位置环
        low_cmd_ros.motor_cmd[i].kp = 0;
        low_cmd_ros.motor_cmd[i].dq = (16000.0f); // VelStopF; // 禁止速度环
        low_cmd_ros.motor_cmd[i].kd = 0;
        low_cmd_ros.motor_cmd[i].tau = 0;
      }

      // In the future, these will be a more complicated function!!
      const auto lspan = 0.10; // half of "stroke length", ie how long it's on the floor
      const auto dl = 0.025; // extra bit to extend by after leaving floor
      const auto ddl = 0.025; // another extra bit to extend by LOL 
      const auto stand_floor = -1.5*l; // y distance when the foot is on the floor.
      const auto swing_height = 0.05; // ???
      const auto dswing_height = 0.025; // ??
      ctrl_x = {-1.0*lspan,
                -1.0*lspan - dl,
                -1.0*lspan - dl - ddl,
                -1.0*lspan - dl - ddl,
                -1.0*lspan - dl - ddl,
                0.0,
                0.0,
                0.0,
                lspan + dl + ddl,
                lspan + dl + ddl,
                lspan + dl,
                lspan};
      ctrl_y = {stand_floor,
                stand_floor,
                stand_floor + swing_height,
                stand_floor + swing_height,
                stand_floor + swing_height,
                stand_floor + swing_height,
                stand_floor + swing_height,
                stand_floor + swing_height + dswing_height,
                stand_floor + swing_height + dswing_height,
                stand_floor + swing_height + dswing_height,
                stand_floor,
                stand_floor};
      std::vector<double> bez_x = bezier(ctrl_x, 1.0/period);
      std::vector<double> bez_y = bezier(ctrl_y, 1.0/period);
      RCLCPP_INFO_STREAM(get_logger(), "Size of bez_x: "<<bez_x.size());

      // std::vector<double> desired_x = linspace(-l, l, period);
      // std::vector<double> desired_y = linspace(-l, -l, period);

      // make_gait();
      make_desired_gait(bez_x, bez_y);
      RCLCPP_INFO_STREAM(get_logger(), "Waiting...");
    }
  private:
    void state_cb(const ros2_unitree_legged_msgs::msg::LowState & msg)
    {
      if (feets.size()==0){
        // first iteration
        // This is a really dumb way of doing it but only way I could get to build ;-;
        feets.push_back(msg.foot_force[0]); // FR
        feets.push_back(msg.foot_force[1]); // FL
        feets.push_back(msg.foot_force[2]); // RR
        feets.push_back(msg.foot_force[3]); // RL
      }else {
        feets[0] = msg.foot_force[0];
        feets[1] = msg.foot_force[1];
        feets[2] = msg.foot_force[2];
        feets[3] = msg.foot_force[3];
      }
      // RCLCPP_INFO_STREAM(get_logger(), "Foot Force:  FR:" << feets[0] << "  FL:" << feets[1] << 
                                      //  "  RR:" << feets[2] << "  RL:" << feets[3]);
    }
    void timer_callback()
    {
      // RCLCPP_INFO_STREAM(get_logger(), "Timer tick!");
      if (!initiated_flag){
        count++;
        if (count>1000){
          RCLCPP_INFO_STREAM(get_logger(), "Start moving!");
          initiated_flag = true;
        }
      }else{
          motiontime += 1;
          // RCLCPP_INFO_STREAM(get_logger(), "Motiontime " << motiontime);
          if (motiontime >= period){
            RCLCPP_INFO_STREAM(get_logger(), "New Step!");
            motiontime = 0;
          }
          low_cmd_ros.motor_cmd[FR_2].q = fr_calf[motiontime];
          low_cmd_ros.motor_cmd[FR_2].dq = 0.0;
          low_cmd_ros.motor_cmd[FR_2].kp = 5.0;
          low_cmd_ros.motor_cmd[FR_2].kd = 1.0;
          low_cmd_ros.motor_cmd[FR_0].q = fr_hip[motiontime];
          low_cmd_ros.motor_cmd[FR_0].dq = 0.0;
          low_cmd_ros.motor_cmd[FR_0].kp = 5.0;
          low_cmd_ros.motor_cmd[FR_0].kd = 1.0;
          low_cmd_ros.motor_cmd[FR_1].q = fr_thigh[motiontime];
          low_cmd_ros.motor_cmd[FR_1].dq = 0.0;
          low_cmd_ros.motor_cmd[FR_1].kp = 5.0;
          low_cmd_ros.motor_cmd[FR_1].kd = 1.0;

          low_cmd_ros.motor_cmd[FL_2].q = fl_calf[motiontime];
          low_cmd_ros.motor_cmd[FL_2].dq = 0.0;
          low_cmd_ros.motor_cmd[FL_2].kp = 5.0;
          low_cmd_ros.motor_cmd[FL_2].kd = 1.0;
          low_cmd_ros.motor_cmd[FL_0].q = fl_hip[motiontime];
          low_cmd_ros.motor_cmd[FL_0].dq = 0.0;
          low_cmd_ros.motor_cmd[FL_0].kp = 5.0;
          low_cmd_ros.motor_cmd[FL_0].kd = 1.0;
          low_cmd_ros.motor_cmd[FL_1].q = fl_thigh[motiontime];
          low_cmd_ros.motor_cmd[FL_1].dq = 0.0;
          low_cmd_ros.motor_cmd[FL_1].kp = 5.0;
          low_cmd_ros.motor_cmd[FL_1].kd = 1.0;

          low_cmd_ros.motor_cmd[RR_2].q = rr_calf[motiontime];
          low_cmd_ros.motor_cmd[RR_2].dq = 0.0;
          low_cmd_ros.motor_cmd[RR_2].kp = 5.0;
          low_cmd_ros.motor_cmd[RR_2].kd = 1.0;
          low_cmd_ros.motor_cmd[RR_0].q = rr_hip[motiontime];
          low_cmd_ros.motor_cmd[RR_0].dq = 0.0;
          low_cmd_ros.motor_cmd[RR_0].kp = 5.0;
          low_cmd_ros.motor_cmd[RR_0].kd = 1.0;
          low_cmd_ros.motor_cmd[RR_1].q = rr_thigh[motiontime];
          low_cmd_ros.motor_cmd[RR_1].dq = 0.0;
          low_cmd_ros.motor_cmd[RR_1].kp = 5.0;
          low_cmd_ros.motor_cmd[RR_1].kd = 1.0;

          low_cmd_ros.motor_cmd[RL_2].q = rl_calf[motiontime];
          low_cmd_ros.motor_cmd[RL_2].dq = 0.0;
          low_cmd_ros.motor_cmd[RL_2].kp = 5.0;
          low_cmd_ros.motor_cmd[RL_2].kd = 1.0;
          low_cmd_ros.motor_cmd[RL_0].q = rl_hip[motiontime];
          low_cmd_ros.motor_cmd[RL_0].dq = 0.0;
          low_cmd_ros.motor_cmd[RL_0].kp = 5.0;
          low_cmd_ros.motor_cmd[RL_0].kd = 1.0;
          low_cmd_ros.motor_cmd[RL_1].q = rl_thigh[motiontime];
          low_cmd_ros.motor_cmd[RL_1].dq = 0.0;
          low_cmd_ros.motor_cmd[RL_1].kp = 5.0;
          low_cmd_ros.motor_cmd[RL_1].kd = 1.0;
      }
      cmd_pub_->publish(low_cmd_ros);
    }

    double calf_func1(long t){
      return (-M_PI / 2 + 0.5 * sin(2 * M_PI / 5.0 * t * 1e-3));
    }

    double calf_func2(long t){
      return (-M_PI / 2 - 0.5 * sin(2 * M_PI / 5.0 * t * 1e-3));
    }

    double thigh_func1(long t){
      return (0.5 * sin(2 * M_PI / 5.0 * t * 1e-3));
    }

    double thigh_func2(long t){
      return (- 0.5 * sin(2 * M_PI / 5.0 * t * 1e-3));
    }

    double hip_func1(long t){
      return 0.0;
    }

    double hip_func2(long t){
      return 0.0;
    }

    // Designed to act like the numpy linspace function.
    // Used for forming/ testing simple linear trajectories
    std::vector<double> linspace(double lo, double hi, double points){
      std::vector<double> res;
      double step = (hi-lo)/period;
      double curr = lo;
      for(int i=0;i<points;i++){
        res.push_back(curr);
        curr += step;
      }
      return res;
    }

    double get_theta_calf(double theta_thigh, double x){
      return asin(-x/l - sin(theta_thigh)) - theta_thigh;
    }

    // Given an x and y (WRT hip joint), return possible joint angles
    // First two in the vector are the "lefty" solution, and last 2 are "righty"
    std::vector<double> ik(double x, double y){
      double alpha = acos(sqrt(x*x + y*y)/(2*l));
      double gamma = atan(x/y);
      double theta_thigh_left = gamma + alpha;
      double theta_calf_left = get_theta_calf(theta_thigh_left, x);
      double theta_thigh_right = gamma - alpha;
      double theta_calf_right = get_theta_calf(theta_thigh_right, x);
      std::vector<double> res;
      res.push_back(theta_thigh_left);
      res.push_back(theta_calf_left);
      res.push_back(theta_thigh_right);
      res.push_back(theta_calf_right);
      return res;
    }

    void make_gait(){
      for(int i=0;i<period;i++){
        fr_calf.push_back(calf_func1(i));
        fl_calf.push_back(calf_func2(i));
        rr_calf.push_back(calf_func2(i));
        rl_calf.push_back(calf_func1(i));

        fr_thigh.push_back(thigh_func1(i));
        fl_thigh.push_back(thigh_func2(i));
        rr_thigh.push_back(thigh_func2(i));
        rl_thigh.push_back(thigh_func1(i));

        fr_hip.push_back(hip_func1(i));
        fl_hip.push_back(hip_func2(i));
        rr_hip.push_back(hip_func2(i));
        rl_hip.push_back(hip_func1(i));
      }
    }

    void make_desired_gait(std::vector<double> desired_x, std::vector<double> desired_y){
      if (desired_x.size() != desired_y.size()){
        RCLCPP_INFO_STREAM(get_logger(), "Desired X and Desired Y different lengths???");
      } else {
        RCLCPP_INFO_STREAM(get_logger(), "Generating Trajectory");
        std::vector<double> ik_result;
        for(int i=0;i<period;i++){
          ik_result = ik(desired_x[i], desired_y[i]);
          // Here we just arbitrarily choose left result (it maintained joint limits in my example)
          // The left thigh result is 0th element and calf result is 1st
          // Keep rest of joints stationary for now
          RCLCPP_INFO_STREAM(get_logger(), "(x,y)=("<<desired_x[i]<<","<<desired_y[i]<<
                                            ") ->\t(theta_t, theta_c)=("<<
                                            ik_result[0]<<","<<ik_result[1]<<")");
          fr_calf.push_back(ik_result[1]);
          fl_calf.push_back(calf_base);
          rr_calf.push_back(calf_base);
          rl_calf.push_back(calf_base);

          fr_thigh.push_back(ik_result[0]);
          fl_thigh.push_back(thigh_base);
          rr_thigh.push_back(thigh_base);
          rl_thigh.push_back(thigh_base);

          fr_hip.push_back(hip_base);
          fl_hip.push_back(hip_base);
          rr_hip.push_back(hip_base);
          rl_hip.push_back(hip_base);
        }
      }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    ros2_unitree_legged_msgs::msg::LowCmd low_cmd_ros;
    long motiontime = 0;
    bool initiated_flag = false;
    int count = 0;
    rclcpp::Publisher<ros2_unitree_legged_msgs::msg::LowCmd>::SharedPtr cmd_pub_;
    rclcpp::Subscription<ros2_unitree_legged_msgs::msg::LowState>::SharedPtr state_sub_;
    std::vector<int> feets;
    // number of ms between timer ticks
    int rate_ms = 2;
    std::chrono::milliseconds rate;
    // Length in seconds of each leg swing
    int period_sec = 5;
    // 1 point/2 ms * 1000 ms / 1sec * 5 sec => period_sec * 1000 / rate_ms
    // number of points per swing. Dependent on rate_ms
    long period = period_sec*1000/rate_ms;
    std::vector<double> fr_calf, fl_calf, rr_calf, rl_calf, fr_thigh, fl_thigh, rr_thigh, rl_thigh,
                   fr_hip, fl_hip, rr_hip, rl_hip;
    // This is the length of the legs.
    double l = 0.213;
    // Define joint limits
    double calf_lo = -2.82;
    double calf_hi = -0.89;
    double thigh_lo = -0.69;
    double thigh_hi =  4.50;
    double hip_lo = -0.86;
    double hip_hi =  0.86;
    // Define nominal joint values
    double calf_base = -1.85;
    double thigh_base = 0.0;
    double hip_base = 0.0;
    // Control points for bezier curve
    std::vector<double> ctrl_x, ctrl_y;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomGait>());
  rclcpp::shutdown();
  return 0;
}