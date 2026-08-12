#include "../debug_config.h"
#include "ros_interface.h"

// Wi-Fi credentials live in an untracked header. Without it the firmware still
// builds and balances; only the ROS link is left out.
#if __has_include("ros_config.h")
#define ROS_ENABLED 1
#include "ros_config.h"
#else
#define ROS_ENABLED 0
#endif

#if ROS_ENABLED

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <WiFi.h>

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "control_state.h"
#include "imu_filter.h"

namespace {

constexpr uint32_t PUBLISH_PERIOD_MS = 20;   // 50Hz
constexpr uint32_t TASK_PERIOD_MS = 10;
constexpr uint32_t PING_PERIOD_MS = 2000;
constexpr size_t PID_GAIN_COUNT = 6;

// Full-scale wheel command. The control task adds this to the balance output, so
// it stays well below the 1023 saturation point to leave the angle PID authority.
constexpr float LINEAR_COMMAND_SCALE = 1.0f;
constexpr float ANGULAR_COMMAND_SCALE = 1.0f;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t timer;

rcl_publisher_t imu_pub;
rcl_publisher_t wheel_pub;
rcl_publisher_t control_pub;
rcl_subscription_t cmd_vel_sub;
rcl_subscription_t enable_sub;
rcl_subscription_t gains_sub;

sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Vector3 wheel_msg;
geometry_msgs__msg__Vector3 control_msg;
geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__Bool enable_msg;
std_msgs__msg__Float32MultiArray gains_msg;

float gains_buffer[PID_GAIN_COUNT];
char imu_frame_id[] = "imu_link";

volatile RosLinkState_t link_state = ROS_LINK_WIFI_WAIT;

enum AgentState {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED,
};
AgentState agent_state = WAITING_AGENT;

bool time_synced = false;
#if BALA_DEBUG_TELEMETRY
uint32_t published = 0;
#endif

void FillStamp(builtin_interfaces__msg__Time* stamp) {
  if (time_synced) {
    int64_t ns = rmw_uros_epoch_nanos();
    stamp->sec = (int32_t)(ns / 1000000000LL);
    stamp->nanosec = (uint32_t)(ns % 1000000000LL);
  } else {
    uint32_t ms = millis();
    stamp->sec = (int32_t)(ms / 1000);
    stamp->nanosec = (uint32_t)((ms % 1000) * 1000000UL);
  }
}

void PublishSample() {
  ImuState_t imu;
  getImuState(&imu);

  FillStamp(&imu_msg.header.stamp);
  imu_msg.orientation.w = imu.quat_w;
  imu_msg.orientation.x = imu.quat_x;
  imu_msg.orientation.y = imu.quat_y;
  imu_msg.orientation.z = imu.quat_z;
  // sensor_msgs/Imu is specified in rad/s and m/s^2; the estimator works in
  // deg/s and g.
  imu_msg.angular_velocity.x = imu.gyro_x * DEG_TO_RAD;
  imu_msg.angular_velocity.y = imu.gyro_y * DEG_TO_RAD;
  imu_msg.angular_velocity.z = imu.gyro_z * DEG_TO_RAD;
  imu_msg.linear_acceleration.x = imu.accel_x * 9.80665f;
  imu_msg.linear_acceleration.y = imu.accel_y * 9.80665f;
  imu_msg.linear_acceleration.z = imu.accel_z * 9.80665f;
  rcl_publish(&imu_pub, &imu_msg, NULL);

  ControlFeedback_t fb;
  ControlGetFeedback(&fb);
  // Vector3 carries three float64 fields with no dynamic arrays, which keeps the
  // publisher allocation-free. x/y are raw encoder counts, z is the filtered
  // wheel speed in counts per control cycle.
  wheel_msg.x = (double)fb.encoder_left;
  wheel_msg.y = (double)fb.encoder_right;
  wheel_msg.z = (double)fb.wheel_speed;
  rcl_publish(&wheel_pub, &wheel_msg, NULL);

  // The controller's own view of itself. Serial telemetry is unavailable once the
  // robot runs untethered, so the balance angle, the motor command and the enable
  // state have to be observable over the network.
  control_msg.x = (double)imu.angle;
  control_msg.y = (double)fb.pwm_output;
  control_msg.z = fb.enabled ? 1.0 : 0.0;
  rcl_publish(&control_pub, &control_msg, NULL);
}

void CmdVelCallback(const void* msgin) {
  const geometry_msgs__msg__Twist* m = (const geometry_msgs__msg__Twist*)msgin;
  VelocityCommand_t cmd;
  cmd.linear = constrain((float)m->linear.x, -1.0f, 1.0f) * LINEAR_COMMAND_SCALE;
  cmd.angular = constrain((float)m->angular.z, -1.0f, 1.0f) * ANGULAR_COMMAND_SCALE;
  ControlSetVelocity(&cmd);
}

void EnableCallback(const void* msgin) {
  const std_msgs__msg__Bool* m = (const std_msgs__msg__Bool*)msgin;
  ControlSetEnabled(m->data);
}

void GainsCallback(const void* msgin) {
  const std_msgs__msg__Float32MultiArray* m = (const std_msgs__msg__Float32MultiArray*)msgin;
  // Applying a partial gain set would leave the controller in a state nobody
  // asked for, so require all six.
  if (m->data.size != PID_GAIN_COUNT) {
#if BALA_DEBUG_TELEMETRY
    Serial.printf("[ros] pid_gains ignored: %u values, expected %u\n",
                  (unsigned)m->data.size, (unsigned)PID_GAIN_COUNT);
#endif
    return;
  }
  PidGains_t g;
  g.kp = m->data.data[0];
  g.ki = m->data.data[1];
  g.kd = m->data.data[2];
  g.s_kp = m->data.data[3];
  g.s_ki = m->data.data[4];
  g.s_kd = m->data.data[5];
  ControlSetGains(&g);
#if BALA_DEBUG_TELEMETRY
  Serial.printf("[ros] pid_gains applied: kp=%.3f ki=%.3f kd=%.3f s_kp=%.3f s_ki=%.4f s_kd=%.3f\n",
                g.kp, g.ki, g.kd, g.s_kp, g.s_ki, g.s_kd);
#endif
}

void InitMessages() {
  sensor_msgs__msg__Imu__init(&imu_msg);
  imu_msg.header.frame_id.data = imu_frame_id;
  imu_msg.header.frame_id.size = strlen(imu_frame_id);
  imu_msg.header.frame_id.capacity = sizeof(imu_frame_id);

  geometry_msgs__msg__Vector3__init(&wheel_msg);
  geometry_msgs__msg__Vector3__init(&control_msg);
  geometry_msgs__msg__Twist__init(&cmd_vel_msg);
  std_msgs__msg__Bool__init(&enable_msg);

  // A sequence in an incoming message needs storage supplied up front; rclc
  // will not allocate it.
  std_msgs__msg__Float32MultiArray__init(&gains_msg);
  gains_msg.data.data = gains_buffer;
  gains_msg.data.size = 0;
  gains_msg.data.capacity = PID_GAIN_COUNT;
}

bool CreateEntities() {
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) { return false; }
  if (rclc_node_init_default(&node, ROS_NODE_NAME, ROS_NAMESPACE, &support) != RCL_RET_OK) {
    return false;
  }

  if (rclc_publisher_init_best_effort(
          &imu_pub, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu") != RCL_RET_OK) {
    return false;
  }
  if (rclc_publisher_init_best_effort(
          &wheel_pub, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "wheel") != RCL_RET_OK) {
    return false;
  }
  if (rclc_publisher_init_best_effort(
          &control_pub, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "control") != RCL_RET_OK) {
    return false;
  }

  // Best effort for commands: a late velocity or enable message is worse than a
  // dropped one for a robot that has to stay upright.
  if (rclc_subscription_init_best_effort(
          &cmd_vel_sub, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel") != RCL_RET_OK) {
    return false;
  }
  if (rclc_subscription_init_default(
          &enable_sub, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "enable") != RCL_RET_OK) {
    return false;
  }
  if (rclc_subscription_init_default(
          &gains_sub, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "pid_gains") != RCL_RET_OK) {
    return false;
  }

  executor = rclc_executor_get_zero_initialized_executor();
  if (rclc_executor_init(&executor, &support.context, 3, &allocator) != RCL_RET_OK) {
    return false;
  }
  rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,
                                 &CmdVelCallback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &enable_sub, &enable_msg,
                                 &EnableCallback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &gains_sub, &gains_msg,
                                 &GainsCallback, ON_NEW_DATA);

  time_synced = (rmw_uros_sync_session(1000) == RMW_RET_OK);

  return true;
}

void DestroyEntities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&imu_pub, &node);
  rcl_publisher_fini(&wheel_pub, &node);
  rcl_publisher_fini(&control_pub, &node);
  rcl_subscription_fini(&cmd_vel_sub, &node);
  rcl_subscription_fini(&enable_sub, &node);
  rcl_subscription_fini(&gains_sub, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  time_synced = false;
}

void RosTask(void* /*arg*/) {
  InitMessages();

#if BALA_DEBUG_TELEMETRY
  // set_microros_wifi_transports blocks until the association succeeds, so a
  // wrong SSID or passphrase looks identical to a missing agent from outside.
  Serial.printf("[ros] connecting to wifi \"%s\", agent %s:%d\n",
                ROS_WIFI_SSID, ROS_AGENT_IP, (int)ROS_AGENT_PORT);
#endif

  set_microros_wifi_transports((char*)ROS_WIFI_SSID, (char*)ROS_WIFI_PASSWORD,
                               (char*)ROS_AGENT_IP, ROS_AGENT_PORT);

#if BALA_DEBUG_TELEMETRY
  Serial.printf("[ros] wifi up, local ip %s, rssi %d dBm\n",
                WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
#endif

  uint32_t last_ticks = xTaskGetTickCount();
  uint32_t last_ping_ms = 0;
  uint32_t last_spin_us = 0;
  uint32_t last_publish_ms = 0;
  AgentState reported = (AgentState)-1;

  for (;;) {
    switch (agent_state) {
      case WAITING_AGENT:
        link_state = ROS_LINK_AGENT_WAIT;
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
          agent_state = AGENT_AVAILABLE;
        }
        break;

      case AGENT_AVAILABLE:
        agent_state = CreateEntities() ? AGENT_CONNECTED : WAITING_AGENT;
        if (agent_state == WAITING_AGENT) {
          DestroyEntities();
        }
        break;

      case AGENT_CONNECTED: {
        link_state = ROS_LINK_CONNECTED;
        // The ping is a synchronous round trip. Issuing one per loop iteration
        // starved the executor badly enough that a 50Hz timer published at 1Hz,
        // so only check liveness periodically.
        uint32_t now = millis();
        if (now - last_ping_ms >= PING_PERIOD_MS) {
          last_ping_ms = now;
          if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
            agent_state = AGENT_DISCONNECTED;
            break;
          }
        }
        if (now - last_publish_ms >= PUBLISH_PERIOD_MS) {
          last_publish_ms = now;
          PublishSample();
#if BALA_DEBUG_TELEMETRY
          published++;
#endif
        }

#if BALA_DEBUG_TELEMETRY
        uint32_t spin_t0 = micros();
#endif
        // Timeout 0: poll for incoming commands without blocking. Anything
        // larger stalls this loop for about a second.
        rclc_executor_spin_some(&executor, 0);
#if BALA_DEBUG_TELEMETRY
        last_spin_us = micros() - spin_t0;
#endif
        break;
      }

      case AGENT_DISCONNECTED:
        // Losing the link must not leave the robot driving on the last command.
        ControlSetEnabled(false);
        DestroyEntities();
        agent_state = WAITING_AGENT;
        break;
    }

#if BALA_DEBUG_TELEMETRY
    {
      // Separate "the loop is starved" from "the executor is not dispatching".
      static uint32_t iters = 0, spin_sum = 0, spin_max = 0, win = 0;
      static uint32_t published_prev = 0;
      iters++;
      spin_sum += last_spin_us;
      if (last_spin_us > spin_max) { spin_max = last_spin_us; }
      uint32_t now2 = millis();
      if (now2 - win >= 1000) {
        Serial.printf("[ros] loop=%lu/s pub=%lu/s spin_avg=%luus spin_max=%luus\n",
                      (unsigned long)iters,
                      (unsigned long)(published - published_prev),
                      (unsigned long)(iters ? spin_sum / iters : 0),
                      (unsigned long)spin_max);
        published_prev = published;
        iters = 0; spin_sum = 0; spin_max = 0; win = now2;
      }
    }

    if (agent_state != reported) {
      static const char* names[] = {"WAITING_AGENT", "AGENT_AVAILABLE",
                                    "AGENT_CONNECTED", "AGENT_DISCONNECTED"};
      Serial.printf("[ros] state -> %s (time_synced=%d)\n", names[agent_state],
                    (int)time_synced);
      reported = agent_state;
    }
#endif

    vTaskDelayUntil(&last_ticks, pdMS_TO_TICKS(TASK_PERIOD_MS));
  }
}

}  // namespace

void RosTaskStart() {
  // Core 0 with the display. Priority above the display so ROS traffic is not
  // held up by a frame, but still below the control tasks on core 1.
  xTaskCreatePinnedToCore(RosTask, "ros_task", 16 * 1024, NULL, 2, NULL, 0);
}

RosLinkState_t RosGetLinkState() { return link_state; }

#else  // !ROS_ENABLED

void RosTaskStart() {}
RosLinkState_t RosGetLinkState() { return ROS_LINK_DISABLED; }

#endif
