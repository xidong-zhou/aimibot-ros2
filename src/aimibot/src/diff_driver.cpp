#include "../include/aimibot/diff_driver.hpp"

namespace Aimi {

/*****************************************************************************
** Implementation
*****************************************************************************/
DiffDriver::DiffDriver():
  last_velocity_left(0.0),
  last_velocity_right(0.0),
  last_tick_left(0),
  last_tick_right(0),
  last_rad_left(0.0),
  last_rad_right(0.0),
//  v(0.0), w(0.0), // command velocities, in [m/s] and [rad/s]
  radius(0.0), speed(0.0), // command velocities, in [mm] and [mm/s]
  point_velocity(2,0.0), // command velocities, in [m/s] and [rad/s]
  bias(0.336), // wheelbase, wheel_to_wheel, in [m]
  wheel_radius(0.063), // radius of main wheel, in [m]
  tick_to_rad(0.0031416),
  diff_drive_kinematics(bias, wheel_radius)
{}

/**
 * @brief Updates the odometry from firmware stamps and encoders.
 *
 * Really horrible - could do with an overhaul.
 *
 * @param time_stamp
 * @param left_encoder
 * @param right_encoder
 * @param pose_update
 * @param pose_update_rates
 */
void DiffDriver::update(const uint16_t &time_stamp,
                       const uint16_t &left_encoder,
                       const uint16_t &right_encoder,
                       ecl::LegacyPose2D<double> &pose_update,
                       ecl::linear_algebra::Vector3d &pose_update_rates) {
//   state_mutex.lock();
  static bool init_l = false;
  static bool init_r = false;
  double left_diff_ticks = 0.0f;
  double right_diff_ticks = 0.0f;
  unsigned short curr_tick_left = 0;
  unsigned short curr_tick_right = 0;
  unsigned short curr_timestamp = 0;
  curr_timestamp = time_stamp;
  curr_tick_left = left_encoder;
  if (!init_l)
  {
    last_tick_left = curr_tick_left;
    init_l = true;
  }
  //left_diff_ticks = (double)(short)((last_tick_left - curr_tick_left) & 0xffff);
  left_diff_ticks = (double)(short)((curr_tick_left - last_tick_left) & 0xffff);
  last_tick_left = curr_tick_left;
  last_rad_left += tick_to_rad * left_diff_ticks;

  curr_tick_right = right_encoder;
  if (!init_r)
  {
    last_tick_right = curr_tick_right;
    init_r = true;
  }
  //right_diff_ticks = (double)(short)((last_tick_right- curr_tick_right ) & 0xffff);
  right_diff_ticks = (double)(short)((curr_tick_right- last_tick_right ) & 0xffff);
  last_tick_right = curr_tick_right;
  last_rad_right += tick_to_rad * right_diff_ticks;

  // TODO this line and the last statements are really ugly; refactor, put in another place
  
  
  double ds = wheel_radius*((tick_to_rad * left_diff_ticks) + (tick_to_rad * right_diff_ticks))/2.0;
  double domega = wheel_radius*((tick_to_rad * right_diff_ticks) - (tick_to_rad * left_diff_ticks))/bias;
  // Local robot frame of reference has the x axis pointing forward
  // Since the pose update is using the robot frame of reference, the y update is zero
  pose_update.translation(ds, 0);
  pose_update.rotation(domega);



  if (curr_timestamp != last_timestamp)
  {
    last_diff_time = ((double)(short)((curr_timestamp - last_timestamp) & 0xffff)) / 1000.0f;
    last_timestamp = curr_timestamp;
    last_velocity_left = (tick_to_rad * left_diff_ticks) / last_diff_time;
    last_velocity_right = (tick_to_rad * right_diff_ticks) / last_diff_time;
  } else {
    // we need to set the last_velocity_xxx to zero?
  }

  pose_update_rates << pose_update.x()/last_diff_time,
                       pose_update.y()/last_diff_time,
                       pose_update.heading()/last_diff_time;
//   state_mutex.unlock();
}

void DiffDriver::reset() {
//   state_mutex.lock();
  last_rad_left = 0.0;
  last_rad_right = 0.0;
  last_velocity_left = 0.0;
  last_velocity_right = 0.0;
//   state_mutex.unlock();
}

void DiffDriver::getWheelJointStates(double &wheel_left_angle, double &wheel_left_angle_rate,
                                    double &wheel_right_angle, double &wheel_right_angle_rate) {
//   state_mutex.lock();
  wheel_left_angle = last_rad_left;
  wheel_right_angle = last_rad_right;
  wheel_left_angle_rate = last_velocity_left;
  wheel_right_angle_rate = last_velocity_right;
//   state_mutex.unlock();
}

void DiffDriver::setVelocityCommands(const double &vx, const double &wz) {
  // vx: in m/s
  // wz: in rad/s
  std::vector<double> cmd_vel;
  cmd_vel.push_back(vx);
  cmd_vel.push_back(wz);
  point_velocity = cmd_vel;
}

void DiffDriver::velocityCommands(const double &vx, const double &wz) {
  // vx: in m/s
  // wz: in rad/s
//   velocity_mutex.lock();
  const double epsilon = 0.0001;

  // Special Case #1 : Straight Run
  if( std::abs(wz) < epsilon ) {
    radius = 0.0f;
    speed  = 1000.0f * vx;
//     velocity_mutex.unlock();
    return;
  }

  radius = vx * 1000.0f / wz;
  // Special Case #2 : Pure Rotation or Radius is less than or equal to 1.0 mm
  if( std::abs(vx) < epsilon || std::abs(radius) <= 1.0f ) {
    speed  = 1000.0f * bias * wz / 2.0f;
    radius = 1.0f;
//     velocity_mutex.unlock();
    return;
  }

  // General Case :
  if( radius > 0.0f ) {
    speed  = (radius + 1000.0f * bias / 2.0f) * wz;
  } else {
    speed  = (radius - 1000.0f * bias / 2.0f) * wz;
  }
//   velocity_mutex.unlock();
  return;
}

void DiffDriver::velocityCommands(const short &cmd_speed, const short &cmd_radius) {
//   velocity_mutex.lock();
  speed = static_cast<double>(cmd_speed);   // In [mm/s]
  radius = static_cast<double>(cmd_radius); // In [mm]
//   velocity_mutex.unlock();
  return;
}

std::vector<short> DiffDriver::velocityCommands() {
//   velocity_mutex.lock();
  std::vector<short> cmd(2);
  cmd[0] = bound(speed);  // In [mm/s]
  cmd[1] = bound(radius); // In [mm]
//   velocity_mutex.unlock();
  return cmd;
}

std::vector<double> DiffDriver::pointVelocity() const {
  return point_velocity;
}

short DiffDriver::bound(const double &value) {
  if (value > static_cast<double>(SHRT_MAX)) return SHRT_MAX;
  if (value < static_cast<double>(SHRT_MIN)) return SHRT_MIN;
  return static_cast<short>(value);
}
}
