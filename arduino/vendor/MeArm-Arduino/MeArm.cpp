/* MeArm library based on the work by York Hack Space May 2014
 * Officially co-opted as the go to IK and control library July 2023
 * A simple control library for MeArm Robotics Classic MeArm Robot Arm
 * Usage:
 *   MeArm arm;
 *   Pca9685ServoOutput output;
 *   output.begin();
 *   arm.begin(output, 0, 1, 2, 3);
 *   arm.openClaw();
 *   arm.moveTo(-80, 100, 140);
 *   arm.closeClaw();
 *   arm.moveTo(70, 200, 10);
 *   arm.openClaw();
 */
#include <Arduino.h>
#include "ik.h"
#include "MeArm.h"

namespace {

constexpr int kMinPulseWidthUs = 544;
constexpr int kMaxPulseWidthUs = 2400;

}  // namespace

bool setup_servo (ServoInfo& svo, const int n_min, const int n_max,
                  const float a_min, const float a_max)
{
    float n_range = n_max - n_min;
    float a_range = a_max - a_min;

    // Must have a non-zero angle range
    if(a_range == 0) return false;

    // Calculate gain and zero
    svo.gain = n_range / a_range;
    svo.zero = n_min - svo.gain * a_min;

    // Set limits
    svo.n_min = n_min;
    svo.n_max = n_max;

    return true;
}

int angle2pwm (const ServoInfo& svo, const float angle)
{
    float pwm = 0.5f + svo.zero + svo.gain * angle;
    return int(pwm*(kMaxPulseWidthUs-kMinPulseWidthUs)/180.0)+kMinPulseWidthUs; // convert 0-180 to microseconds
}

//Full constructor with calibration data
MeArm::MeArm(int sweepMinBase, int sweepMaxBase, float angleMinBase, float angleMaxBase,
          int sweepMinShoulder, int sweepMaxShoulder, float angleMinShoulder, float angleMaxShoulder,
          int sweepMinElbow, int sweepMaxElbow, float angleMinElbow, float angleMaxElbow,
          int sweepMinClaw, int sweepMaxClaw, float angleMinClaw, float angleMaxClaw) {
  //calroutine();
  setup_servo(_svoBase, sweepMinBase, sweepMaxBase, angleMinBase, angleMaxBase);
  setup_servo(_svoShoulder, sweepMinShoulder, sweepMaxShoulder, angleMinShoulder, angleMaxShoulder);
  setup_servo(_svoElbow, sweepMinElbow, sweepMaxElbow, angleMinElbow, angleMaxElbow);
  setup_servo(_svoClaw, sweepMinClaw, sweepMaxClaw, angleMinClaw, angleMaxClaw);
}

void MeArm::begin(MeArmServoOutput& output, uint8_t baseChannel, uint8_t shoulderChannel, uint8_t elbowChannel, uint8_t clawChannel) {
  _output = &output;
  _baseChannel = baseChannel;
  _shoulderChannel = shoulderChannel;
  _elbowChannel = elbowChannel;
  _clawChannel = clawChannel;

  //snapToXYZ(0, 100, 50);
  snapTo(0, 100, 50);
  openClaw();
}

void MeArm::end() {
  if (!_output) {
    return;
  }

  _output->detach(_baseChannel);
  _output->detach(_shoulderChannel);
  _output->detach(_elbowChannel);
  _output->detach(_clawChannel);
}

void MeArm::writeServo(uint8_t channel, const ServoInfo& servo, float angle) {
  if (!_output) {
    return;
  }

  int pulse = angle2pwm(servo, angle);
  if (pulse < kMinPulseWidthUs) {
    pulse = kMinPulseWidthUs;
  }
  if (pulse > kMaxPulseWidthUs) {
    pulse = kMaxPulseWidthUs;
  }
  _output->writeMicroseconds(channel, static_cast<uint16_t>(pulse));
}

//Set servos to reach a certain point directly without caring how we get there 
void MeArm::snapToXYZ(float x, float y, float z) {
  float radBase,radShoulder,radElbow;
  if (solve(x, y, z, radBase, radShoulder, radElbow)) {
    writeServo(_baseChannel, _svoBase, radBase);
    writeServo(_shoulderChannel, _svoShoulder, radShoulder);
    writeServo(_elbowChannel, _svoElbow, radElbow);
    _x = x; _y = y; _z = z;
  }    
}

//Travel smoothly from current point to another point
void MeArm::moveToXYZ(float x, float y, float z) {
  //Starting points - current pos
  float x0 = _x; 
  float y0 = _y; 
  float z0 = _z;
  float dist = sqrt((x0-x)*(x0-x)+(y0-y)*(y0-y)+(z0-z)*(z0-z));
  int step = 10;
  for (int i = 0; i<dist; i+= step) {
    snapToXYZ(x0 + (x-x0)*i/dist, y0 + (y-y0) * i/dist, z0 + (z-z0) * i/dist);
    delay(50);
  }
  snapToXYZ(x, y, z);
  delay(50);
}

//Get x and y from theta and r
void MeArm::polarToCartesian(float theta, float r, float& x, float& y){
    _r = r;
    _t = theta;
    x = r*sin(theta);
    y = r*cos(theta);
}

//Same as above but for cylindrical polar coodrinates
void MeArm::moveTo(float theta, float r, float z){
    float x, y;
    polarToCartesian(theta, r, x, y);
    moveToXYZ(x,y,z);
}

void MeArm::snapTo(float theta, float r, float z){
    float x, y;
    polarToCartesian(theta, r, x, y);
    snapToXYZ(x,y,z);
}

//Check to see if possible
bool MeArm::isReachable(float x, float y, float z) {
  float radBase,radShoulder,radElbow;
  return (solve(x, y, z, radBase, radShoulder, radElbow));
}

//Grab something
void MeArm::openClaw() {
  writeServo(_clawChannel, _svoClaw, pi/2);
  delay(300);
}

//Let go of something
void MeArm::closeClaw() {
  writeServo(_clawChannel, _svoClaw, 0);
  delay(300);
}

//Current x, y and z
float MeArm::getX() {
  return _x;
}
float MeArm::getY() {
  return _y;
}
float MeArm::getZ() {
  return _z;
}


float MeArm::getR() {
  return _r;
}
float MeArm::getTheta() {
  return _t;
}
