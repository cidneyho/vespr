/*
 * The sim's flow sensor: PX4's gz optical flow sensor (BSD-3-Clause, PX4 Development Team,
 * src/modules/simulation/gz_plugins/optical_flow), with its flow from RotationFreeFlow.
 * Same camera, topic and message, so PX4's simulation bridge reads it as it reads PX4's.
 */
#pragma once

#include <memory>

#include <gz/msgs/image.pb.h>
#include <gz/sensors/Sensor.hh>
#include <gz/transport/Node.hh>
#include <opencv2/opencv.hpp>

#include "RotationFreeFlow.hpp"

namespace perch_sim
{

// The custom sensor type this plugin handles: <sensor type="custom" gz:type="...">
inline constexpr const char *kSensorType = "perch_optical_flow";

class OpticalFlowSensor : public gz::sensors::Sensor
{
public:
	explicit OpticalFlowSensor(bool shift_only = true) : _shift_only(shift_only) {}
	bool Load(const sdf::Sensor &_sdf) override;
	bool Update(const std::chrono::steady_clock::duration &_now) override;

private:
	void OnImage(const gz::msgs::Image &_msg);

	bool _shift_only;
	gz::transport::Node _node;
	gz::transport::Node::Publisher _publisher;
	std::shared_ptr<RotationFreeFlow> _optical_flow {nullptr};
	int _integration_time_us {0};
	cv::Mat _last_image_gray;
	uint32_t _last_image_timestamp {0};
	bool _new_image_available {false};
};

} // namespace perch_sim
