/*
 * PX4's OpticalFlowSensor.cpp (BSD-3-Clause, PX4 Development Team), with RotationFreeFlow in
 * place of OpticalFlowOpenCV and our sensor type. Otherwise unchanged.
 */
#include "OpticalFlowSensor.hpp"

#include <cmath>
#include <cstring>

#include <gz/common/Console.hh>
#include <gz/sensors/Util.hh>

using namespace perch_sim;

namespace
{
// PX4's flow message (PX4-Autopilot src/modules/simulation/gz_msgs/opticalflow.proto), written
// out by hand: compiling that .proto here registers it a second time beside PX4's own flow
// plugin, and protobuf aborts
constexpr const char *kMsgType = "px4.msgs.OpticalFlow";

void put_varint(std::string &out, uint64_t value)
{
	while (value >= 0x80) {
		out.push_back(static_cast<char>((value & 0x7F) | 0x80));
		value >>= 7;
	}

	out.push_back(static_cast<char>(value));
}

void put_float(std::string &out, int field, float value)
{
	put_varint(out, (field << 3) | 5);  // wire type 5: 32-bit
	char bytes[4];
	std::memcpy(bytes, &value, 4);      // little-endian, as protobuf
	out.append(bytes, 4);
}

std::string encode_flow(int64_t time_usec, uint32_t integration_time_us, float integrated_x,
			float integrated_y, float quality)
{
	std::string out;
	put_varint(out, (1 << 3) | 0);      // time_usec, varint
	put_varint(out, static_cast<uint64_t>(time_usec));
	put_varint(out, (2 << 3) | 0);      // integration_time_us, varint
	put_varint(out, integration_time_us);
	put_float(out, 3, integrated_x);
	put_float(out, 4, integrated_y);
	put_float(out, 5, quality);
	return out;
}
} // namespace

bool OpticalFlowSensor::Load(const sdf::Sensor &_sdf)
{
	auto type = gz::sensors::customType(_sdf);

	if (type != kSensorType) {
		gzerr << "Trying to load [" << kSensorType << "] sensor, but got type [" << type
		      << "] instead." << std::endl;
		return false;
	}

	gz::sensors::Sensor::Load(_sdf);

	std::string output_topic = this->Topic();
	_publisher = _node.Advertise(output_topic, kMsgType);

	std::string camera_topic = output_topic;
	size_t last_segment = camera_topic.rfind("/optical_flow/optical_flow");

	if (last_segment != std::string::npos) {
		camera_topic = camera_topic.substr(0, last_segment) + "/flow_camera/image";
	}

	int image_width = 0;
	int image_height = 0;
	int update_rate = 0;
	float hfov = 0;
	auto sensorElem = _sdf.Element()->GetParent()->GetElement("sensor");

	while (sensorElem) {
		if (sensorElem->Get<std::string>("name") == "flow_camera") {
			auto cameraElem = sensorElem->GetElement("camera");
			update_rate = sensorElem->GetElement("update_rate")->Get<int>();
			hfov = cameraElem->GetElement("horizontal_fov")->Get<double>();
			auto imageElem = cameraElem->GetElement("image");
			image_width = imageElem->GetElement("width")->Get<int>();
			image_height = imageElem->GetElement("height")->Get<int>();
			break;
		}

		sensorElem = sensorElem->GetNextElement("sensor");
	}

	if (!_node.Subscribe(camera_topic, &OpticalFlowSensor::OnImage, this)) {
		gzerr << "Failed to subscribe to camera topic: " << camera_topic << std::endl;
		return false;
	}

	// Assume pinhole camera and 1:1 aspect ratio
	float focal_length = (image_width / 2.0f) / std::tan(hfov / 2.0f);
	_optical_flow = std::make_shared<RotationFreeFlow>(focal_length, focal_length, update_rate,
			image_width, image_height, _shift_only);
	gzmsg << "Optical flow on " << output_topic << ", rotation "
	      << (_shift_only ? "and zoom removed" : "and zoom not removed (as PX4's)") << std::endl;
	return true;
}

void OpticalFlowSensor::OnImage(const gz::msgs::Image &image_msg)
{
	if (image_msg.width() == 0 || image_msg.height() == 0) {
		gzerr << "Invalid image dimensions" << std::endl;
		return;
	}

	if (image_msg.pixel_format_type() == gz::msgs::PixelFormatType::RGB_INT8) {
		cv::Mat temp(image_msg.height(), image_msg.width(), CV_8UC3);
		std::memcpy(temp.data, image_msg.data().c_str(), image_msg.data().size());
		cv::cvtColor(temp, _last_image_gray, cv::COLOR_RGB2GRAY);

	} else if (image_msg.pixel_format_type() == gz::msgs::PixelFormatType::L_INT8) {
		std::memcpy(_last_image_gray.data, image_msg.data().c_str(), image_msg.data().size());

	} else {
		gzerr << "Unsupported image format" << std::endl;
		return;
	}

	uint32_t current_timestamp = (image_msg.header().stamp().sec() * 1000000ULL +
				      image_msg.header().stamp().nsec() / 1000ULL) & 0xFFFFFFFF;

	if (_last_image_timestamp != 0) {
		_integration_time_us = (current_timestamp - _last_image_timestamp) & 0xFFFFFFFF;
	}

	_last_image_timestamp = current_timestamp;
	_new_image_available = true;
}

bool OpticalFlowSensor::Update(const std::chrono::steady_clock::duration &)
{
	if (!_new_image_available) {
		return true;
	}

	float flow_x = 0.f;
	float flow_y = 0.f;
	int quality = _optical_flow->calcFlow(_last_image_gray.data, _last_image_timestamp,
					      _integration_time_us, flow_x, flow_y);
	std::string msg = encode_flow(_last_image_timestamp, _integration_time_us, flow_x, flow_y,
				      quality);

	if (!_publisher.PublishRaw(msg, kMsgType)) {
		gzwarn << "Failed to publish optical flow message" << std::endl;
	}

	_new_image_available = false;
	return true;
}
