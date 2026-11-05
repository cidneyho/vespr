/*
 * Optical flow from a downward camera, for the sim's flow sensor.
 *
 * A copy of PX4-OpticalFlow's OpticalFlowOpenCV (BSD-3-Clause, PX4 Development Team), same
 * feature tracker, same rate limiting, with one change in how the tracked features' movements
 * become one flow value:
 *   shift_only false: their mean, as PX4's (OpticalFlowOpenCV::calcFlow)
 *   shift_only true:  fit "shift + rotation + zoom about the image centre", report the shift
 *
 * Why: when the drone yaws, every feature moves round the image centre; when it climbs or
 * descends, towards or away from it. The mean of those movements cancels only if the features
 * happen to be spread evenly round the centre, so PX4's version reports sideways motion that
 * isn't there (about 8 cm/s while turning at 30 deg/s, perch_eval, 2026-11-05). A real flow
 * sensor reports the movement of the ground under its centre, which is the shift.
 */
#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#include "optical_flow.hpp"

namespace perch_sim
{

class RotationFreeFlow : public OpticalFlow
{
public:
	RotationFreeFlow(float focal_length_x, float focal_length_y, int output_rate, int image_width,
			 int image_height, bool shift_only);

	// As OpticalFlowOpenCV::calcFlow: flow_x, flow_y in rad over dt_us; returns the quality
	// (0 to 255), or -1 while the rate limiter holds the frame back
	int calcFlow(uint8_t *img_current, const uint32_t &img_time_us, int &dt_us,
		     float &flow_x, float &flow_y) override;

private:
	// The mean movement of the active features, dropping those outside the confidence interval
	void meanFlow(float &flow_x, float &flow_y, int &count);
	// The shift of the image centre, from a shift + rotation + zoom fit, dropping poorly fitting
	// features
	void shiftFlow(float &flow_x, float &flow_y, int &count);
	// Least-squares rotation and zoom about the image centre, and shift, over features where use[i]
	bool fitShift(const std::vector<bool> &use, cv::Point2f &turn_zoom, cv::Point2f &shift) const;

	bool shift_only;
	int num_features {20};               // as PX4's DEFAULT_NUMBER_OF_FEATURES
	float confidence_multiplier {1.645f}; // 90 % confidence interval, as PX4's
	std::vector<int> updateVector;
	std::vector<cv::Point2f> features_current, features_previous, useless;
};

} // namespace perch_sim
