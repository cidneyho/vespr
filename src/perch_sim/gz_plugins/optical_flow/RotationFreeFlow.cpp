/*
 * See RotationFreeFlow.hpp. The tracking, the feature bookkeeping, the quality and the rate
 * limiting are PX4-OpticalFlow's OpticalFlowOpenCV::calcFlow (BSD-3-Clause, PX4 Development
 * Team), without camera undistortion, which the sim's flow camera doesn't use.
 */
#include "RotationFreeFlow.hpp"

#include <algorithm>
#include <cmath>

#include "trackFeatures.h"

using namespace perch_sim;

RotationFreeFlow::RotationFreeFlow(float focal_length_x, float focal_length_y, int output_rate,
				   int image_width, int image_height, bool shift_only) :
	shift_only(shift_only)
{
	setImageWidth(image_width);
	setImageHeight(image_height);
	setFocalLengthX(focal_length_x);
	setFocalLengthY(focal_length_y);
	setOutputRate(output_rate);
	initLimitRate();
}

int RotationFreeFlow::calcFlow(uint8_t *img_current, const uint32_t &img_time_us, int &dt_us,
			       float &flow_x, float &flow_y)
{
	if (updateVector.empty()) {
		updateVector.resize(num_features, 2);
	}

	cv::Mat frame_gray = cv::Mat(image_height, image_width, CV_8UC1);
	frame_gray.data = (uchar *)img_current;

	trackFeatures(frame_gray, frame_gray, features_current, useless, updateVector, 0);

	float pixel_flow_x = 0.f;
	float pixel_flow_y = 0.f;
	int count = 0;

	if (!features_current.empty() && !features_previous.empty()) {
		if (shift_only) {
			shiftFlow(pixel_flow_x, pixel_flow_y, count);

		} else {
			meanFlow(pixel_flow_x, pixel_flow_y, count);
		}
	}

	features_previous = features_current;

	for (size_t i = 0; i < updateVector.size(); i++) {
		// new and now active
		if (updateVector[i] == 2) {
			updateVector[i] = 1;
		}

		// inactive
		if (updateVector[i] == 0) {
			updateVector[i] = 2;
		}
	}

	flow_x = pixel_flow_x;
	flow_y = pixel_flow_y;

	int flow_quality = std::round(255.0 * count / updateVector.size());
	flow_quality = limitRate(flow_quality, img_time_us, &dt_us, &flow_x, &flow_y);

	flow_x = std::atan2(flow_x, focal_length_x); // pixel flow to angular flow
	flow_y = std::atan2(flow_y, focal_length_y);

	return flow_quality;
}

void RotationFreeFlow::meanFlow(float &flow_x, float &flow_y, int &count)
{
	// PX4's: the mean, then the mean again over features within the confidence interval
	float mean_x = 0.f;
	float mean_y = 0.f;
	count = 0;

	for (size_t i = 0; i < updateVector.size(); i++) {
		if (updateVector[i] == 1) {
			mean_x += features_current[i].x - features_previous[i].x;
			mean_y += features_current[i].y - features_previous[i].y;
			count++;
		}
	}

	if (count == 0) {
		return;
	}

	mean_x /= count;
	mean_y /= count;
	float std_x = 0.f;
	float std_y = 0.f;

	for (size_t i = 0; i < updateVector.size(); i++) {
		if (updateVector[i] == 1) {
			std_x += std::pow(features_current[i].x - features_previous[i].x - mean_x, 2);
			std_y += std::pow(features_current[i].y - features_previous[i].y - mean_y, 2);
		}
	}

	std_x = std::sqrt(std_x / count);
	std_y = std::sqrt(std_y / count);
	float sum_x = 0.f;
	float sum_y = 0.f;
	count = 0;

	for (size_t i = 0; i < updateVector.size(); i++) {
		if (updateVector[i] == 1) {
			float fx = features_current[i].x - features_previous[i].x;
			float fy = features_current[i].y - features_previous[i].y;

			if (std::fabs(fx - mean_x) < std_x * confidence_multiplier
			    && std::fabs(fy - mean_y) < std_y * confidence_multiplier) {
				sum_x += fx;
				sum_y += fy;
				count++;

			} else {
				updateVector[i] = 0;
			}
		}
	}

	if (count) {
		mean_x = sum_x / count;
		mean_y = sum_y / count;
	}

	flow_x = mean_x;
	flow_y = mean_y;
}

void RotationFreeFlow::shiftFlow(float &flow_x, float &flow_y, int &count)
{
	// As meanFlow, with "shift + rotation + zoom" in place of the mean: fit, drop features whose
	// movement the fit explains poorly (same confidence interval, on the fit's residuals), refit
	std::vector<bool> use(updateVector.size());

	for (size_t i = 0; i < updateVector.size(); i++) {
		use[i] = updateVector[i] == 1;
	}

	count = std::count(use.begin(), use.end(), true);
	cv::Point2f turn_zoom;
	cv::Point2f shift;

	if (count < 3 || !fitShift(use, turn_zoom, shift)) {
		meanFlow(flow_x, flow_y, count); // too few features to fit a rotation and zoom
		return;
	}

	const cv::Point2f centre(image_width / 2.f, image_height / 2.f);
	const float c = turn_zoom.x;
	const float s = turn_zoom.y;
	std::vector<cv::Point2f> residual(use.size());
	float std_x = 0.f;
	float std_y = 0.f;

	for (size_t i = 0; i < use.size(); i++) {
		if (use[i]) {
			cv::Point2f a = features_previous[i] - centre;
			cv::Point2f predicted(c * a.x - s * a.y + centre.x + shift.x,
					      s * a.x + c * a.y + centre.y + shift.y);
			residual[i] = features_current[i] - predicted;
			std_x += residual[i].x * residual[i].x;
			std_y += residual[i].y * residual[i].y;
		}
	}

	std_x = std::sqrt(std_x / count);
	std_y = std::sqrt(std_y / count);
	// A perfect fit has no spread: keep every feature then, not none
	const float tol_x = std_x * confidence_multiplier + 1e-3f;
	const float tol_y = std_y * confidence_multiplier + 1e-3f;
	int kept = 0;

	for (size_t i = 0; i < use.size(); i++) {
		if (use[i]) {
			if (std::fabs(residual[i].x) < tol_x && std::fabs(residual[i].y) < tol_y) {
				kept++;

			} else {
				use[i] = false;
				updateVector[i] = 0;
			}
		}
	}

	if (kept >= 3) {
		fitShift(use, turn_zoom, shift);
		count = kept;
	}

	flow_x = shift.x;
	flow_y = shift.y;
}

bool RotationFreeFlow::fitShift(const std::vector<bool> &use, cv::Point2f &turn_zoom,
				cv::Point2f &shift) const
{
	// current = zoom R(angle) (previous - centre) + centre + shift. Least squares: the turn and
	// zoom that best map the centred previous features onto the centred current ones, then the
	// shift of means. turn_zoom = zoom (cos angle, sin angle)
	const cv::Point2f centre(image_width / 2.f, image_height / 2.f);
	cv::Point2f mean_a, mean_b;
	int n = 0;

	for (size_t i = 0; i < use.size(); i++) {
		if (use[i]) {
			mean_a += features_previous[i] - centre;
			mean_b += features_current[i] - centre;
			n++;
		}
	}

	if (n == 0) {
		return false;
	}

	mean_a *= 1.f / n;
	mean_b *= 1.f / n;
	float cross = 0.f;
	float dot = 0.f;
	float spread = 0.f;

	for (size_t i = 0; i < use.size(); i++) {
		if (use[i]) {
			cv::Point2f a = features_previous[i] - centre - mean_a;
			cv::Point2f b = features_current[i] - centre - mean_b;
			cross += a.x * b.y - a.y * b.x;
			dot += a.x * b.x + a.y * b.y;
			spread += a.x * a.x + a.y * a.y;
		}
	}

	if (dot <= 0.f || spread < 1.f) {
		return false; // features too close together, or tracking went wrong
	}

	turn_zoom = cv::Point2f(dot / spread, cross / spread);
	const float c = turn_zoom.x;
	const float s = turn_zoom.y;
	shift = mean_b - cv::Point2f(c * mean_a.x - s * mean_a.y, s * mean_a.x + c * mean_a.y);
	return true;
}
