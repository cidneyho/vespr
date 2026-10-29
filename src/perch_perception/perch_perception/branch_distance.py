"""
Live branch distance from the D435i, using two clicks along the branch.

Click two points along the branch in the IR window. The script samples depth
in a thin strip along that segment, ignores invalid (zero) pixels, and shows
the median distance plus the fraction of valid pixels. Press 'c' to clear
the clicks, 'q' to quit.

Keep the camera on a tripod: the clicked line is fixed in the image, so if
the camera moves, the line no longer sits on the branch.
"""

import cv2
import numpy as np
import pyrealsense2 as rs

WIDTH, HEIGHT, FPS = 848, 480, 30
HALF_WIDTH_PX = 2  # strip is 2*2+1 = 5 px wide


def start_camera():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
    # Left IR (index 1) shares the depth stream's viewpoint, so a pixel in
    # this image is the same pixel in the depth map. No alignment needed.
    config.enable_stream(rs.stream.infrared, 1, WIDTH, HEIGHT, rs.format.y8, FPS)
    profile = pipeline.start(config)
    # Raw depth values are integers; depth_scale converts them to meters.
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    return pipeline, depth_scale


def sample_strip(depth, p0, p1, half_width):
    """Return (valid depth values, total pixels) in a thin strip from p0 to p1."""
    strip = np.zeros(depth.shape, dtype=np.uint8)
    cv2.line(strip, p0, p1, color=255, thickness=2 * half_width + 1)
    values = depth[strip > 0]
    valid = values[values > 0]  # zero means "no depth", never "zero meters"
    return valid, values.size


def main():
    pipeline, depth_scale = start_camera()
    clicks = []

    def on_mouse(event, x, y, _flags, _param):  # OpenCV's callback signature
        if event == cv2.EVENT_LBUTTONDOWN:
            clicks.append((x, y))
            if len(clicks) > 2:
                clicks.pop(0)

    cv2.namedWindow('ir')
    cv2.setMouseCallback('ir', on_mouse)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            depth = np.asanyarray(frames.get_depth_frame().get_data())
            ir = np.asanyarray(frames.get_infrared_frame(1).get_data())
            view = cv2.cvtColor(ir, cv2.COLOR_GRAY2BGR)

            for point in clicks:
                cv2.circle(view, point, 4, (0, 255, 0), -1)

            if len(clicks) == 2:
                cv2.line(view, clicks[0], clicks[1], (0, 255, 0), 1)
                valid, total = sample_strip(depth, clicks[0], clicks[1], HALF_WIDTH_PX)
                if valid.size > 0:
                    distance_m = float(np.median(valid)) * depth_scale
                    valid_fraction = valid.size / total
                    text = f'{distance_m:.3f} m   valid {valid_fraction:.0%}'
                else:
                    text = 'no valid depth on this line'
                cv2.putText(view, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0, 255, 0), 2)

            cv2.imshow('ir', view)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            if key == ord('c'):
                clicks.clear()
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
