#pragma once

/// RGB + depth -> the cube's pose and top colour. sys1's only sensor.
/// Port of vibe `planner/clips.py::CubeSight`; every threshold's measurement
/// lives there. Depth is unprojected into the robot's gravity-aligned base
/// frame first, so every test below is geometry, not an image statistic.

#include <array>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "sys1/cfg.hpp"

namespace cpp_control {
namespace sys1 {

/// MEASURED lit/shaded chromaticity refs, 6 colours x 2 rows, RGB.
/// 0 red 1 orange 2 green 3 yellow 4 blue 5 pink. Painted refs read orange as
/// yellow 100% of the time — every lit face clips a channel to 255.
using Palette = std::array<float, 36>;
extern const Palette PALETTE_SIM;

/// Pinhole, in one plane's own pixels — carried per-plane on the camera wire,
/// so a downscaled depth plane stays exact and nothing needs calibrating here.
struct Intrinsics {
  float fx = 0.f, fy = 0.f, cx = 0.f, cy = 0.f;
  Intrinsics scaled(float s) const { return {fx * s, fy * s, cx * s, cy * s}; }
  bool valid() const { return fx > 0.f && fy > 0.f; }
};

/// Camera pose in the robot's gravity-aligned base frame (FK + IMU).
struct CameraPose {
  std::array<float, 9> R{};  ///< row-major R_bc
  std::array<float, 3> t{};
};

/// What one settled look at the cube yields, in the ROBOT BASE frame.
struct Sight {
  bool ok = false;        ///< top face proven AND whole enough to carry a POSE
  bool color_ok = false;  ///< top face proven HORIZONTAL, cut or not
  int color = -1;
  std::array<float, 3> pos{};  ///< cube centre (x fwd, y left, z up), metres
  float phi = 0.0f;   ///< spin about vertical, folded to [-pi/4, pi/4)
  int n_px = 0;
  float hint = 0.0f;  ///< bearing of the biggest blob — valid even if REJECTED
  std::string reason = "init";

  float range_m() const { return std::hypot(pos[0], pos[1]); }
  float bearing() const { return std::atan2(pos[1], pos[0]); }
};

/// Per-candidate stats, kept for palette calibration (§8.1) — not the loop.
struct Candidate {
  int color;
  int n_px;
  float up_dot;
  float vis, big, cx, cy;
};

class CubeSight {
 public:
  CubeSight(const Cfg& cfg, const Palette& palette, float half_extent);

  /// `bgr` CV_8UC3 (wire order), `depth_m` CV_32FC1 metres with `intr` in the
  /// depth plane's pixels. Colour is resized onto depth: it is the plane the
  /// producer already downscaled, so nothing resamples depth twice.
  Sight operator()(const cv::Mat& bgr, const cv::Mat& depth_m,
                   const Intrinsics& intr, const CameraPose& cam);

  const cv::Mat& labels() const { return labels_; }  ///< int16, -1 = not live
  const std::vector<Candidate>& last() const { return last_; }
  float half_extent() const { return half_extent_; }

  /// Classifier output as an image — the one view that explains a bad exit.
  static cv::Mat paint(const cv::Mat& labels);

 private:
  void classify(const cv::Mat& bgr);
  void build_rays(int w, int h, const Intrinsics& intr);

  Cfg cfg_;
  float half_extent_;
  std::array<float, 24> ref_{};  ///< 12 refs x (r, g) chromaticity
  cv::Mat labels_;               ///< CV_16SC1
  std::vector<float> rays_;      ///< (H*W*3) unit-depth camera rays, cached
  Intrinsics ray_intr_{};
  int ray_w_ = 0, ray_h_ = 0;
  std::vector<Candidate> last_;
  std::vector<int> idx_;         ///< scratch: live-pixel indices
  std::vector<float> pts_;       ///< scratch: (N*3) base-frame points
};

}  // namespace sys1
}  // namespace cpp_control
