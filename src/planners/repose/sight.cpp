#include "cpp_control/planners/repose/sight.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <opencv2/imgproc.hpp>

namespace cpp_control {
namespace planners {
namespace repose {

const Palette PALETTE_SIM = {
    255.f, 105.f, 107.f, 53.f, 11.f, 12.f,  // red
    255.f, 239.f, 53.f,  49.f, 23.f, 5.f,   // orange
    106.f, 255.f, 107.f, 6.f,  28.f, 7.f,   // green
    255.f, 255.f, 107.f, 71.f, 71.f, 16.f,  // yellow
    104.f, 104.f, 255.f, 10.f, 10.f, 51.f,  // blue
    255.f, 107.f, 255.f, 54.f, 13.f, 47.f,  // pink
};

namespace {

constexpr int NUM_COLORS = 6;
constexpr int REFS_PER_COLOR = 2;

/// Angle mod 90 deg, into [-pi/4, pi/4) — the cube cannot tell them apart.
float fold(float a) {
  constexpr float kQuarter = static_cast<float>(M_PI) / 2.0f;
  float m = std::fmod(a + kQuarter / 2.0f, kQuarter);
  if (m < 0.0f) m += kQuarter;
  return m - kQuarter / 2.0f;
}

/// Unit normal of the best-fit plane — smallest eigenvector of the scatter.
/// eigh(q'q) rather than svd(q): same vector, O(1) memory, 4.4x (sys1_cpp §6).
std::array<float, 3> plane_normal(const float* pts, int n) {
  double mx = 0.0, my = 0.0, mz = 0.0;
  for (int i = 0; i < n; ++i) {
    mx += pts[i * 3];
    my += pts[i * 3 + 1];
    mz += pts[i * 3 + 2];
  }
  mx /= n;
  my /= n;
  mz /= n;
  double c[6] = {0, 0, 0, 0, 0, 0};  // xx xy xz yy yz zz
  for (int i = 0; i < n; ++i) {
    const double x = pts[i * 3] - mx, y = pts[i * 3 + 1] - my,
                 z = pts[i * 3 + 2] - mz;
    c[0] += x * x;
    c[1] += x * y;
    c[2] += x * z;
    c[3] += y * y;
    c[4] += y * z;
    c[5] += z * z;
  }
  const cv::Matx33d cov(c[0], c[1], c[2], c[1], c[3], c[4], c[2], c[4], c[5]);
  cv::Matx33d vecs;
  cv::Vec3d vals;
  if (!cv::eigen(cov, vals, vecs)) return {0.0f, 0.0f, 0.0f};
  return {static_cast<float>(vecs(2, 0)), static_cast<float>(vecs(2, 1)),
          static_cast<float>(vecs(2, 2))};  // eigenvalues come back descending
}

float median_of(std::vector<float>& v) {
  const size_t n = v.size();
  const size_t mid = n / 2;
  std::nth_element(v.begin(), v.begin() + mid, v.end());
  const float hi = v[mid];
  if (n % 2 == 1) return hi;
  const float lo = *std::max_element(v.begin(), v.begin() + mid);
  return 0.5f * (lo + hi);
}

float percentile_of(std::vector<float> v, float q) {
  if (v.empty()) return 0.0f;
  const size_t k = std::min(
      v.size() - 1,
      static_cast<size_t>(std::lround(q * static_cast<float>(v.size() - 1))));
  std::nth_element(v.begin(), v.begin() + k, v.end());
  return v[k];
}

/// Centre of the interval with the most samples. The interval width is the
/// known cube edge plus a small sensor slack; unlike min/max, one flying depth
/// pixel cannot move it.
float densest_interval_center(const std::vector<float>& values, float width) {
  if (values.empty()) return 0.0f;
  const auto mm = std::minmax_element(values.begin(), values.end());
  if (*mm.second - *mm.first <= width) return 0.5f * (*mm.first + *mm.second);

  // Five-millimetre bins are far below the D435i depth fringe and turn the
  // orientation sweep from repeated O(N log N) sorts into bounded O(N)
  // histograms.
  constexpr float kBin = 0.005f;
  const float lo = *mm.first;
  const int n_bins =
      std::max(1, static_cast<int>(std::ceil((*mm.second - lo) / kBin)) + 1);
  std::vector<int> hist(static_cast<size_t>(n_bins), 0);
  for (float value : values) {
    const int bin =
        std::clamp(static_cast<int>((value - lo) / kBin), 0, n_bins - 1);
    ++hist[bin];
  }
  const int span = std::max(1, static_cast<int>(std::floor(width / kBin)));
  int count = 0, best_count = -1, best_begin = 0;
  for (int end = 0; end < n_bins; ++end) {
    count += hist[end];
    if (end - span >= 0) count -= hist[end - span];
    if (count > best_count) {
      best_count = count;
      best_begin = std::max(0, end - span + 1);
    }
  }
  const float interval_lo = lo + best_begin * kBin;
  const float interval_hi = interval_lo + width + kBin;
  float kept_lo = std::numeric_limits<float>::infinity();
  float kept_hi = -std::numeric_limits<float>::infinity();
  for (float value : values)
    if (value >= interval_lo && value <= interval_hi) {
      kept_lo = std::min(kept_lo, value);
      kept_hi = std::max(kept_hi, value);
    }
  return std::isfinite(kept_lo) ? 0.5f * (kept_lo + kept_hi)
                                : 0.5f * (interval_lo + interval_hi);
}

std::array<float, 2> robust_bounds(std::vector<float> values) {
  const size_t hi = values.size() - 1;
  const size_t lo_k = static_cast<size_t>(std::lround(0.02f * hi));
  const size_t hi_k = static_cast<size_t>(std::lround(0.98f * hi));
  std::nth_element(values.begin(), values.begin() + lo_k, values.end());
  const float lo = values[lo_k];
  std::nth_element(values.begin(), values.begin() + hi_k, values.end());
  return {lo, values[hi_k]};
}

struct SquareFit {
  bool ok = false;
  float cx = 0.0f, cy = 0.0f, phi = 0.0f;
  float vis = 0.0f, big = 0.0f;
  std::vector<uint8_t> keep;
};

/// Fit a known-size square without assuming that its depth pixels form one
/// clean contour. For every cube-unique orientation in [0, 90 deg), find the
/// densest edge-sized window in each axis. The winning window discards depth
/// fringe while interior holes cost support but not footprint.
SquareFit fit_square(const std::vector<cv::Point2f>& xy, float edge,
                     float slack) {
  SquareFit best;
  if (xy.size() < 3) return best;
  const float width = edge * (1.0f + slack);
  float best_score = -std::numeric_limits<float>::infinity();
  float best_cu = 0.0f, best_cv = 0.0f, best_angle = 0.0f;

  std::vector<float> u(xy.size()), v(xy.size()), work;
  // Five-degree bins are enough to choose the inlier window; continuous
  // heading is recovered from those inliers below. This keeps the 20 Hz
  // observe budget on the Orin.
  for (int degree = 0; degree < 90; degree += 5) {
    const float a = degree * static_cast<float>(M_PI) / 180.0f;
    const float ca = std::cos(a), sa = std::sin(a);
    for (size_t i = 0; i < xy.size(); ++i) {
      u[i] = ca * xy[i].x + sa * xy[i].y;
      v[i] = -sa * xy[i].x + ca * xy[i].y;
    }
    float cu = densest_interval_center(u, width);
    float cv = densest_interval_center(v, width);
    // Two alternating refinements make the two 1-D windows one 2-D square.
    for (int pass = 0; pass < 2; ++pass) {
      work.clear();
      for (size_t i = 0; i < xy.size(); ++i)
        if (std::fabs(v[i] - cv) <= 0.5f * width) work.push_back(u[i]);
      if (!work.empty()) cu = densest_interval_center(work, width);
      work.clear();
      for (size_t i = 0; i < xy.size(); ++i)
        if (std::fabs(u[i] - cu) <= 0.5f * width) work.push_back(v[i]);
      if (!work.empty()) cv = densest_interval_center(work, width);
    }

    std::vector<float> kept_u, kept_v;
    kept_u.reserve(xy.size());
    kept_v.reserve(xy.size());
    for (size_t i = 0; i < xy.size(); ++i)
      if (std::fabs(u[i] - cu) <= 0.5f * width &&
          std::fabs(v[i] - cv) <= 0.5f * width) {
        kept_u.push_back(u[i]);
        kept_v.push_back(v[i]);
      }
    if (kept_u.size() < 3) continue;
    const auto ub = robust_bounds(kept_u);
    const auto vb = robust_bounds(kept_v);
    const float ulo = ub[0], uhi = ub[1];
    const float vlo = vb[0], vhi = vb[1];
    const float du = uhi - ulo, dv = vhi - vlo;
    // Support is primary; the metre-scale term breaks broad plateaus in favour
    // of the orientation whose robust footprint is actually one square.
    const float score = static_cast<float>(kept_u.size()) -
                        50.0f * (std::fabs(du - edge) + std::fabs(dv - edge));
    if (score <= best_score) continue;
    best_score = score;
    best.ok = true;
    best_angle = a;
    best_cu = 0.5f * (ulo + uhi);
    best_cv = 0.5f * (vlo + vhi);
    best.vis = std::min(du, dv) / edge;
    best.big = std::max(du, dv) / edge;
    best.keep.assign(xy.size(), 0);
    for (size_t i = 0; i < xy.size(); ++i)
      if (std::fabs(u[i] - cu) <= 0.5f * width &&
          std::fabs(v[i] - cv) <= 0.5f * width)
        best.keep[i] = 1;
  }
  if (!best.ok) return best;
  const float ca = std::cos(best_angle), sa = std::sin(best_angle);
  best.cx = ca * best_cu - sa * best_cv;
  best.cy = sa * best_cu + ca * best_cv;
  // The orientation sweep is deliberately coarse and exists to reject fringe,
  // not to quantise heading. Once fringe is gone, minAreaRect sees the actual
  // square edges and supplies the continuous spin that clip retrieval needs.
  std::vector<cv::Point2f> kept_xy;
  kept_xy.reserve(xy.size());
  for (size_t i = 0; i < xy.size(); ++i)
    if (best.keep[i]) kept_xy.push_back(xy[i]);
  const cv::RotatedRect rect = cv::minAreaRect(kept_xy);
  best.phi = fold(rect.angle * static_cast<float>(M_PI) / 180.0f);
  return best;
}

struct GroundPlane {
  bool ok = false;
  std::array<float, 3> n{0.0f, 0.0f, 1.0f};
  float d = 0.0f;
};

/// Deterministic RANSAC over the lower part of the cloud. A least-squares fit
/// to that subset fails when a near cube fills the frame: it averages the floor
/// and top into a plane that is neither. RANSAC keeps those two levels
/// separate.
GroundPlane fit_ground_plane(const std::vector<float>& pts,
                             const std::vector<int>& sample_slots,
                             float floor_quantile, float distance,
                             float up_dot_min, int min_px) {
  GroundPlane out;
  if (sample_slots.size() < 3) return out;
  std::vector<float> zs;
  zs.reserve(sample_slots.size());
  for (int slot : sample_slots) zs.push_back(pts[slot * 3 + 2]);
  const float z_cut = percentile_of(zs, floor_quantile);
  std::vector<int> floor_slots;
  floor_slots.reserve(sample_slots.size());
  for (int slot : sample_slots)
    if (pts[slot * 3 + 2] <= z_cut) floor_slots.push_back(slot);
  if (floor_slots.size() < 3) return out;

  int best_support = 0;
  std::array<float, 3> best_n{};
  float best_d = 0.0f;
  uint32_t state = 0x9e3779b9U ^ static_cast<uint32_t>(floor_slots.size());
  auto pick = [&]() {
    state = state * 1664525U + 1013904223U;
    return floor_slots[state % floor_slots.size()];
  };
  constexpr int kIterations = 128;
  for (int trial = 0; trial < kIterations; ++trial) {
    const int ia = pick(), ib = pick(), ic = pick();
    if (ia == ib || ia == ic || ib == ic) continue;
    const float* a = &pts[ia * 3];
    const float* b = &pts[ib * 3];
    const float* c = &pts[ic * 3];
    float nx = (b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1]);
    float ny = (b[2] - a[2]) * (c[0] - a[0]) - (b[0] - a[0]) * (c[2] - a[2]);
    float nz = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
    const float norm = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (norm < 1e-6f) continue;
    nx /= norm;
    ny /= norm;
    nz /= norm;
    if (nz < 0.0f) {
      nx = -nx;
      ny = -ny;
      nz = -nz;
    }
    if (nz < up_dot_min) continue;
    const float d = -(nx * a[0] + ny * a[1] + nz * a[2]);
    int support = 0;
    for (int slot : floor_slots) {
      const float* p = &pts[slot * 3];
      if (std::fabs(nx * p[0] + ny * p[1] + nz * p[2] + d) <= distance)
        ++support;
    }
    if (support > best_support) {
      best_support = support;
      best_n = {nx, ny, nz};
      best_d = d;
    }
  }
  if (best_support < std::max(3 * min_px, 24)) return out;

  // Refit the winning consensus rather than returning one sampled triangle.
  std::vector<float> inliers;
  inliers.reserve(static_cast<size_t>(best_support) * 3);
  std::array<double, 3> mean{};
  int n = 0;
  for (int slot : floor_slots) {
    const float* p = &pts[slot * 3];
    const float e =
        best_n[0] * p[0] + best_n[1] * p[1] + best_n[2] * p[2] + best_d;
    if (std::fabs(e) > 1.5f * distance) continue;
    inliers.insert(inliers.end(), p, p + 3);
    for (int j = 0; j < 3; ++j) mean[j] += p[j];
    ++n;
  }
  if (n < 3) return out;
  out.n = plane_normal(inliers.data(), n);
  if (out.n[2] < 0.0f)
    for (float& value : out.n) value = -value;
  if (out.n[2] < up_dot_min) return out;
  for (double& value : mean) value /= n;
  out.d = -(out.n[0] * static_cast<float>(mean[0]) +
            out.n[1] * static_cast<float>(mean[1]) +
            out.n[2] * static_cast<float>(mean[2]));
  out.ok = true;
  return out;
}

}  // namespace

CubeSight::CubeSight(const Cfg& cfg, float half_extent)
    : cfg_(cfg), half_extent_(half_extent) {
  for (int i = 0; i < NUM_COLORS * REFS_PER_COLOR; ++i) {
    const float r = cfg.palette[i * 3], g = cfg.palette[i * 3 + 1],
                b = cfg.palette[i * 3 + 2];
    const float s = std::max(r + g + b, 1e-6f);
    ref_[i * 2] = r / s;
    ref_[i * 2 + 1] = g / s;
  }
}

// ── Classification ───────────────────────────────────────────────
//
// Only LIVE pixels are classified: the full-frame intermediate cost 22 MB in
// numpy and the argmin is identical either way. 12 squared distances per live
// pixel — at a 160x120 plane that is ~0.1 ms, so the doc's GEMM reformulation
// buys nothing and would perturb the argmin on boundary pixels.

void CubeSight::classify(const cv::Mat& bgr) {
  labels_.create(bgr.rows, bgr.cols, CV_16SC1);
  labels_.setTo(-1);
  const float min_value = static_cast<float>(cfg_.min_value);
  const float reject2 = cfg_.chroma_reject * cfg_.chroma_reject;
  for (int r = 0; r < bgr.rows; ++r) {
    const uint8_t* src = bgr.ptr<uint8_t>(r);
    int16_t* dst = labels_.ptr<int16_t>(r);
    for (int c = 0; c < bgr.cols; ++c) {
      const float b = src[c * 3], g = src[c * 3 + 1], rr = src[c * 3 + 2];
      const float hi = std::max(rr, std::max(g, b));
      const float lo = std::min(rr, std::min(g, b));
      // Relative saturation: shading scales all three channels, so an absolute
      // threshold does not survive it.
      if (hi < min_value || (hi - lo) / std::max(hi, 1e-6f) < cfg_.min_rel_sat)
        continue;
      const float s = std::max(rr + g + b, 1e-6f);
      const float cr = rr / s, cg = g / s;
      float best = std::numeric_limits<float>::max();
      int arg = 0;
      for (int k = 0; k < NUM_COLORS * REFS_PER_COLOR; ++k) {
        const float dr = cr - ref_[k * 2], dg = cg - ref_[k * 2 + 1];
        const float d2 = dr * dr + dg * dg;
        if (d2 < best) {
          best = d2;
          arg = k;
        }
      }
      // Beyond `chroma_reject` a pixel is NO colour rather than the nearest of
      // six. Off by default: the shipping classifier has no "none" class.
      if (reject2 > 0.0f && best > reject2) continue;
      dst[c] = static_cast<int16_t>(arg / REFS_PER_COLOR);
    }
  }
}

void CubeSight::build_rays(int w, int h, const Intrinsics& intr) {
  if (ray_w_ == w && ray_h_ == h && intr.fx == ray_intr_.fx &&
      intr.fy == ray_intr_.fy && intr.cx == ray_intr_.cx &&
      intr.cy == ray_intr_.cy)
    return;
  // Camera convention (mujoco, and the realsense colour frame after align):
  // +x right, +y up, looks down -z. Constant per (w, h, intrinsics).
  rays_.resize(static_cast<size_t>(w) * h * 3);
  for (int v = 0; v < h; ++v)
    for (int u = 0; u < w; ++u) {
      float* p = &rays_[(static_cast<size_t>(v) * w + u) * 3];
      p[0] = (static_cast<float>(u) - intr.cx) / intr.fx;
      p[1] = -(static_cast<float>(v) - intr.cy) / intr.fy;
      p[2] = -1.0f;
    }
  ray_w_ = w;
  ray_h_ = h;
  ray_intr_ = intr;
}

// ── The read ─────────────────────────────────────────────────────

Sight CubeSight::operator()(const cv::Mat& bgr, const cv::Mat& depth_m,
                            const Intrinsics& intr, const CameraPose& cam) {
  cv::Mat colour = bgr, depth = depth_m;
  Intrinsics k = intr;
  if (cfg_.proc_width > 0 && depth.cols > cfg_.proc_width) {
    const int h = std::max(
        1,
        static_cast<int>(std::lround(
            depth.rows * static_cast<double>(cfg_.proc_width) / depth.cols)));
    k = intr.scaled(static_cast<float>(cfg_.proc_width) / depth.cols);
    cv::resize(depth, depth, cv::Size(cfg_.proc_width, h), 0, 0,
               cv::INTER_NEAREST);
  }
  if (colour.size() != depth.size())
    cv::resize(colour, colour, depth.size(), 0, 0, cv::INTER_AREA);

  classify(colour);
  build_rays(depth.cols, depth.rows, k);

  const int total = depth.rows * depth.cols;
  const int min_px =
      std::max(cfg_.min_px_floor, static_cast<int>(cfg_.min_area_frac * total));
  const float edge = 2.0f * half_extent_;
  if (cfg_.read == Cfg::Read::PLANE)
    return read_plane(depth, min_px, edge, cam);
  return cfg_.read == Cfg::Read::MASK ? read_mask(depth, min_px, edge, cam)
                                      : read_blobs(depth, min_px, edge, cam);
}

// ── BLOBS: colour first, six candidates, highest wins ────────────
//
// The shipping read. Every gate below is per COLOUR, so one physical face that
// straddles two chromaticity cells becomes two candidates that then compete on
// height — which is where this read loses on hardware (`Cfg::Read`).

Sight CubeSight::read_blobs(const cv::Mat& depth, int min_px, float edge,
                            const CameraPose& cam) {
  const int total = depth.rows * depth.cols;
  const float vis_color = cfg_.color_gate();
  const float* R = cam.R.data();

  std::string why = "no_blob";
  float hint = 0.0f;
  int hint_px = 0;
  bool have_best = false, have_seen = false;
  int best_col = -1, best_n = 0, seen_col = -1, seen_n = 0;
  std::array<float, 3> best_pos{}, seen_pos{};
  float best_phi = 0.0f, seen_phi = 0.0f;
  last_.clear();

  std::vector<cv::Point2f> xy;
  std::vector<float> zs;
  for (int col = 0; col < NUM_COLORS; ++col) {
    // Gather this colour's valid pixels, then unproject ONLY those: most of the
    // frame is never cube-coloured, and the maths is unchanged.
    idx_.clear();
    for (int i = 0; i < total; ++i) {
      const float z = depth.ptr<float>()[i];
      if (labels_.ptr<int16_t>()[i] == col && std::isfinite(z) &&
          z > cfg_.z_min_m)
        idx_.push_back(i);
    }
    if (static_cast<int>(idx_.size()) < min_px) continue;

    pts_.resize(idx_.size() * 3);
    double sx = 0.0, sy = 0.0;
    for (size_t n = 0; n < idx_.size(); ++n) {
      const float z = depth.ptr<float>()[idx_[n]];
      const float* ray = &rays_[static_cast<size_t>(idx_[n]) * 3];
      const float cx = ray[0] * z, cy = ray[1] * z, cz = ray[2] * z;
      float* p = &pts_[n * 3];
      p[0] = R[0] * cx + R[1] * cy + R[2] * cz + cam.t[0];
      p[1] = R[3] * cx + R[4] * cy + R[5] * cz + cam.t[1];
      p[2] = R[6] * cx + R[7] * cy + R[8] * cz + cam.t[2];
      sx += p[0];
      sy += p[1];
    }
    if (static_cast<int>(idx_.size()) > hint_px) {  // survives a rejection
      hint_px = static_cast<int>(idx_.size());
      hint = std::atan2(static_cast<float>(sy / idx_.size()),
                        static_cast<float>(sx / idx_.size()));
    }

    // TOP SLAB. A same-coloured floor touching the cube is ONE connected
    // component and the floor is the larger half, so cut by height. max(z)
    // cannot set it — stray pixels put the slab above the robot's root — so
    // use the min_px-th highest z, which ignores that many outliers by
    // construction.
    const int n_all = static_cast<int>(idx_.size());
    zs.resize(n_all);
    for (int n = 0; n < n_all; ++n) zs[n] = pts_[n * 3 + 2];
    std::nth_element(zs.begin(), zs.begin() + (n_all - min_px), zs.end());
    const float z_cut = zs[n_all - min_px] - cfg_.slab_frac * edge;

    int n_top = 0;
    for (int n = 0; n < n_all; ++n) {
      if (pts_[n * 3 + 2] <= z_cut) continue;
      std::copy(&pts_[n * 3], &pts_[n * 3] + 3, &pts_[n_top * 3]);
      ++n_top;
    }
    if (n_top < min_px) continue;

    // The face is a SQUARE, so fit one: min-area rect gives centre, edge
    // direction and size together. A centroid is foreshortening-biased and PCA
    // is degenerate on a square — both measured, neither fixable by a
    // threshold.
    xy.resize(n_top);
    zs.resize(n_top);
    for (int n = 0; n < n_top; ++n) {
      xy[n] = {pts_[n * 3], pts_[n * 3 + 1]};
      zs[n] = pts_[n * 3 + 2];
    }
    const cv::RotatedRect rect = cv::minAreaRect(xy);
    const float vis = std::min(rect.size.width, rect.size.height) / edge;
    const float big = std::max(rect.size.width, rect.size.height) / edge;
    // Pose-invariant: a fitted plane normal in the gravity-aligned base frame
    // survives any camera pitch, which a depth-slope threshold did not. Keep
    // the rejected candidate's scalar too: calibration bags need to show how
    // far hardware reads sit from the gate, not only that they failed it.
    const float up_dot = std::fabs(plane_normal(pts_.data(), n_top)[2]);
    last_.push_back(
        {col, n_top, up_dot, vis, big, rect.center.x, rect.center.y});
    if (up_dot < cfg_.up_dot_min) {
      why = "side_face";
      continue;
    }
    if (big > cfg_.big_max) {  // wider than a cube: this is the floor
      why = "too_big";
      continue;
    }
    const float zc = median_of(zs) - half_extent_;  // -> cube CENTRE
    const std::array<float, 3> pos{rect.center.x, rect.center.y, zc};
    const float phi = fold(rect.angle * static_cast<float>(M_PI) / 180.0f);

    // TWO GATES ON ONE CANDIDATE. The plane test already proved this is the top
    // face, so the COLOUR is settled; only the CENTRE needs the face whole — a
    // partial view puts the rect centre off by half the missing strip (~0.08
    // m).
    if (vis >= vis_color && (!have_seen || zc > seen_pos[2])) {
      have_seen = true;
      seen_col = col;
      seen_pos = pos;
      seen_phi = phi;
      seen_n = n_top;
    }
    if (vis < cfg_.min_visible) {
      why = "cut_face";
      continue;
    }
    if (!have_best || zc > best_pos[2]) {  // highest wins
      have_best = true;
      best_col = col;
      best_pos = pos;
      best_phi = phi;
      best_n = n_top;
    }
  }

  Sight s;
  s.hint = hint;
  if (!have_seen) {
    s.reason = why;
    return s;
  }
  s.ok = have_best;
  s.color_ok = true;
  s.color = have_best ? best_col : seen_col;
  s.pos = have_best ? best_pos : seen_pos;
  s.phi = have_best ? best_phi : seen_phi;
  s.n_px = have_best ? best_n : seen_n;
  s.reason = have_best ? "ok" : why;
  return s;
}

// ── MASK: geometry first, one mask, modal colour ─────────────────
//
// The cube's top face is ONE horizontal plane, whichever colours the classifier
// happened to split it into. Find that plane from every coloured pixel at once,
// then ask what colour it mostly is. The gates are the same numbers as BLOBS
// and mean the same things — they are applied to one candidate instead of six,
// so a face that straddles two chromaticity cells can no longer compete with
// itself, and the loser can no longer win on height.

Sight CubeSight::read_mask(const cv::Mat& depth, int min_px, float edge,
                           const CameraPose& cam) {
  const int total = depth.rows * depth.cols;
  const float* R = cam.R.data();
  Sight s;
  last_.clear();

  // LIVE: every pixel the gates called a colour, with a usable depth. Unproject
  // once — BLOBS does this per colour, which is the same arithmetic six times.
  idx_.clear();
  for (int i = 0; i < total; ++i) {
    const float z = depth.ptr<float>()[i];
    if (labels_.ptr<int16_t>()[i] >= 0 && std::isfinite(z) && z > cfg_.z_min_m)
      idx_.push_back(i);
  }
  const int n_live = static_cast<int>(idx_.size());
  if (n_live < min_px) {
    s.reason = "no_blob";
    return s;
  }
  pts_.resize(static_cast<size_t>(n_live) * 3);
  double sx = 0.0, sy = 0.0;
  for (int n = 0; n < n_live; ++n) {
    const float z = depth.ptr<float>()[idx_[n]];
    const float* ray = &rays_[static_cast<size_t>(idx_[n]) * 3];
    const float cx = ray[0] * z, cy = ray[1] * z, cz = ray[2] * z;
    float* p = &pts_[n * 3];
    p[0] = R[0] * cx + R[1] * cy + R[2] * cz + cam.t[0];
    p[1] = R[3] * cx + R[4] * cy + R[5] * cz + cam.t[1];
    p[2] = R[6] * cx + R[7] * cy + R[8] * cz + cam.t[2];
    sx += p[0];
    sy += p[1];
  }
  // Bearing of everything coloured — the SCAN's hint, and it survives every
  // rejection below exactly as it does in BLOBS.
  s.hint = std::atan2(static_cast<float>(sy / n_live),
                      static_cast<float>(sx / n_live));

  // TOP PLANE. A percentile, not the max: stray pixels put the max above the
  // robot's own root, and one of them would carry the band with it.
  std::vector<float> zs(n_live);
  for (int n = 0; n < n_live; ++n) zs[n] = pts_[n * 3 + 2];
  const int kth = std::min(
      n_live - 1,
      std::max(0, static_cast<int>(cfg_.mask_top_pct * 0.01f * (n_live - 1))));
  std::nth_element(zs.begin(), zs.begin() + kth, zs.end());
  const float z_top = zs[kth];

  cv::Mat band(depth.rows, depth.cols, CV_8UC1, cv::Scalar(0));
  for (int n = 0; n < n_live; ++n)
    if (std::fabs(pts_[n * 3 + 2] - z_top) <= cfg_.mask_band_m)
      band.ptr<uint8_t>()[idx_[n]] = 255;

  // ONE component: the band still holds anything else at cube height, and the
  // cube is the connected thing the camera is pointed at.
  cv::Mat cc, stats, centroids;
  const int n_cc =
      cv::connectedComponentsWithStats(band, cc, stats, centroids, 8);
  if (n_cc < 2) {
    s.reason = "no_blob";
    return s;
  }
  int best_cc = 1, best_area = 0;
  for (int c = 1; c < n_cc; ++c) {
    const int a = stats.at<int>(c, cv::CC_STAT_AREA);
    if (a > best_area) {
      best_area = a;
      best_cc = c;
    }
  }
  cv::Mat mask = (cc == best_cc);
  if (cfg_.mask_erode > 0)
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), cfg_.mask_erode);

  // Keep only the live points the mask survived — no second unprojection.
  int n_top = 0;
  std::vector<int> vote(NUM_COLORS, 0);
  for (int n = 0; n < n_live; ++n) {
    if (!mask.ptr<uint8_t>()[idx_[n]]) continue;
    std::copy(&pts_[n * 3], &pts_[n * 3] + 3, &pts_[n_top * 3]);
    ++vote[labels_.ptr<int16_t>()[idx_[n]]];
    ++n_top;
  }
  if (n_top < min_px) {
    s.reason = "no_blob";
    return s;
  }

  std::vector<cv::Point2f> xy(n_top);
  std::vector<float> zt(n_top);
  for (int n = 0; n < n_top; ++n) {
    xy[n] = {pts_[n * 3], pts_[n * 3 + 1]};
    zt[n] = pts_[n * 3 + 2];
  }
  const cv::RotatedRect rect = cv::minAreaRect(xy);
  const float vis = std::min(rect.size.width, rect.size.height) / edge;
  const float big = std::max(rect.size.width, rect.size.height) / edge;
  const float up_dot = std::fabs(plane_normal(pts_.data(), n_top)[2]);
  const int color = static_cast<int>(
      std::max_element(vote.begin(), vote.end()) - vote.begin());

  // Every colour that got a vote, sharing the one geometry — a calibration bag
  // has to show what the face was SPLIT into, not only what won.
  for (int c = 0; c < NUM_COLORS; ++c)
    if (vote[c] > 0)
      last_.push_back(
          {c, vote[c], up_dot, vis, big, rect.center.x, rect.center.y});

  if (up_dot < cfg_.up_dot_min) {
    s.reason = "side_face";
    return s;
  }
  if (big > cfg_.big_max) {
    s.reason = "too_big";
    return s;
  }
  if (vis < cfg_.color_gate()) {
    s.reason = "cut_face";
    return s;
  }
  s.color_ok = true;
  s.color = color;
  s.pos = {rect.center.x, rect.center.y, median_of(zt) - half_extent_};
  s.phi = fold(rect.angle * static_cast<float>(M_PI) / 180.0f);
  s.n_px = vote[color];
  // The COLOUR is settled by the plane test; only the CENTRE needs the face
  // whole, because a partial view puts the rect centre off by half the missing
  // strip. Same two gates as BLOBS, same reason.
  s.ok = vis >= cfg_.min_visible;
  s.reason = s.ok ? "ok" : "cut_face";
  return s;
}

// ── PLANE: depth first, constrained geometry, colour last ────────────
//
// MASK improved colour association but still made colour classification an
// admission ticket to geometry (`labels >= 0 && valid depth`). On the D435i,
// depth fringe and desaturated face pixels therefore shrink the same physical
// top into a different rectangle from frame to frame. PLANE starts with every
// valid depth point, estimates the floor, and looks exactly one known cube edge
// above it. Only after a square pose exists do RGB labels get a vote.

Sight CubeSight::read_plane(const cv::Mat& depth, int min_px, float edge,
                            const CameraPose& cam) {
  const int total = depth.rows * depth.cols;
  const float* R = cam.R.data();
  Sight s;
  s.reason = "no_plane";
  last_.clear();

  idx_.clear();
  pts_.clear();
  idx_.reserve(total);
  pts_.reserve(static_cast<size_t>(total) * 3);
  std::vector<int> sample_slots;
  sample_slots.reserve(total / 4);
  double color_x = 0.0, color_y = 0.0;
  int color_points = 0;
  const float workspace_max = cfg_.plane_range_max_m + edge;
  for (int i = 0; i < total; ++i) {
    const float z = depth.ptr<float>()[i];
    if (!std::isfinite(z) || z <= cfg_.z_min_m) continue;
    const float* ray = &rays_[static_cast<size_t>(i) * 3];
    const float cx = ray[0] * z, cy = ray[1] * z, cz = ray[2] * z;
    const float x = R[0] * cx + R[1] * cy + R[2] * cz + cam.t[0];
    const float y = R[3] * cx + R[4] * cy + R[5] * cz + cam.t[1];
    const float bz = R[6] * cx + R[7] * cy + R[8] * cz + cam.t[2];
    // The planner only retrieves a cube in front of the robot. Bounding the
    // floor fit to that workspace prevents distant walls from becoming its
    // dominant "horizontal" plane.
    if (x <= cfg_.z_min_m || x >= workspace_max ||
        std::fabs(y) >= workspace_max)
      continue;
    const int slot = static_cast<int>(idx_.size());
    idx_.push_back(i);
    pts_.push_back(x);
    pts_.push_back(y);
    pts_.push_back(bz);
    const int row = i / depth.cols, col = i % depth.cols;
    if ((row & 1) == 0 && (col & 1) == 0) sample_slots.push_back(slot);
    if (labels_.ptr<int16_t>()[i] >= 0) {
      color_x += x;
      color_y += y;
      ++color_points;
    }
  }
  if (static_cast<int>(idx_.size()) < min_px ||
      static_cast<int>(sample_slots.size()) < min_px)
    return s;
  if (color_points > 0)
    s.hint = std::atan2(static_cast<float>(color_y / color_points),
                        static_cast<float>(color_x / color_points));

  const GroundPlane ground =
      fit_ground_plane(pts_, sample_slots, cfg_.plane_floor_quantile,
                       cfg_.plane_ransac_dist_m, cfg_.up_dot_min, min_px);
  if (!ground.ok) return s;

  // Raw support and associated support are different on purpose. Closing says
  // which depth islands belong together; only raw support contributes a 3-D
  // point to the fit.
  cv::Mat band(depth.rows, depth.cols, CV_8UC1, cv::Scalar(0));
  for (size_t slot = 0; slot < idx_.size(); ++slot) {
    const float* p = &pts_[slot * 3];
    const float height =
        ground.n[0] * p[0] + ground.n[1] * p[1] + ground.n[2] * p[2] + ground.d;
    if (std::fabs(height - edge) <= cfg_.plane_top_band_m)
      band.ptr<uint8_t>()[idx_[slot]] = 255;
  }
  if (cv::countNonZero(band) < min_px) {
    s.reason = "no_blob";
    return s;
  }

  cv::Mat associated = band.clone();
  if (cfg_.plane_close_px > 0) {
    const int width = 2 * cfg_.plane_close_px + 1;
    cv::morphologyEx(
        band, associated, cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(width, width)));
  }
  cv::Mat cc, stats, centroids;
  const int n_cc =
      cv::connectedComponentsWithStats(associated, cc, stats, centroids, 8);
  if (n_cc < 2) {
    s.reason = "no_blob";
    return s;
  }

  struct Best {
    bool have = false;
    int support = 0;
    int color = -1;
    int color_px = 0;
    float up = 0.0f, vis = 0.0f, big = 0.0f;
    float cx = 0.0f, cy = 0.0f, z = 0.0f, phi = 0.0f;
  } best;
  int hint_support = 0;
  std::string rejected = "no_blob";
  std::vector<int> component_slots;
  std::vector<cv::Point2f> xy;
  std::vector<float> candidate_pts, candidate_z;

  for (int component = 1; component < n_cc; ++component) {
    if (stats.at<int>(component, cv::CC_STAT_AREA) < min_px) continue;
    component_slots.clear();
    xy.clear();
    for (size_t slot = 0; slot < idx_.size(); ++slot) {
      const int pixel = idx_[slot];
      if (!band.ptr<uint8_t>()[pixel] || cc.ptr<int>()[pixel] != component)
        continue;
      component_slots.push_back(static_cast<int>(slot));
      xy.push_back({pts_[slot * 3], pts_[slot * 3 + 1]});
    }
    if (static_cast<int>(xy.size()) < min_px) continue;
    const SquareFit square = fit_square(xy, edge, cfg_.plane_fit_slack);
    if (!square.ok) continue;

    candidate_pts.clear();
    candidate_z.clear();
    std::array<int, NUM_COLORS> vote{};
    for (size_t i = 0; i < component_slots.size(); ++i) {
      if (!square.keep[i]) continue;
      const int slot = component_slots[i];
      const float* p = &pts_[slot * 3];
      candidate_pts.insert(candidate_pts.end(), p, p + 3);
      candidate_z.push_back(p[2]);
      const int color = labels_.ptr<int16_t>()[idx_[slot]];
      if (color >= 0 && color < NUM_COLORS) ++vote[color];
    }
    const int support = static_cast<int>(candidate_z.size());
    if (support < min_px) continue;
    const float up = std::fabs(plane_normal(candidate_pts.data(), support)[2]);
    const int color = static_cast<int>(
        std::max_element(vote.begin(), vote.end()) - vote.begin());
    const int color_px = vote[color];
    bool wrote = false;
    for (int c = 0; c < NUM_COLORS; ++c)
      if (vote[c] > 0) {
        last_.push_back(
            {c, vote[c], up, square.vis, square.big, square.cx, square.cy});
        wrote = true;
      }
    if (!wrote)
      last_.push_back(
          {-1, support, up, square.vis, square.big, square.cx, square.cy});

    if (support > hint_support) {
      hint_support = support;
      s.hint = std::atan2(square.cy, square.cx);
    }
    const float range = std::hypot(square.cx, square.cy);
    if (range > cfg_.plane_range_max_m) {
      rejected = "out_of_range";
      continue;
    }
    if (up < cfg_.up_dot_min) {
      rejected = "side_face";
      continue;
    }
    if (square.big > cfg_.big_max) {
      rejected = "too_big";
      continue;
    }
    if (!best.have || support > best.support) {
      best.have = true;
      best.support = support;
      best.color = color_px >= min_px ? color : -1;
      best.color_px = color_px;
      best.up = up;
      best.vis = square.vis;
      best.big = square.big;
      best.cx = square.cx;
      best.cy = square.cy;
      best.z = median_of(candidate_z) - half_extent_;
      best.phi = square.phi;
    }
  }

  if (!best.have) {
    s.reason = rejected;
    return s;
  }
  if (cfg_.plane_color_pool) {
    // Geometry has already paid for the square. Project every image ray onto
    // its known top plane and vote RGB inside the fitted footprint, including
    // pixels whose D435i depth is missing at an edge or texture boundary.
    // A small inset keeps side-face/background antialiasing out of the pool.
    std::array<int, NUM_COLORS> vote{};
    const float half = edge * (0.5f - cfg_.plane_color_inset_frac);
    const float cp = std::cos(best.phi), sp = std::sin(best.phi);
    const float camera_height = ground.n[0] * cam.t[0] +
                                ground.n[1] * cam.t[1] +
                                ground.n[2] * cam.t[2] + ground.d;
    for (int pixel = 0; pixel < total; ++pixel) {
      const int color = labels_.ptr<int16_t>()[pixel];
      if (color < 0 || color >= NUM_COLORS) continue;
      const float* ray = &rays_[static_cast<size_t>(pixel) * 3];
      const float dx = R[0] * ray[0] + R[1] * ray[1] + R[2] * ray[2];
      const float dy = R[3] * ray[0] + R[4] * ray[1] + R[5] * ray[2];
      const float dz = R[6] * ray[0] + R[7] * ray[1] + R[8] * ray[2];
      const float denom =
          ground.n[0] * dx + ground.n[1] * dy + ground.n[2] * dz;
      if (std::fabs(denom) < 1e-6f) continue;
      const float lambda = (edge - camera_height) / denom;
      if (lambda <= 0.0f) continue;
      const float x = cam.t[0] + lambda * dx - best.cx;
      const float y = cam.t[1] + lambda * dy - best.cy;
      const float u = cp * x + sp * y;
      const float v = -sp * x + cp * y;
      if (std::fabs(u) <= half && std::fabs(v) <= half) ++vote[color];
    }
    const int pooled = static_cast<int>(
        std::max_element(vote.begin(), vote.end()) - vote.begin());
    best.color_px = vote[pooled];
    best.color = best.color_px >= min_px ? pooled : -1;
    for (int color = 0; color < NUM_COLORS; ++color)
      if (vote[color] > 0)
        last_.push_back({color, vote[color], best.up, best.vis, best.big,
                         best.cx, best.cy});
  }
  s.pos = {best.cx, best.cy, best.z};
  s.phi = best.phi;
  s.n_px = best.color >= 0 ? best.color_px : best.support;
  s.ok = best.vis >= cfg_.min_visible;
  if (best.color >= 0 && best.vis >= cfg_.color_gate()) {
    s.color_ok = true;
    s.color = best.color;
  }
  if (!s.color_ok)
    s.reason = best.vis < cfg_.color_gate() ? "cut_face" : "no_color";
  else
    s.reason = s.ok ? "ok" : "cut_face";
  return s;
}

cv::Mat CubeSight::paint(const cv::Mat& labels) {
  static const cv::Vec3b lut[7] = {
      {30, 30, 30},   {51, 51, 230}, {26, 115, 242}, {51, 230, 51},
      {51, 230, 230}, {230, 51, 51}, {166, 51, 204},  // BGR of _HEX
  };
  cv::Mat out(labels.rows, labels.cols, CV_8UC3);
  for (int r = 0; r < labels.rows; ++r) {
    const int16_t* src = labels.ptr<int16_t>(r);
    cv::Vec3b* dst = out.ptr<cv::Vec3b>(r);
    for (int c = 0; c < labels.cols; ++c)
      dst[c] = lut[std::clamp<int>(src[c], -1, 5) + 1];
  }
  return out;
}

}  // namespace repose
}  // namespace planners
}  // namespace cpp_control
