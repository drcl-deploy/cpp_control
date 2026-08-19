#include "cpp_control/planners/repose/sight.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace cpp_control {
namespace planners {
namespace repose {

const Palette PALETTE_SIM = {
    255.f, 105.f, 107.f, 53.f, 11.f, 12.f,   // red
    255.f, 239.f, 53.f,  49.f, 23.f, 5.f,    // orange
    106.f, 255.f, 107.f, 6.f,  28.f, 7.f,    // green
    255.f, 255.f, 107.f, 71.f, 71.f, 16.f,   // yellow
    104.f, 104.f, 255.f, 10.f, 10.f, 51.f,   // blue
    255.f, 107.f, 255.f, 54.f, 13.f, 47.f,   // pink
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
    const int h = std::max(1, static_cast<int>(std::lround(
                                  depth.rows * static_cast<double>(cfg_.proc_width) /
                                  depth.cols)));
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
    // is degenerate on a square — both measured, neither fixable by a threshold.
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
    // partial view puts the rect centre off by half the missing strip (~0.08 m).
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
  const int kth = std::min(n_live - 1,
                           std::max(0, static_cast<int>(cfg_.mask_top_pct * 0.01f *
                                                        (n_live - 1))));
  std::nth_element(zs.begin(), zs.begin() + kth, zs.end());
  const float z_top = zs[kth];

  cv::Mat band(depth.rows, depth.cols, CV_8UC1, cv::Scalar(0));
  for (int n = 0; n < n_live; ++n)
    if (std::fabs(pts_[n * 3 + 2] - z_top) <= cfg_.mask_band_m)
      band.ptr<uint8_t>()[idx_[n]] = 255;

  // ONE component: the band still holds anything else at cube height, and the
  // cube is the connected thing the camera is pointed at.
  cv::Mat cc, stats, centroids;
  const int n_cc = cv::connectedComponentsWithStats(band, cc, stats, centroids, 8);
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
  const int color =
      static_cast<int>(std::max_element(vote.begin(), vote.end()) - vote.begin());

  // Every colour that got a vote, sharing the one geometry — a calibration bag
  // has to show what the face was SPLIT into, not only what won.
  for (int c = 0; c < NUM_COLORS; ++c)
    if (vote[c] > 0)
      last_.push_back({c, vote[c], up_dot, vis, big, rect.center.x, rect.center.y});

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

cv::Mat CubeSight::paint(const cv::Mat& labels) {
  static const cv::Vec3b lut[7] = {
      {30, 30, 30},    {51, 51, 230},   {26, 115, 242}, {51, 230, 51},
      {51, 230, 230},  {230, 51, 51},   {166, 51, 204},  // BGR of _HEX
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
