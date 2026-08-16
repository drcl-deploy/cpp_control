#include "sys1/table.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <numeric>
#include <stdexcept>

#include <zlib.h>

#include "cnpy/cnpy.h"
#include "common/g1/joint_orders.hpp"
#include "common/g1/motion.hpp"
#include "sys1/cfg.hpp"

namespace cpp_control {
namespace sys1 {

namespace {

// ── A small npz reader ───────────────────────────────────────────
//
// cnpy cannot open this artifact at all: numpy writes ZIP64 local headers
// (0xFFFFFFFF sizes + a 20-byte extra field) and cnpy reads the sentinel as the
// length. It also cannot parse the `object` dtype that `motion_files` uses. So
// the table brings its own reader — 60 lines, no dtype surprises, and cnpy
// stays exactly as it is for the motion clips it already reads fine.

/// Raw (inflated) bytes of a zip member. Empty if absent.
std::vector<char> zip_member(const std::string& path, const std::string& name) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("sys1 table: cannot open " + path);
  const std::string want = name + ".npy";
  std::vector<char> header(30);
  while (in.read(header.data(), 30)) {
    if (static_cast<uint8_t>(header[2]) != 0x03 ||
        static_cast<uint8_t>(header[3]) != 0x04)
      break;  // central directory: no more members
    uint16_t name_len, extra_len, method;
    uint32_t c32, u32;
    std::memcpy(&method, header.data() + 8, 2);
    std::memcpy(&c32, header.data() + 18, 4);
    std::memcpy(&u32, header.data() + 22, 4);
    std::memcpy(&name_len, header.data() + 26, 2);
    std::memcpy(&extra_len, header.data() + 28, 2);
    std::string member(name_len, '\0');
    in.read(member.data(), name_len);

    uint64_t compressed = c32, uncompressed = u32;
    std::vector<char> extra(extra_len);
    if (extra_len) in.read(extra.data(), extra_len);
    for (size_t at = 0; at + 4 <= extra.size();) {  // ZIP64 extended info
      uint16_t tag, len;
      std::memcpy(&tag, extra.data() + at, 2);
      std::memcpy(&len, extra.data() + at + 2, 2);
      if (tag == 0x0001 && len >= 16) {
        std::memcpy(&uncompressed, extra.data() + at + 4, 8);
        std::memcpy(&compressed, extra.data() + at + 12, 8);
      }
      at += 4 + len;
    }

    if (member != want) {
      in.seekg(static_cast<std::streamoff>(compressed), std::ios::cur);
      continue;
    }
    std::vector<char> raw(compressed);
    in.read(raw.data(), static_cast<std::streamsize>(compressed));
    if (method == 0) return raw;
    std::vector<char> out(uncompressed);
    z_stream zs{};
    zs.next_in = reinterpret_cast<Bytef*>(raw.data());
    zs.avail_in = static_cast<uInt>(compressed);
    zs.next_out = reinterpret_cast<Bytef*>(out.data());
    zs.avail_out = static_cast<uInt>(uncompressed);
    if (inflateInit2(&zs, -MAX_WBITS) != Z_OK)
      throw std::runtime_error("sys1 table: inflate init failed for " + name);
    const int rc = inflate(&zs, Z_FINISH);
    inflateEnd(&zs);
    if (rc != Z_STREAM_END)
      throw std::runtime_error("sys1 table: corrupt member '" + name + "'");
    return out;
  }
  return {};
}

/// One decoded .npy member: numeric payload plus what it is.
struct Npy {
  std::vector<char> blob;
  size_t off = 0, word = 0, count = 1;
  char kind = 0;  ///< 'i' 'u' 'f' 'b' — 'O' never reaches here
};

Npy npy_parse(std::vector<char> blob, const std::string& what) {
  if (blob.size() < 12 || std::memcmp(blob.data(), "\x93NUMPY", 6) != 0)
    throw std::runtime_error("sys1 table: '" + what + "' is not a .npy member");
  const auto major = static_cast<uint8_t>(blob[6]);
  size_t head_len, at;
  if (major == 1) {
    uint16_t n;
    std::memcpy(&n, blob.data() + 8, 2);
    head_len = n;
    at = 10;
  } else {
    uint32_t n;
    std::memcpy(&n, blob.data() + 8, 4);
    head_len = n;
    at = 12;
  }
  const std::string head(blob.data() + at, head_len);
  const size_t d = head.find("'descr'");
  const size_t q = head.find('\'', head.find(':', d) + 1);
  const std::string descr = head.substr(q + 1, head.find('\'', q + 1) - q - 1);
  if (descr.size() < 2)
    throw std::runtime_error("sys1 table: '" + what + "' has no dtype");

  Npy out;
  out.kind = descr[1];
  out.word = descr.size() > 2 ? std::stoul(descr.substr(2)) : 0;
  const size_t s = head.find('(', head.find("'shape'"));
  for (size_t i = s + 1; i < head.size() && head[i] != ')'; ++i)
    if (std::isdigit(static_cast<unsigned char>(head[i]))) {
      size_t used = 0;
      out.count *= std::stoul(head.substr(i), &used);
      i += used - 1;
    }
  out.blob = std::move(blob);
  out.off = at + head_len;
  return out;
}

Npy need(const std::string& path, const char* key) {
  auto blob = zip_member(path, key);
  if (blob.empty())
    throw std::runtime_error("sys1 table: '" + std::string(key) +
                             "' missing in " + path);
  return npy_parse(std::move(blob), key);
}

template <typename T>
const T* at(const Npy& a) {
  return reinterpret_cast<const T*>(a.blob.data() + a.off);
}

std::vector<float> floats(const Npy& a) {
  std::vector<float> out(a.count);
  if (a.kind == 'f' && a.word == 4)
    std::copy(at<float>(a), at<float>(a) + a.count, out.begin());
  else if (a.kind == 'f' && a.word == 8)
    for (size_t i = 0; i < a.count; ++i) out[i] = static_cast<float>(at<double>(a)[i]);
  else
    throw std::runtime_error("sys1 table: expected a float array");
  return out;
}

std::vector<int> ints(const Npy& a) {
  std::vector<int> out(a.count);
  if ((a.kind == 'i' || a.kind == 'u') && a.word == 4)
    std::copy(at<int32_t>(a), at<int32_t>(a) + a.count, out.begin());
  else if ((a.kind == 'i' || a.kind == 'u') && a.word == 8)
    for (size_t i = 0; i < a.count; ++i) out[i] = static_cast<int>(at<int64_t>(a)[i]);
  else
    throw std::runtime_error("sys1 table: expected an integer array");
  return out;
}

std::vector<uint8_t> bools(const Npy& a) {
  if (a.kind != 'b')
    throw std::runtime_error("sys1 table: expected a bool array");
  return {at<uint8_t>(a), at<uint8_t>(a) + a.count};
}

/// npz object arrays hold pickled python strings; the paths we need are plain
/// ASCII, so pull them out of the pickle stream by their known prefix.
std::vector<std::string> object_strings(const std::vector<char>& blob,
                                        size_t count) {
  const char* p = blob.data();
  const size_t n = blob.size();
  std::vector<std::string> out;
  for (size_t i = 0; i + 1 < n && out.size() < count; ++i) {
    if (p[i] != '/') continue;
    size_t j = i;
    while (j < n && p[j] >= 0x20 && p[j] < 0x7f) ++j;
    const std::string s(p + i, j - i);
    if (s.size() > 8 && s.rfind(".npz") == s.size() - 4) out.push_back(s);
    i = j;
  }
  return out;
}

/// Re-root a baked absolute path onto this machine's dataset copy.
std::string reroot(const std::string& path, const std::string& root) {
  if (root.empty() || std::ifstream(path).good()) return path;
  static const std::string kMark = "retargeted_motions/";
  const size_t at = path.find(kMark);
  const std::string tail =
      at == std::string::npos ? path : path.substr(at + kMark.size());
  return root.back() == '/' ? root + tail : root + "/" + tail;
}

}  // namespace

ClipTable ClipTable::load(const std::string& path) {
  ClipTable t;
  t.cols_ = g1::WIRE_COLS_FULL;

  const auto clip = ints(need(path, "clip"));
  const auto entry = ints(need(path, "entry"));
  const auto exit_ = ints(need(path, "exit"));
  const auto delta = ints(need(path, "delta"));
  const auto clip_end = ints(need(path, "clip_end"));
  const auto qx = floats(need(path, "qx"));
  const auto qy = floats(need(path, "qy"));
  const auto qth = floats(need(path, "qth"));
  const auto ex_phi = floats(need(path, "ex_phi"));
  const auto exit_range = floats(need(path, "exit_range"));
  const auto clean = bools(need(path, "clean"));
  const size_t n = clip.size();
  if (clean.size() != n)
    throw std::runtime_error("sys1 table: 'clean' must be (N,) bool: " + path);

  t.rows_.resize(n);
  for (size_t i = 0; i < n; ++i)
    t.rows_[i] = {clip[i],       entry[i],   exit_[i], delta[i],
                  clean[i] != 0, qx[i],      qy[i],    qth[i],
                  ex_phi[i],     exit_range[i], 0,     0};

  // clip_end is cumulative, so it names every clip's span in one pass.
  t.clip_start_.resize(clip_end.size());
  t.clip_len_.resize(clip_end.size());
  for (size_t c = 0; c < clip_end.size(); ++c) {
    t.clip_start_[c] = c == 0 ? 0 : clip_end[c - 1];
    t.clip_len_[c] = clip_end[c] - t.clip_start_[c];
  }

  t.stand_frame_ = ints(need(path, "stand_frame"))[0];
  t.half_extent_ = floats(need(path, "half_extent"))[0];
  t.fps_ = floats(need(path, "fps"))[0];

  t.finish_pools();
  return t;
}

void ClipTable::finish_pools() {
  for (auto& p : pools_) p.clear();
  for (size_t i = 0; i < rows_.size(); ++i) {
    const int d = rows_[i].delta;
    if (rows_[i].clean && d >= 2 && d <= 5) pools_[d - 2].push_back(static_cast<int>(i));
  }
}

const std::vector<int>& ClipTable::pool(char delta) const {
  static const std::vector<int> kEmpty;
  const char* at = std::strchr(SLOTS, delta);
  if (!at) return kEmpty;
  const int s = static_cast<int>(at - SLOTS);
  return (s >= 2 && s <= 5) ? pools_[s - 2] : kEmpty;
}

// ── Frames ───────────────────────────────────────────────────────
//
// One wire row per frame: jp29 | jv29 | apos3 | aquat4 | alin3 | aang3 |
// contact12, IL-ordered — exactly what MotionReference carries, so publishing
// a plan is a memcpy and the warp is one scalar on the side.

namespace {

void write_row(const g1::Motion& m, int f, float* row) {
  const int J = g1::NUM_JOINTS;
  m.jp_il(f, row);
  m.jv_il(f, row + J);
  const auto pos = m.root_pos(f);
  const auto quat = m.root_quat(f);
  std::copy(pos.begin(), pos.end(), row + 2 * J);
  std::copy(quat.begin(), quat.end(), row + 2 * J + 3);
  const size_t base = static_cast<size_t>(f) * m.num_bodies * 3;
  std::copy(&m.body_lin_vel_w[base], &m.body_lin_vel_w[base] + 3,
            row + g1::WIRE_COLS_MIN);
  std::copy(&m.body_ang_vel_w[base], &m.body_ang_vel_w[base] + 3,
            row + g1::WIRE_COLS_MIN + 3);
  std::copy(m.contact(f), m.contact(f) + g1::NUM_CONTACT_BODIES,
            row + g1::WIRE_COLS_MIN + 6);
}

}  // namespace

void ClipTable::load_frames_retargeted(const std::string& library_npz,
                                       const std::string& root) {
  motion_files_ = object_strings(zip_member(library_npz, "motion_files"),
                                 clip_start_.size());
  if (motion_files_.size() != clip_start_.size())
    throw std::runtime_error(
        "sys1 table: " + library_npz + " lists " +
        std::to_string(motion_files_.size()) + " motion files, the table has " +
        std::to_string(clip_start_.size()) + " clips");

  frames_.clear();
  stand_.assign(cols_, 0.0f);
  const int stand_clip = static_cast<int>(
      std::upper_bound(clip_start_.begin(), clip_start_.end(), stand_frame_) -
      clip_start_.begin() - 1);

  std::vector<float> scratch(cols_);
  for (size_t c = 0; c < motion_files_.size(); ++c) {
    // Only clips that own a needed span (or the stand pose) get opened.
    std::vector<int> mine;
    for (size_t i = 0; i < rows_.size(); ++i)
      if (rows_[i].clip == static_cast<int>(c) && rows_[i].clean)
        mine.push_back(static_cast<int>(i));
    if (mine.empty() && static_cast<int>(c) != stand_clip) continue;

    const std::string path = reroot(motion_files_[c], root);
    const g1::Motion m = g1::Motion::from_npz(path, g1::MJ2IL);
    if (m.num_frames != clip_len_[c])
      throw std::runtime_error("sys1 table: " + path + " has " +
                               std::to_string(m.num_frames) +
                               " frames, table says " +
                               std::to_string(clip_len_[c]));

    for (int i : mine) {
      ClipRow& r = rows_[i];
      const int lo = r.entry - clip_start_[c], hi = r.exit - clip_start_[c];
      if (lo < 0 || hi >= m.num_frames || hi < lo)
        throw std::runtime_error("sys1 table: row " + std::to_string(i) +
                                 " span is outside clip " + std::to_string(c));
      r.span_off = static_cast<int>(frames_.size() / cols_);
      r.span_len = hi - lo + 1;
      frames_.resize(frames_.size() + static_cast<size_t>(r.span_len) * cols_);
      for (int f = 0; f < r.span_len; ++f)
        write_row(m, lo + f, &frames_[(static_cast<size_t>(r.span_off) + f) * cols_]);
    }
    if (static_cast<int>(c) == stand_clip)
      write_row(m, stand_frame_ - clip_start_[c], stand_.data());
  }
}

void ClipTable::load_frames_baked(const std::string& path) {
  const auto off = ints(need(path, "span_off"));
  const auto len = ints(need(path, "span_len"));
  const auto rows = need(path, "rows");
  if (off.size() != rows_.size() || len.size() != rows_.size())
    throw std::runtime_error("sys1 bundle: row count != table row count: " +
                             path);
  if (rows.count % static_cast<size_t>(cols_) != 0)
    throw std::runtime_error("sys1 bundle: rows are not a multiple of " +
                             std::to_string(cols_) + " wide: " + path);
  frames_ = floats(rows);
  stand_ = floats(need(path, "stand"));
  if (static_cast<int>(stand_.size()) != cols_)
    throw std::runtime_error("sys1 bundle: stand row is the wrong width");
  for (size_t i = 0; i < rows_.size(); ++i) {
    rows_[i].span_off = off[i];
    rows_[i].span_len = len[i];
  }
}

// The v7 still pose, and the whole of v7. `g1::Motion::stand` is already the
// STAND-mode reference builder, and its channels ARE the sim's `_fk_nominal`
// one for one: nominal joints, zero velocity, IDENTITY root quat, zero twist,
// zero contact. So there is no new pose math here — only a different source.
//
// Root POSITION is left at the origin and is inert: no observation port reads a
// reference position (§1 F1), and `MotionClock::engage` rebases the heading
// onto the live robot, which is the C++ form of the sim's `still_yaw` latch.
void ClipTable::set_nominal_stand(const std::vector<float>& default_angles_mj) {
  if (cols_ == 0 || stand_.size() != static_cast<size_t>(cols_))
    throw std::runtime_error(
        "sys1 table: load frames before setting the nominal stand");
  if (default_angles_mj.size() != static_cast<size_t>(g1::NUM_JOINTS))
    throw std::runtime_error("sys1 table: nominal stance is not " +
                             std::to_string(g1::NUM_JOINTS) + " MJ joints");
  write_row(g1::Motion::stand(default_angles_mj, fps_), 0, stand_.data());
  nominal_stand_ = true;
}

void ClipTable::bake(const std::string& out) const {
  if (frames_.empty()) throw std::runtime_error("sys1 bake: no frames loaded");
  std::vector<int32_t> off(rows_.size()), len(rows_.size());
  for (size_t i = 0; i < rows_.size(); ++i) {
    off[i] = rows_[i].span_off;
    len[i] = rows_[i].span_len;
  }
  const size_t n = frames_.size() / cols_;
  cnpy::npz_save(out, "rows", frames_.data(), {n, static_cast<size_t>(cols_)}, "w");
  cnpy::npz_save(out, "stand", stand_.data(), {static_cast<size_t>(cols_)}, "a");
  cnpy::npz_save(out, "span_off", off.data(), {off.size()}, "a");
  cnpy::npz_save(out, "span_len", len.data(), {len.size()}, "a");
}

}  // namespace sys1
}  // namespace cpp_control
