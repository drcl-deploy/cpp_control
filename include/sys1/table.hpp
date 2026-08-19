#pragma once

/// The baked clip alphabet: 78 scalar rows + the reference frames they name.
///
/// Scalars come from vibe `data/sys1_clips.npz` (bench-produced, offline —
/// the bake stays Python). Frames come from either the retargeted dataset
/// (loaded and sliced at boot) or a bundle this class writes itself, so the
/// Orin never parses 78 npz files. Rows are stored in MotionReference wire
/// layout, IL-ordered: writing a reference is then a memcpy.

#include <string>
#include <vector>

namespace cpp_control {
namespace sys1 {

/// One clean single quarter-turn, indexed by its roll delta letter.
struct ClipRow {
  int clip = 0;
  int entry = 0, exit = 0;  ///< GLOBAL library frames (see clip_start)
  int delta = 0;            ///< index into SLOTS
  bool clean = false;
  float qx = 0.f, qy = 0.f, qth = 0.f;  ///< recording's robot SE(2) in the cube
  float ex_phi = 0.f;                   ///< cube spin at entry
  float exit_range = 0.f;               ///< where the clip LEAVES us, metres
  int span_off = 0, span_len = 0;       ///< into `frames_`
};

class ClipTable {
 public:
  /// Scalars only; call one of the frame loaders next.
  static ClipTable load(const std::string& table_npz);

  /// Slice [entry, exit] out of each clip. The file list lives in the library
  /// npz (`motion_files`), not the table; `root` re-roots those absolute paths,
  /// which carry the bench machine's prefix.
  void load_frames_retargeted(const std::string& library_npz,
                              const std::string& root);
  void load_frames_baked(const std::string& bundle_npz);
  void bake(const std::string& out_npz) const;

  /// v7: replace the still pose with the robot's NOMINAL STANCE (`nominal_stand`).
  /// `default_angles_mj` is the manifest's `default_joint_pos` — the same array
  /// sys0 holds in NOMINAL_POSE and the same one mjlab FKs, so the two sides
  /// cannot drift. Call after a frame loader; the still path then never reads
  /// the table again. No-op'd by config, so v5/v6 keep the library frame.
  void set_nominal_stand(const std::vector<float>& default_angles_mj);

  const std::vector<ClipRow>& rows() const { return rows_; }
  /// Clean rows for a delta letter (F | B | L | R); empty if none.
  const std::vector<int>& pool(char delta) const;
  const float* span(int row) const {
    return &frames_[static_cast<size_t>(rows_[row].span_off) * cols_];
  }
  const float* stand_row() const { return stand_.data(); }
  int cols() const { return cols_; }
  float fps() const { return fps_; }
  float half_extent() const { return half_extent_; }
  bool has_frames() const { return !frames_.empty(); }

 private:
  void finish_pools();

  std::vector<ClipRow> rows_;
  std::vector<std::string> motion_files_;
  std::vector<int> clip_start_, clip_len_;
  std::vector<int> pools_[4];  ///< SLOTS index 2..5 -> F B L R
  std::vector<float> frames_;  ///< (N, cols_) wire rows
  std::vector<float> stand_;   ///< (cols_,) the pose a still mode holds
  int stand_frame_ = 0;
  int cols_ = 0;
  float fps_ = 50.f, half_extent_ = 0.3048f;
};

}  // namespace sys1
}  // namespace cpp_control
