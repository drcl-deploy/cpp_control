#pragma once

// Per-control-step run logging, for measuring what changes between a policy's
// sim2sim run and the same policy's run on a robot.
//
// WHY THIS IS IN THE CONTROLLER AND NOT ON THE WIRE
//
// scripts/record_policy_actions.py makes the opposite choice deliberately: it
// recovers the actions from /lowcmd so that measuring a run cannot change it.
// That works because an action IS on the wire. A tracking error is not. The
// reference the policy was given is a clip placed on the robot by an anchor
// resolved at the instant `A` was pressed, in a frame the node chose, at a step
// index the node owns -- none of which is published, and none of which can be
// reconstructed afterwards from a bag. Neither can the world pose the policy
// actually observed: it has been through a staleness check, a finite difference
// over the measured mocap interval, and possibly a body->world rotation.
//
// The second reason is alignment. Off the wire, a state stream and a command
// stream have to be joined by timestamp, and a dropped message silently deletes
// a control step. Here one row IS one control step: the observation, the action,
// the reference, the measured state and the command that went out are the same
// tick by construction, and a step that did not happen leaves a visible gap in
// `t_wall` and `state_tick`.
//
// WHAT IT COSTS THE CONTROL LOOP, which is the part that matters on hardware:
//
//   * nothing is allocated after begin(). The whole run is written into one
//     buffer reserved up front, and a run that outgrows it stops appending and
//     says so (`truncated`) rather than reallocating under a walking robot.
//   * nothing is serialised in the control thread. end() moves the buffer to a
//     writer thread and returns; the npz is written, and the file system
//     touched, on that thread. This matters because a run ends on the tick the
//     clip does, which is the tick the rest state has to catch a robot at
//     ~1 m/s -- exactly where a 30 ms fwrite must not be.
//   * one row is a memset and a few dozen stores, at the control rate.
//
// THE FILE. One npz per run, self-contained:
//
//   <channel>    (rows, width) float32, one array per channel, in the order the
//                schema declares. Width-1 channels are written 1-D.
//   meta_json    uint8, the run's metadata -- everything needed to interpret
//                the columns and to know which export, which gains, which world
//                source and which parameters produced them.
//   <blob>       uint8, whatever the caller attached verbatim (the difftrack
//                tracker attaches the export's own difftrack_config.json, so a
//                run can be analysed without the model directory beside it).
//
// and one line appended to `index.jsonl` in the same directory per run, so a
// session is greppable without opening every file.

#include <condition_variable>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace cpp_control::run_log
{

/// One named block of consecutive float columns in a row.
struct Channel
{
    std::string name;
    int width = 1;
};

/**
 * A JSON object, built key by key.
 *
 * Small on purpose: the metadata is written once per run and read by numpy, so
 * a hand-rolled emitter is cheaper than adding a JSON library to a package that
 * parses its JSON with yaml-cpp. Non-finite doubles are emitted as `null`,
 * because JSON has no NaN and a bare `nan` token is what makes a metrics file
 * unreadable three weeks later.
 */
class Json
{
public:
    Json& add(const std::string& key, const std::string& value);
    Json& add(const std::string& key, const char* value);
    Json& add(const std::string& key, double value);
    Json& add(const std::string& key, float value) { return add(key, static_cast<double>(value)); }
    Json& add(const std::string& key, int value);
    Json& add(const std::string& key, long value) { return add(key, static_cast<int>(value)); }
    Json& add(const std::string& key, bool value);
    Json& add(const std::string& key, const std::vector<double>& value);
    Json& add(const std::string& key, const std::vector<float>& value);
    Json& add(const std::string& key, const std::vector<int>& value);
    Json& add(const std::string& key, const std::vector<std::string>& value);
    /// A value that is already JSON -- a nested object, or another Json's str().
    Json& raw(const std::string& key, const std::string& json);

    std::string str() const { return body_ + "}"; }

    /// Escape a string into a JSON string literal, quotes included.
    static std::string quote(const std::string& s);
    /// A finite number, or `null`.
    static std::string number(double v);

private:
    void sep(const std::string& key);
    std::string body_ = "{";
    bool first_ = true;
};

/**
 * A fixed-schema, fixed-capacity run log with a background writer.
 *
 * Lifecycle, all from the control thread:
 *
 *     begin("<run name>")            take a buffer, start counting rows
 *     float* r = row(); ... commit() once per control step
 *     end(meta_json, index_json)     hand it to the writer, get the path back
 *
 * `row()` returns nullptr when nothing is being recorded or the run has filled
 * its buffer, so the call site is one `if` and never a special case.
 */
class RunRecorder
{
public:
    /// @p max_rows is the hard capacity of one run. Two buffers of
    /// max_rows * stride floats are reserved here, so that neither begin() nor
    /// end() has to allocate while a robot is moving.
    RunRecorder(std::string dir, std::vector<Channel> schema, int max_rows);
    ~RunRecorder();

    RunRecorder(const RunRecorder&) = delete;
    RunRecorder& operator=(const RunRecorder&) = delete;

    /// False when the output directory could not be made; error() says why and
    /// every other call is then a no-op.
    bool ok() const { return ok_; }
    const std::string& error() const { return setup_error_; }

    const std::string& dir() const { return dir_; }
    int stride() const { return stride_; }
    int max_rows() const { return max_rows_; }
    const std::vector<Channel>& schema() const { return schema_; }

    /// Column offset of @p name, for the put() helpers. A name the schema does
    /// not have throws: resolved once at startup, it is a typo; resolved on the
    /// fly it would be a column written silently into the wrong place.
    int offset(const std::string& name) const;

    /// A blob copied verbatim into every run file. Call before the first
    /// begin(); @p name becomes the npz key (`.npy` is appended by the writer,
    /// so read it back as `bytes(z["name"]).decode()`).
    void attach(std::string name, std::string blob);

    void begin(std::string run_name);
    bool recording() const { return recording_; }
    int rows() const { return rows_; }
    /// True when the run hit max_rows and stopped appending.
    bool truncated() const { return truncated_; }

    /// A zeroed row, or nullptr. Never allocates, never blocks, never throws.
    float* row()
    {
        if (!recording_)
            return nullptr;
        if (rows_ >= max_rows_)
        {
            truncated_ = true;
            return nullptr;
        }
        float* r = active_.data() + static_cast<size_t>(rows_) * static_cast<size_t>(stride_);
        std::memset(r, 0, sizeof(float) * static_cast<size_t>(stride_));
        return r;
    }
    void commit() { ++rows_; }

    /// Close the run and queue it for writing. Returns the path the file will
    /// appear at, or "" when nothing was recorded.
    std::string end(const std::string& meta_json, const std::string& index_json);

    /// The first write error since the last call, and clears it. Poll it from
    /// somewhere that can log -- the writer thread has no logger.
    std::string take_write_error();

private:
    struct Job
    {
        std::string path;
        std::vector<float> data;
        int rows = 0;
        std::string meta;
        std::string index;
    };

    void writer_main();
    void write_job(const Job& job);

    std::string dir_;
    std::vector<Channel> schema_;
    std::vector<int> offsets_;
    std::vector<std::pair<std::string, std::string>> blobs_;
    int stride_ = 0;
    int max_rows_ = 0;
    bool ok_ = false;
    std::string setup_error_;

    // --- control thread only ---
    std::vector<float> active_;
    std::string run_name_;
    bool recording_ = false;
    bool truncated_ = false;
    int rows_ = 0;

    // --- shared with the writer ---
    std::mutex mu_;
    std::condition_variable cv_;
    std::deque<Job> queue_;
    std::vector<std::vector<float>> pool_;
    std::string write_error_;
    bool stop_ = false;
    std::thread writer_;
};

// ── row writers ───────────────────────────────────────────────
//
// Free functions rather than members: the row pointer is the hot object and
// there is nothing for a method call to look up.

inline void put(float* row, int off, double v) { row[off] = static_cast<float>(v); }
inline void put(float* row, int off, bool v) { row[off] = v ? 1.0f : 0.0f; }
inline void put(float* row, int off, int v) { row[off] = static_cast<float>(v); }

inline void put(float* row, int off, const float* v, int n)
{
    std::memcpy(row + off, v, sizeof(float) * static_cast<size_t>(n));
}
inline void put(float* row, int off, const double* v, int n)
{
    for (int i = 0; i < n; ++i)
        row[off + i] = static_cast<float>(v[i]);
}

}  // namespace cpp_control::run_log
