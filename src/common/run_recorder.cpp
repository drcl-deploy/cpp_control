#include "common/run_recorder.hpp"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "cnpy/cnpy.h"

namespace cpp_control::run_log
{

// ══════════════════════════════════════════════════════════════
//  Json
// ══════════════════════════════════════════════════════════════

std::string Json::quote(const std::string& s)
{
    std::string out = "\"";
    for (const char c : s)
    {
        switch (c)
        {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20)
                {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                }
                else
                {
                    out += c;
                }
        }
    }
    return out + "\"";
}

std::string Json::number(double v)
{
    // JSON has no NaN and no Infinity. A bare `nan` token is accepted by
    // nothing, so an unmeasurable quantity is null and the reader decides.
    if (!std::isfinite(v))
        return "null";
    std::ostringstream ss;
    ss.precision(9);
    ss << v;
    return ss.str();
}

void Json::sep(const std::string& key)
{
    if (!first_)
        body_ += ", ";
    first_ = false;
    body_ += quote(key) + ": ";
}

Json& Json::add(const std::string& key, const std::string& value)
{
    sep(key);
    body_ += quote(value);
    return *this;
}

Json& Json::add(const std::string& key, const char* value)
{
    return add(key, std::string(value ? value : ""));
}

Json& Json::add(const std::string& key, double value)
{
    sep(key);
    body_ += number(value);
    return *this;
}

Json& Json::add(const std::string& key, int value)
{
    sep(key);
    body_ += std::to_string(value);
    return *this;
}

Json& Json::add(const std::string& key, bool value)
{
    sep(key);
    body_ += value ? "true" : "false";
    return *this;
}

Json& Json::add(const std::string& key, const std::vector<double>& value)
{
    sep(key);
    body_ += "[";
    for (size_t i = 0; i < value.size(); ++i)
        body_ += (i ? ", " : "") + number(value[i]);
    body_ += "]";
    return *this;
}

Json& Json::add(const std::string& key, const std::vector<float>& value)
{
    sep(key);
    body_ += "[";
    for (size_t i = 0; i < value.size(); ++i)
        body_ += (i ? ", " : "") + number(static_cast<double>(value[i]));
    body_ += "]";
    return *this;
}

Json& Json::add(const std::string& key, const std::vector<int>& value)
{
    sep(key);
    body_ += "[";
    for (size_t i = 0; i < value.size(); ++i)
        body_ += (i ? ", " : "") + std::to_string(value[i]);
    body_ += "]";
    return *this;
}

Json& Json::add(const std::string& key, const std::vector<std::string>& value)
{
    sep(key);
    body_ += "[";
    for (size_t i = 0; i < value.size(); ++i)
        body_ += (i ? ", " : "") + quote(value[i]);
    body_ += "]";
    return *this;
}

Json& Json::raw(const std::string& key, const std::string& json)
{
    sep(key);
    body_ += json.empty() ? "null" : json;
    return *this;
}

// ══════════════════════════════════════════════════════════════
//  RunRecorder
// ══════════════════════════════════════════════════════════════

RunRecorder::RunRecorder(std::string dir, std::vector<Channel> schema, int max_rows)
    : dir_(std::move(dir)), schema_(std::move(schema)), max_rows_(max_rows)
{
    offsets_.reserve(schema_.size());
    for (const auto& ch : schema_)
    {
        offsets_.push_back(stride_);
        stride_ += ch.width;
    }
    if (stride_ <= 0 || max_rows_ <= 0)
    {
        setup_error_ = "empty schema or zero capacity";
        return;
    }

    try
    {
        std::filesystem::create_directories(dir_);
    }
    catch (const std::exception& e)
    {
        setup_error_ = std::string("cannot create ") + dir_ + ": " + e.what();
        return;
    }
    // Writable, not merely present: cnpy fwrites into whatever fopen returns
    // and does not check it, so a read-only directory would crash the writer
    // thread rather than report anything.
    {
        const std::string probe = dir_ + "/.run_recorder_probe";
        std::ofstream f(probe);
        if (!f)
        {
            setup_error_ = "cannot write in " + dir_;
            return;
        }
        f.close();
        std::error_code ec;
        std::filesystem::remove(probe, ec);
    }

    // Two buffers: the run in progress and a spare, so the hand-off at end()
    // and the take at begin() are both moves rather than allocations.
    const size_t cells = static_cast<size_t>(max_rows_) * static_cast<size_t>(stride_);
    try
    {
        pool_.emplace_back(cells, 0.0f);
        pool_.emplace_back(cells, 0.0f);
    }
    catch (const std::bad_alloc&)
    {
        setup_error_ = "cannot reserve " +
                       std::to_string(2 * cells * sizeof(float) / (1024 * 1024)) +
                       " MB for the run buffers — lower record_max_seconds";
        pool_.clear();
        return;
    }

    ok_ = true;
    writer_ = std::thread([this] { this->writer_main(); });
}

RunRecorder::~RunRecorder()
{
    // An unfinished run is dropped rather than written with no metadata: a file
    // that cannot say which export, which gains and which anchor produced it is
    // not a measurement. Owners end() theirs in their own destructor.
    recording_ = false;
    {
        std::lock_guard<std::mutex> lk(mu_);
        stop_ = true;
    }
    cv_.notify_all();
    if (writer_.joinable())
        writer_.join();
}

int RunRecorder::offset(const std::string& name) const
{
    for (size_t i = 0; i < schema_.size(); ++i)
        if (schema_[i].name == name)
            return offsets_[i];
    throw std::runtime_error("run_log: no channel named '" + name + "'");
}

void RunRecorder::attach(std::string name, std::string blob)
{
    blobs_.emplace_back(std::move(name), std::move(blob));
}

void RunRecorder::begin(std::string run_name)
{
    if (!ok_)
        return;
    if (recording_)
    {
        // Should not happen — the owner ends a run before starting the next —
        // and if it does, the rows already taken are worth more than the
        // metadata they are missing.
        end("{\"note\": \"superseded by a new run\"}", "{}");
    }

    {
        std::lock_guard<std::mutex> lk(mu_);
        if (!pool_.empty())
        {
            active_ = std::move(pool_.back());
            pool_.pop_back();
        }
    }
    const size_t cells = static_cast<size_t>(max_rows_) * static_cast<size_t>(stride_);
    if (active_.size() != cells)
        active_.assign(cells, 0.0f);   // only when the writer is still holding both

    run_name_ = std::move(run_name);
    rows_ = 0;
    truncated_ = false;
    recording_ = true;
}

std::string RunRecorder::end(const std::string& meta_json, const std::string& index_json)
{
    if (!recording_)
        return {};
    recording_ = false;
    if (rows_ == 0)
    {
        std::lock_guard<std::mutex> lk(mu_);
        if (pool_.size() < 2)
            pool_.push_back(std::move(active_));
        active_.clear();
        return {};
    }

    Job job;
    job.path = dir_ + "/" + run_name_ + ".npz";
    job.rows = rows_;
    job.meta = meta_json;
    job.index = index_json;
    job.data = std::move(active_);   // O(1); active_ is refilled by the next begin()
    active_.clear();
    const std::string path = job.path;

    {
        std::lock_guard<std::mutex> lk(mu_);
        queue_.push_back(std::move(job));
    }
    cv_.notify_one();
    return path;
}

std::string RunRecorder::take_write_error()
{
    std::lock_guard<std::mutex> lk(mu_);
    std::string e;
    e.swap(write_error_);
    return e;
}

void RunRecorder::writer_main()
{
    for (;;)
    {
        Job job;
        {
            std::unique_lock<std::mutex> lk(mu_);
            cv_.wait(lk, [this] { return stop_ || !queue_.empty(); });
            if (queue_.empty())
                return;   // stop_ and drained
            job = std::move(queue_.front());
            queue_.pop_front();
        }

        try
        {
            write_job(job);
        }
        catch (const std::exception& e)
        {
            std::lock_guard<std::mutex> lk(mu_);
            if (write_error_.empty())
                write_error_ = job.path + ": " + e.what();
        }

        // Give the buffer back with its size intact: begin() then takes it
        // without touching a page, and row() zeroes what it hands out anyway.
        std::lock_guard<std::mutex> lk(mu_);
        if (pool_.size() < 2)
            pool_.push_back(std::move(job.data));
    }
}

void RunRecorder::write_job(const Job& job)
{
    // Written aside and renamed, so a reader never opens a half-written npz and
    // an interrupted write leaves a .tmp rather than a corrupt run.
    const std::string tmp = job.path + ".tmp";
    {
        std::ofstream probe(tmp, std::ios::binary | std::ios::trunc);
        if (!probe)
            throw std::runtime_error("cannot open for writing");
    }

    const size_t rows = static_cast<size_t>(job.rows);
    std::vector<float> column;
    bool first = true;
    for (size_t c = 0; c < schema_.size(); ++c)
    {
        const int width = schema_[c].width;
        const int off = offsets_[c];
        column.resize(rows * static_cast<size_t>(width));
        for (size_t r = 0; r < rows; ++r)
            std::memcpy(column.data() + r * static_cast<size_t>(width),
                        job.data.data() + r * static_cast<size_t>(stride_) +
                            static_cast<size_t>(off),
                        sizeof(float) * static_cast<size_t>(width));

        const std::vector<size_t> shape =
            width == 1 ? std::vector<size_t>{rows}
                       : std::vector<size_t>{rows, static_cast<size_t>(width)};
        cnpy::npz_save(tmp, schema_[c].name, column.data(), shape, first ? "w" : "a");
        first = false;
    }

    const auto save_text = [&](const std::string& name, const std::string& text) {
        std::vector<size_t> shape{text.size()};
        cnpy::npz_save(tmp, name, reinterpret_cast<const unsigned char*>(text.data()), shape,
                       first ? "w" : "a");
        first = false;
    };
    save_text("meta_json", job.meta);
    for (const auto& blob : blobs_)
        save_text(blob.first, blob.second);

    std::filesystem::rename(tmp, job.path);

    // One line per run, so a session is a table without opening any of them.
    const std::string base = std::filesystem::path(job.path).filename().string();
    std::string line = "{" + Json::quote("file") + ": " + Json::quote(base) + ", " +
                       Json::quote("rows") + ": " + std::to_string(job.rows);
    if (job.index.size() > 2 && job.index.front() == '{' && job.index.back() == '}')
        line += ", " + job.index.substr(1, job.index.size() - 2);
    line += "}";
    std::ofstream index(dir_ + "/index.jsonl", std::ios::app);
    if (index)
        index << line << "\n";
}

}  // namespace cpp_control::run_log
