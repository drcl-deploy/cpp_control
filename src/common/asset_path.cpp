#include "common/asset_path.hpp"

#include <cstdlib>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace cpp_control {
namespace {

namespace fs = std::filesystem;

/// No shell runs for a yaml value, so `~/` is expanded here or not at all.
std::string expand_user(const std::string& p)
{
    if (p.rfind("~/", 0) != 0) return p;
    const char* home = std::getenv("HOME");
    return home ? home + p.substr(1) : p;
}

/// The roots a relative path may name, in order:
///   1. $VIBE_ASSET_ROOT   untracked checkpoints and motion data, rsync'd per box
///   2. <prefix>/share/cpp_control/models   the policies that ship with the package
/// The ament layout is read straight off AMENT_PREFIX_PATH so this stays
/// ROS-free and the selftests keep running without a sourced overlay. First
/// prefix wins, exactly as ament_index resolves it — two prefixes carrying the
/// same package is an overlay, not an ambiguity.
std::vector<fs::path> roots()
{
    std::vector<fs::path> out;
    const char* asset_root = std::getenv("VIBE_ASSET_ROOT");
    if (asset_root && *asset_root) out.emplace_back(asset_root);

    const char* prefixes = std::getenv("AMENT_PREFIX_PATH");
    if (prefixes && *prefixes)
    {
        std::stringstream ss(prefixes);
        std::string prefix;
        while (std::getline(ss, prefix, ':'))
        {
            if (prefix.empty()) continue;
            const fs::path models = fs::path(prefix) / "share" / "cpp_control" / "models";
            std::error_code ec;
            if (fs::is_directory(models, ec)) { out.push_back(models); break; }
        }
    }
    return out;
}

}  // namespace

std::string asset_path(const std::string& p)
{
    if (p.empty()) return p;
    const std::string s = expand_user(p);
    if (s.front() == '/') return s;

    const std::vector<fs::path> search = roots();
    std::vector<fs::path> hits;
    for (const fs::path& root : search)
    {
        std::error_code ec;
        const fs::path full = root / s;
        if (fs::exists(full, ec)) hits.push_back(full);
    }
    if (hits.size() == 1) return hits.front().string();

    // A path that silently resolves to the wrong file reads as tuned and
    // behaves as stock. Name every root that was tried and stop.
    std::string msg = "asset path '" + p + "' ";
    if (search.empty())
    {
        msg += "is relative and no asset root is set. Source setup.sh, which "
               "exports VIBE_ASSET_ROOT, or name the file absolutely.";
    }
    else if (hits.empty())
    {
        msg += "is under none of:";
        for (const fs::path& root : search) msg += "\n  " + (root / s).string();
    }
    else
    {
        msg += "is ambiguous — it exists under:";
        for (const fs::path& hit : hits) msg += "\n  " + hit.string();
        msg += "\nrename one, or name the file absolutely.";
    }
    throw std::runtime_error(msg);
}

}  // namespace cpp_control
