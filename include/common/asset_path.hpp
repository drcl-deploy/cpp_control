#pragma once

/// One resolution rule for every path a config or a launch file names, so a
/// yaml is the same string on the desktop and on the Orin — docs/vibe/assets.md

#include <string>

namespace cpp_control {

/// Absolute form of `p`. "" stays "" (it means "none"); absolute and `~` paths
/// pass through untouched. A relative path is searched under $VIBE_ASSET_ROOT
/// and then the package's installed `models/`, and throws when it is missing
/// from every root or present under more than one.
std::string asset_path(const std::string& p);

}  // namespace cpp_control
