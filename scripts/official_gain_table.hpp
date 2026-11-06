#pragma once
// Offline scoring tools and tests only; never linked into eda_router and not part of
// the submission package. Per-case thermal gains G as used for local readback of the
// public cases; the platform passes no G, and the router estimates its own
// (hottest-K bound, --thermal-gain-estimate hottest-k). Unknown names fall back to 20.
#include <filesystem>
#include <map>
#include <string>

namespace eda {
inline double official_thermal_gain_for_def_file(const std::string& path) {
    const auto name=std::filesystem::path(path).filename().string();
    static const std::map<std::string,double> table{
        {"open_3d_case001.input.def",11.684},{"ispd19_test001.input.def",11.684},
        {"aes_sky130hs.input.def",1.719},{"open_3d_case002.input.def",1.719},
        {"open_3d_case003.input.def",4.224},{"ispd19_test004.input.def",4.224},
        {"ispd18_test005.input.def",12.350},{"open_3d_case004.input.def",12.350},
        {"open_3d_case005.input.def",8.323},{"ispd18_test007.input.def",8.323},
        {"ispd18_test010.input.def",18.996},{"open_3d_case006.input.def",18.996},
        {"toy.input.def",21.492}};
    const auto f=table.find(name);return f==table.end()?20.0:f->second;
}
} // namespace eda
