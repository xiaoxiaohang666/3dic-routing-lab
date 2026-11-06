#pragma once

#include "common/types.hpp"
#include "parser/text_tokens.hpp"
#include <algorithm>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace eda {

// Constructs that the public cases do not contain but LEF/DEF allows (official
// Q&A A97/A123: hidden cases may differ). Each is read conservatively instead of
// aborting the whole run; every use is reported once per category on stderr.
class ParseWarnings {
public:
    explicit ParseWarnings(std::string file) : file_(std::move(file)) {}
    void add(const std::string& category, const std::string& detail) {
        if (++counts_[category] <= 3)
            std::cerr << "[WARN] " << file_ << ": " << category << ": " << detail << "\n";
    }
    void summary() const {
        for (const auto& [category, count] : counts_)
            std::cerr << "[WARN] " << file_ << ": " << category << " x" << count << " (read tolerantly)\n";
    }

private:
    std::string file_;
    std::map<std::string, int> counts_;
};

// Rectangles covering a polygon given by its vertices, cut into horizontal
// slabs at every vertex y. Each slab keeps the x intervals inside the polygon at
// the slab's middle height (even-odd rule), so a rectilinear polygon is covered
// exactly and a non-rectilinear one approximately. Degenerate input yields none.
inline std::vector<Rect> polygon_to_rects(const std::vector<Point>& vertices) {
    std::vector<Rect> out;
    if (vertices.size() < 3) return out;
    std::vector<double> ys;
    for (const auto& p : vertices) ys.push_back(p.y);
    std::sort(ys.begin(), ys.end());
    ys.erase(std::unique(ys.begin(), ys.end()), ys.end());
    for (size_t s = 0; s + 1 < ys.size(); ++s) {
        const double y = (ys[s] + ys[s + 1]) * 0.5;
        std::vector<double> xs;
        for (size_t i = 0; i < vertices.size(); ++i) {
            const auto& a = vertices[i];
            const auto& b = vertices[(i + 1) % vertices.size()];
            if ((a.y <= y) == (b.y <= y)) continue;
            xs.push_back(a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y));
        }
        std::sort(xs.begin(), xs.end());
        for (size_t i = 0; i + 1 < xs.size(); i += 2)
            if (xs[i + 1] > xs[i]) out.emplace_back(xs[i], ys[s], xs[i + 1], ys[s + 1]);
    }
    return out;
}

// Reads "x y x y ..." up to ';' (LEF POLYGON body, optional MASK n first).
inline std::vector<Point> read_lef_polygon(TextTokens& t) {
    if (t.peek() == "MASK") { t.take(); t.take(); }
    std::vector<Point> points;
    while (t.peek() != ";") {
        const double x = t.number(), y = t.number();
        points.emplace_back(x, y);
    }
    t.expect(";");
    return points;
}

// After "LAYER name": skips optional qualifiers (SPACING, DESIGNRULEWIDTH,
// EXCEPTPGNET, MASK, ...) through the closing ';'.
inline void skip_to_semicolon(TextTokens& t) {
    while (t.take() != ";") {
        if (t.peek() == "END") throw std::runtime_error("missing semicolon before END");
    }
}

} // namespace eda
