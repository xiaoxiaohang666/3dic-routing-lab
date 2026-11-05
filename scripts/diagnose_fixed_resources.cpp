// Offline semantic discrimination of fixed-resource hypotheses against an official
// sparse overflow CSV for one Guide. Never linked into routing or score acceptance.
//
// Every hypothesis is a purely geometric rule applied to all inputs. For each cell-layer
//   demand = fixed(model) + route(guide)
// is compared with the CSV. Non-listed cells must not overflow. A model that reproduces
// the map exactly is still an observation on this sample, not an official rule.
#include "objectives/official_metrics.hpp"
#include "objectives/net_cell_contribution.hpp"
#include "scoring/compat_validation.hpp"
#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <unordered_map>

using namespace eda;
using Span = std::pair<uint32_t, uint32_t>;
namespace {
struct Shape { Rect rect; bool pin; };
enum Boundary { CLOSED, OPEN, BOUNDARIES };
enum Fallback { NONE, NEXT, PREV, NEAREST_IN_CELL, NEXT_INSIDE, PREV_INSIDE, VIRTUAL_ADD, CELL_VIRTUAL,
                WHOLE_NEXT, WHOLE_PREV, FALLBACKS };
const std::array<const char*, BOUNDARIES> boundary_names = {"closed", "open"};
const std::array<const char*, FALLBACKS> fallback_names = {"none", "next", "prev", "nearest_in_cell", "next_inside",
    "prev_inside", "virtual_add", "cell_virtual", "whole_next", "whole_prev"};

struct CellEvents {
    std::array<std::vector<Span>, BOUNDARIES> pin, obs;
    std::array<std::array<std::vector<Span>, FALLBACKS>, BOUNDARIES> floor;
    std::array<uint32_t, BOUNDARIES> zeros{};
};
struct Model { int pin_boundary, obs_boundary, fallback; std::vector<uint16_t> fixed; };

std::pair<int, int> cells(double low, double high, const std::vector<double>& lines) {
    if (high < low) std::swap(high, low);
    const int n = int(lines.size()) - 1;
    if (high < lines.front() || low > lines.back()) return {0, 0};
    if (high == low) {
        const int c = std::clamp(int(std::upper_bound(lines.begin(), lines.end(), low) - lines.begin()) - 1, 0, n - 1);
        return {c, c + 1};
    }
    if (high <= lines.front() || low >= lines.back()) return {0, 0};
    return {std::clamp(int(std::upper_bound(lines.begin(), lines.end(), low) - lines.begin()) - 1, 0, n),
            std::clamp(int(std::lower_bound(lines.begin(), lines.end(), high) - lines.begin()), 0, n)};
}
uint32_t merged(std::vector<Span> spans) {
    if (spans.empty()) return 0;
    std::sort(spans.begin(), spans.end());
    auto [b, e] = spans.front();
    uint32_t count = 0;
    for (size_t i = 1; i < spans.size(); ++i) {
        auto [nb, ne] = spans[i];
        if (nb <= e) e = std::max(e, ne);
        else { count += e - b; b = nb; e = ne; }
    }
    return count + e - b;
}
std::vector<std::string> fields(const std::string& line) {
    std::vector<std::string> v;
    std::stringstream in(line);
    std::string word;
    while (std::getline(in, word, ',')) {
        if (!word.empty() && word.back() == '\r') word.pop_back();
        v.push_back(word);
    }
    return v;
}
std::string model_name(const Model& m, bool capped) {
    return std::string("pin_") + boundary_names[size_t(m.pin_boundary)] + "/obs_" + boundary_names[size_t(m.obs_boundary)] +
           "/fb_" + fallback_names[size_t(m.fallback)] + (capped ? "/cap_fixed" : "/uncapped");
}
} // namespace

int main(int argc, char** argv) {
    try {
        if (argc != 6) throw std::runtime_error("DEF LEF GUIDE official.csv output.json required");
        if (std::filesystem::exists(argv[5])) throw std::runtime_error("output must be new");
        LefParser lef;
        if (!lef.parse(argv[2])) throw std::runtime_error("LEF parse failed");
        DefParser def;
        if (!def.parse(argv[1], lef, DefGridMapping::OFFICIAL_FULL)) throw std::runtime_error("DEF parse failed");
        RoutingGraph graph(def, lef, {RoutingGeometryMode::GLOBAL_GUIDE, false, true, false, true, true});
        const auto& g = def.gcell_grid();
        const size_t plane = size_t(g.cols) * g.rows;
        std::array<std::vector<double>, 21> tracks;
        std::array<std::vector<Shape>, 21> shapes;
        for (const auto& p : def.tracks())
            for (const auto& name : p.layers) {
                const int l = get_layer_index(name);
                if (!l) continue;
                const bool h = lef.get_layer_direction(name) == Direction::HORIZONTAL;
                if (p.axis != (h ? 'Y' : 'X')) continue;
                const auto& lines = h ? g.y_lines : g.x_lines;
                for (int i = 0; i < p.count; ++i) {
                    double t = p.start + i * p.step;
                    if (t >= lines.front() && t <= lines.back()) tracks[l].push_back(t);
                }
            }
        for (auto& v : tracks) {
            std::sort(v.begin(), v.end());
            v.erase(std::unique(v.begin(), v.end()), v.end());
        }
        for (const auto& [name, c] : def.components()) {
            (void)name;
            const auto* m = lef.get_macro(c.macro_type);
            if (!m) throw std::runtime_error("missing macro");
            for (const auto& s : m->obstructions) {
                int l = get_layer_index(s.layer);
                if (l) shapes[l].push_back({transform_macro_rect(s.rect, *m, c, def.dbu_per_micron()), false});
            }
            for (const auto& [pn, p] : m->pins) {
                (void)pn;
                for (const auto& s : p.port_rects) {
                    int l = get_layer_index(s.layer);
                    if (l) shapes[l].push_back({transform_macro_rect(s.rect, *m, c, def.dbu_per_micron()), true});
                }
            }
        }
        for (const auto& [pn, p] : def.io_pins()) {
            (void)pn;
            for (const auto& s : p.port_rects)
                if (s.layer_idx > 0) shapes[s.layer_idx].push_back({s.rect, true});
        }

        std::vector<Model> models;
        for (int pb = 0; pb < BOUNDARIES; ++pb)
            for (int ob = 0; ob < BOUNDARIES; ++ob)
                for (int fb = 0; fb < FALLBACKS; ++fb)
                    models.push_back({pb, ob, fb, std::vector<uint16_t>(graph.size())});

        uint64_t zero_events = 0;
        for (int l = 1; l <= 20; ++l) {
            const auto& v = tracks[l];
            const bool h = lef.get_layer_direction(get_layer_name(l)) == Direction::HORIZONTAL;
            const auto& lines = h ? g.y_lines : g.x_lines;
            std::unordered_map<size_t, CellEvents> events;
            for (const auto& s : shapes[l]) {
                auto [cb, ce] = cells(s.rect.x1, s.rect.x2, g.x_lines);
                auto [rb, re] = cells(s.rect.y1, s.rect.y2, g.y_lines);
                const double shape_low = h ? s.rect.y1 : s.rect.x1, shape_high = h ? s.rect.y2 : s.rect.x2;
                const bool globally_empty =
                    std::lower_bound(v.begin(), v.end(), shape_low) >= std::upper_bound(v.begin(), v.end(), shape_high);
                for (int r = rb; r < re; ++r)
                    for (int c = cb; c < ce; ++c) {
                        const size_t band = size_t(h ? r : c);
                        const double low = std::max(shape_low, lines[band]), high = std::min(shape_high, lines[band + 1]);
                        const auto begin = std::lower_bound(v.begin(), v.end(), low);
                        const auto band_begin = std::lower_bound(v.begin(), v.end(), lines[band]);
                        const auto band_end = band + 2 == lines.size() ? std::upper_bound(v.begin(), v.end(), lines[band + 1])
                                                                       : std::lower_bound(v.begin(), v.end(), lines[band + 1]);
                        const auto closed_end = high == lines[band + 1] && band + 2 < lines.size()
                                                    ? std::lower_bound(v.begin(), v.end(), high)
                                                    : std::upper_bound(v.begin(), v.end(), high);
                        const auto open_end = std::lower_bound(v.begin(), v.end(), high);
                        auto& e = events[size_t(r) * g.cols + c];
                        for (int boundary = 0; boundary < BOUNDARIES; ++boundary) {
                            const auto end = boundary == CLOSED ? closed_end : open_end;
                            if (begin < end) {
                                (s.pin ? e.pin : e.obs)[size_t(boundary)].emplace_back(uint32_t(begin - v.begin()), uint32_t(end - v.begin()));
                                continue;
                            }
                            if (!s.pin) continue;
                            ++e.zeros[size_t(boundary)];
                            if (boundary == CLOSED) ++zero_events;
                            auto& f = e.floor[size_t(boundary)];
                            const uint32_t next = uint32_t(begin - v.begin());
                            const uint32_t prev = next > 0 ? next - 1 : 0;
                            f[NEXT].emplace_back(next, next + 1);
                            f[PREV].emplace_back(prev, prev + 1);
                            if (begin < band_end) f[NEXT_INSIDE].emplace_back(next, next + 1);
                            if (begin > band_begin) f[PREV_INSIDE].emplace_back(prev, prev + 1);
                            if (globally_empty) {
                                f[WHOLE_NEXT].emplace_back(next, next + 1);
                                f[WHOLE_PREV].emplace_back(prev, prev + 1);
                            }
                            if (band_begin < band_end) {
                                const double center = (low + high) / 2;
                                auto nearest = std::lower_bound(band_begin, band_end, center);
                                if (nearest == band_end) --nearest;
                                else if (nearest != band_begin && center - *(nearest - 1) <= *nearest - center) --nearest;
                                const uint32_t j = uint32_t(nearest - v.begin());
                                f[NEAREST_IN_CELL].emplace_back(j, j + 1);
                            }
                        }
                    }
            }
            for (auto& [cell, e] : events) {
                const size_t n = size_t(l - 1) * plane + cell;
                for (auto& m : models) {
                    auto spans = e.pin[size_t(m.pin_boundary)];
                    const auto& obs = e.obs[size_t(m.obs_boundary)];
                    spans.insert(spans.end(), obs.begin(), obs.end());
                    uint32_t value;
                    if (m.fallback == VIRTUAL_ADD) value = merged(std::move(spans)) + e.zeros[size_t(m.pin_boundary)];
                    else if (m.fallback == CELL_VIRTUAL) value = merged(std::move(spans)) + (e.zeros[size_t(m.pin_boundary)] > 0);
                    else {
                        const auto& f = e.floor[size_t(m.pin_boundary)][size_t(m.fallback)];
                        spans.insert(spans.end(), f.begin(), f.end());
                        value = merged(std::move(spans));
                    }
                    m.fixed[n] = uint16_t(std::min<uint32_t>(value, 65535));
                }
            }
            std::vector<Shape>().swap(shapes[l]);
        }

        const auto guides = compat::read_guides(argv[3]);
        if (guides.size() != def.nets().size()) throw std::runtime_error("net count differs");
        std::vector<int> route(graph.size()), mandatory(graph.size());
        OfficialMetricOptions options;
        options.score_20261020 = true;
        options.a150_rules = true;
        options.tsv_track_demand = true;
        options.gcell_width_dbu = def.gcell_pitch_x();
        options.gcell_height_dbu = def.gcell_pitch_y();
        for (const auto& net : def.nets()) {
            auto i = guides.find(net.name);
            if (i == guides.end()) throw std::runtime_error("missing net");
            auto m = measure_official_net_guides(net, i->second, graph, lef, options, 0);
            for (auto [n, d] : m.demand) route[size_t(n)] += d;
            const auto pins = graph.own_pin_nodes(net);
            for (auto point : m.tsv_points)
                for (auto [n, d] : mandatory_tsv_contribution(graph, point, 1, true, pins, true)) mandatory[size_t(n)] += d;
        }

        struct Expected { int capacity, demand; };
        std::unordered_map<int, Expected> expected;
        {
            std::ifstream in(argv[4]);
            if (!in) throw std::runtime_error("missing official CSV");
            std::string line;
            std::getline(in, line);
            const auto labels = fields(line);
            std::map<std::string, size_t> column;
            for (size_t i = 0; i < labels.size(); ++i) column[labels[i]] = i;
            while (std::getline(in, line)) {
                const auto v = fields(line);
                auto val = [&](const std::string& key) { return v.at(column.at(key)); };
                const int n = graph.node(std::stoi(val("gcell_col")), std::stoi(val("gcell_row")), get_layer_index(val("layer")));
                if (!expected.emplace(n, Expected{std::stoi(val("capacity")), std::stoi(val("demand"))}).second)
                    throw std::runtime_error("duplicate official cell");
            }
        }

        std::ofstream out(argv[5]);
        out << std::setprecision(17) << "{\"diagnostic_only\":true,\"production_allowed\":false,\"cells\":" << graph.size()
            << ",\"official_entries\":" << expected.size() << ",\"closed_zero_track_events\":" << zero_events << ",\n";

        // Empirical view independent of any fixed-resource hypothesis:
        // implied = official demand - route, split by layer and capacity.
        std::array<std::map<std::pair<int, int>, uint64_t>, 21> implied;
        uint64_t capacity_differences = 0, negative_implied = 0, implied_above_capacity = 0;
        for (const auto& [n, e] : expected) {
            const int i = e.demand - route[size_t(n)];
            implied[size_t(graph.layer(n))][{i, e.capacity}]++;
            capacity_differences += graph.official_capacity(n) != e.capacity;
            negative_implied += i < 0;
            implied_above_capacity += i > e.capacity;
        }
        out << "\"capacity_differences\":" << capacity_differences << ",\"negative_implied_fixed\":" << negative_implied
            << ",\"implied_fixed_above_capacity\":" << implied_above_capacity << ",\n\"implied_fixed_histogram\":{";
        bool first_layer = true;
        for (int l = 1; l <= 20; ++l) {
            if (implied[size_t(l)].empty()) continue;
            std::vector<std::pair<uint64_t, std::pair<int, int>>> rows;
            for (const auto& [key, count] : implied[size_t(l)]) rows.push_back({count, key});
            std::sort(rows.rbegin(), rows.rend());
            if (rows.size() > 12) rows.resize(12);
            out << (first_layer ? "" : ",") << "\"" << l << "\":[";
            first_layer = false;
            for (size_t i = 0; i < rows.size(); ++i)
                out << (i ? "," : "") << "{\"implied\":" << rows[i].second.first << ",\"capacity\":" << rows[i].second.second
                    << ",\"count\":" << rows[i].first << "}";
            out << "]";
        }
        out << "},\n\"models\":[\n";

        struct Row { std::string name; uint64_t extra, missing, different, overflow, entries; double q, fixed_tsv_q; std::array<uint64_t, 21> layer_mismatch; };
        std::vector<Row> rows;
        for (const auto& m : models)
            for (int capped = 0; capped < 2; ++capped) {
                Row row{model_name(m, capped), 0, 0, 0, 0, 0, 0, 0, {}};
                long double q = 0, minimum = 0;
                for (size_t n = 0; n < graph.size(); ++n) {
                    const int cap = graph.official_capacity(int(n));
                    int fixed = m.fixed[n];
                    if (capped) fixed = std::min(fixed, cap);
                    const int d = fixed + route[n], excess = std::max(0, d - cap);
                    row.overflow += uint64_t(excess);
                    row.entries += excess > 0;
                    if (cap > 0) {
                        q += (static_cast<long double>(excess) / cap) * (static_cast<long double>(excess) / cap);
                        const long double min_excess = std::max(0, fixed + mandatory[n] - cap);
                        minimum += (min_excess / cap) * (min_excess / cap);
                    }
                    const auto i = expected.find(int(n));
                    bool mismatch = false;
                    if (i == expected.end()) { row.extra += excess > 0; mismatch = excess > 0; }
                    else { row.missing += excess == 0; row.different += d != i->second.demand; mismatch = d != i->second.demand; }
                    if (mismatch) row.layer_mismatch[size_t(graph.layer(int(n)))]++;
                }
                row.q = double(q);
                row.fixed_tsv_q = double(minimum);
                rows.push_back(row);
            }
        std::sort(rows.begin(), rows.end(), [](const Row& a, const Row& b) {
            return a.extra + a.different < b.extra + b.different;
        });
        for (size_t i = 0; i < rows.size(); ++i) {
            const auto& r = rows[i];
            out << (i ? ",\n" : "") << "{\"name\":\"" << r.name << "\",\"extra\":" << r.extra << ",\"missing\":" << r.missing
                << ",\"different\":" << r.different << ",\"overflow\":" << r.overflow << ",\"entries\":" << r.entries << ",\"Q\":" << r.q
                << ",\"C\":" << 100 / std::sqrt(1 + r.q) << ",\"fixed_plus_required_tsv_Q\":" << r.fixed_tsv_q
                << ",\"C_upper_bound_relaxed\":" << 100 / std::sqrt(1 + r.fixed_tsv_q) << ",\"layer_mismatch\":[";
            for (int l = 1; l <= 20; ++l) out << (l > 1 ? "," : "") << r.layer_mismatch[size_t(l)];
            out << "]}";
        }
        out << "\n]}\n";
        if (!out) throw std::runtime_error("output failed");
        for (size_t i = 0; i < std::min<size_t>(12, rows.size()); ++i)
            std::cout << rows[i].name << " extra=" << rows[i].extra << " missing=" << rows[i].missing << " different=" << rows[i].different
                      << " Q=" << rows[i].q << " relaxedC=" << 100 / std::sqrt(1 + rows[i].fixed_tsv_q) << '\n';
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
