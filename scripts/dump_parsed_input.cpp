// Canonical text dump of what LefParser/DefParser read from one case, used to
// show that a parser change leaves real inputs unchanged (compare the dumps of
// two builds). Usage: dump_parsed_input LEF DEF > dump.txt
// Only the public parser API is used so the same file builds against older trees.
#include "parser/def_parser.hpp"

#include <algorithm>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

using namespace eda;
namespace {
std::string num(double v) { char b[64]; std::snprintf(b, sizeof b, "%.17g", v); return b; }
std::string rect(const Rect& r) { return num(r.x1) + " " + num(r.y1) + " " + num(r.x2) + " " + num(r.y2); }
template <class Map> std::vector<std::string> keys(const Map& m) {
    std::vector<std::string> k;
    for (const auto& e : m) k.push_back(e.first);
    std::sort(k.begin(), k.end());
    return k;
}
void pin_line(const Pin& p) {
    std::cout << " pin " << p.name << " " << p.layer << " " << p.layer_idx << " " << num(p.coord.x) << " "
              << num(p.coord.y) << " g " << p.gcell_col << " " << p.gcell_row << " e "
              << num(p.evaluator_coord.x) << " " << num(p.evaluator_coord.y) << " " << p.has_evaluator_coord
              << " i " << p.instance_name << " o " << int(p.owner_type) << " " << p.has_owner_bbox << " "
              << rect(p.owner_bbox) << " r " << p.port_rects.size() << "\n";
    for (const auto& s : p.port_rects) std::cout << "  pr " << s.layer_idx << " " << rect(s.rect) << "\n";
}
} // namespace
int main(int argc, char** argv) {
    if (argc != 3) { std::cerr << "usage: dump_parsed_input LEF DEF\n"; return 2; }
    LefParser lef;
    if (!lef.parse(argv[1])) return 1;
    DefParser def;
    if (!def.parse(argv[2], lef, DefGridMapping::OFFICIAL_FULL)) return 1;
    std::cout.setf(std::ios::unitbuf);
    for (const auto& name : keys(lef.vias())) {
        const auto& v = lef.vias().at(name);
        std::cout << "via " << name << " " << v.is_default << " " << v.top_of_stack_only << "\n";
        for (const auto& s : v.shapes) std::cout << " s " << s.layer << " " << rect(s.rect) << "\n";
    }
    for (const auto& name : keys(lef.macros())) {
        const auto& m = lef.macros().at(name);
        std::cout << "macro " << name << " " << num(m.width) << " " << num(m.height) << " " << num(m.origin.x)
                  << " " << num(m.origin.y) << " " << m.macro_class << "|" << m.macro_subclass << "\n";
        for (const auto& pn : keys(m.pins)) {
            const auto& p = m.pins.at(pn);
            std::cout << " mpin " << pn << " " << p.layer << " " << rect(p.bbox) << " " << p.has_rect << "\n";
            for (const auto& s : p.port_rects) std::cout << "  pr " << s.layer << " " << rect(s.rect) << "\n";
        }
        for (const auto& s : m.obstructions) std::cout << " obs " << s.layer << " " << rect(s.rect) << "\n";
    }
    for (int i = 1; i <= 20; ++i) {
        const auto layer = get_layer_name(i);
        const auto* info = lef.get_layer(layer);
        std::cout << "layer " << layer << " " << int(lef.get_layer_direction(layer)) << " "
                  << (info ? num(info->width) + " " + info->type : std::string("-")) << "\n";
    }
    std::cout << "die " << rect(def.die_area()) << " dbu " << num(def.dbu_per_micron()) << " pitch "
              << num(def.gcell_pitch_x()) << " " << num(def.gcell_pitch_y()) << "\n";
    const auto& g = def.gcell_grid();
    std::cout << "grid " << g.cols << " " << g.rows << "\n";
    for (double x : g.x_lines) std::cout << " x " << num(x) << "\n";
    for (double y : g.y_lines) std::cout << " y " << num(y) << "\n";
    for (const auto& t : def.tracks()) {
        std::cout << "track " << t.axis << " " << num(t.start) << " " << t.count << " " << num(t.step);
        for (const auto& l : t.layers) std::cout << " " << l;
        std::cout << "\n";
    }
    for (const auto& name : keys(def.components())) {
        const auto& c = def.components().at(name);
        std::cout << "comp " << name << " " << c.macro_type << " " << num(c.origin.x) << " " << num(c.origin.y)
                  << " " << c.orient << "\n";
    }
    for (const auto& name : keys(def.io_pins())) { std::cout << "io " << name << "\n"; pin_line(def.io_pins().at(name)); }
    for (const auto& n : def.nets()) {
        std::cout << "net " << n.name << " " << n.is_cross_die << " " << n.pins.size() << "\n";
        for (const auto& p : n.pins) pin_line(p);
    }
    return 0;
}
