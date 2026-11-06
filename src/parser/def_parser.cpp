#include "parser/def_parser.hpp"
#include "parser/text_tokens.hpp"
#include "parser/tolerant_geometry.hpp"
#include <iostream>
#include <set>
#include <tuple>

namespace eda {
namespace {
Point read_point(TextTokens& t) {
    t.expect("(");
    const double x = t.number(), y = t.number();
    t.expect(")"); return {x, y};
}
Point orient_point(Point p, const std::string& o) {
    if (o == "N") return p;
    if (o == "S") return {-p.x, -p.y};
    if (o == "W") return {-p.y, p.x};
    if (o == "E") return {p.y, -p.x};
    if (o == "FN") return {-p.x, p.y};
    if (o == "FS") return {p.x, -p.y};
    if (o == "FW") return {p.y, p.x};
    if (o == "FE") return {-p.y, -p.x};
    throw std::runtime_error("unsupported DEF orientation: " + o);
}
Point macro_point(Point p, const MacroInfo& m, const std::string& o) {
    // LEF ORIGIN shifts geometry; DEF places the oriented SIZE box's lower left.
    p.x += m.origin.x; p.y += m.origin.y;
    p = orient_point(p, o);
    const auto corner = orient_point({m.width, m.height}, o);
    p.x -= std::min(0.0, corner.x); p.y -= std::min(0.0, corner.y);
    return p;
}
void skip_property(TextTokens& t) {
    while (t.peek() != "+" && t.peek() != ";") {
        if (t.peek() == "END" || t.peek() == "-")
            throw std::runtime_error("unterminated DEF record");
        t.take();
    }
}
std::vector<double> grid_lines(const std::vector<std::tuple<double, int, double>>& statements,
                               double lower, double upper) {
    if (statements.empty()) throw std::runtime_error("both GCELLGRID axes are required");
    std::set<double> lines{lower, upper};
    for (const auto& [start, count, step] : statements) {
        if (count < 1 || (count > 1 && step <= 0)) throw std::runtime_error("invalid GCELLGRID count/step");
        const double first = std::max(0.0, std::ceil((lower - start) / step));
        const double last = std::min(static_cast<double>(count - 1), std::floor((upper - start) / step));
        if (last - first > 1000000) throw std::runtime_error("GCELLGRID axis exceeds supported size");
        for (double i = first; i <= last; ++i) {
            const double value = start + i * step;
            if (value >= lower && value <= upper) lines.insert(value);
        }
    }
    return {lines.begin(), lines.end()};
}
} // namespace

Point transform_macro_point(Point point, const MacroInfo& macro,
                            const Component& component, double dbu_per_micron) {
    if (!std::isfinite(dbu_per_micron) || dbu_per_micron <= 0)
        throw std::runtime_error("invalid geometry DBU scale");
    const auto local = macro_point(point, macro, component.orient);
    return {component.origin.x + local.x * dbu_per_micron,
            component.origin.y + local.y * dbu_per_micron};
}

Rect transform_macro_rect(const Rect& rect, const MacroInfo& macro,
                          const Component& component, double dbu_per_micron) {
    const auto a = transform_macro_point({rect.x1, rect.y1}, macro, component, dbu_per_micron);
    const auto b = transform_macro_point({rect.x2, rect.y2}, macro, component, dbu_per_micron);
    return {a.x, a.y, b.x, b.y};
}

Rect component_bbox(const MacroInfo& macro, const Component& component, double dbu_per_micron) {
    // Removing ORIGIN here gives the true placement SIZE box. ORIGIN continues
    // to shift terminal/obstruction geometry through transform_macro_rect.
    return transform_macro_rect({-macro.origin.x, -macro.origin.y,
                                 macro.width - macro.origin.x, macro.height - macro.origin.y},
                                macro, component, dbu_per_micron);
}

bool DefParser::parse(const std::string& def_file_path, const LefParser& lef_parser, DefGridMapping mapping) {
    *this = DefParser{};
    try {
        TextTokens t(def_file_path);
        ParseWarnings w(def_file_path);
        std::set<std::string> unusable_io, unplaced_components;
        std::vector<std::tuple<double, int, double>> xs, ys;
        bool units = false, die = false, nets_seen = false, design_end = false;
        std::set<std::string> sections, net_names;
        while (!t.done()) {
            const auto key = t.take();
            if (key == "UNITS") {
                if (units) throw std::runtime_error("duplicate UNITS");
                t.expect("DISTANCE"); t.expect("MICRONS");
                dbu_per_micron_ = t.number(); t.expect(";");
                if (dbu_per_micron_ <= 0) throw std::runtime_error("invalid DEF units");
                units = true;
            } else if (key == "DIEAREA") {
                if (die) throw std::runtime_error("duplicate DIEAREA");
                std::vector<Point> corners;
                while (t.peek() == "(") corners.push_back(read_point(t));
                t.expect(";");
                if (corners.size() < 2) throw std::runtime_error("invalid DIEAREA");
                if (corners.size() > 2) w.add("polygon DIEAREA uses its bounding box", std::to_string(corners.size()) + " points");
                double x1 = corners[0].x, y1 = corners[0].y, x2 = x1, y2 = y1;
                for (const auto& p : corners) {
                    x1 = std::min(x1, p.x); y1 = std::min(y1, p.y); x2 = std::max(x2, p.x); y2 = std::max(y2, p.y);
                }
                if (x2 <= x1 || y2 <= y1) throw std::runtime_error("invalid rectangular DIEAREA");
                die_area_ = Rect(x1, y1, x2, y2); die = true;
            } else if (key == "GCELLGRID") {
                const auto axis = t.take();
                const double start = t.number(); t.expect("DO");
                const int count = t.integer(); t.expect("STEP");
                const double step = t.number(); t.expect(";");
                if (axis != "X" && axis != "Y") throw std::runtime_error("invalid GCELLGRID axis");
                (axis == "X" ? xs : ys).emplace_back(start, count, step);
            } else if (key == "TRACKS") {
                TrackInfo track;
                const auto axis = t.take();
                if (axis != "X" && axis != "Y") throw std::runtime_error("invalid TRACKS axis");
                track.axis = axis[0]; track.start = t.number(); t.expect("DO");
                track.count = t.integer(); t.expect("STEP"); track.step = t.number();
                if (t.peek() == "MASK") {
                    t.take(); t.take();
                    if (t.peek() == "SAMEMASK") t.take();
                }
                t.expect("LAYER");
                bool any_layer = false;
                while (t.peek() != ";") {
                    const auto layer = t.take();
                    any_layer = true;
                    if (!get_layer_index(layer) || lef_parser.get_layer_direction(layer) == Direction::UNKNOWN) {
                        w.add("TRACKS on a non-routing layer ignored", layer);
                        continue;
                    }
                    track.layers.push_back(layer);
                }
                t.expect(";");
                if (track.count <= 0 || track.step <= 0 || !any_layer)
                    throw std::runtime_error("invalid TRACKS count, step or layers");
                if (!track.layers.empty()) tracks_.push_back(std::move(track));
            } else if (key == "COMPONENTS" || key == "PINS" || key == "NETS") {
                if (!units || !die) throw std::runtime_error("UNITS/DIEAREA must precede design objects");
                if (!sections.insert(key).second) throw std::runtime_error("duplicate section " + key);
                const int declared = t.integer(); t.expect(";");
                int records = 0;
                while (t.peek() != "END") {
                    t.expect("-");
                    const auto name = t.take();
                    if (key == "COMPONENTS") {
                        Component c{name, t.take(), {}, {}};
                        bool placed = false;
                        while (t.peek() != ";") {
                            t.expect("+");
                            const auto property = t.take();
                            if (property == "PLACED" || property == "FIXED" || property == "COVER") {
                                if (placed) throw std::runtime_error("duplicate component placement");
                                c.origin = read_point(t); c.orient = t.take();
                                orient_point({}, c.orient); placed = true;
                            } else if (property == "UNPLACED") skip_property(t);
                            else skip_property(t);
                        }
                        if (!placed && lef_parser.get_macro(c.macro_type) && !components_.count(name) &&
                            unplaced_components.insert(name).second) {
                            // Only an error for the nets that use it; their pins are dropped below.
                            w.add("unplaced component", name);
                            t.expect(";"); ++records;
                            continue;
                        }
                        if (!placed || !lef_parser.get_macro(c.macro_type) || !components_.emplace(name, c).second)
                            throw std::runtime_error("unplaced/duplicate component or missing macro: " + name);
                    } else if (key == "PINS") {
                        Pin pin; pin.name = name;
                        pin.owner_type = PinOwnerType::TOP_LEVEL;
                        // Each + PORT carries its own shapes and placement (DEF 5.7). A pin
                        // without + PORT is one implicit port.
                        struct IoPort {
                            std::vector<PinShape> shapes;
                            std::string first_layer;
                            Point placement;
                            std::string orient;
                            bool placed{false};
                        };
                        std::vector<IoPort> ports;
                        auto current = [&]() -> IoPort& { if (ports.empty()) ports.emplace_back(); return ports.back(); };
                        auto add_shape = [&](const std::string& layer, const Rect& rect) {
                            const auto index = get_layer_index(layer);
                            if (!index || lef_parser.get_layer_direction(layer) == Direction::UNKNOWN) {
                                w.add("IO shape on a non-routing layer ignored", name + " " + layer);
                                return;
                            }
                            auto& port = current();
                            port.shapes.push_back({index, rect});
                            if (port.first_layer.empty()) port.first_layer = layer;
                        };
                        while (t.peek() != ";") {
                            t.expect("+");
                            const auto property = t.take();
                            if (property == "PORT") {
                                if (!ports.empty()) w.add("multiple IO PORTs", name);
                                ports.emplace_back();
                            } else if (property == "LAYER") {
                                const auto layer = t.take();
                                while (t.peek() != "(") {  // MASK n, SPACING d, DESIGNRULEWIDTH d
                                    if (t.peek() == "+" || t.peek() == ";") throw std::runtime_error("IO LAYER without rectangle " + name);
                                    t.take();
                                }
                                const auto a = read_point(t), b = read_point(t);
                                if (b.x <= a.x || b.y <= a.y) throw std::runtime_error("invalid IO rectangle " + name);
                                add_shape(layer, Rect(a.x, a.y, b.x, b.y));
                            } else if (property == "POLYGON") {
                                const auto layer = t.take();
                                while (t.peek() != "(") {
                                    if (t.peek() == "+" || t.peek() == ";") throw std::runtime_error("IO POLYGON without points " + name);
                                    t.take();
                                }
                                std::vector<Point> points;
                                while (t.peek() == "(") points.push_back(read_point(t));
                                w.add("IO POLYGON split into rectangles", name);
                                for (const auto& rect : polygon_to_rects(points)) add_shape(layer, rect);
                            } else if (property == "PLACED" || property == "FIXED" || property == "COVER") {
                                auto& port = current();
                                if (port.placed) throw std::runtime_error("duplicate IO placement " + name);
                                port.placement = read_point(t); port.orient = t.take(); port.placed = true;
                                orient_point({}, port.orient);
                            } else {
                                // VIA, UNPLACED, NET, DIRECTION, USE, SPECIAL, ...
                                if (property == "VIA" || property == "UNPLACED")
                                    w.add("IO " + property + " skipped", name);
                                skip_property(t);
                            }
                        }
                        const IoPort* primary = nullptr;
                        for (const auto& port : ports)
                            if (port.placed && !port.shapes.empty()) { primary = &port; break; }
                        if (!primary) {
                            // Only an error for the nets that use it; their pins are dropped below.
                            w.add("IO pin without placed routing-layer shape", name);
                            if (io_pins_.count(name) || !unusable_io.insert(name).second)
                                throw std::runtime_error("duplicate IO pin: " + name);
                            t.expect(";"); ++records;
                            continue;
                        }
                        pin.layer = primary->first_layer;
                        Point offset = primary->shapes.front().rect.center();
                        offset = orient_point(offset, primary->orient);
                        const auto& placement = primary->placement;
                        pin.coord = {placement.x + offset.x, placement.y + offset.y};
                        auto place = [&](const IoPort& port) {
                            if (!port.placed) return;
                            for (const auto& shape : port.shapes) {
                                const auto a = orient_point({shape.rect.x1, shape.rect.y1}, port.orient);
                                const auto b = orient_point({shape.rect.x2, shape.rect.y2}, port.orient);
                                pin.port_rects.push_back({shape.layer_idx, Rect(port.placement.x + a.x, port.placement.y + a.y,
                                                                               port.placement.x + b.x, port.placement.y + b.y)});
                            }
                        };
                        place(*primary);
                        for (const auto& port : ports) if (&port != primary) place(port);
                        pin.evaluator_coord = placement; pin.has_evaluator_coord = true;
                        pin.layer_idx = get_layer_index(pin.layer);
                        if (!pin.layer_idx || lef_parser.get_layer_direction(pin.layer) == Direction::UNKNOWN ||
                            unusable_io.count(name) || !io_pins_.emplace(name, pin).second)
                            throw std::runtime_error("duplicate IO pin or unknown routing layer: " + name);
                    } else {
                        Net net; net.name = name;
                        while (t.peek() == "(") {
                            t.expect("(");
                            const auto instance = t.take(), pin_name = t.take();
                            if (t.peek() != ")") {  // + SYNTHESIZED and similar qualifiers
                                w.add("net pin qualifier skipped", name + " " + instance + "." + pin_name);
                                while (t.peek() != ")") {
                                    if (t.peek() == ";") throw std::runtime_error("unterminated net pin in " + name);
                                    t.take();
                                }
                            }
                            t.expect(")");
                            Pin pin;
                            if (instance == "PIN") {
                                const auto found = io_pins_.find(pin_name);
                                if (found == io_pins_.end() && unusable_io.count(pin_name)) {
                                    w.add("net pin on an unusable IO pin dropped", name + " " + pin_name);
                                    continue;
                                }
                                if (found == io_pins_.end()) throw std::runtime_error("missing IO pin: " + pin_name);
                                pin = found->second;
                            } else {
                                const auto found = components_.find(instance);
                                if (found == components_.end() && unplaced_components.count(instance)) {
                                    w.add("net pin on an unplaced component dropped", name + " " + instance);
                                    continue;
                                }
                                if (found == components_.end()) throw std::runtime_error("missing component: " + instance);
                                const auto& c = found->second;
                                const auto* macro = lef_parser.get_macro(c.macro_type);
                                const auto mp = macro->pins.find(pin_name);
                                if (mp == macro->pins.end() || !mp->second.has_rect)
                                    throw std::runtime_error("missing macro pin: " + instance + "." + pin_name);
                                // The terminal layer is that of its first rectangle on a routing layer.
                                auto routing = [&](const std::string& layer) {
                                    return get_layer_index(layer) && lef_parser.get_layer_direction(layer) != Direction::UNKNOWN;
                                };
                                const LayerRect* first = nullptr;
                                if (routing(mp->second.layer)) first = &mp->second.port_rects.front();
                                else {
                                    for (const auto& shape : mp->second.port_rects)
                                        if (routing(shape.layer)) { first = &shape; break; }
                                    if (!first) {
                                        w.add("net pin without routing-layer shape dropped", name + " " + instance + "." + pin_name);
                                        continue;
                                    }
                                    w.add("macro pin first shape not on a routing layer", instance + "." + pin_name);
                                }
                                pin.name = instance + "." + pin_name;
                                pin.instance_name = instance;
                                pin.owner_type = macro->macro_class == "CORE" ? PinOwnerType::CORE_CELL :
                                    macro->macro_class == "BLOCK" ? PinOwnerType::BLOCK_MACRO : PinOwnerType::OTHER_MACRO;
                                pin.owner_bbox = component_bbox(*macro, c, dbu_per_micron_);
                                pin.has_owner_bbox = true;
                                for (const auto& shape : mp->second.port_rects) {
                                    if (!routing(shape.layer)) continue;
                                    pin.port_rects.push_back({get_layer_index(shape.layer),
                                        transform_macro_rect(shape.rect, *macro, c, dbu_per_micron_)});
                                }
                                const auto checker_offset = first->rect.center();
                                pin.evaluator_coord = {c.origin.x + checker_offset.x * dbu_per_micron_,
                                                       c.origin.y + checker_offset.y * dbu_per_micron_};
                                pin.has_evaluator_coord = true;
                                pin.layer = first->layer; pin.layer_idx = get_layer_index(pin.layer);
                                pin.coord = transform_macro_point(first->rect.center(), *macro, c,
                                                                  dbu_per_micron_);
                            }
                            net.pins.push_back(std::move(pin));
                        }
                        while (t.peek() != ";") {
                            t.expect("+");
                            const auto property = t.take();
                            if (property == "ROUTED" || property == "FIXED" || property == "COVER" ||
                                property == "NOSHIELD" || property == "SUBNET" || property == "VPIN")
                                w.add("net wiring/subnet ignored", name + " " + property);
                            skip_property(t);
                        }
                        if (!net_names.insert(name).second)
                            throw std::runtime_error("duplicate net: " + name);
                        if (net.pins.empty()) w.add("net without usable pins skipped", name);
                        else nets_.push_back(std::move(net));
                    }
                    t.expect(";"); ++records;
                }
                t.expect("END"); t.expect(key);
                if (records != declared) throw std::runtime_error("record count mismatch in " + key);
                if (key == "NETS") nets_seen = true;
            } else if (key == "VIAS" || key == "SPECIALNETS" || key == "REGIONS" || key == "GROUPS") {
                t.skip_block(key);
            } else if (key == "FILLS" || key == "SLOTS" || key == "NONDEFAULTRULES" || key == "STYLES" ||
                       key == "SCANCHAINS" || key == "PINPROPERTIES") {
                w.add("DEF section skipped", key);
                t.skip_block(key);
            } else if (key == "BEGINEXT") {
                w.add("DEF section skipped", key);
                while (t.take() != "ENDEXT") {}
            } else if (key == "BLOCKAGES") {
                const int count = t.integer(); t.expect(";");
                // Not modelled as obstacles yet: reading on beats aborting the case.
                if (count) w.add("DEF BLOCKAGES ignored (not modelled)", std::to_string(count) + " records");
                t.skip_block(key);
            } else if (key == "PROPERTYDEFINITIONS") t.skip_block(key);
            else if (key == "END") {
                t.expect("DESIGN"); design_end = true;
                if (!t.done()) throw std::runtime_error("content after END DESIGN");
            } else t.skip_statement();
        }
        if (!units || !die || !nets_seen || !design_end || nets_.empty())
            throw std::runtime_error("missing UNITS, DIEAREA, NETS or END DESIGN");
        if(xs.empty() || ys.empty()) throw std::runtime_error("both GCELLGRID axes are required");
        pitch_x_=std::get<2>(xs.front());pitch_y_=std::get<2>(ys.front());
        auto official_lines=[](const auto& statements,double lower,double upper) {
            std::vector<double> lines;
            for(auto [start,count,step]:statements) {
                if(count<1 || count>1000000 || (count>1 && step<=0)) throw std::runtime_error("invalid official GCELLGRID");
                for(int i=0;i<count;++i) lines.push_back(start+i*step);
            }
            std::sort(lines.begin(),lines.end());lines.erase(std::unique(lines.begin(),lines.end()),lines.end());
            if(lines.front()>lower) lines.insert(lines.begin(),lower);
            if(lines.back()<upper) lines.push_back(upper);
            return lines;
        };
        grid_.x_lines=mapping==DefGridMapping::OFFICIAL_FULL?official_lines(xs,die_area_.x1,die_area_.x2):grid_lines(xs,die_area_.x1,die_area_.x2);
        grid_.y_lines=mapping==DefGridMapping::OFFICIAL_FULL?official_lines(ys,die_area_.y1,die_area_.y2):grid_lines(ys,die_area_.y1,die_area_.y2);
        grid_.cols = static_cast<int>(grid_.x_lines.size()) - 1;
        grid_.rows = static_cast<int>(grid_.y_lines.size()) - 1;
        if (!grid_.valid()) throw std::runtime_error("invalid GCell grid");
        for (const auto* lines : {&grid_.x_lines, &grid_.y_lines}) {
            for (double value : *lines) {
                if (std::floor(value) != value || std::abs(value) > 9007199254740991.0)
                    throw std::runtime_error("guide grid coordinates must be exact integer DBU");
            }
        }
        for (auto& net : nets_) {
            for (auto& pin : net.pins) {
                pin.gcell_col = grid_.get_col(pin.coord.x); pin.gcell_row = grid_.get_row(pin.coord.y);
                const double margin_x = (grid_.x_lines.back() - grid_.x_lines.front()) / grid_.cols;
                const double margin_y = (grid_.y_lines.back() - grid_.y_lines.front()) / grid_.rows;
                if (pin.layer_idx && (pin.gcell_col < 0 || pin.gcell_row < 0) &&
                    pin.coord.x >= grid_.x_lines.front() - margin_x && pin.coord.x <= grid_.x_lines.back() + margin_x &&
                    pin.coord.y >= grid_.y_lines.front() - margin_y && pin.coord.y <= grid_.y_lines.back() + margin_y) {
                    // A terminal centre at most one GCell outside the die maps to the nearest
                    // boundary GCell; anything farther is a broken input and still rejected.
                    w.add("pin outside the die clamped to its edge", pin.name);
                    pin.coord.x = std::clamp(pin.coord.x, grid_.x_lines.front(), grid_.x_lines.back());
                    pin.coord.y = std::clamp(pin.coord.y, grid_.y_lines.front(), grid_.y_lines.back());
                    pin.gcell_col = grid_.get_col(pin.coord.x); pin.gcell_row = grid_.get_row(pin.coord.y);
                }
                if (!pin.layer_idx || lef_parser.get_layer_direction(pin.layer) == Direction::UNKNOWN ||
                    pin.gcell_col < 0 || pin.gcell_row < 0)
                    throw std::runtime_error("invalid pin layer or out-of-die coordinate: " + pin.name);
            }
            net.is_cross_die = net.has_bottom_pin() && net.has_top_pin();
            // The current official evaluator derives the die from terminal
            // layers. Prefix restrictions belong only to the legacy checker.
            if (mapping != DefGridMapping::OFFICIAL_FULL &&
                ((net.name.starts_with("BOTTOM__") && net.has_top_pin()) ||
                 (net.name.starts_with("TOP__") && net.has_bottom_pin())))
                throw std::runtime_error("net prefix conflicts with pin die: " + net.name);
        }
        w.summary();
        std::cout << "[INFO] DEF parsed: " << components_.size() << " components, "
                  << io_pins_.size() << " IO pins, " << nets_.size() << " nets, "
                  << grid_.cols << "x" << grid_.rows << " GCells\n";
        return true;
    } catch (const std::exception& error) {
        std::cerr << "[ERROR] " << def_file_path << ": " << error.what() << "\n";
        *this = DefParser{};
        return false;
    }
}
} // namespace eda
