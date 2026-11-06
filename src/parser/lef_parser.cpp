#include "parser/lef_parser.hpp"
#include "parser/text_tokens.hpp"
#include "parser/tolerant_geometry.hpp"
#include <iostream>

namespace eda {
namespace {
LayerRect read_rect(TextTokens& t, const std::string& layer) {
    if (t.peek() == "MASK") { t.take(); t.take(); }
    const double x1 = t.number(), y1 = t.number(), x2 = t.number(), y2 = t.number();
    t.expect(";");
    if (layer.empty() || x2 == x1 || y2 == y1)
        throw std::runtime_error("invalid LEF RECT on layer " + layer);
    return {layer, Rect(x1, y1, x2, y2)};
}
std::vector<LayerRect> parse_shapes(TextTokens& t, ParseWarnings& w) {
    std::vector<LayerRect> shapes;
    std::string layer;
    while (t.peek() != "END") {
        const auto key = t.take();
        if (key == "LAYER") { layer = read_layer_name(t, w); skip_to_semicolon(t); }
        else if (key == "RECT" && t.peek() != "ITERATE") shapes.push_back(read_rect(t, layer));
        else if (key == "POLYGON" && t.peek() != "ITERATE") {
            const auto points = read_lef_polygon(t);
            w.add("OBS POLYGON split into rectangles", layer);
            if (!layer.empty())
                for (const auto& rect : polygon_to_rects(points)) shapes.push_back({layer, rect});
        } else {
            // WIDTH, PATH, VIA and ITERATE forms carry no rectangle we model.
            w.add("OBS geometry skipped", key);
            skip_to_semicolon(t);
        }
    }
    return shapes;
}
ViaInfo parse_via(TextTokens& t, ParseWarnings& w) {
    ViaInfo via; via.name = t.take();
    if (t.peek() == "DEFAULT") { t.take(); via.is_default = true; }
    std::string layer;
    while (t.peek() != "END") {
        const auto key = t.take();
        if (key == "TOPOFSTACKONLY") {
            if (via.top_of_stack_only)
                throw std::runtime_error("duplicate TOPOFSTACKONLY on VIA " + via.name);
            via.top_of_stack_only = true;
        }
        else if (key == "LAYER") { layer = read_layer_name(t, w); skip_to_semicolon(t); }
        else if (key == "RECT") via.shapes.push_back(read_rect(t, layer));
        else if (key == "RESISTANCE" || key == "PROPERTY" || key == "FOREIGN") t.skip_statement();
        else {
            // e.g. VIARULE-generated parameters or POLYGON cuts: keep the RECTs we have.
            w.add("VIA statement skipped", via.name + " " + key);
            skip_to_semicolon(t);
        }
    }
    t.expect("END"); t.expect(via.name);
    return via;
}
void add_port_rect(MacroPinInfo& pin, const std::string& layer, const Rect& rect) {
    pin.port_rects.push_back({layer, rect});
    // Select a real rectangle, never a bounding box across disconnected shapes.
    // The package checker also uses the first rectangle.
    if (!pin.has_rect) {
        pin.layer = layer; pin.bbox = rect; pin.has_rect = true;
    }
}
void parse_port(TextTokens& t, MacroPinInfo& pin, ParseWarnings& w) {
    std::string layer;
    while (t.peek() != "END") {
        const auto key = t.take();
        if (key == "LAYER") {
            layer = read_layer_name(t, w); skip_to_semicolon(t);
        } else if (key == "RECT" && t.peek() != "ITERATE") {
            if (t.peek() == "MASK") { t.take(); t.take(); }
            const double x1 = t.number(), y1 = t.number(), x2 = t.number(), y2 = t.number();
            t.expect(";");
            if (layer.empty() || x2 == x1 || y2 == y1)
                throw std::runtime_error("invalid RECT in pin " + pin.name);
            add_port_rect(pin, layer, Rect(x1, y1, x2, y2));
        } else if (key == "POLYGON" && t.peek() != "ITERATE") {
            const auto points = read_lef_polygon(t);
            w.add("pin POLYGON split into rectangles", pin.name);
            if (layer.empty()) throw std::runtime_error("POLYGON without LAYER in pin " + pin.name);
            for (const auto& rect : polygon_to_rects(points)) add_port_rect(pin, layer, rect);
        } else if (key == "CLASS") t.skip_statement();
        else {
            // WIDTH, PATH, VIA and ITERATE forms: no rectangle we model.
            w.add("pin PORT geometry skipped", pin.name + " " + key);
            skip_to_semicolon(t);
        }
    }
    t.expect("END");
}
MacroInfo parse_macro(TextTokens& t, ParseWarnings& w) {
    MacroInfo macro;
    macro.name = t.take();
    while (t.peek() != "END") {
        const auto key = t.take();
        if (key == "SIZE") {
            macro.width = t.number(); t.expect("BY"); macro.height = t.number(); t.expect(";");
        } else if (key == "CLASS") {
            if (!macro.macro_class.empty()) throw std::runtime_error("duplicate CLASS in macro " + macro.name);
            macro.macro_class = t.take();
            while (t.peek() != ";") {
                if (!macro.macro_subclass.empty()) macro.macro_subclass += " ";
                macro.macro_subclass += t.take();
            }
            t.expect(";");
        } else if (key == "ORIGIN") {
            macro.origin.x = t.number(); macro.origin.y = t.number(); t.expect(";");
        } else if (key == "PIN") {
            MacroPinInfo pin; pin.name = t.take();
            while (t.peek() != "END") {
                if (t.take() == "PORT") parse_port(t, pin, w);
                else t.skip_statement();
            }
            t.expect("END"); t.expect(pin.name);
            if (!pin.has_rect) {
                // A pin without modelled geometry (e.g. power pins) only matters if a net uses it.
                w.add("macro pin without rectangles ignored", macro.name + "." + pin.name);
                continue;
            }
            if (!macro.pins.emplace(pin.name, pin).second)
                throw std::runtime_error("duplicate pin: " + pin.name);
        } else if (key == "OBS") {
            auto shapes = parse_shapes(t, w);
            macro.obstructions.insert(macro.obstructions.end(), shapes.begin(), shapes.end());
            t.expect("END");
        } else t.skip_statement();
    }
    t.expect("END"); t.expect(macro.name);
    if (macro.width <= 0 || macro.height <= 0)
        throw std::runtime_error("missing/invalid SIZE for macro " + macro.name);
    return macro;
}
} // namespace
bool LefParser::parse(const std::string& lef_file_path) {
    try {
        TextTokens t(lef_file_path);
        ParseWarnings w(lef_file_path);
        // A failed additional --lefs file must not corrupt previously loaded data.
        auto directions = layer_directions_;
        auto macros = macros_;
        auto layers = layers_;
        auto vias = vias_;
        bool saw_content = false;
        while (!t.done()) {
            const auto key = t.take();
            if (key == "LAYER") {
                const auto raw_name = t.take();
                const auto name = canonical_layer_name(raw_name);
                if (name != raw_name) w.add("layer name mapped to metalN", raw_name + " -> " + name);
                std::string type;
                Direction direction = Direction::UNKNOWN;
                double width = 0.0;
                while (t.peek() != "END") {
                    const auto field = t.take();
                    if (field == "TYPE") { type = t.take(); t.expect(";"); }
                    else if (field == "DIRECTION") {
                        const auto value = t.take();
                        if (value == "HORIZONTAL") direction = Direction::HORIZONTAL;
                        else if (value == "VERTICAL") direction = Direction::VERTICAL;
                        else throw std::runtime_error("invalid layer DIRECTION: " + value);
                        t.expect(";");
                    } else if (field == "WIDTH") {
                        width = t.number(); t.expect(";");
                        if (width <= 0) throw std::runtime_error("invalid layer WIDTH: " + name);
                    } else t.skip_statement();
                }
                t.expect("END"); t.expect(raw_name);
                if (type == "ROUTING") {
                    if (!get_layer_index(name) || direction == Direction::UNKNOWN)
                        throw std::runtime_error("expected metal1..metal20 and DIRECTION: " + name);
                    if (!directions.emplace(name, direction).second)
                        throw std::runtime_error("duplicate routing layer: " + name);
                }
                if (!layers.emplace(name, LayerInfo{direction, width, type}).second)
                    throw std::runtime_error("duplicate layer: " + name);
                saw_content = true;
            } else if (key == "MACRO") {
                auto macro = parse_macro(t, w);
                if (!macros.emplace(macro.name, macro).second)
                    throw std::runtime_error("duplicate MACRO: " + macro.name);
                saw_content = true;
            } else if (key == "VIA") {
                auto via = parse_via(t, w);
                if (via.shapes.empty()) {
                    // Nothing to model (e.g. a VIARULE-generated via); unusable by the router.
                    w.add("VIA without rectangles ignored", via.name);
                    saw_content = true;
                    continue;
                }
                if (!vias.emplace(via.name, via).second)
                    throw std::runtime_error("duplicate VIA: " + via.name);
                saw_content = true;
            } else if (key == "VIARULE" || key == "SITE") {
                t.skip_block(t.take()); saw_content = true;
            } else if (key == "UNITS" || key == "PROPERTYDEFINITIONS" || key == "SPACING") {
                t.skip_block(key);
            } else if (key == "NONDEFAULTRULE") {
                // Contains its own LAYER/VIA blocks that must not redefine the technology.
                w.add("NONDEFAULTRULE skipped", t.peek());
                t.skip_block(t.take());
            } else if (key == "BEGINEXT") {
                w.add("BEGINEXT skipped", t.peek());
                while (t.take() != "ENDEXT") {}
            } else if (key == "END") {
                t.expect("LIBRARY");
                if (!t.done()) throw std::runtime_error("content after END LIBRARY");
            } else t.skip_statement();
        }
        if (!saw_content) throw std::runtime_error("no layer or macro definitions");
        w.summary();
        layer_directions_ = std::move(directions);
        macros_ = std::move(macros);
        layers_ = std::move(layers);
        vias_ = std::move(vias);
        std::cout << "[INFO] LEF parsed: " << macros_.size() << " macros\n";
        return true;
    } catch (const std::exception& error) {
        std::cerr << "[ERROR] " << lef_file_path << ": " << error.what() << "\n";
        return false;
    }
}
} // namespace eda
