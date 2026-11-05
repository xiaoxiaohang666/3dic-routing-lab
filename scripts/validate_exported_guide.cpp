#include "router/routing_graph.hpp"
#include "router/tsv_policy.hpp"
#include "validation/guide_validator.hpp"
#include "scoring/compat_validation.hpp"
#include "validation/gcell_encoding.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <set>
#include <sstream>
#include <tuple>
#include <unordered_map>

// Independent export boundary: reload original inputs and the serialized
// Guide. No router NetRoute, resource ledger, cached validation or score is used.
// Shared parsers, physical OBS checks and TSV policy remain engineering
// contracts; this executable is not an independent contest DRC oracle.
using namespace eda;
namespace fs = std::filesystem;
namespace {
struct Options {
    std::vector<std::string> lefs;
    std::string def, thermal, guide, report;
    bool official_model{false}, strict_obs{true}, strict_obs_explicit{false};
    bool score_20261020{false}, a150{true}, supply_only{false},pin_floor{true};
    TsvObstacleScope scope{TsvObstacleScope::INTERFACE};
    TsvPolicyOptions tsv;
};
std::string quoted(const std::string& value) {
    std::ostringstream out; out << '"';
    for (unsigned char c : value) {
        if (c == '"' || c == '\\') out << '\\' << c;
        else if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << unsigned(c) << std::dec;
        else out << c;
    }
    out << '"'; return out.str();
}
Options options(int argc, char** argv) {
    Options result;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if (key == "--help") {
            std::cout << "eda_validate_guide --lef FILE (repeatable) --def FILE --thermal FILE --guide FILE\n"
                      << "  [--model physical|official] [--tsv-resources auto|paired|projected|global-grid]\n"
                      << "  [--strict-obs 0|1] [--report JSON]\n"
                      << "  [--score-profile legacy-20261011|20261020]\n"
                      << "  [--rules-profile a150-20261029|a150|historical] [--tsv-obstacle-scope all-layers|interface]\n"
                      << "  [--tsv-supply-only 0|1] (supply audit requires only --lef and --def)\n"
                      << "  [--tsv-shared-cell-limit N] (default 1; global-grid nets per TSV landing, A62/A150 per-net demand)\n"
                      << "Physical defaults to strict OBS. Official uses full DEF grids and OBS/PIN capacity demand;\n"
                      << "  explicit --strict-obs 1 conflicts with --model official. Pin/TSV checks remain mandatory.\n"
                      << "Checks oriented terminals, Guide connectivity, DIE/OBS and engineering TSV mapping.\n"
                      << "Does not run the official evaluator or certify detailed-route DRC.\n";
            std::exit(0);
        }
        if (i + 1 == argc) throw std::runtime_error("missing value for " + key);
        const std::string value = argv[++i];
        if (key == "--lef") result.lefs.push_back(value);
        else if (key == "--def") result.def = value;
        else if (key == "--thermal") result.thermal = value;
        else if (key == "--guide") result.guide = value;
        else if (key == "--report") result.report = value;
        else if (key == "--rules-profile") {
            if(value!="a150-20261029" && value!="a150" && value!="historical") throw std::runtime_error("rules-profile must be a150-20261029, a150 or historical");
            result.a150=value!="historical";result.pin_floor=value=="a150-20261029";
        }
        else if (key == "--tsv-obstacle-scope") {
            if(value=="all-layers") result.scope=TsvObstacleScope::ALL_LAYERS;
            else if(value=="interface") result.scope=TsvObstacleScope::INTERFACE;
            else throw std::runtime_error("tsv-obstacle-scope must be all-layers or interface");
        }
        else if (key == "--tsv-shared-cell-limit") {
            size_t end=0;const long v=std::stol(value,&end);
            if(end!=value.size() || v<1 || v>64) throw std::runtime_error("tsv-shared-cell-limit must be 1..64");
            result.tsv.shared_cell_limit=size_t(v);
        }
        else if (key == "--tsv-supply-only") {
            if(value!="0" && value!="1") throw std::runtime_error("tsv-supply-only must be 0 or 1");
            result.supply_only=value=="1";
        }
        else if (key == "--score-profile") {
            if(value!="legacy-20261011" && value!="20261020") throw std::runtime_error("invalid score profile");
            result.score_20261020=value=="20261020";
        }
        else if (key == "--model") {
            if (value != "physical" && value != "official") throw std::runtime_error("model must be physical or official");
            result.official_model = value == "official";
        }
        else if (key == "--strict-obs") {
            if (value != "0" && value != "1") throw std::runtime_error("strict-obs must be 0 or 1");
            result.strict_obs = value == "1"; result.strict_obs_explicit = true;
        }
        else if (key == "--tsv-resources") {
            if (value == "auto") result.tsv.resource_mode = TsvResourceMode::AUTO;
            else if (value == "paired") result.tsv.resource_mode = TsvResourceMode::PAIRED;
            else if (value == "projected") result.tsv.resource_mode = TsvResourceMode::PROJECTED;
            else if (value == "global-grid") result.tsv.resource_mode = TsvResourceMode::GLOBAL_GRID;
            else throw std::runtime_error("unknown TSV resource mode: " + value);
        } else throw std::runtime_error("unknown option: " + key);
    }
    // A150 is the default for official production validation. Physical teaching
    // experiments retain their existing strict geometric contract.
    if(!result.official_model) {result.a150=false;result.pin_floor=false;}
    if(result.a150) result.score_20261020=true;
    if(result.score_20261020 && !result.official_model)
        throw std::runtime_error("20261020 score profile requires --model official");
    if(result.tsv.resource_mode==TsvResourceMode::GLOBAL_GRID && !result.official_model)
        throw std::runtime_error("global-grid TSV resources require --model official");
    if (result.official_model) {
        if (result.strict_obs_explicit && result.strict_obs)
            throw std::runtime_error("--strict-obs 1 conflicts with --model official: OBS/PIN are capacity demand in the global-Guide model");
        result.strict_obs = false;
    }
    result.tsv.tsv_obstacle_scope=result.a150?result.scope:TsvObstacleScope::DISABLED;
    if (result.lefs.empty() || result.def.empty() ||
        (!result.supply_only && (result.thermal.empty() || result.guide.empty())))
        throw std::runtime_error("required: --lef --def (and --thermal --guide unless --tsv-supply-only 1)");
    if (!result.report.empty()) {
        std::vector<std::string> inputs = result.lefs;
        inputs.push_back(result.def);
        if(!result.thermal.empty()) inputs.push_back(result.thermal);
        if(!result.guide.empty()) inputs.push_back(result.guide);
        for (const auto& input : inputs)
            if (fs::weakly_canonical(input) == fs::weakly_canonical(result.report) ||
                (fs::exists(input) && fs::exists(result.report) && fs::equivalent(input, result.report)))
                throw std::runtime_error("report cannot overwrite input: " + input);
    }
    return result;
}
GuideMap read_guide(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open guide: " + path);
    GuideMap result; std::string name, token;
    while (in >> name) {
        if (result.contains(name)) throw std::runtime_error("duplicate guide net: " + name);
        if (!(in >> token) || token != "(") throw std::runtime_error("missing '(' for " + name);
        auto& rectangles = result[name]; bool ended = false;
        while (in >> token) {
            if (token == ")") { ended = true; break; }
            GuideRect rect; size_t consumed = 0;
            rect.x1 = std::stod(token, &consumed);
            if (consumed != token.size() || !(in >> rect.y1 >> rect.x2 >> rect.y2 >> rect.layer))
                throw std::runtime_error("malformed rectangle for " + name);
            // Preserve raw coordinates: a reversed export must not be normalized.
            rectangles.push_back(rect);
        }
        if (!ended) throw std::runtime_error("unterminated guide net: " + name);
        if (rectangles.empty()) throw std::runtime_error("empty guide net: " + name);
    }
    if (!in.eof()) throw std::runtime_error("guide read error: " + path);
    if (result.empty()) throw std::runtime_error("empty guide file");
    return result;
}
bool contains(const GuideRect& rect, Point point) {
    return point.x >= rect.x1 && point.x <= rect.x2 && point.y >= rect.y1 && point.y <= rect.y2;
}
bool positive_overlap(const GuideRect& a, const Rect& b) {
    return std::min(a.x2, b.x2) > std::max(a.x1, b.x1) &&
           std::min(a.y2, b.y2) > std::max(a.y1, b.y1);
}
struct Dsu {
    std::vector<size_t> parent, rank;
    explicit Dsu(size_t n) : parent(n), rank(n) { std::iota(parent.begin(), parent.end(), 0); }
    size_t find(size_t x) { while (x != parent[x]) { parent[x] = parent[parent[x]]; x = parent[x]; } return x; }
    void join(size_t a, size_t b) {
        a = find(a); b = find(b); if (a == b) return;
        if (rank[a] < rank[b]) std::swap(a, b);
        parent[b] = a; if (rank[a] == rank[b]) ++rank[a];
    }
};
struct Results {
    size_t checked_nets{}, checked_pins{}, checked_rectangles{}, checked_cross_nets{}, errors{}, checker_uncovered_pins{};
    size_t pin_errors{}, connectivity_errors{}, geometry_errors{}, mapping_errors{}, fallback_layers{}, interpolated_cells{};
    size_t guide_envelope_obs_intersections{}, pin_port_precedence_rectangles{};
    bool official_model{false}, strict_obs{true};
    bool score_20261020{false};
    size_t grid_rows{}, grid_cols{};
    std::string thermal_mode, tsv_mode, capacity_model;
    bool a150{false},supply_only{false},pin_floor{false};
    uint64_t pin_floor_demand{};
    size_t shared_cell_limit{1}, shared_landings{}, peak_landing_load{};
    TsvObstacleScope scope{TsvObstacleScope::DISABLED};
    uint64_t interface_violations{},all_layers_violations{},interface_eligible_cells{},all_layers_eligible_cells{},
             selected_eligible_cells{},geometric_cells{},cross_die_nets{},fixed_interface_violations{},fixed_all_layers_violations{};
    TsvObstacleStats obstacle_stats;
    std::vector<std::string> examples;
    void fail(const std::string& message, size_t& category) {
        ++errors; ++category; if (examples.size() < 50) examples.push_back(message);
    }
};
std::string report(const Results& result) {
    std::ostringstream out;
    out << "{\n\"status\":" << quoted(result.errors ? "FAIL" : "PASS")
        << ",\n\"score_profile\":" << quoted(result.score_20261020?"20261020":"legacy-20261011")
        << ",\n\"rules_profile\":" << quoted(result.pin_floor?"a150-20261029":result.a150?"a150":"historical")
        << ",\n\"pin_zero_track_floor\":{\"enabled\":" << (result.pin_floor?"true":"false")
        << ",\"demand\":" << result.pin_floor_demand << ",\"granularity_assumption\":\"per-logical-PIN-layer-GCell\"}"
        << ",\n\"congestion_formula_status\":\"UNCONFIRMED_20261029_CONGESTION_FORMULA\""
        << ",\n\"supply_only\":" << (result.supply_only?"true":"false")
        << ",\n\"tsv_sharing\":{\"limit\":" << result.shared_cell_limit << ",\"shared_landings\":" << result.shared_landings
        << ",\"peak_load\":" << result.peak_landing_load << "}"
        << ",\n\"a150\":{\"scope\":" << quoted(tsv_obstacle_scope_name(result.scope))
        << ",\"interface_violations\":" << result.interface_violations
        << ",\"all_layers_violations\":" << result.all_layers_violations
        << ",\"interface_eligible_cells\":" << result.interface_eligible_cells
        << ",\"all_layers_eligible_cells\":" << result.all_layers_eligible_cells
        << ",\"selected_eligible_cells\":" << result.selected_eligible_cells
        << ",\"geometric_cells\":" << result.geometric_cells
        << ",\"cross_die_nets\":" << result.cross_die_nets
        << ",\"selected_scope_supply_sufficient\":" << (result.cross_die_nets<=result.selected_eligible_cells?"true":"false")
        << ",\"fixed_interface_violations\":" << result.fixed_interface_violations
        << ",\"fixed_all_layers_violations\":" << result.fixed_all_layers_violations
        << ",\"source_shape_counts\":{\"macro_pin\":" << result.obstacle_stats.shape_counts[0]
        << ",\"obs\":" << result.obstacle_stats.shape_counts[1] << ",\"io_pin\":" << result.obstacle_stats.shape_counts[2] << '}'
        << ",\"source_cell_counts\":{\"macro_pin\":" << result.obstacle_stats.source_cells[0]
        << ",\"obs\":" << result.obstacle_stats.source_cells[1] << ",\"io_pin\":" << result.obstacle_stats.source_cells[2] << '}'
        << ",\"geometry_boundary\":\"positive-area-cell-interior;degenerate-half-open-last-closed\""
        << ",\"shape_track_boundary\":\"closed-shape-edges;internal-gcell-upper-exclusive\"}"
        << ",\n\"validation_scope\":" << quoted(result.official_model
            ? "reloaded-export-oriented-pin-connectivity-die-global-guide-engineering-tsv-policy"
            : "reloaded-export-oriented-pin-connectivity-die-obs-engineering-tsv-policy")
        << ",\n\"official_evaluator_status\":\"not_run\",\n\"full_drc_certified\":false"
        << ",\n\"gcell_encoding_enforced\":" << (result.official_model && result.tsv_mode=="global-grid" ? "true" : "false")
        << ",\n\"unpublished_platform_compatibility\":\"unverified; use audit_guide_geometry.py for observed-rule and unresolved-boundary reporting\""
        << ",\n\"strict_obs_enforced\":" << (result.strict_obs ? "true" : "false")
        << ",\n\"guide_envelope_obs_intersections\":" << result.guide_envelope_obs_intersections
        << ",\n\"pin_port_precedence_rectangles\":" << result.pin_port_precedence_rectangles
        << ",\n\"existing_pin_port_connectivity_enforced\":" << (result.official_model ? "false" : "true")
        << ",\n\"checked\":{\"nets\":" << result.checked_nets << ",\"pins\":" << result.checked_pins
        << ",\"rectangles\":" << result.checked_rectangles << ",\"cross_die_nets\":" << result.checked_cross_nets << '}'
        << ",\n\"errors\":{\"total\":" << result.errors << ",\"pins\":" << result.pin_errors
        << ",\"connectivity\":" << result.connectivity_errors << ",\"die_or_obs\":" << result.geometry_errors
        << ",\"tsv_mapping\":" << result.mapping_errors << '}'
        << ",\n\"input_models\":{\"model\":" << quoted(result.official_model ? "official" : "physical")
        << ",\"grid\":" << quoted(result.official_model ? "official-full" : "clipped-physical")
        << ",\"grid_rows\":" << result.grid_rows << ",\"grid_cols\":" << result.grid_cols
        << ",\"geometry\":" << quoted(result.official_model ? "global-guide" : "physical-strict")
        << ",\"obs_interpretation\":" << quoted(result.official_model ? "capacity-demand" : "physical-geometry")
        << ",\"capacity_model\":" << quoted(result.capacity_model)
        << ",\"thermal_strategy\":" << quoted(result.official_model ? "official-previous" : "legacy-bilinear")
        << ",\"thermal\":" << quoted(result.thermal_mode)
        << ",\"interpolated_cells\":" << result.interpolated_cells
        << ",\"physical_track_proxy_layers\":" << result.fallback_layers
        << ",\"tsv_policy\":" << quoted(result.tsv_mode) << '}'
        << ",\n\"fixed_checker_coordinate_comparison\":{\"uncovered_checker_points\":" << result.checker_uncovered_pins
        << ",\"affects_actual_geometry_status\":false}"
        << ",\n\"limitations\":[\"Shared input parsers and engineering TSV policy are not an independent contest legality oracle.\","
        << quoted(result.official_model
            ? "Official global Guides may cover OBS; fixed OBS/PIN capacity demand is represented by the shared graph. This tool does not calculate congestion or certify physical wire legality."
            : "Exact own-instance PIN PORT precedence over OBS is checked. CORE-cell narrow terminal guides remain engineering abstractions; minimum-width and pin-access DRC are not certified.") << ","
        << "\"A legacy GCell Guide is a routing search envelope and may contain OBS although its retained center-wire corridor is legal; this tool cannot reconstruct or certify that private corridor ledger.\","
        << "\"No spacing, cut enclosure, detailed track assignment, inter-net short, or official quality score is certified.\"]"
        << ",\n\"error_examples\":[";
    for (size_t i = 0; i < result.examples.size(); ++i) { if (i) out << ','; out << quoted(result.examples[i]); }
    out << "]\n}\n"; return out.str();
}
void audit_supply(Results& result,const RoutingGraph& graph,const DefParser& def) {
    result.obstacle_stats=graph.tsv_obstacle_stats();
    const auto& grid=graph.grid();const auto& die=def.die_area();
    for(int row=0;row<grid.rows;++row) for(int col=0;col<grid.cols;++col) {
        const auto box=grid.get_cell_rect(col,row);
        if(!graph.legal_node(graph.node(col,row,10)) || !graph.legal_node(graph.node(col,row,11)) ||
            std::ceil(std::max(box.x1,die.x1))>=std::floor(std::min(box.x2,die.x2)) ||
            std::ceil(std::max(box.y1,die.y1))>=std::floor(std::min(box.y2,die.y2))) continue;
        ++result.geometric_cells;
        result.interface_eligible_cells+=graph.tsv_landing_legal(col,row,TsvObstacleScope::INTERFACE);
        result.all_layers_eligible_cells+=graph.tsv_landing_legal(col,row,TsvObstacleScope::ALL_LAYERS);
    }
    result.selected_eligible_cells=result.scope==TsvObstacleScope::DISABLED?result.geometric_cells:
        (result.scope==TsvObstacleScope::INTERFACE?result.interface_eligible_cells:result.all_layers_eligible_cells);
    for(const auto& net:def.nets()) if(net.is_cross_die) {
        ++result.cross_die_nets;bool interface_blocked=false,all_blocked=false;
        for(const auto& lower:net.pins) if(lower.layer_idx==10)
            for(const auto& upper:net.pins) if(upper.layer_idx==11 && lower.coord.distance_to(upper.coord)<=1e-6) {
                const int col=grid.get_col(lower.coord.x),row=grid.get_row(lower.coord.y);
                if(col<0 || row<0) continue;
                interface_blocked|=!graph.tsv_landing_legal(col,row,TsvObstacleScope::INTERFACE);
                all_blocked|=!graph.tsv_landing_legal(col,row,TsvObstacleScope::ALL_LAYERS);
            }
        result.fixed_interface_violations+=interface_blocked;
        result.fixed_all_layers_violations+=all_blocked;
    }
}
void emit_report(const Results& result,const Options& config) {
    const std::string json=report(result);
    if(!config.report.empty()) {
        const auto parent=fs::path(config.report).parent_path();if(!parent.empty()) fs::create_directories(parent);
        std::ofstream out(config.report);out<<json;out.close();if(!out) throw std::runtime_error("cannot write report");
    }
    std::cout<<json;
}
}
int main(int argc, char** argv) {
    try {
        const auto config = options(argc, argv);
        LefParser lef; for (const auto& path : config.lefs) if (!lef.parse(path)) return 2;
        DefParser def; if (!def.parse(config.def, lef, config.official_model
            ? DefGridMapping::OFFICIAL_FULL : DefGridMapping::CLIPPED_PHYSICAL)) return 2;
        RoutingGraphOptions graph_options;
        graph_options.score_20261020=config.score_20261020;
        graph_options.documented_rules=config.a150 && config.score_20261020;
        graph_options.pin_zero_track_floor=config.pin_floor;
        if(config.official_model) graph_options.geometry=RoutingGeometryMode::GLOBAL_GUIDE;
        RoutingGraph graph(def,lef,graph_options);
        Results result;result.a150=config.a150;result.supply_only=config.supply_only;
        result.pin_floor=graph.pin_zero_track_floor();result.pin_floor_demand=graph.official_pin_zero_track_floor_total();
        result.scope=config.tsv.tsv_obstacle_scope;result.tsv_mode=config.tsv.resource_mode==TsvResourceMode::GLOBAL_GRID?"global-grid":"supply-audit";
        result.official_model=config.official_model;result.strict_obs=config.strict_obs;
        result.shared_cell_limit=config.tsv.shared_cell_limit;
        result.score_20261020=config.score_20261020;result.grid_rows=graph.grid().rows;result.grid_cols=graph.grid().cols;
        result.capacity_model=graph.capacity_model();audit_supply(result,graph,def);
        if(config.supply_only) {
            if(result.cross_die_nets>result.selected_eligible_cells)
                result.fail("A150 supply insufficient under conservative one-net-per-GCell implementation policy",result.mapping_errors);
            const auto fixed=result.scope==TsvObstacleScope::ALL_LAYERS?result.fixed_all_layers_violations:
                (result.scope==TsvObstacleScope::INTERFACE?result.fixed_interface_violations:0);
            if(fixed) result.fail("A150 fixed M10/M11 PIN crossing is forbidden; input terminals cannot be moved or exempted",result.mapping_errors);
            emit_report(result,config);return result.errors?1:0;
        }
        ThermalParser thermal;
        if(!thermal.parse_json(config.thermal,def.gcell_grid(),def.dbu_per_micron(),def.die_area(),
            config.official_model?ThermalMapping::OFFICIAL_PREVIOUS:ThermalMapping::LEGACY_BILINEAR)) return 2;
        const auto guides=config.score_20261020?compat::read_guides(config.guide):read_guide(config.guide);
        auto audit_options=config.tsv;audit_options.supply_audit_only=true;
        TsvPolicy policy(def,lef,graph,audit_options);
        result.thermal_mode=thermal.mapping_mode();result.tsv_mode=policy.pairing_mode();
        if(config.score_20261020) {
            const auto checked=compat::validate(def,lef,guides);
            result.errors+=checked.error_count;result.geometry_errors+=checked.error_count;
            result.examples=checked.errors;
        }
        result.grid_rows = graph.grid().rows; result.grid_cols = graph.grid().cols;
        result.capacity_model = graph.capacity_model();
        result.interpolated_cells = thermal.interpolated_cells();
        for (int layer = 1; layer <= 20; ++layer) result.fallback_layers += graph.guide_uses_physical_tracks(layer);
        std::unordered_map<int, std::vector<int>> slot_cells;
        for (const auto& slot : policy.slots()) {
            int c = graph.grid().get_col(slot.point.x), r = graph.grid().get_row(slot.point.y);
            if (c >= 0 && r >= 0) slot_cells[r * graph.grid().cols + c].push_back(slot.id);
        }
        std::set<std::string> expected; std::unordered_map<int, std::string> occupied; std::unordered_map<int, int> load;
        for (const auto& net : def.nets()) {
            expected.insert(net.name); ++result.checked_nets;
            auto found = guides.find(net.name);
            if (found == guides.end()) { result.fail(net.name + ": missing exported net", result.connectivity_errors); continue; }
            const auto& rects = found->second;
            if(config.official_model && config.tsv.resource_mode==TsvResourceMode::GLOBAL_GRID) {
                std::string error;
                if(!validate_gcell_encoding(net,def.die_area(),graph.grid(),lef,rects,error))
                    result.fail(error,result.geometry_errors);
            }
            std::array<std::vector<size_t>, 21> by_layer; bool valid = true;
            for (size_t i = 0; i < rects.size(); ++i) {
                const auto& rect = rects[i]; ++result.checked_rectangles; const int layer = get_layer_index(rect.layer);
                bool coordinate = true;
                for (double value : {rect.x1, rect.y1, rect.x2, rect.y2})
                    coordinate = coordinate && std::isfinite(value) && std::floor(value) == value && std::abs(value) <= 9007199254740991.0;
                const auto& die = def.die_area();
                if (!coordinate || !layer || lef.get_layer_direction(rect.layer) == Direction::UNKNOWN ||
                    rect.x1 >= rect.x2 || rect.y1 >= rect.y2 || rect.x1 < die.x1 || rect.y1 < die.y1 ||
                    rect.x2 > die.x2 || rect.y2 > die.y2) {
                    valid = false; result.fail(net.name + ": rectangle " + std::to_string(i) + " violates DIE/export geometry", result.geometry_errors);
                    continue;
                }
                const Rect physical{rect.x1, rect.y1, rect.x2, rect.y2};
                if (!graph.legal_rect(physical, layer)) {
                    ++result.guide_envelope_obs_intersections;
                    // LEF PIN geometry takes precedence over OBS belonging to
                    // its own instance. The graph checks the exact intersection
                    // against the actual oriented PORT union; other-owner OBS
                    // and geometry outside that PORT stay forbidden.
                    bool pin_precedence = false;
                    for (const auto& pin : net.pins)
                        if (graph.legal_pin_rect(physical, layer, pin)) { pin_precedence = true; break; }
                    if (pin_precedence) ++result.pin_port_precedence_rectangles;
                    else if (config.strict_obs) result.fail(net.name + ": Guide envelope rectangle " + std::to_string(i) +
                        " intersects OBS outside its own actual PIN PORT; private route corridors are not inferred by this check", result.geometry_errors);
                }
                if (!net.is_cross_die && ((net.has_bottom_pin() && layer > 10) || (net.has_top_pin() && layer <= 10)))
                    result.fail(net.name + ": intra-die net enters the other die", result.mapping_errors);
                by_layer[layer].push_back(i);
            }
            if (!valid) continue;
            for (const auto& pin : net.pins) {
                ++result.checked_pins; bool center = false, port = false, checker = !pin.has_evaluator_coord;
                for (const auto& rect : rects) {
                    int layer = get_layer_index(rect.layer);
                    if (layer == pin.layer_idx && contains(rect, pin.coord)) center = true;
                    if (layer == pin.layer_idx && pin.has_evaluator_coord && contains(rect, pin.evaluator_coord)) checker = true;
                    for (const auto& shape : pin.port_rects)
                        if (shape.layer_idx == layer && positive_overlap(rect, shape.rect)) port = true;
                }
                if (!center || (!pin.port_rects.empty() && !port))
                    result.fail(net.name + ": actual oriented Pin is uncovered: " + pin.name, result.pin_errors);
                if (!checker) ++result.checker_uncovered_pins;
            }
            Dsu connected(rects.size());
            for (auto& indices : by_layer) std::sort(indices.begin(), indices.end(), [&](size_t a, size_t b) {
                return std::tie(rects[a].x1, a) < std::tie(rects[b].x1, b);
            });
            for (int layer = 1; layer <= 20; ++layer) for (size_t a : by_layer[layer])
                for (int next = layer; next <= std::min(20, layer + 1); ++next)
                    for (size_t b : by_layer[next]) {
                        if (rects[b].x1 > rects[a].x2) break;
                        if (next == layer && b <= a) continue;
                        const double dx = std::min(rects[a].x2, rects[b].x2) - std::max(rects[a].x1, rects[b].x1);
                        const double dy = std::min(rects[a].y2, rects[b].y2) - std::max(rects[a].y1, rects[b].y1);
                        // The written rule only connects same-layer Guides
                        // along the preferred direction. Side contact between
                        // adjacent track bands must not bridge those bands.
                        const bool same_layer_contact = config.official_model
                            ? (lef.get_layer_direction(get_layer_name(layer)) == Direction::HORIZONTAL
                                ? dx >= 0 && dy > 0 : dy >= 0 && dx > 0)
                            : dx >= 0 && dy >= 0 && (dx > 0 || dy > 0);
                        const bool contact = layer == next ? same_layer_contact : dx > 0 && dy > 0;
                        if (contact) connected.join(a, b);
                    }
            // Physical geometry can connect through existing terminal metal.
            // The official checker builds adjacency from Guide rectangles
            // alone and requires every ordinary layer transition explicitly.
            if (!config.official_model) for (const auto& pin : net.pins) {
                size_t first = rects.size();
                for (size_t i = 0; i < rects.size(); ++i) {
                    const int layer = get_layer_index(rects[i].layer);
                    bool contacts_port = false;
                    for (const auto& shape : pin.port_rects)
                        if (shape.layer_idx == layer && positive_overlap(rects[i], shape.rect)) {
                            contacts_port = true; break;
                        }
                    if (!contacts_port) continue;
                    if (first == rects.size()) first = i;
                    else connected.join(first, i);
                }
            }
            bool all_connected = true;
            for (size_t i = 1; i < rects.size(); ++i) all_connected = all_connected && connected.find(i) == connected.find(0);
            if (!all_connected) result.fail(net.name + ": exported rectangles/terminals are disconnected", result.connectivity_errors);
            if (!net.is_cross_die) {
                std::string error;
                if (!policy.validate_mapping(net, rects, -1, error)) result.fail(error, result.mapping_errors);
                continue;
            }
            ++result.checked_cross_nets;
            std::set<std::tuple<double, double, double, double>> positive_crosses;
            for (size_t a : by_layer[10]) for (size_t b : by_layer[11]) {
                double x1 = std::max(rects[a].x1, rects[b].x1), y1 = std::max(rects[a].y1, rects[b].y1);
                double x2 = std::min(rects[a].x2, rects[b].x2), y2 = std::min(rects[a].y2, rects[b].y2);
                if (x1 < x2 && y1 < y2) positive_crosses.emplace(x1, y1, x2, y2);
            }
            if (positive_crosses.size() != 1) {
                result.fail(net.name + ": expected one positive-area TSV overlap, found " + std::to_string(positive_crosses.size()), result.mapping_errors); continue;
            }
            const auto [x1, y1, x2, y2] = *positive_crosses.begin(); const Point p{(x1 + x2) * 0.5, (y1 + y2) * 0.5};
            const int col=graph.grid().get_col(p.x),row=graph.grid().get_row(p.y);
            if(col<0 || row<0) {result.fail(net.name+": TSV is outside GCell grid",result.mapping_errors);continue;}
            const bool interface_forbidden=!graph.tsv_landing_legal(col,row,TsvObstacleScope::INTERFACE),
                       all_forbidden=!graph.tsv_landing_legal(col,row,TsvObstacleScope::ALL_LAYERS);
            result.interface_violations+=interface_forbidden;result.all_layers_violations+=all_forbidden;
            if(config.a150 && !policy.tsv_landing_legal(col,row)) {
                result.fail(net.name+": A150 PIN/OBS forbids exported TSV GCell; col="+std::to_string(col)+
                    ", row="+std::to_string(row)+", scope="+tsv_obstacle_scope_name(result.scope),result.mapping_errors);
                continue;
            }
            int cell=row*graph.grid().cols+col;
            std::vector<int> matches;
            // Outward-rounded integer landing rectangles can shift the inferred
            // center by half a DBU. Match slots in its thermal GCell, then let
            // validate_mapping enforce the actual VIA envelope and reservation;
            // do not require the inferred center to equal the physical slot point.
            if (auto candidates = slot_cells.find(cell); candidates != slot_cells.end())
                for (int slot : candidates->second) { std::string error; if (policy.validate_mapping(net, rects, slot, error)) matches.push_back(slot); }
            if (matches.size() != 1) {
                result.fail(net.name + ": TSV reservation mapping has " + std::to_string(matches.size()) + " candidates; unique mapping cannot be verified", result.mapping_errors); continue;
            }
            const int slot = matches.front();
            // One net per landing unless an explicit shared-cell budget is given.
            const int used = ++load[slot];
            result.peak_landing_load = std::max(result.peak_landing_load, size_t(used));
            if (used == 2) ++result.shared_landings;
            if (used > policy.slot_capacity(slot)) result.fail(net.name + ": TSV reservation reused by " + occupied.at(slot), result.mapping_errors);
            for (int conflict : policy.slots()[slot].conflicts)
                if (occupied.contains(conflict)) result.fail(net.name + ": geometric TSV conflict with " + occupied.at(conflict), result.mapping_errors);
            if (used == 1) occupied[slot] = net.name;
        }
        for (const auto& [name, rects] : guides) { (void)rects; if (!expected.contains(name)) result.fail(name + ": unexpected exported net", result.connectivity_errors); }
        emit_report(result,config);return result.errors?1:0;
    } catch (const std::exception& error) { std::cerr << "[EXPORT_VALIDATOR_ERROR] " << error.what() << '\n'; return 2; }
}
