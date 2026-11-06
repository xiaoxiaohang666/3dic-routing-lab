// LEF/DEF constructs absent from the public cases (official Q&A A97/A123) must be
// read conservatively rather than abort the run: a failed parse leaves the case
// without a Guide, and one illegal case fails all cases (A81).
#include "parser/def_parser.hpp"
#include "parser/tolerant_geometry.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>

namespace fs = std::filesystem;
using namespace eda;
namespace {
fs::path work;
size_t checks = 0;
void require(bool value, const std::string& message) {
    ++checks;
    if (!value) throw std::runtime_error(message);
}
std::string file(const std::string& name, const std::string& text) {
    const auto path = work / name;
    std::ofstream out(path); out << text; out.close();
    if (!out) throw std::runtime_error("cannot write parser tolerance fixture");
    return path.string();
}
bool equals(const Rect& r, double x1, double y1, double x2, double y2) {
    return r.x1 == x1 && r.y1 == y1 && r.x2 == x2 && r.y2 == y2;
}

void test_polygon_to_rects() {
    // L shape: (3,3)-(6,3)-(6,4)-(4,4)-(4,6)-(3,6).
    const auto rects = polygon_to_rects({{3, 3}, {6, 3}, {6, 4}, {4, 4}, {4, 6}, {3, 6}});
    require(rects.size() == 2 && equals(rects[0], 3, 3, 6, 4) && equals(rects[1], 3, 4, 4, 6),
            "rectilinear polygon split exactly into slabs");
    require(polygon_to_rects({{0, 0}, {1, 1}}).empty(), "degenerate polygon yields nothing");
    const auto square = polygon_to_rects({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
    require(square.size() == 1 && equals(square[0], 0, 0, 2, 2), "rectangle polygon stays one rectangle");
}

const std::string lef_text =
    "VERSION 5.8 ; BUSBITCHARS \"[]\" ; MANUFACTURINGGRID 0.005 ; "
    "LAYER metal1 TYPE ROUTING ; DIRECTION HORIZONTAL ; WIDTH 0.1 ; END metal1 "
    "LAYER via1 TYPE CUT ; END via1 "
    "LAYER metal2 TYPE ROUTING ; DIRECTION VERTICAL ; WIDTH 0.1 ; END metal2 "
    "NONDEFAULTRULE wide LAYER metal1 WIDTH 0.4 ; END metal1 USEVIA via12 ; END wide "
    "BEGINEXT \"vendor\" anything goes here ENDEXT "
    "VIA via12 DEFAULT LAYER metal1 ; RECT -0.1 -0.1 0.1 0.1 ; LAYER via1 ; RECT -0.05 -0.05 0.05 0.05 ; "
    "LAYER metal2 ; RECT -0.1 -0.1 0.1 0.1 ; END via12 "
    "VIA genvia DEFAULT VIARULE M1M2 ; CUTSIZE 0.1 0.1 ; LAYERS metal1 via1 metal2 ; "
    "CUTSPACING 0.1 0.1 ; ENCLOSURE 0 0 0 0 ; END genvia "
    "MACRO core CLASS CORE ; SIZE 10 BY 20 ; "
    "PIN A DIRECTION INPUT ; PORT LAYER metal1 SPACING 0.1 ; RECT MASK 1 1 1 2 2 ; "
    "POLYGON 3 3 6 3 6 4 4 4 4 6 3 6 ; VIA 1 1 via12 ; WIDTH 0.1 ; PATH 0 0 1 0 ; END END A "
    "PIN B PORT LAYER via1 ; RECT 4 4 4.5 4.5 ; LAYER metal2 ; RECT 4 4 5 5 ; END END B "
    "PIN VDD USE POWER ; PORT LAYER via1 ; RECT 7 7 8 8 ; END END VDD "
    "PIN VSS USE GROUND ; END VSS "
    "OBS LAYER metal1 EXCEPTPGNET ; RECT 0 0 1 1 ; POLYGON 0 0 2 0 2 2 0 2 ; PATH 0 0 1 1 ; "
    "RECT ITERATE 0 0 1 1 DO 2 BY 1 STEP 2 0 ; END "
    "END core END LIBRARY";

void test_lef() {
    LefParser lef;
    require(lef.parse(file("tolerant.lef", lef_text)), "LEF with uncommon constructs parses");
    require(lef.get_layer("metal1") && lef.get_layer("metal1")->width == 0.1,
            "NONDEFAULTRULE does not redefine the technology layer");
    require(lef.get_via("via12") && !lef.get_via("genvia"), "VIARULE-generated via without rectangles ignored");
    const auto* core = lef.get_macro("core");
    require(core != nullptr, "macro retained");
    const auto& a = core->pins.at("A");
    require(a.port_rects.size() == 3 && equals(a.port_rects[0].rect, 1, 1, 2, 2) &&
            equals(a.port_rects[1].rect, 3, 3, 6, 4) && equals(a.port_rects[2].rect, 3, 4, 4, 6),
            "RECT MASK and POLYGON pin geometry read, VIA/WIDTH/PATH skipped");
    require(a.layer == "metal1" && equals(a.bbox, 1, 1, 2, 2), "first real rectangle stays the pin anchor");
    require(core->pins.at("B").layer == "via1", "LEF keeps the literal first layer");
    require(!core->pins.count("VSS"), "pin without geometry ignored instead of aborting");
    require(core->obstructions.size() == 2 && equals(core->obstructions[1].rect, 0, 0, 2, 2),
            "OBS RECT and POLYGON kept, PATH/ITERATE skipped");
}

void test_def() {
    LefParser lef;
    require(lef.parse(file("tolerant-def.lef", lef_text)), "LEF for DEF fixture");
    const std::string def_text =
        "VERSION 5.8 ; DESIGN d ; UNITS DISTANCE MICRONS 10 ; "
        "DIEAREA ( 0 0 ) ( 1000 0 ) ( 1000 1000 ) ( 0 1000 ) ; "
        "TRACKS X 50 DO 10 STEP 100 MASK 1 SAMEMASK LAYER metal1 poly ; "
        "TRACKS Y 50 DO 10 STEP 100 LAYER poly ; "
        "TRACKS Y 50 DO 10 STEP 100 LAYER metal2 ; "
        "GCELLGRID X 0 DO 11 STEP 100 ; GCELLGRID X 1000 DO 1 STEP 0 ; "
        "GCELLGRID Y 0 DO 11 STEP 100 ; GCELLGRID Y 1000 DO 1 STEP 0 ; "
        "NONDEFAULTRULES 1 ; - wide + HARDSPACING + LAYER metal1 WIDTH 400 ; END NONDEFAULTRULES "
        "COMPONENTS 3 ; - u core + PLACED ( 100 200 ) N ; - v core + UNPLACED ; "
        "- w core + SOURCE DIST + FIXED ( 300 300 ) FS ; END COMPONENTS "
        "PINS 3 ; "
        "- io1 + NET n + DIRECTION INPUT + PORT + LAYER metal1 MASK 1 ( -1 -1 ) ( 1 1 ) + PLACED ( 100 100 ) N "
        "+ PORT + POLYGON metal2 ( 0 0 ) ( 4 0 ) ( 4 2 ) ( 0 2 ) + FIXED ( 500 500 ) S ; "
        "- io2 + NET m + USE POWER ; "
        "- io3 + NET n + LAYER metal1 ( -1 -1 ) ( 1 1 ) + PLACED ( 1001 500 ) N ; END PINS "
        "BLOCKAGES 2 ; - LAYER metal1 RECT ( 0 0 ) ( 10 10 ) ; - PLACEMENT RECT ( 0 0 ) ( 5 5 ) ; END BLOCKAGES "
        "FILLS 1 ; - LAYER metal1 RECT ( 0 0 ) ( 5 5 ) ; END FILLS "
        "SPECIALNETS 1 ; - VDD ( * VDD ) + USE POWER ; END SPECIALNETS "
        "NETS 3 ; - n ( u A + SYNTHESIZED ) ( PIN io1 ) ( v A ) ( PIN io3 ) ( u B ) ( w A ) "
        "+ ROUTED metal1 ( 0 0 ) ( 10 * ) NEW metal2 ( 10 0 ) ( 10 10 ) + USE SIGNAL ; "
        "- m ( PIN io2 ) ( u VDD ) ; "
        "- k ( u A ) ( w B ) ; END NETS "
        "SCANCHAINS 1 ; - c + START u A + STOP w A ; END SCANCHAINS "
        "END DESIGN";
    DefParser def;
    require(def.parse(file("tolerant.def", def_text), lef, DefGridMapping::OFFICIAL_FULL),
            "DEF with uncommon constructs parses");
    require(equals(def.die_area(), 0, 0, 1000, 1000), "polygon DIEAREA uses its bounding box");
    require(def.tracks().size() == 2 && def.tracks()[0].layers.size() == 1 && def.tracks()[0].layers[0] == "metal1",
            "TRACKS MASK read and non-routing layers ignored");
    require(def.gcell_grid().cols == 10 && def.gcell_grid().rows == 10, "single-line GCELLGRID accepted");
    require(def.components().size() == 2 && !def.components().count("v"), "unplaced component kept out");
    require(def.io_pins().size() == 2 && !def.io_pins().count("io2"), "IO pin without geometry kept out");
    const auto& io1 = def.io_pins().at("io1");
    require(io1.port_rects.size() == 2 && equals(io1.port_rects[0].rect, 99, 99, 101, 101) &&
            io1.port_rects[1].layer_idx == 2 && equals(io1.port_rects[1].rect, 496, 498, 500, 500),
            "every IO PORT placed with its own location and orientation");
    require(io1.coord.x == 100 && io1.coord.y == 100 && io1.layer == "metal1", "first placed PORT anchors the pin");
    require(def.nets().size() == 2, "net without usable pins skipped");
    const auto& n = def.nets()[0];
    require(n.name == "n" && n.pins.size() == 5, "net pins on unplaced components dropped, qualifiers skipped");
    require(n.pins[0].layer == "metal1" && n.pins[0].coord.x == 115 && n.pins[0].coord.y == 215,
            "macro pin anchored on its first rectangle");
    require(n.pins[2].coord.x == 1000 && n.pins[2].gcell_col == 9, "pin just outside the die clamped");
    const auto& b = n.pins[3];
    require(b.layer == "metal2" && b.layer_idx == 2 && b.coord.x == 145 && b.coord.y == 245 &&
            b.port_rects.size() == 1 && b.port_rects[0].layer_idx == 2,
            "macro pin anchored on its first routing-layer rectangle; cut shapes dropped");
    require(def.nets()[1].name == "k" && def.nets()[1].pins.size() == 2, "later nets unaffected");
}

void test_still_rejected() {
    LefParser lef;
    require(lef.parse(file("rejected.lef", lef_text)), "LEF for rejection fixture");
    const std::string head =
        "UNITS DISTANCE MICRONS 10 ; DIEAREA ( 0 0 ) ( 1000 1000 ) ; "
        "GCELLGRID X 0 DO 11 STEP 100 ; GCELLGRID Y 0 DO 11 STEP 100 ; "
        "COMPONENTS 1 ; - u core + PLACED ( 100 200 ) N ; END COMPONENTS ";
    DefParser def;
    require(!def.parse(file("missing-comp.def", head + "NETS 1 ; - n ( x A ) ; END NETS END DESIGN"), lef),
            "undeclared component still rejected");
    require(!def.parse(file("missing-pin.def", head + "NETS 1 ; - n ( u Q ) ; END NETS END DESIGN"), lef),
            "unknown macro pin still rejected");
    require(!def.parse(file("count.def", head + "NETS 2 ; - n ( u A ) ; END NETS END DESIGN"), lef),
            "record count mismatch still rejected");
    require(!def.parse(file("dup.def", head + "NETS 2 ; - n ( u A ) ; - n ( u B ) ; END NETS END DESIGN"), lef),
            "duplicate net still rejected");
    const std::string far =
        "UNITS DISTANCE MICRONS 10 ; DIEAREA ( 0 0 ) ( 1000 1000 ) ; "
        "GCELLGRID X 0 DO 11 STEP 100 ; GCELLGRID Y 0 DO 11 STEP 100 ; "
        "COMPONENTS 1 ; - u core + PLACED ( 1200 200 ) N ; END COMPONENTS "
        "NETS 1 ; - n ( u A ) ; END NETS END DESIGN";
    require(!def.parse(file("far.def", far), lef), "pin more than one GCell outside the die still rejected");
}
void test_layer_names() {
    require(canonical_layer_name("Metal10") == "metal10" && canonical_layer_name("M2") == "metal2" &&
            canonical_layer_name("metal20") == "metal20" && canonical_layer_name("met1") == "met1" &&
            canonical_layer_name("M21") == "M21" && canonical_layer_name("M01") == "M01" &&
            canonical_layer_name("via1") == "via1" && canonical_layer_name("MASTERSLICE") == "MASTERSLICE",
            "only Metal<N>/M<N> spellings with N in 1..20 are mapped");
    const std::string lef =
        "LAYER Metal1 TYPE ROUTING ; DIRECTION HORIZONTAL ; END Metal1 "
        "LAYER via1 TYPE CUT ; END via1 "
        "LAYER M2 TYPE ROUTING ; DIRECTION VERTICAL ; END M2 "
        "VIA v12 DEFAULT LAYER Metal1 ; RECT -1 -1 1 1 ; LAYER via1 ; RECT -0.5 -0.5 0.5 0.5 ; "
        "LAYER M2 ; RECT -1 -1 1 1 ; END v12 "
        "MACRO c CLASS CORE ; SIZE 10 BY 10 ; PIN A PORT LAYER Metal1 ; RECT 1 1 2 2 ; END END A "
        "OBS LAYER M2 ; RECT 0 0 3 3 ; END END c END LIBRARY";
    LefParser l;
    require(l.parse(file("names.lef", lef)), "LEF with Metal<N>/M<N> layer names parses");
    require(l.get_layer_direction("metal1") == Direction::HORIZONTAL &&
            l.get_layer_direction("metal2") == Direction::VERTICAL, "routing layers stored as metalN");
    require(l.get_via("v12")->shapes[0].layer == "metal1" && l.get_via("v12")->shapes[2].layer == "metal2",
            "via shapes use metalN");
    require(l.get_macro("c")->pins.at("A").layer == "metal1" && l.get_macro("c")->obstructions[0].layer == "metal2",
            "pin and OBS shapes use metalN");
    const std::string def =
        "UNITS DISTANCE MICRONS 10 ; DIEAREA ( 0 0 ) ( 1000 1000 ) ; "
        "TRACKS X 50 DO 10 STEP 100 LAYER M2 ; TRACKS Y 50 DO 10 STEP 100 LAYER Metal1 ; "
        "GCELLGRID X 0 DO 11 STEP 100 ; GCELLGRID Y 0 DO 11 STEP 100 ; "
        "COMPONENTS 1 ; - u c + PLACED ( 100 100 ) N ; END COMPONENTS "
        "PINS 1 ; - io + NET n + LAYER M2 ( -1 -1 ) ( 1 1 ) + PLACED ( 500 500 ) N ; END PINS "
        "NETS 1 ; - n ( u A ) ( PIN io ) ; END NETS END DESIGN";
    DefParser d;
    require(d.parse(file("names.def", def), l, DefGridMapping::OFFICIAL_FULL), "DEF with Metal<N>/M<N> names parses");
    require(d.tracks()[0].layers[0] == "metal2" && d.tracks()[1].layers[0] == "metal1", "TRACKS layers use metalN");
    require(d.nets()[0].pins[0].layer_idx == 1 && d.nets()[0].pins[1].layer == "metal2" &&
            d.nets()[0].pins[1].layer_idx == 2, "pins resolve to metalN layer indices");
}
} // namespace

int main(int argc, char** argv) {
    try {
        if (argc != 2) throw std::runtime_error("usage: parser_tolerance_tests output_directory");
        work = argv[1]; fs::create_directories(work);
        test_polygon_to_rects(); test_lef(); test_def(); test_still_rejected(); test_layer_names();
        std::cout << "Parser tolerance tests passed: " << checks << " checks\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Parser tolerance tests failed: " << e.what() << "\n";
        return 1;
    }
}
