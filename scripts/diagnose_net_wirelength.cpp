// Read-only per-net comparison of a Guide's charged wirelength against the official
// reference. Reference uses one FLUTE tree over all pins in one plane; a cross-die net
// is really routed as one tree per die joined at its TSV, so also report a FLUTE tree
// per die with the Guide's TSV point. Those per-die trees are a comparison value only.
#include "objectives/official_metrics.hpp"
#include "objectives/net_cell_contribution.hpp"
#include "scoring/compat_validation.hpp"
#include "validation/gcell_encoding.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
using namespace eda;
namespace fs=std::filesystem;
int main(int argc,char** argv) {
 try {
    std::string def_file,guide_file,runtime,output;std::vector<std::string> lefs;
    for(int i=1;i<argc;++i) {
        const std::string key=argv[i];
        if(i+1>=argc) throw std::runtime_error("expected option value");
        const std::string value=argv[++i];
        if(key=="--def") def_file=value;else if(key=="--lef") lefs.push_back(value);
        else if(key=="--guide") guide_file=value;else if(key=="--runtime") runtime=value;
        else if(key=="--output") output=value;else throw std::runtime_error("unknown option");
    }
    if(def_file.empty()||lefs.empty()||guide_file.empty()||runtime.empty()||output.empty())
        throw std::runtime_error("required --def --lef --guide --runtime --output");
    if(fs::exists(output)) throw std::runtime_error("output must be new");
    LefParser lef;for(const auto& file:lefs) if(!lef.parse(file)) throw std::runtime_error("LEF parse failed");
    DefParser def;if(!def.parse(def_file,lef,DefGridMapping::OFFICIAL_FULL)) throw std::runtime_error("DEF parse failed");
    RoutingGraph graph(def,lef,{RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true,true,1,true});
    const auto guides=compat::read_guides(guide_file);
    OfficialMetricOptions options;options.score_20261020=true;options.tsv_track_demand=true;options.a150_rules=true;
    options.all_adjacent_overlaps=true;options.snap_reference_coordinates=true;
    options.gcell_width_dbu=def.gcell_pitch_x();options.gcell_height_dbu=def.gcell_pitch_y();
    NativeFluteBaseline flute(runtime);
    const double pitch=std::max(def.gcell_pitch_x(),def.gcell_pitch_y());
    std::ofstream out(output);if(!out) throw std::runtime_error("cannot create output");
    out<<std::setprecision(15)
       <<"net,pins,bottom,top,cross,min_layer,max_layer,flute_wire,flute_vias,reference,planar,via,tsv,L,rects,single_cell_rects,per_die_wire,per_die_vias,skipped\n";
    for(const auto& net:def.nets()) {
        const auto found=guides.find(net.name);
        if(found==guides.end()) throw std::runtime_error("missing Guide net "+net.name);
        const double reference=flute.measure_net(net,pitch,options);
        const auto m=measure_official_net_guides(net,found->second,graph,lef,options,reference);
        std::vector<Point> all,side[2];std::set<std::string> names;int low=99,high=0;
        for(const auto& pin:net.pins) if(names.insert(pin.name).second) {
            const Point p=pin.port_rects.empty()?pin.coord:pin.port_rects.front().rect.center();
            all.push_back(p);side[pin.layer_idx>10].push_back(p);
            low=std::min(low,pin.layer_idx);high=std::max(high,pin.layer_idx);
        }
        const auto whole=flute.evaluate_points(all);
        double per_die_wire=0;int per_die_vias=0;bool skipped=whole.skipped_degree_limit;
        for(auto& points:side) {
            if(points.empty()) continue;
            if(net.is_cross_die) for(const auto& t:m.tsv_points) points.push_back(t);
            const auto f=flute.evaluate_points(points);
            per_die_wire+=f.wirelength_dbu;per_die_vias+=f.via_count;skipped=skipped||f.skipped_degree_limit;
        }
        size_t single=0;
        for(const auto& g:found->second) {
            const double w=std::abs(g.x2-g.x1),h=std::abs(g.y2-g.y1);
            single+=w<=def.gcell_pitch_x()&&h<=def.gcell_pitch_y();
        }
        out<<net.name<<','<<all.size()<<','<<side[0].size()-(net.is_cross_die&&!side[0].empty()?m.tsv_points.size():0)
           <<','<<side[1].size()-(net.is_cross_die&&!side[1].empty()?m.tsv_points.size():0)<<','<<int(net.is_cross_die)
           <<','<<low<<','<<high<<','<<whole.wirelength_dbu<<','<<whole.via_count<<','<<reference
           <<','<<m.planar_wirelength<<','<<m.via_wirelength<<','<<m.tsv_wirelength<<','<<m.wirelength
           <<','<<found->second.size()<<','<<single<<','<<per_die_wire<<','<<per_die_vias<<','<<int(skipped)<<'\n';
    }
    return 0;
 } catch(const std::exception& error) {std::cerr<<"[ERROR] "<<error.what()<<'\n';return 1;}
}
