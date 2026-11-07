// Run the router's local Guide editing stage on a saved Guide. TSV positions do
// not move. Output scores are offline estimates, not official results.
#include "objectives/net_cell_contribution.hpp"
#include "router/guide_local_edit.hpp"
#include "scoring/compat_validation.hpp"
#include "validation/gcell_encoding.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace eda;
namespace fs=std::filesystem;
namespace {
void metrics(std::ostream& s,const OfficialGuideMetrics& m) {
    s<<"{\"L\":"<<m.guide.wirelength<<",\"Q\":"<<m.squared_normalized_overflow<<",\"rho\":"<<m.guide.correlation
     <<",\"W\":"<<m.wirelength_score<<",\"C\":"<<m.congestion_score<<",\"T\":"<<m.thermal_score<<",\"S\":"<<m.guide.quality*100<<'}';
}
}
int main(int argc,char** argv) {
    try {
        std::string def_file,lef_file,thermal_file,guide_file,runtime,fixed_model="production";fs::path out;
        double seconds=900,gain=0;GuideLocalEditOptions options;
        for(int i=1;i<argc;++i) {
            const std::string key=argv[i];
            if(i+1>=argc) throw std::runtime_error("missing value for "+key);
            const std::string value=argv[++i];
            if(key=="--def") def_file=value;else if(key=="--lef") lef_file=value;else if(key=="--thermal") thermal_file=value;
            else if(key=="--guide") guide_file=value;else if(key=="--runtime") runtime=value;else if(key=="--output-dir") out=value;
            else if(key=="--fixed-resource-model") fixed_model=value;else if(key=="--seconds") seconds=std::stod(value);
            else if(key=="--thermal-gain") gain=std::stod(value);else if(key=="--max-L-growth") options.max_length_growth=std::stod(value);
            else if(key=="--passes") options.passes=std::stoul(value);
            else throw std::runtime_error("unknown option "+key);
        }
        if(def_file.empty()||lef_file.empty()||thermal_file.empty()||guide_file.empty()||runtime.empty()||out.empty()||!(gain>0)||!(seconds>0))
            throw std::runtime_error("required: --def --lef --thermal --guide --runtime --thermal-gain --output-dir NEW");
        if(fixed_model!="production"&&fixed_model!="observed-20261104") throw std::runtime_error("invalid fixed resource model");
        if(fs::exists(out)) throw std::runtime_error("output directory must be new");
        fs::create_directories(out);
        options.deadline=std::chrono::steady_clock::now()+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(seconds));
        LefParser lef;if(!lef.parse(lef_file)) throw std::runtime_error("LEF");
        DefParser def;if(!def.parse(def_file,lef,DefGridMapping::OFFICIAL_FULL)) throw std::runtime_error("DEF");
        RoutingGraph graph(def,lef,{RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true,true,1,fixed_model=="observed-20261104"});
        ThermalParser thermal;
        if(!thermal.parse_json(thermal_file,graph.grid(),def.dbu_per_micron(),def.die_area(),ThermalMapping::OFFICIAL_PREVIOUS)) throw std::runtime_error("thermal");
        auto input=compat::read_guides(guide_file);
        if(input.size()!=def.nets().size()) throw std::runtime_error("net count mismatch");
        OfficialMetricOptions mo;mo.score_20261020=true;mo.tsv_track_demand=true;mo.a150_rules=true;mo.all_adjacent_overlaps=true;
        mo.snap_reference_coordinates=true;mo.thermal_gain=gain;mo.gcell_width_dbu=def.gcell_pitch_x();mo.gcell_height_dbu=def.gcell_pitch_y();
        NativeFluteBaseline flute(runtime);
        std::vector<std::vector<GuideRect>> guides;std::vector<NetGuideMetrics> measured;std::vector<int> demand(graph.size());
        guides.reserve(def.nets().size());measured.reserve(def.nets().size());
        for(const auto& net:def.nets()) {
            const auto it=input.find(net.name);std::string error;
            if(it==input.end()||!validate_gcell_encoding(net,def.die_area(),graph.grid(),lef,it->second,error)||
               !validate_net_guides(net,def.die_area(),lef,it->second,error,mo.gcell_width_dbu,mo.gcell_height_dbu))
                throw std::runtime_error("input Guide invalid for "+net.name+": "+error);
            guides.push_back(it->second);
            measured.push_back(measure_official_net_guides(net,it->second,graph,lef,mo,flute.measure_net(net,std::max(def.gcell_pitch_x(),def.gcell_pitch_y()),mo)));
            for(auto [n,d]:measured.back().demand) demand[n]+=d;
        }
        const auto aggregate=[&]{
            std::vector<const NetGuideMetrics*> cache;for(const auto& m:measured) cache.push_back(&m);
            return aggregate_official_metrics(cache,demand,graph,thermal,mo,false);
        };
        const auto before=aggregate();
        const auto stats=edit_guides_locally(def,graph,lef,mo,guides,measured,demand,before.guide.reference,options);
        const auto after=aggregate();
        if(std::abs(after.guide.wirelength-stats.length_after)>1e-6||std::abs(after.squared_normalized_overflow-stats.q_after)>1e-9||
           after.guide.correlation!=before.guide.correlation||after.thermal_score!=before.thermal_score)
            throw std::runtime_error("full measurement disagrees with the edited totals");
        {
            std::ofstream guide(out/"result.guide");guide<<std::fixed<<std::setprecision(0);
            for(size_t i=0;i<guides.size();++i) {
                guide<<def.nets()[i].name<<"\n(\n";
                for(const auto& r:guides[i]) guide<<r.x1<<' '<<r.y1<<' '<<r.x2<<' '<<r.y2<<' '<<r.layer<<'\n';
                guide<<")\n";
            }
            guide.close();if(!guide) throw std::runtime_error("Guide export failed");
        }
        std::ofstream report(out/"stats.json");report<<std::setprecision(17)
            <<"{\"scope\":\"local Guide edits with TSV positions fixed; offline estimate\",\"official_evaluation\":false"
            <<",\"seconds_limit\":"<<seconds<<",\"fixed_resource_model\":\""<<fixed_model<<"\",\"passes\":"<<stats.passes
            <<",\"stack_accepted\":"<<stats.stack_accepted<<",\"subtree_accepted\":"<<stats.subtree_accepted
            <<",\"attempted\":"<<stats.attempted<<",\"trials\":"<<stats.trials<<",\"invalid\":"<<stats.invalid
            <<",\"pin_rejections\":"<<stats.pin_rejections<<",\"canonical_rejections\":"<<stats.canonical_rejections
            <<",\"search_bound_hits\":"<<stats.search_bound_hits<<",\"timed_out\":"<<(stats.timed_out?"true":"false")
            <<",\"wall_seconds\":"<<stats.seconds<<",\"before\":";
        metrics(report,before);report<<",\"after\":";metrics(report,after);report<<"}\n";
        report.close();if(!report) throw std::runtime_error("stats failed");
        std::cout<<"Q "<<before.squared_normalized_overflow<<" -> "<<after.squared_normalized_overflow
                 <<" S "<<before.guide.quality*100<<" -> "<<after.guide.quality*100<<'\n';
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
