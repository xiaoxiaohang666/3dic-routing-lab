#include "router/shared_hub_optimizer.hpp"
#include "scoring/compat_validation.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
using namespace eda;
namespace fs=std::filesystem;
namespace {
size_t integer(const std::string& s) {
    size_t end=0;if(s.empty()||s.front()=='-') throw std::invalid_argument("negative/empty budget");
    const auto n=std::stoull(s,&end);if(end!=s.size()) throw std::invalid_argument("invalid integer budget");return size_t(n);
}
double number(const std::string& s) {
    size_t end=0;double n=std::stod(s,&end);if(end!=s.size()||!std::isfinite(n)) throw std::invalid_argument("invalid scalar");return n;
}
}
int main(int argc,char** argv) {
 try {
    std::string def_path,thermal_path,input,output,report_path,runtime;std::vector<std::string> lefs;
    SharedHubOptions options;double gain=0;
    for(int i=1;i<argc;++i) {
        const std::string key=argv[i];
        if(key=="--help") {std::cout<<"Required: --def FILE --lef FILE --thermal FILE --guide FILE --output NEW.guide --report NEW.json --runtime DIR --thermal-gain G\n"
            <<"Optional: --max-groups 16 --max-moves 8 --max-candidates 12 --radius 3 --seconds 120 --search-nodes 20000 --max-tail-nodes 512\n"
            <<"Analysis/control: --max-moves 0 preserves the input Guide bytes. No platform upload.\n";return 0;}
        if(i+1>=argc) throw std::invalid_argument("missing option value");
        const std::string value=argv[++i];
        if(key=="--def") def_path=value;else if(key=="--lef") lefs.push_back(value);else if(key=="--thermal") thermal_path=value;
        else if(key=="--guide") input=value;else if(key=="--output") output=value;else if(key=="--report") report_path=value;
        else if(key=="--runtime") runtime=value;else if(key=="--thermal-gain") gain=number(value);
        else if(key=="--max-groups") options.max_groups=integer(value);else if(key=="--max-moves") options.max_moves=integer(value);
        else if(key=="--max-candidates") options.max_candidates=integer(value);else if(key=="--radius") {const auto n=integer(value);if(n>32) throw std::invalid_argument("radius exceeds 32");options.radius=int(n);}
        else if(key=="--seconds") options.seconds=number(value);else if(key=="--search-nodes") options.search_nodes=integer(value);
        else if(key=="--max-tail-nodes") options.max_tail_nodes=integer(value);else throw std::invalid_argument("unknown option: "+key);
    }
    if(def_path.empty()||lefs.empty()||thermal_path.empty()||input.empty()||output.empty()||report_path.empty()||runtime.empty()||gain<=0)
        throw std::invalid_argument("missing required input, output, runtime or explicit positive G");
    if(fs::exists(output)||fs::exists(report_path)||fs::absolute(output).lexically_normal()==fs::absolute(report_path).lexically_normal())
        throw std::invalid_argument("outputs must be distinct new files");
    LefParser lef;for(const auto& p:lefs) if(!lef.parse(p)) throw std::runtime_error("LEF parse failed");
    DefParser def;if(!def.parse(def_path,lef,DefGridMapping::OFFICIAL_FULL)) throw std::runtime_error("DEF parse failed");
    RoutingGraph graph(def,lef,{RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true,true,1});
    ThermalParser thermal;if(!thermal.parse_json(thermal_path,graph.grid(),def.dbu_per_micron(),def.die_area(),ThermalMapping::OFFICIAL_PREVIOUS))
        throw std::runtime_error("thermal parse failed");
    OfficialMetricOptions metric;metric.score_20261020=true;metric.tsv_track_demand=true;metric.a150_rules=true;
    metric.all_adjacent_overlaps=true;metric.snap_reference_coordinates=true;metric.thermal_gain=gain;
    metric.gcell_width_dbu=def.gcell_pitch_x();metric.gcell_height_dbu=def.gcell_pitch_y();
    NativeFluteBaseline flute(runtime);std::vector<double> references;
    for(const auto& net:def.nets()) references.push_back(flute.measure_net(net,std::max(def.gcell_pitch_x(),def.gcell_pitch_y()),metric));
    auto guides=compat::read_guides(input);
    const auto result=optimize_shared_hubs(def,lef,graph,thermal,metric,references,guides,options);
    // A no-op must remain byte-identical, including net/rectangle ordering.
    if(result.moves==0) fs::copy_file(input,output,fs::copy_options::none);
    else {
        std::ofstream out(output);if(!out) throw std::runtime_error("cannot create Guide output");out<<std::setprecision(17);
        for(const auto& [name,rects]:guides) {
            out<<name<<"\n(\n";
            for(const auto& g:rects) out<<g.x1<<' '<<g.y1<<' '<<g.x2<<' '<<g.y2<<' '<<g.layer<<'\n';
            out<<")\n";
        }
        out.close();if(!out) throw std::runtime_error("Guide write failed");
    }
    std::ofstream report(report_path);write_shared_hub_report(report,result,def);report.close();
    if(!report) throw std::runtime_error("report write failed");
    std::cout<<std::setprecision(12)<<"shared_hub groups="<<result.groups.size()<<" evaluated="<<result.evaluated_groups<<" moves="<<result.moves
        <<" delta_L="<<result.after.L-result.before.L<<" delta_S="<<result.after.S-result.before.S<<'\n';return 0;
 } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
