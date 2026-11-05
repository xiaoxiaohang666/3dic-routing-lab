// Independent serialized-Guide readback under the production resource contract.
// No router caches, compatibility resources, or hidden scorer participate.
#include "objectives/official_metrics.hpp"
#include "objectives/net_cell_contribution.hpp"
#include "scoring/compat_validation.hpp"
#include "validation/gcell_encoding.hpp"
#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <set>
using namespace eda;
namespace fs=std::filesystem;
int main(int argc,char** argv) {
 try {
    std::string def_file,thermal_file,guide_file,runtime,output,csv,rules="a150-20261029",scope="all-layers",fixed_model="production";
    std::vector<std::string> lefs;double penalty_scale=0.1,thermal_gain=20;
    for(int i=1;i<argc;++i) {
        const std::string key=argv[i];
        if(i+1>=argc) throw std::runtime_error("expected option value");
        const std::string value=argv[++i];
        if(key=="--penalty-scale") penalty_scale=std::stod(value);else if(key=="--thermal-gain") thermal_gain=std::stod(value);else if(key=="--rules-profile") rules=value;else if(key=="--tsv-obstacle-scope") scope=value;
        else if(key=="--def") def_file=value;else if(key=="--lef") lefs.push_back(value);
        else if(key=="--thermal") thermal_file=value;else if(key=="--guide") guide_file=value;
        else if(key=="--runtime") runtime=value;else if(key=="--output") output=value;
        else if(key=="--cells-csv") csv=value;else if(key=="--fixed-resource-model") fixed_model=value;else throw std::runtime_error("unknown measurement option");
    }
    if(def_file.empty()||lefs.empty()||thermal_file.empty()||guide_file.empty()||runtime.empty()||output.empty())
        throw std::runtime_error("required --def --lef --thermal --guide --runtime --output [--cells-csv]");
    if(rules!="a150-20261029"&&rules!="a150"&&rules!="historical") throw std::runtime_error("invalid rules profile");
    if(scope!="all-layers"&&scope!="interface") throw std::runtime_error("invalid TSV obstacle scope");
    if(fixed_model!="production"&&fixed_model!="observed-20261104") throw std::runtime_error("invalid fixed resource model");
    if((penalty_scale!=0.1&&penalty_scale!=1)||!std::isfinite(thermal_gain)||thermal_gain<=0) throw std::runtime_error("invalid scalar parameters");
    const bool a150=rules!="historical",pin_floor=rules=="a150-20261029";
    const auto obstacle_scope=scope=="all-layers"?TsvObstacleScope::ALL_LAYERS:TsvObstacleScope::INTERFACE;
    if(fs::exists(output)||(!csv.empty()&&fs::exists(csv))) throw std::runtime_error("measurement output must be new");
    LefParser lef;for(const auto& file:lefs) if(!lef.parse(file)) throw std::runtime_error("LEF parse failed");
    DefParser def;if(!def.parse(def_file,lef,DefGridMapping::OFFICIAL_FULL)) throw std::runtime_error("DEF parse failed");
    RoutingGraph graph(def,lef,{RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true,pin_floor,penalty_scale,fixed_model=="observed-20261104"});
    ThermalParser thermal;if(!thermal.parse_json(thermal_file,graph.grid(),def.dbu_per_micron(),def.die_area(),ThermalMapping::OFFICIAL_PREVIOUS))
        throw std::runtime_error("thermal parse failed");
    const auto guides=compat::read_guides(guide_file);
    if(guides.size()!=def.nets().size()) throw std::runtime_error("serialized Guide net count mismatch");
    OfficialMetricOptions options;options.score_20261020=true;options.tsv_track_demand=true;options.a150_rules=a150;options.thermal_gain=thermal_gain;
    options.all_adjacent_overlaps=penalty_scale==1;options.snap_reference_coordinates=penalty_scale==1;
    options.gcell_width_dbu=def.gcell_pitch_x();options.gcell_height_dbu=def.gcell_pitch_y();
    NativeFluteBaseline flute(runtime);
    std::vector<NetGuideMetrics> nets;nets.reserve(def.nets().size());
    std::vector<int> demand(graph.size()),mandatory(graph.size());
    std::vector<int> occupancy(size_t(graph.grid().rows)*graph.grid().cols);
    double planar=0,via=0,tsv=0;size_t ordinary_connections=0,interface_violations=0,all_layer_violations=0;
    for(const auto& net:def.nets()) {
        const auto found=guides.find(net.name);std::string error;
        if(found==guides.end() || !validate_gcell_encoding(net,def.die_area(),graph.grid(),lef,found->second,error) ||
           !validate_net_guides(net,def.die_area(),lef,found->second,error,def.gcell_pitch_x(),def.gcell_pitch_y()))
            throw std::runtime_error("serialized net validation failed: "+net.name+" "+error);
        const double reference=flute.measure_net(net,std::max(def.gcell_pitch_x(),def.gcell_pitch_y()),options);
        nets.push_back(measure_official_net_guides(net,found->second,graph,lef,options,reference));
        const auto& measured=nets.back();
        for(auto [node,amount]:measured.demand) demand[node]+=amount;
        const auto pins=graph.own_pin_nodes(net);
        for(const auto point:measured.tsv_points) for(auto [node,amount]:mandatory_tsv_contribution(graph,point,1,true,pins,a150)) mandatory[node]+=amount;
        for(const auto point:measured.tsv_points) {
            const int col=graph.grid().get_col(point.x),row=graph.grid().get_row(point.y);
            interface_violations+=!graph.tsv_landing_legal(col,row,TsvObstacleScope::INTERFACE);
            all_layer_violations+=!graph.tsv_landing_legal(col,row,TsvObstacleScope::ALL_LAYERS);
            if(a150&&!graph.tsv_landing_legal(col,row,obstacle_scope))
                throw std::runtime_error("A150 TSV intersects PIN/OBS GCell: "+net.name);
        }
        for(int cell:measured.tsv_cells) ++occupancy.at(size_t(cell));
        planar+=measured.planar_wirelength;via+=measured.via_wirelength;tsv+=measured.tsv_wirelength;
        ordinary_connections+=measured.ordinary_connections;
    }
    std::vector<const NetGuideMetrics*> cached;for(const auto& net:nets) cached.push_back(&net);
    const auto result=aggregate_official_metrics(cached,demand,graph,thermal,options,false);
    struct Layer {long double q{},fixed_q{},required_q{};uint64_t fixed{},guide{},required{},overflow{};};
    std::array<Layer,21> layers;
    std::ofstream cells;if(!csv.empty()) {cells.open(csv);if(!cells) throw std::runtime_error("cannot create cells CSV");
        cells<<"node,layer,col,row,capacity,fixed,guide,mandatory,removable,overflow,Q\n"<<std::setprecision(17);}
    for(size_t node=0;node<graph.size();++node) {
        const auto contribution=official_resource_cell_contribution(graph,int(node),demand[node]);
        auto& layer=layers[graph.layer(int(node))];layer.q+=contribution.squared;
        layer.fixed_q+=official_resource_cell_contribution(graph,int(node),0).squared;
        layer.required_q+=official_resource_cell_contribution(graph,int(node),mandatory[node]).squared;
        layer.fixed+=graph.official_obstacle_demand(int(node));layer.guide+=demand[node];layer.required+=mandatory[node];layer.overflow+=contribution.overflow;
        if(cells && (contribution.overflow || mandatory[node])) cells<<node<<','<<graph.layer(int(node))<<','<<graph.col(int(node))<<','<<graph.row(int(node))<<','
            <<graph.official_capacity(int(node))<<','<<graph.official_obstacle_demand(int(node))<<','<<demand[node]<<','<<mandatory[node]<<','
            <<demand[node]-mandatory[node]<<','<<contribution.overflow<<','<<double(contribution.squared)<<'\n';
    }
    std::vector<double> heat;heat.reserve(occupancy.size());for(int row=0;row<graph.grid().rows;++row) for(int col=0;col<graph.grid().cols;++col)
        heat.push_back(std::max(0.,thermal.get_temperature(col,row)-thermal.ambient_temperature()));
    const size_t count=std::accumulate(occupancy.begin(),occupancy.end(),size_t(0));
    bool binary=std::all_of(occupancy.begin(),occupancy.end(),[](int n){return n<=1;});
    double upper=0;bool valid_upper=false;
    if(binary && count>0 && count<heat.size()) {
        auto sorted=heat;std::sort(sorted.begin(),sorted.end(),std::greater<double>());
        const long double sum=std::accumulate(heat.begin(),heat.end(),0.L),mean=sum/heat.size();
        long double squares=0;for(double h:heat) squares+=(h-mean)*(h-mean);
        const long double product=std::accumulate(sorted.begin(),sorted.begin()+count,0.L)-count*mean;
        const long double denom=std::sqrt(squares*count*(1-(long double)count/heat.size()));
        if(denom>0) {upper=double(product/denom);valid_upper=true;}
    }
    std::ofstream out(output);if(!out) throw std::runtime_error("cannot create measurement output");out<<std::setprecision(17);
    out<<"{\"model\":\""<<(pin_floor?"a150-20261029-pin-floor-tsv-tracks":a150?"a150-v9-tsv-tracks-geometric-obstacles":"documented-rules-tsv-landing-tracks-v1")
       <<"\",\"a150\":{\"enabled\":"<<(a150?"true":"false")<<",\"obstacle_scope\":\""<<scope
       <<"\",\"interface_violations\":"<<interface_violations<<",\"all_layer_violations\":"<<all_layer_violations
       <<"},\"pin_zero_track_floor\":{\"enabled\":"<<(pin_floor?"true":"false")
       <<",\"demand\":"<<graph.official_pin_zero_track_floor_total()<<",\"granularity_assumption\":\"per-logical-PIN-layer-GCell\"}"
       <<",\"congestion_formula_status\":\"UNCONFIRMED_20261029_CONGESTION_FORMULA\",\"congestion_overflow_scale_proxy\":"<<penalty_scale
       <<",\"resource_alignment_passed\":false,\"all_adjacent_overlaps\":"<<(options.all_adjacent_overlaps?"true":"false")<<",\"official_equivalence\":false,\"G\":"<<thermal_gain<<",\"metrics\":{\"W\":"<<result.wirelength_score
       <<",\"C\":"<<result.congestion_score<<",\"T\":"<<result.thermal_score<<",\"S\":"<<100*result.guide.quality<<",\"Q\":"<<result.squared_normalized_overflow
       <<",\"rho\":"<<result.guide.correlation<<",\"L\":"<<result.guide.wirelength<<",\"reference\":"<<result.guide.reference
       <<",\"overflow\":"<<result.guide.overflow<<",\"ordinary_connections\":"<<ordinary_connections<<",\"planar_L\":"<<planar<<",\"via_L\":"<<via<<",\"tsv_L\":"<<tsv
       <<"},\"thermal_relaxation\":{\"binary\":"<<(binary?"true":"false")<<",\"K\":"<<count<<",\"N\":"<<heat.size()
       <<",\"valid\":"<<(valid_upper?"true":"false")<<",\"hottest_K_rho_upper_bound\":";
    if(valid_upper) out<<upper;else out<<"null";
    out<<",\"reason\":\""<<(valid_upper?"binary-fixed-K-nondegenerate":"nonbinary-or-degenerate")<<"\",\"ignores_connectivity_and_resources\":true},\"layers\":[";
    for(int l=1;l<=20;++l) {if(l>1) out<<',';const auto& m=layers[l];out<<"{\"layer\":"<<l<<",\"Q\":"<<double(m.q)<<",\"fixed_Q\":"<<double(m.fixed_q)
        <<",\"mandatory_floor_Q\":"<<double(m.required_q)<<",\"fixed\":"<<m.fixed<<",\"guide\":"<<m.guide<<",\"mandatory\":"<<m.required<<",\"overflow\":"<<m.overflow<<'}';}
    out<<"]}\n";out.close();if(!out) throw std::runtime_error("measurement output write failed");
    std::cout<<"documented serialized readback S="<<100*result.guide.quality<<" Q="<<result.squared_normalized_overflow<<'\n';return 0;
 } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}