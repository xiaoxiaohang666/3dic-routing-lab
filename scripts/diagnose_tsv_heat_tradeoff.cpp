// Read-only estimate of how far single-TSV relocation can raise the thermal score
// and what it costs in wirelength. Each cross-die net keeps exactly one TSV and each
// cell holds at most one. Moving inside the net's pin bounding box is priced as free;
// moving outside costs twice the distance. Ignores real routing, so results are
// optimistic planning numbers, not achievable scores. Never linked into the router.
#include "objectives/official_metrics.hpp"
#include "scoring/compat_validation.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <functional>
#include <set>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <queue>
#include <sstream>
using namespace eda;
namespace fs=std::filesystem;
namespace {
struct CrossNet {size_t cell{};int c0{},c1{},r0{},r1{};};
struct Option {double value;size_t cell;double added;};
int outside(int c,int r,const CrossNet& n) {
    return std::max({0,n.c0-c,c-n.c1})+std::max({0,n.r0-r,r-n.r1});
}
}
int main(int argc,char** argv) {
 try {
    std::string def_file,thermal_file,guide_file,output,scope_text="all-layers",margins_text="0,1,2,4,8,16,32,64,128",scales_text="1";
    std::vector<std::string> lefs;double gain=0,length=0,reference=0;bool need_free=false;
    for(int i=1;i<argc;++i) {
        const std::string key=argv[i];if(i+1>=argc) throw std::runtime_error("missing value for "+key);
        const std::string value=argv[++i];
        if(key=="--def") def_file=value;else if(key=="--lef") lefs.push_back(value);else if(key=="--thermal") thermal_file=value;
        else if(key=="--guide") guide_file=value;else if(key=="--output") output=value;else if(key=="--scope") scope_text=value;
        else if(key=="--margins") margins_text=value;else if(key=="--price-scales") scales_text=value;
        else if(key=="--thermal-gain") gain=std::stod(value);else if(key=="--length") length=std::stod(value);
        else if(key=="--reference") reference=std::stod(value);else if(key=="--need-free") need_free=std::stoi(value)!=0;
        else throw std::runtime_error("unknown option "+key);
    }
    if(def_file.empty()||lefs.empty()||thermal_file.empty()||guide_file.empty()||output.empty()||!(gain>0)||!(length>0)||!(reference>0))
        throw std::runtime_error("required: --def --lef --thermal --guide --output --thermal-gain --length --reference");
    if(scope_text!="all-layers"&&scope_text!="interface") throw std::runtime_error("bad --scope");
    if(fs::exists(output)) throw std::runtime_error("output must be new");
    const auto scope=scope_text=="all-layers"?TsvObstacleScope::ALL_LAYERS:TsvObstacleScope::INTERFACE;
    const auto parse=[](const std::string& text){std::vector<double> v;std::stringstream in(text);std::string item;while(std::getline(in,item,',')) v.push_back(std::stod(item));return v;};
    LefParser lef;for(const auto& f:lefs) if(!lef.parse(f)) throw std::runtime_error("LEF parse failed");
    DefParser def;if(!def.parse(def_file,lef,DefGridMapping::OFFICIAL_FULL)) throw std::runtime_error("DEF parse failed");
    RoutingGraph graph(def,lef,{RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true,true,1,true});
    ThermalParser thermal;
    if(!thermal.parse_json(thermal_file,graph.grid(),def.dbu_per_micron(),def.die_area(),ThermalMapping::OFFICIAL_PREVIOUS)) throw std::runtime_error("thermal parse failed");
    const auto& grid=graph.grid();const int cols=grid.cols,rows=grid.rows;const size_t cells=size_t(cols)*rows;
    std::vector<double> heat(cells);long double sum=0,sum2=0;
    for(int r=0;r<rows;++r) for(int c=0;c<cols;++c) {
        const double h=std::max(0.0,thermal.get_temperature(c,r)-thermal.ambient_temperature());
        heat[size_t(r)*cols+c]=h;sum+=h;sum2+=(long double)h*h;
    }
    const double mean=double(sum/cells),sigma=double(std::sqrt(sum2/cells-(sum/cells)*(sum/cells)));
    const auto guides=compat::read_guides(guide_file);
    OfficialMetricOptions mo;mo.score_20261020=true;mo.tsv_track_demand=true;mo.a150_rules=true;mo.thermal_gain=gain;mo.all_adjacent_overlaps=true;
    mo.snap_reference_coordinates=true;mo.gcell_width_dbu=def.gcell_pitch_x();mo.gcell_height_dbu=def.gcell_pitch_y();
    std::vector<CrossNet> nets;std::vector<int> count(cells),route(cells);std::vector<unsigned char> pin_cell(cells);
    std::vector<double> pin_share(cells);
    for(const auto& net:def.nets()) {
        const auto found=guides.find(net.name);if(found==guides.end()) throw std::runtime_error("Guide misses net "+net.name);
        const auto measured=measure_official_net_guides(net,found->second,graph,lef,mo,0);
        std::set<size_t> own_tsv;for(int cell:measured.tsv_cells) own_tsv.insert(size_t(cell));
        for(auto [node,amount]:measured.demand) {
            const int layer=graph.layer(node);if(layer!=10&&layer!=11) continue;
            const size_t cell=size_t(graph.row(node))*cols+graph.col(node);
            if(!own_tsv.count(cell)) route[cell]+=amount;
        }
        if(!net.is_cross_die) continue;
        if(measured.tsv_cells.size()!=1) throw std::runtime_error("cross-die net without one TSV: "+net.name);
        CrossNet n;n.cell=size_t(measured.tsv_cells.front());n.c0=cols;n.r0=rows;n.c1=0;n.r1=0;
        for(const auto& pin:net.pins) {
            const int c=std::clamp(grid.get_col(pin.coord.x),0,cols-1),r=std::clamp(grid.get_row(pin.coord.y),0,rows-1);
            n.c0=std::min(n.c0,c);n.c1=std::max(n.c1,c);n.r0=std::min(n.r0,r);n.r1=std::max(n.r1,r);
            pin_share[size_t(r)*cols+c]+=1.0/net.pins.size();
        }
        ++count[n.cell];nets.push_back(n);
    }
    const size_t k=nets.size();
    const auto pearson=[&](const std::vector<int>& x) {
        long double c=0,cc=0,ch=0;for(size_t i=0;i<cells;++i) if(x[i]) {c+=x[i];cc+=(long double)x[i]*x[i];ch+=x[i]*(long double)heat[i];}
        const long double n=cells,vc=cc-c*c/n,vh=sum2-sum*sum/n;return double((ch-c*sum/n)/std::sqrt(vc*vh));
    };
    const auto thermal_score=[&](double rho){return 50*(1+std::tanh(gain*rho));};
    const auto wire_score=[&](double l){return l<=reference?100:100*reference/l;};
    const double rho0=pearson(count),pitch=std::max(def.gcell_pitch_x(),def.gcell_pitch_y());
    std::vector<unsigned char> allowed(cells);size_t allowed_count=0;
    for(int r=0;r<rows;++r) for(int c=0;c<cols;++c) {
        const size_t i=size_t(r)*cols+c;
        allowed[i]=graph.tsv_landing_legal(c,r,scope)&&(!need_free||route[i]==0);
        allowed_count+=allowed[i];
    }
    std::ofstream out(output);out.exceptions(std::ios::badbit|std::ios::failbit);out<<std::setprecision(10);
    std::vector<int> spans;double span_sum=0;size_t currently_inside=0;
    for(const auto& n:nets) {const int s=(n.c1-n.c0)+(n.r1-n.r0);spans.push_back(s);span_sum+=s;currently_inside+=outside(int(n.cell%cols),int(n.cell/cols),n)==0;}
    std::sort(spans.begin(),spans.end());
    const auto pct=[&](double q){return spans[size_t(q*(spans.size()-1))];};
    out<<"{\"scope\":\"read-only estimate: bbox moves priced free, outside moves cost twice the distance, real routing ignored; optimistic\",\n"
       <<"\"tsv_scope\":\""<<scope_text<<"\",\"need_no_other_route_on_M10_M11\":"<<need_free<<",\"K\":"<<k<<",\"cells\":"<<cells<<",\"allowed_cells\":"<<allowed_count
       <<",\"pitch_dbu\":"<<pitch<<",\"rho\":"<<rho0<<",\"T\":"<<thermal_score(rho0)<<",\"heat_mean\":"<<mean<<",\"heat_sigma\":"<<sigma
       <<",\"tsv_inside_own_pin_bbox\":"<<currently_inside<<",\n\"pin_bbox_half_perimeter_gcells\":{\"mean\":"<<span_sum/k<<",\"p50\":"<<pct(.5)<<",\"p90\":"<<pct(.9)<<",\"p99\":"<<pct(.99)<<"},\n";
    const int n8=8;
    const auto regional=[&](const char* name,const std::function<double(size_t)>& value,bool average) {
        out<<'"'<<name<<"\":[";
        for(int i=0;i<n8;++i) {out<<(i?",":"")<<'[';for(int j=0;j<n8;++j) {
            double total=0;size_t seen=0;
            for(int r=i*rows/n8;r<(i+1)*rows/n8;++r) for(int c=j*cols/n8;c<(j+1)*cols/n8;++c) {total+=value(size_t(r)*cols+c);++seen;}
            out<<(j?",":"")<<(average?total/double(seen):total);
        }out<<']';}
        out<<"],\n";
    };
    regional("allowed_fraction_8x8",[&](size_t i){return double(allowed[i]);},true);
    regional("cross_net_pin_share_8x8",[&](size_t i){return pin_share[i]/double(k);},false);
    regional("heat_z_8x8",[&](size_t i){return (heat[i]-mean)/sigma;},true);
    // Marginal S per TSV per heat unit is evaluated at rho=0, the steepest part.
    const long double var_count=(long double)k*(1-(long double)k/cells);
    const double drho_dheat=double(1/std::sqrt((sum2-sum*sum/cells)*var_count));
    const double ds_drho=12.5*gain,ds_dlength=length>reference?45*reference/(length*length):0;
    out<<"\"prices\":{\"dS_dL_per_dbu\":"<<ds_dlength<<",\"dS_per_tsv_per_sigma_at_rho0\":"<<ds_drho*drho_dheat*sigma
       <<",\"break_even_gcells_per_sigma\":"<<(ds_dlength>0?ds_drho*drho_dheat*sigma/ds_dlength/pitch:0)<<"},\n\"sweeps\":[";
    bool first=true;
    for(double scale:parse(scales_text)) for(double margin_value:parse(margins_text)) {
        const int margin=int(margin_value);
        const double price=ds_dlength>0?scale*ds_drho*drho_dheat/ds_dlength:1e18; // dbu per heat unit
        auto moved=count;auto current=nets;std::vector<unsigned char> touched(k);double added=0;size_t rounds=0;
        long double free_choice_heat=0;double free_choice_added=0;
        // Repeat so that nets losing a contested hot cell can take the next one.
        for(int round=0;round<8;++round) {
            std::vector<std::vector<Option>> options(k);
            for(size_t n=0;n<k;++n) {
                const auto& net=current[n];const int base=outside(int(net.cell%cols),int(net.cell/cols),net);
                auto& list=options[n];double best=0;double best_added=0;double best_heat=heat[net.cell];
                for(int r=std::max(0,net.r0-margin);r<=std::min(rows-1,net.r1+margin);++r) for(int c=std::max(0,net.c0-margin);c<=std::min(cols-1,net.c1+margin);++c) {
                    const size_t cell=size_t(r)*cols+c;
                    if(cell==net.cell||!allowed[cell]||heat[cell]<=heat[net.cell]) continue;
                    const double extra=2*pitch*(outside(c,r,net)-base);
                    const double value=price*(heat[cell]-heat[net.cell])-extra;
                    if(value<=0) continue;
                    if(value>best){best=value;best_added=extra;best_heat=heat[cell];}
                    if(moved[cell]) continue;
                    list.push_back({value,cell,extra});
                    if(list.size()>256) {
                        std::nth_element(list.begin(),list.begin()+64,list.end(),[](const Option& x,const Option& y){return x.value>y.value;});
                        list.resize(64);
                    }
                }
                std::sort(list.begin(),list.end(),[](const Option& x,const Option& y){return x.value>y.value;});
                if(list.size()>64) list.resize(64);
                if(!round){free_choice_heat+=best_heat;free_choice_added+=best_added;}
            }
            using Item=std::tuple<double,size_t,size_t>;std::priority_queue<Item> queue;
            for(size_t n=0;n<k;++n) if(!options[n].empty()) queue.push({options[n][0].value,n,0});
            size_t round_moves=0;
            while(!queue.empty()) {
                const auto [value,n,index]=queue.top();queue.pop();(void)value;
                const auto& option=options[n][index];
                if(moved[option.cell]) {if(index+1<options[n].size()) queue.push({options[n][index+1].value,n,index+1});continue;}
                --moved[current[n].cell];++moved[option.cell];current[n].cell=option.cell;touched[n]=1;++round_moves;added+=option.added;
            }
            ++rounds;
            if(!round_moves) break;
        }
        size_t moves=0;for(unsigned char t:touched) moves+=t;
        const double rho=pearson(moved);
        const double dT=.25*(thermal_score(rho)-thermal_score(rho0)),dW=.45*(wire_score(length+added)-wire_score(length));
        // Upper bound: every net takes its best cell as if no cell were contested.
        const double f=std::sqrt(double(k)/double(cells)/(1-double(k)/double(cells)));
        const double rho_free=(double(free_choice_heat/k)-mean)/sigma*f;
        const double dS_free=.25*(thermal_score(rho_free)-thermal_score(rho0))+.45*(wire_score(length+free_choice_added)-wire_score(length));
        out<<(first?"":",")<<"\n{\"margin_gcells\":"<<margin<<",\"price_scale\":"<<scale<<",\"rounds\":"<<rounds<<",\"moves\":"<<moves<<",\"rho\":"<<rho<<",\"T\":"<<thermal_score(rho)
           <<",\"added_length\":"<<added<<",\"added_length_percent\":"<<100*added/length<<",\"dS_T\":"<<dT<<",\"dS_W\":"<<dW<<",\"dS\":"<<dT+dW
           <<",\"no_contest_rho\":"<<rho_free<<",\"no_contest_added_length_percent\":"<<100*free_choice_added/length<<",\"no_contest_dS\":"<<dS_free<<'}';
        first=false;
        std::cout<<"margin="<<margin<<" scale="<<scale<<" moves="<<moves<<" rho="<<rho<<" dS="<<dT+dW<<std::endl;
    }
    out<<"]}\n";
    return 0;
 } catch(const std::exception& e) {std::cerr<<"diagnose_tsv_heat_tradeoff: "<<e.what()<<'\n';return 1;}
}
