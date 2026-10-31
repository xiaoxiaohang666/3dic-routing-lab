// Offline TSV thermal headroom diagnostic. Never used by routing or acceptance.
// It reads a serialized Guide, finds each cross-die net's actual TSV cell and
// estimates how far exact Pearson/T could move under the production all-layer
// PIN/OBS landing ban. Wirelength changes use a terminal-attachment proxy and
// congestion is ignored, so every estimate is an optimistic planning number.
#include "objectives/official_metrics.hpp"
#include "scoring/compat_validation.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <queue>
#include <sstream>

using namespace eda;
namespace fs=std::filesystem;
namespace {
struct Pearson {
    long double sum_h{},sum_hh{};size_t cells{};
    const std::vector<double>* heat{};
    double rho(const std::vector<int>& counts) const {
        long double c=0,cc=0,ch=0;
        for(size_t i=0;i<counts.size();++i) if(counts[i]) {
            c+=counts[i];cc+=(long double)counts[i]*counts[i];ch+=counts[i]*(long double)(*heat)[i];
        }
        const long double n=cells,vc=cc-c*c/n,vh=sum_hh-sum_h*sum_h/n;
        if(vc<=0||vh<=0) return 0;
        return double((ch-c*sum_h/n)/std::sqrt(vc*vh));
    }
};
double thermal_score(double gain,double rho) {return 50*(1+std::tanh(gain*rho));}
double wirelength_score(double length,double reference) {return length<=reference?100:100*reference/length;}
// Best hottest-cells bound when each legal cell may hold at most `cap` TSVs.
double capped_bound(const Pearson& p,const std::vector<size_t>& legal_by_heat,size_t k,int cap) {
    std::vector<int> counts(p.cells,0);size_t left=k;
    for(size_t cell:legal_by_heat) {
        if(!left) break;
        const int put=int(std::min<size_t>(left,size_t(cap)));counts[cell]=put;left-=size_t(put);
    }
    if(left) return std::nan("");
    return p.rho(counts);
}
std::vector<int> parse_list(const std::string& text) {
    std::vector<int> values;std::stringstream in(text);std::string item;
    while(std::getline(in,item,',')) values.push_back(std::stoi(item));
    return values;
}
}
int main(int argc,char** argv) {
 try {
    std::string def_file,thermal_file,guide_file,output,radii_text="2,4,8,16,32";
    std::vector<std::string> lefs;double gain=0,length=0,reference=0,detour=2;
    for(int i=1;i<argc;++i) {
        const std::string key=argv[i];
        if(i+1>=argc) throw std::runtime_error("expected option value");
        const std::string value=argv[++i];
        if(key=="--def") def_file=value;else if(key=="--lef") lefs.push_back(value);
        else if(key=="--thermal") thermal_file=value;else if(key=="--guide") guide_file=value;
        else if(key=="--output") output=value;else if(key=="--radii") radii_text=value;
        else if(key=="--thermal-gain") gain=std::stod(value);
        else if(key=="--length") length=std::stod(value);else if(key=="--reference") reference=std::stod(value);
        else if(key=="--detour-factor") detour=std::stod(value);
        else throw std::runtime_error("unknown option "+key);
    }
    if(def_file.empty()||lefs.empty()||thermal_file.empty()||guide_file.empty()||output.empty()||
       !(gain>0)||!(length>0)||!(reference>0)||!(detour>0))
        throw std::runtime_error("required: --def --lef --thermal --guide --output --thermal-gain G --length L --reference R [--radii 2,4,8] [--detour-factor 2]");
    if(fs::exists(output)) throw std::runtime_error("output must be new");
    const auto radii=parse_list(radii_text);
    LefParser lef;for(const auto& file:lefs) if(!lef.parse(file)) throw std::runtime_error("LEF parse failed");
    DefParser def;if(!def.parse(def_file,lef,DefGridMapping::OFFICIAL_FULL)) throw std::runtime_error("DEF parse failed");
    RoutingGraph graph(def,lef,{RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true,true,1.0});
    ThermalParser thermal;
    if(!thermal.parse_json(thermal_file,graph.grid(),def.dbu_per_micron(),def.die_area(),ThermalMapping::OFFICIAL_PREVIOUS))
        throw std::runtime_error("thermal parse failed");
    const auto& grid=graph.grid();const int cols=grid.cols,rows=grid.rows;
    const size_t cells=size_t(cols)*rows;
    std::vector<double> heat(cells);
    for(int r=0;r<rows;++r) for(int c=0;c<cols;++c)
        heat[size_t(r)*cols+c]=std::max(0.0,thermal.get_temperature(c,r)-thermal.ambient_temperature());
    Pearson pearson;pearson.cells=cells;pearson.heat=&heat;
    for(double h:heat) {pearson.sum_h+=h;pearson.sum_hh+=(long double)h*h;}
    const long double mean=pearson.sum_h/cells;
    const long double sxx=pearson.sum_hh-pearson.sum_h*pearson.sum_h/cells;
    const double sigma=double(std::sqrt(sxx/cells));
    std::vector<unsigned char> legal(cells),legal_interface(cells);
    size_t legal_count=0,interface_count=0;long double legal_heat=0;
    for(int r=0;r<rows;++r) for(int c=0;c<cols;++c) {
        const size_t i=size_t(r)*cols+c;
        legal[i]=graph.tsv_landing_legal(c,r,TsvObstacleScope::ALL_LAYERS);
        legal_interface[i]=graph.tsv_landing_legal(c,r,TsvObstacleScope::INTERFACE);
        legal_count+=legal[i];interface_count+=legal_interface[i];if(legal[i]) legal_heat+=heat[i];
    }
    // Actual TSV cells of the serialized Guide.
    const auto guides=compat::read_guides(guide_file);
    OfficialMetricOptions options;options.score_20261020=true;options.tsv_track_demand=true;options.a150_rules=true;
    options.thermal_gain=gain;options.all_adjacent_overlaps=true;options.snap_reference_coordinates=true;
    options.gcell_width_dbu=def.gcell_pitch_x();options.gcell_height_dbu=def.gcell_pitch_y();
    struct Cross {size_t net;size_t cell;std::vector<Point> bottom,top;};
    std::vector<Cross> cross;std::vector<int> counts(cells,0);
    for(size_t n=0;n<def.nets().size();++n) {
        const auto& net=def.nets()[n];if(!net.is_cross_die) continue;
        const auto found=guides.find(net.name);
        if(found==guides.end()) throw std::runtime_error("Guide misses cross-die net "+net.name);
        const auto measured=measure_official_net_guides(net,found->second,graph,lef,options,0.0);
        if(measured.tsv_cells.size()!=1) throw std::runtime_error("cross-die net without exactly one TSV: "+net.name);
        Cross item{n,size_t(measured.tsv_cells.front()),{},{}};
        for(const auto& pin:net.pins) (pin.layer_idx>=11?item.top:item.bottom).push_back(pin.coord);
        ++counts[item.cell];cross.push_back(std::move(item));
    }
    const size_t k=cross.size();
    const bool binary=std::all_of(counts.begin(),counts.end(),[](int c){return c<=1;});
    const double rho0=pearson.rho(counts);
    long double tsv_heat=0;for(const auto& c:cross) tsv_heat+=heat[c.cell];
    // Hottest-K bounds: unrestricted, all-layer legal, interface-only legal.
    std::vector<size_t> order(cells);std::iota(order.begin(),order.end(),size_t(0));
    std::sort(order.begin(),order.end(),[&](size_t a,size_t b){return heat[a]!=heat[b]?heat[a]>heat[b]:a<b;});
    std::vector<size_t> legal_order,interface_order;
    for(size_t i:order) {if(legal[i]) legal_order.push_back(i);if(legal_interface[i]) interface_order.push_back(i);}
    const double bound_all=capped_bound(pearson,order,k,1);
    // Marginal prices at the current state, in weighted S points.
    const double tanh0=std::tanh(gain*rho0);
    const double ds_drho=12.5*gain*(1-tanh0*tanh0);
    const long double count_var=(long double)k*(1-(long double)k/cells);
    const double drho_dheat=double(1/std::sqrt(sxx*count_var)); // one TSV, one heat unit
    const double ds_dlength=length>reference?45*reference/(length*length):0;
    const double lambda=ds_dlength>0?ds_drho*drho_dheat/ds_dlength:std::numeric_limits<double>::infinity();
    const double pitch=std::max(def.gcell_pitch_x(),def.gcell_pitch_y());
    std::ofstream out(output);out<<std::setprecision(10);
    out<<"{\"scope\":\"offline-diagnostic; ignores congestion, routing feasibility and exact MST\",\n"
       <<"\"grid\":{\"cols\":"<<cols<<",\"rows\":"<<rows<<",\"cells\":"<<cells<<",\"pitch_dbu\":"<<pitch<<"},\n"
       <<"\"heat\":{\"mean\":"<<double(mean)<<",\"sigma\":"<<sigma<<",\"max\":"<<heat[order.front()]
       <<",\"legal_mean\":"<<double(legal_heat/std::max<size_t>(1,legal_count))
       <<",\"tsv_mean\":"<<double(tsv_heat/std::max<size_t>(1,k))<<"},\n"
       <<"\"legal_cells\":{\"all_layers\":"<<legal_count<<",\"interface_only_diagnostic\":"<<interface_count<<"},\n"
       <<"\"tsv\":{\"K\":"<<k<<",\"binary\":"<<(binary?"true":"false")<<",\"rho\":"<<rho0
       <<",\"T\":"<<thermal_score(gain,rho0)<<"},\n"
       <<"\"bounds\":{\"hottest_K_any_cell\":"<<bound_all
       <<",\"hottest_K_all_layer_legal\":"<<capped_bound(pearson,legal_order,k,1)
       <<",\"hottest_K_interface_legal_diagnostic\":"<<capped_bound(pearson,interface_order,k,1);
    for(int cap:{2,4,8}) out<<",\"hottest_all_layer_legal_cap"<<cap<<"\":"<<capped_bound(pearson,legal_order,k,cap);
    out<<"},\n\"prices\":{\"dS_drho\":"<<ds_drho<<",\"drho_per_tsv_heat\":"<<drho_dheat<<",\"dS_dL\":"<<ds_dlength
       <<",\"break_even_dbu_per_heat\":"<<lambda<<",\"break_even_gcells_per_sigma\":"<<lambda*sigma/pitch<<"},\n";
    // Attachment proxy: each die side joins at its nearest terminal (bbox
    // distance for wide nets). The detour factor scales the routed penalty.
    const auto side_cost=[&](const std::vector<Point>& pins,Point p) {
        if(pins.empty()) return 0.0;
        if(pins.size()>32) {
            double x1=pins[0].x,x2=x1,y1=pins[0].y,y2=y1;
            for(const auto& q:pins) {x1=std::min(x1,q.x);x2=std::max(x2,q.x);y1=std::min(y1,q.y);y2=std::max(y2,q.y);}
            return std::max({0.0,x1-p.x,p.x-x2})+std::max({0.0,y1-p.y,p.y-y2});
        }
        double best=std::numeric_limits<double>::infinity();
        for(const auto& q:pins) best=std::min(best,std::abs(q.x-p.x)+std::abs(q.y-p.y));
        return best;
    };
    const auto center=[&](size_t cell) {
        const int c=int(cell%size_t(cols)),r=int(cell/size_t(cols));
        return Point{(grid.x_lines[c]+grid.x_lines[c+1])*0.5,(grid.y_lines[r]+grid.y_lines[r+1])*0.5};
    };
    // Natural positions: the pin-median anchor used by the initial assignment.
    // (a) anchor cells as-is (may stack or be illegal), (b) nearest free legal
    // cell to the anchor, unique, assigned in net order. Both ignore routing.
    {
        std::vector<int> anchor_counts(cells,0),nearest_counts(cells,0);
        std::vector<unsigned char> taken(cells,0);
        double displacement=0,anchor_heat=0;size_t unresolved=0;
        std::vector<double> current_offsets;current_offsets.reserve(k);
        for(const auto& item:cross) {
            const auto& net=def.nets()[item.net];
            std::vector<double> xs,ys;
            for(const auto& pin:net.pins) {xs.push_back(pin.coord.x);ys.push_back(pin.coord.y);}
            std::sort(xs.begin(),xs.end());std::sort(ys.begin(),ys.end());
            const Point anchor{(xs[(xs.size()-1)/2]+xs[xs.size()/2])*0.5,(ys[(ys.size()-1)/2]+ys[ys.size()/2])*0.5};
            const int c0=std::clamp(grid.get_col(anchor.x),0,cols-1),r0=std::clamp(grid.get_row(anchor.y),0,rows-1);
            const size_t cell0=size_t(r0)*cols+c0;++anchor_counts[cell0];anchor_heat+=heat[cell0];
            const int cc=int(item.cell%size_t(cols)),cr=int(item.cell/size_t(cols));
            current_offsets.push_back(std::max(std::abs(cc-c0),std::abs(cr-r0)));
            size_t found=cells;
            for(int ring=0;ring<=std::max(cols,rows) && found==cells;++ring) {
                double best=std::numeric_limits<double>::infinity();
                for(int r=r0-ring;r<=r0+ring;++r) for(int c=c0-ring;c<=c0+ring;c+=(r==r0-ring||r==r0+ring||ring==0)?1:2*ring) {
                    if(r<0||c<0||r>=rows||c>=cols) continue;
                    const size_t cell=size_t(r)*cols+c;
                    if(!legal[cell]||taken[cell]) continue;
                    const double d=std::abs(c-c0)+std::abs(r-r0);
                    if(d<best) {best=d;found=cell;}
                }
            }
            if(found==cells) {++unresolved;continue;}
            taken[found]=1;++nearest_counts[found];
            displacement+=std::abs(int(found%size_t(cols))-c0)+std::abs(int(found/size_t(cols))-r0);
        }
        std::sort(current_offsets.begin(),current_offsets.end());
        const auto pct=[&](double q){return current_offsets.empty()?0.0:current_offsets[size_t(q*(current_offsets.size()-1))];};
        out<<"\"natural_positions\":{\"anchor_cells_rho\":"<<pearson.rho(anchor_counts)
           <<",\"anchor_mean_heat\":"<<anchor_heat/std::max<size_t>(1,k)
           <<",\"nearest_legal_unique_rho\":"<<pearson.rho(nearest_counts)
           <<",\"nearest_legal_unique_T\":"<<thermal_score(gain,pearson.rho(nearest_counts))
           <<",\"nearest_mean_displacement_gcells\":"<<displacement/std::max<size_t>(1,k-unresolved)
           <<",\"unresolved\":"<<unresolved
           <<",\"current_tsv_to_anchor_chebyshev_p50\":"<<pct(.5)<<",\"p90\":"<<pct(.9)<<",\"p99\":"<<pct(.99)<<"},\n";
    }
    out<<"\"radius_sweeps\":[";
    bool first=true;
    for(int radius:radii) for(double lambda_scale:{0.25,1.0,4.0}) {
        // Greedy unique relocation on linearized gains, then exact Pearson.
        const double price=lambda*lambda_scale;
        struct Option {double gain;size_t target;double dlen;};
        std::vector<std::vector<Option>> options_by_net(k);
        for(size_t n=0;n<k;++n) {
            const auto& item=cross[n];
            const int c0=int(item.cell%size_t(cols)),r0=int(item.cell/size_t(cols));
            const Point p0=center(item.cell);
            const double base=side_cost(item.bottom,p0)+side_cost(item.top,p0);
            std::vector<Option> best;
            for(int r=std::max(0,r0-radius);r<=std::min(rows-1,r0+radius);++r)
                for(int c=std::max(0,c0-radius);c<=std::min(cols-1,c0+radius);++c) {
                    const size_t cell=size_t(r)*cols+c;
                    if(cell==item.cell || !legal[cell] || heat[cell]<=heat[item.cell]) continue;
                    const Point p=center(cell);
                    const double dlen=detour*(side_cost(item.bottom,p)+side_cost(item.top,p)-base);
                    const double value=price*(heat[cell]-heat[item.cell])-dlen;
                    if(value<=0) continue;
                    best.push_back({value,cell,dlen});
                    if(best.size()>24) {
                        std::nth_element(best.begin(),best.begin()+8,best.end(),[](const auto& a,const auto& b){return a.gain>b.gain;});
                        best.resize(8);
                    }
                }
            std::sort(best.begin(),best.end(),[](const auto& a,const auto& b){return a.gain>b.gain;});
            if(best.size()>8) best.resize(8);
            options_by_net[n]=std::move(best);
        }
        using Item=std::tuple<double,size_t,size_t>;
        std::priority_queue<Item> queue;
        for(size_t n=0;n<k;++n) if(!options_by_net[n].empty()) queue.push({options_by_net[n][0].gain,n,0});
        auto moved=counts;size_t moves=0;double added=0;
        while(!queue.empty()) {
            const auto [value,n,index]=queue.top();queue.pop();(void)value;
            const auto& option=options_by_net[n][index];
            if(moved[option.target]) {
                if(index+1<options_by_net[n].size()) queue.push({options_by_net[n][index+1].gain,n,index+1});
                continue;
            }
            --moved[cross[n].cell];++moved[option.target];++moves;added+=option.dlen;
        }
        const double rho=pearson.rho(moved);
        const double dT=.25*(thermal_score(gain,rho)-thermal_score(gain,rho0));
        const double dW=.45*(wirelength_score(length+added,reference)-wirelength_score(length,reference));
        out<<(first?"":",")<<"\n {\"radius_gcells\":"<<radius<<",\"lambda_scale\":"<<lambda_scale
           <<",\"moves\":"<<moves<<",\"rho\":"<<rho<<",\"T\":"<<thermal_score(gain,rho)
           <<",\"added_length_proxy\":"<<added<<",\"dS_T\":"<<dT<<",\"dS_W\":"<<dW<<",\"dS_est\":"<<dT+dW<<'}';
        first=false;
        std::cerr<<"radius="<<radius<<" scale="<<lambda_scale<<" moves="<<moves<<" rho="<<rho<<" dS="<<dT+dW<<'\n';
    }
    out<<"]}\n";out.close();if(!out) throw std::runtime_error("output write failed");
    std::cout<<"TSV thermal diagnostic K="<<k<<" rho="<<rho0<<" legal="<<legal_count<<'\n';
    return 0;
 } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
