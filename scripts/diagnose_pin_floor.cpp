// Offline semantic discrimination only. Never used by routing or scoring acceptance.
#include "objectives/official_metrics.hpp"
#include "objectives/net_cell_contribution.hpp"
#include "scoring/compat_validation.hpp"
#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <unordered_map>

using namespace eda;
using Span = std::pair<uint32_t,uint32_t>;
namespace {
constexpr size_t models=10;
const std::array<const char*,models> names={"production_logical_additive", "real_tracks_only",
    "geometry_next_union", "geometry_drop_outside", "whole_shape_empty_next_union",
    "geometry_virtual_additive", "cell_virtual_union", "geometry_nearest_in_cell_union",
    "geometry_next_union_open_pins", "geometry_next_union_open_all"};
struct Shape {Rect rect; bool pin;};
struct Events {std::vector<Span> actual,next,inside,whole,nearest,open_pins,open_all,open_next; uint32_t zeros=0;};
std::pair<int,int> cells(double low,double high,const std::vector<double>& lines) {
    if(high<low) std::swap(high,low);
    const int n=int(lines.size())-1;
    if(high<lines.front()||low>lines.back()) return {0,0};
    if(high==low) {int c=std::clamp(int(std::upper_bound(lines.begin(),lines.end(),low)-lines.begin())-1,0,n-1);return {c,c+1};}
    if(high<=lines.front()||low>=lines.back()) return {0,0};
    return {std::clamp(int(std::upper_bound(lines.begin(),lines.end(),low)-lines.begin())-1,0,n),
        std::clamp(int(std::lower_bound(lines.begin(),lines.end(),high)-lines.begin()),0,n)};
}
uint32_t merged(std::vector<Span> spans) {
    if(spans.empty()) return 0;
    std::sort(spans.begin(),spans.end());auto [b,e]=spans.front();uint32_t count=0;
    for(size_t i=1;i<spans.size();++i) {auto [nb,ne]=spans[i];if(nb<=e)e=std::max(e,ne);else{count+=e-b;b=nb;e=ne;}}
    return count+e-b;
}
uint32_t join(const std::vector<Span>& actual,const std::vector<Span>& floor) {
    auto spans=actual;spans.insert(spans.end(),floor.begin(),floor.end());return merged(std::move(spans));
}
std::vector<std::string> fields(const std::string& line) {
    std::vector<std::string> v;std::stringstream in(line);std::string word;
    while(std::getline(in,word,',')){if(!word.empty()&&word.back()=='\r')word.pop_back();v.push_back(word);}return v;
}
}
int main(int argc,char**argv) {try {
    if(argc!=6) throw std::runtime_error("DEF LEF GUIDE official.csv output.json required");
    if(std::filesystem::exists(argv[5])||std::filesystem::exists(std::string(argv[5])+".diff.csv"))
        throw std::runtime_error("output must be new");
    LefParser lef;if(!lef.parse(argv[2]))throw std::runtime_error("LEF parse failed");
    DefParser def;if(!def.parse(argv[1],lef,DefGridMapping::OFFICIAL_FULL))throw std::runtime_error("DEF parse failed");
    RoutingGraph graph(def,lef,{RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true,true});
    const auto& g=def.gcell_grid();const size_t plane=size_t(g.cols)*g.rows;
    std::array<std::vector<double>,21> tracks;std::array<std::vector<Shape>,21> shapes;
    for(const auto& p:def.tracks()) for(const auto& name:p.layers) {
        const int l=get_layer_index(name);if(!l)continue;
        const bool h=lef.get_layer_direction(name)==Direction::HORIZONTAL;
        if(p.axis!=(h?'Y':'X'))continue;const auto& lines=h?g.y_lines:g.x_lines;
        for(int i=0;i<p.count;++i){double t=p.start+i*p.step;if(t>=lines.front()&&t<=lines.back())tracks[l].push_back(t);}
    }
    for(auto& v:tracks){std::sort(v.begin(),v.end());v.erase(std::unique(v.begin(),v.end()),v.end());}
    for(const auto& [name,c]:def.components()) {
        (void)name;const auto* m=lef.get_macro(c.macro_type);if(!m)throw std::runtime_error("missing macro");
        for(const auto& s:m->obstructions){int l=get_layer_index(s.layer);if(l)shapes[l].push_back({transform_macro_rect(s.rect,*m,c,def.dbu_per_micron()),false});}
        for(const auto& [pn,p]:m->pins){(void)pn;for(const auto& s:p.port_rects){int l=get_layer_index(s.layer);if(l)shapes[l].push_back({transform_macro_rect(s.rect,*m,c,def.dbu_per_micron()),true});}}
    }
    for(const auto& [pn,p]:def.io_pins()){(void)pn;for(const auto& s:p.port_rects)if(s.layer_idx>0)shapes[s.layer_idx].push_back({s.rect,true});}
    std::array<std::vector<uint32_t>,models> fixed;for(auto& v:fixed)v.resize(graph.size());
    uint64_t fallback=0,outside=0,whole_nonempty=0,raw_differences=0;
    for(int l=1;l<=20;++l) {
        const auto& v=tracks[l];const bool h=lef.get_layer_direction(get_layer_name(l))==Direction::HORIZONTAL;
        const auto& lines=h?g.y_lines:g.x_lines;std::unordered_map<size_t,Events> event;
        for(const auto& s:shapes[l]) {
            auto [cb,ce]=cells(s.rect.x1,s.rect.x2,g.x_lines);auto [rb,re]=cells(s.rect.y1,s.rect.y2,g.y_lines);
            const double shape_low=h?s.rect.y1:s.rect.x1,shape_high=h?s.rect.y2:s.rect.x2;
            const bool globally_empty=std::lower_bound(v.begin(),v.end(),shape_low)>=std::upper_bound(v.begin(),v.end(),shape_high);
            for(int r=rb;r<re;++r)for(int c=cb;c<ce;++c) {
                const size_t band=size_t(h?r:c);const double low=std::max(shape_low,lines[band]),high=std::min(shape_high,lines[band+1]);
                const auto begin=std::lower_bound(v.begin(),v.end(),low);
                const auto band_begin=std::lower_bound(v.begin(),v.end(),lines[band]);
                const auto band_end=band+2==lines.size()?std::upper_bound(v.begin(),v.end(),lines[band+1]):std::lower_bound(v.begin(),v.end(),lines[band+1]);
                const auto end=high==lines[band+1]&&band+2<lines.size()?std::lower_bound(v.begin(),v.end(),high):std::upper_bound(v.begin(),v.end(),high);
                auto& e=event[size_t(r)*g.cols+c];
                // Joint hypothesis: a half-open physical shape interval and
                // a per-RECT minimum index union. Changing the endpoint only
                // under the old logical-PIN model cannot test this combination.
                const auto open_end=std::lower_bound(v.begin(),v.end(),high);
                if(begin<open_end) {
                    e.open_all.emplace_back(uint32_t(begin-v.begin()),uint32_t(open_end-v.begin()));
                    if(s.pin)e.open_pins.emplace_back(uint32_t(begin-v.begin()),uint32_t(open_end-v.begin()));
                } else if(s.pin) {
                    const uint32_t i=uint32_t(begin-v.begin());e.open_next.emplace_back(i,i+1);
                }
                if(!s.pin&&begin<end)e.open_pins.emplace_back(uint32_t(begin-v.begin()),uint32_t(end-v.begin()));
                if(begin<end)e.actual.emplace_back(uint32_t(begin-v.begin()),uint32_t(end-v.begin()));
                else if(s.pin) {
                    ++fallback;++e.zeros;outside+=begin>=band_end;whole_nonempty+=!globally_empty;
                    const uint32_t i=uint32_t(begin-v.begin());e.next.emplace_back(i,i+1);
                    if(begin<band_end)e.inside.emplace_back(i,i+1);
                    if(globally_empty)e.whole.emplace_back(i,i+1);
                    if(band_begin<band_end) {
                        const double center=(low+high)/2;
                        auto nearest=std::lower_bound(band_begin,band_end,center);
                        if(nearest==band_end)--nearest;
                        else if(nearest!=band_begin&&center-*(nearest-1)<=*nearest-center)--nearest;
                        const uint32_t j=uint32_t(nearest-v.begin());e.nearest.emplace_back(j,j+1);
                    } else e.nearest.emplace_back(i,i+1);
                }
            }
        }
        for(auto& [cell,e]:event) {
            const size_t n=size_t(l-1)*plane+cell;const uint32_t raw=merged(e.actual);
            raw_differences+=raw!=uint32_t(graph.official_obstacle_demand(int(n))-graph.official_pin_zero_track_floor_demand(int(n)));
            fixed[1][n]=raw;fixed[2][n]=join(e.actual,e.next);fixed[3][n]=join(e.actual,e.inside);
            fixed[4][n]=join(e.actual,e.whole);fixed[5][n]=raw+e.zeros;
            fixed[6][n]=raw+(e.zeros>0);fixed[7][n]=join(e.actual,e.nearest);
            fixed[8][n]=join(e.open_pins,e.open_next);fixed[9][n]=join(e.open_all,e.open_next);
        }
        std::vector<Shape>().swap(shapes[l]);
    }
    for(size_t n=0;n<graph.size();++n)fixed[0][n]=graph.official_obstacle_demand(int(n));
    const auto guides=compat::read_guides(argv[3]);if(guides.size()!=def.nets().size())throw std::runtime_error("net count differs");
    std::vector<int> route(graph.size()),mandatory(graph.size());OfficialMetricOptions options;
    options.score_20261020=true;options.a150_rules=true;options.tsv_track_demand=true;
    options.gcell_width_dbu=def.gcell_pitch_x();options.gcell_height_dbu=def.gcell_pitch_y();
    for(const auto& net:def.nets()) {
        auto i=guides.find(net.name);if(i==guides.end())throw std::runtime_error("missing net");
        auto m=measure_official_net_guides(net,i->second,graph,lef,options,0);for(auto [n,d]:m.demand)route[n]+=d;
        const auto pins=graph.own_pin_nodes(net);
        for(auto point:m.tsv_points)for(auto [n,d]:mandatory_tsv_contribution(graph,point,1,true,pins,true))mandatory[n]+=d;
    }
    struct Expected {int cap,demand;};std::unordered_map<int,Expected> expected;
    std::ifstream in(argv[4]);if(!in)throw std::runtime_error("missing official CSV");
    std::string line;std::getline(in,line);const auto labels=fields(line);std::map<std::string,size_t> column;
    for(size_t i=0;i<labels.size();++i)column[labels[i]]=i;
    while(std::getline(in,line)) {
        const auto v=fields(line);auto val=[&](const std::string& key){return v.at(column.at(key));};
        const int n=graph.node(std::stoi(val("gcell_col")),std::stoi(val("gcell_row")),get_layer_index(val("layer")));
        if(!expected.emplace(n,Expected{std::stoi(val("capacity")),std::stoi(val("demand"))}).second)throw std::runtime_error("duplicate official cell");
    }
    std::ofstream diff(std::string(argv[5])+".diff.csv");
    diff<<"model,layer,col,row,capacity,fixed,route,total,official_total_or_minus1\n";
    std::ofstream out(argv[5]);out<<std::setprecision(17)<<"{\"diagnostic_only\":true,\"production_allowed\":false,\"raw_track_union_differences\":"<<raw_differences
        <<",\"zero_geometry_cell_events\":"<<fallback<<",\"outside_band_events\":"<<outside<<",\"clipped_empty_full_shape_nonempty\":"<<whole_nonempty<<",\"models\":{\n";
    for(size_t mode=0;mode<models;++mode) {
        uint64_t overflow=0,entries=0,extra=0,missing=0,different=0,capdiff=0,zero_capacity=0,fixed_total=0;
        long double q=0,mandatory_q=0;size_t examples=0;
        std::array<long double,21> layer_q{},layer_mandatory_q{};
        std::vector<long double> relief;
        for(size_t n=0;n<graph.size();++n) {
            const int cap=graph.official_capacity(int(n)),d=int(fixed[mode][n])+route[n],excess=std::max(0,d-cap);
            overflow+=excess;entries+=excess>0;fixed_total+=fixed[mode][n];
            if(mandatory[n]>route[n])throw std::runtime_error("mandatory TSV demand exceeds measured route demand");
            if(cap>0) {
                const long double cell_q=(static_cast<long double>(excess)/cap)*(static_cast<long double>(excess)/cap);
                const long double minimum_excess=std::max(0,int(fixed[mode][n])+mandatory[n]-cap);
                const long double minimum_q=(minimum_excess/cap)*(minimum_excess/cap);
                q+=cell_q;mandatory_q+=minimum_q;
                layer_q[graph.layer(int(n))]+=cell_q;layer_mandatory_q[graph.layer(int(n))]+=minimum_q;
                if(cell_q>minimum_q)relief.push_back(cell_q-minimum_q);
            }else zero_capacity+=d>0;
            const auto i=expected.find(int(n));bool mismatch=false;
            if(i==expected.end()){extra+=excess>0;mismatch=excess>0;}
            else{missing+=excess==0;different+=d!=i->second.demand;capdiff+=cap!=i->second.cap;mismatch=d!=i->second.demand||cap!=i->second.cap;}
            if(mismatch&&(mode==2||examples++<40))diff<<names[mode]<<','<<graph.layer(int(n))<<','<<graph.col(int(n))<<','<<graph.row(int(n))<<','<<cap<<','<<fixed[mode][n]<<','<<route[n]<<','<<d<<','<<(i==expected.end()?-1:i->second.demand)<<'\n';
        }
        if(mode)out<<",\n";
        out<<'"'<<names[mode]<<"\":{\"overflow\":"<<overflow<<",\"entries\":"<<entries<<",\"extra\":"<<extra<<",\"missing\":"<<missing<<",\"different\":"<<different
            <<",\"capacity_differences\":"<<capdiff<<",\"fixed_total\":"<<fixed_total<<",\"zero_capacity_positive\":"<<zero_capacity<<",\"Q\":"<<double(q)<<",\"C\":"<<100/std::sqrt(1+double(q))
            <<",\"observed_overflow_map_exact\":"<<(!extra&&!missing&&!different&&!capdiff&&!raw_differences?"true":"false");
        std::sort(relief.begin(),relief.end(),std::greater<long double>());
        long double top100=0;for(size_t i=0;i<std::min(size_t(100),relief.size());++i)top100+=relief[i];
        out<<",\"relaxation\":{\"valid\":"<<(zero_capacity?"false":"true")
           <<",\"ignores_connectivity_and_pin_access\":true,\"fixed_tsv_Q\":"<<double(mandatory_q)
           <<",\"C_upper_bound\":"<<100/std::sqrt(1+double(mandatory_q))
           <<",\"removable_Q_upper_bound\":"<<double(q-mandatory_q)
           <<",\"removable_cells\":"<<relief.size()<<",\"top100_removable_Q_upper_bound\":"<<double(top100)<<",\"layers\":[";
        for(int l=1;l<=20;++l) {if(l>1)out<<',';
            out<<"{\"layer\":"<<l<<",\"Q\":"<<double(layer_q[l])<<",\"fixed_tsv_Q\":"<<double(layer_mandatory_q[l])<<'}';}
        out<<"]}}";
    }
    out<<"},\"pair_disagreements\":[";
    const std::array<std::pair<size_t,size_t>,2> pairs={{{2,8},{8,9}}};
    for(size_t p=0;p<pairs.size();++p) {
        const auto [a,b]=pairs[p];uint64_t cells_different=0,one_more_route=0;
        int64_t units=0;std::vector<size_t> examples;
        for(size_t n=0;n<graph.size();++n)if(fixed[a][n]!=fixed[b][n]) {
            ++cells_different;units+=int64_t(fixed[a][n])-fixed[b][n];
            const int cap=graph.official_capacity(int(n));
            auto marginal=[&](size_t mode) {
                const long double overflow=std::max(0,int(fixed[mode][n])+route[n]-cap);
                const long double after=std::max(0,int(fixed[mode][n])+route[n]+1-cap);
                return cap>0?(after*after-overflow*overflow)/(static_cast<long double>(cap)*cap):0;
            };
            if(cap>0&&std::abs(marginal(a)-marginal(b))>1e-18L) {
                ++one_more_route;if(examples.size()<10)examples.push_back(n);
            }
        }
        if(p)out<<',';
        out<<"{\"a\":\""<<names[a]<<"\",\"b\":\""<<names[b]<<"\",\"fixed_different_cells\":"<<cells_different
            <<",\"fixed_units_a_minus_b\":"<<units<<",\"one_more_route_delta_Q_different_cells\":"<<one_more_route<<",\"examples\":[";
        for(size_t i=0;i<examples.size();++i) {
            const size_t n=examples[i];if(i)out<<',';
            out<<"{\"layer\":"<<graph.layer(int(n))<<",\"col\":"<<graph.col(int(n))<<",\"row\":"<<graph.row(int(n))
                <<",\"capacity\":"<<graph.official_capacity(int(n))<<",\"fixed_a\":"<<fixed[a][n]<<",\"fixed_b\":"<<fixed[b][n]<<",\"route\":"<<route[n]<<'}';
        }
        out<<"]}";
    }
    out<<"]}\n";if(!out||!diff)throw std::runtime_error("output failed");
    std::cout<<"pin floor discrimination completed; no production contract adopted\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
