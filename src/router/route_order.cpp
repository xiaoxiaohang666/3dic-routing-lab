#include "router/negotiated_router.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <numeric>

namespace eda {
namespace {
// Window used when the sideways step is off: templates can still run along the
// pin layer, so a terminal's neighbourhood matters without it too.
constexpr int order_window_without_escape=3;
}
bool NegotiatedRouter::full_before_routing(size_t net,int node) const {
    if(node<0) return true;
    if(pin_exempt(net,node)) return false;
    return graph_.official_obstacle_demand(node)>=graph_.official_capacity(node);
}
// Difficulty of leaving one terminal, from the input only:
// - straight_full: full cells crossed in its own column before reaching two
//   adjacent open layers with different directions, where the net can turn.
// - usable_columns: columns of the window reachable through open cells where
//   both directions are open. Sideways steps can only use these.
RouteOrderTerminal NegotiatedRouter::route_order_terminal(size_t net,int col,int row,int layer,int low,int high) const {
    RouteOrderTerminal result;
    const auto horizontal=[&](int l){return lef_.get_layer_direction(get_layer_name(l))==Direction::HORIZONTAL;};
    // A side with no place to turn does not count: a landing on M10 has nothing
    // above it, and that must not make the blocked layers below look easy.
    constexpr int no_turn=std::numeric_limits<int>::max();
    int full_on_die=0;
    const auto walk=[&](int step) {
        int crossed=0;
        for(int l=layer;l+step>=low && l+step<=high;l+=step) {
            const bool next_full=full_before_routing(net,graph_.node(col,row,l+step));
            if(!next_full && (l==layer || !full_before_routing(net,graph_.node(col,row,l))) && horizontal(l)!=horizontal(l+step))
                return crossed;
            crossed+=next_full;
            full_on_die+=next_full;
        }
        return no_turn;
    };
    const int shortest=std::min(walk(1),walk(-1));
    result.straight_full=shortest==no_turn?full_on_die+1:shortest;
    if(!result.straight_full) return result;

    const int radius=options_.climb_escape_radius>0?options_.climb_escape_radius:order_window_without_escape;
    const auto& grid=graph_.grid();
    result.window_columns=(std::min(col+radius,grid.cols-1)-std::max(col-radius,0)+1)*
                          (std::min(row+radius,grid.rows-1)-std::max(row-radius,0)+1);
    const int side=2*radius+1,bottom=std::min(low,layer),layers=std::max(high,layer)-bottom+1;
    const auto column_at=[&](int n){return (graph_.row(n)-row+radius)*side+graph_.col(n)-col+radius;};
    std::vector<char> seen(size_t(side*side*layers));
    std::vector<unsigned char> directions(size_t(side*side));
    const int root=graph_.node(col,row,layer);
    std::vector<int> todo{root};
    seen[size_t(column_at(root)*layers+layer-bottom)]=1;
    while(!todo.empty()) {
        const int n=todo.back();todo.pop_back();
        directions[size_t(column_at(n))]|=horizontal(graph_.layer(n))?1:2;
        for(int next:graph_.neighbors(n)) {
            const int l=graph_.layer(next);
            if(l<low || l>high || std::abs(graph_.col(next)-col)>radius || std::abs(graph_.row(next)-row)>radius) continue;
            char& mark=seen[size_t(column_at(next)*layers+l-bottom)];
            if(mark || full_before_routing(net,next)) continue;
            mark=1;todo.push_back(next);
        }
    }
    result.usable_columns=int(std::count(directions.begin(),directions.end(),3));
    return result;
}
std::vector<size_t> NegotiatedRouter::initial_route_order(const std::vector<int>& assignment) {
    const auto& nets=def_.nets();
    std::vector<size_t> order(nets.size());std::iota(order.begin(),order.end(),0);
    auto hpwl=[](const Net& n) {Rect b(n.pins[0].coord.x,n.pins[0].coord.y,n.pins[0].coord.x,n.pins[0].coord.y);
        for(const auto& p:n.pins) b=Rect(std::min(b.x1,p.coord.x),std::min(b.y1,p.coord.y),std::max(b.x2,p.coord.x),std::max(b.y2,p.coord.y));
        return b.width()+b.height();};
    std::vector<double> spans;for(const auto& n:nets) spans.push_back(hpwl(n));
    const auto later_keys=[&](size_t a,size_t b,bool cross_first) {
        const auto& x=nets[a];const auto& y=nets[b];
        if(cross_first && x.is_cross_die!=y.is_cross_die) return x.is_cross_die>y.is_cross_die;
        if(x.pins.size()!=y.pins.size()) return x.pins.size()>y.pins.size();
        if(spans[a]!=spans[b]) return spans[a]>spans[b];
        return x.name<y.name;
    };
    std::stable_sort(order.begin(),order.end(),[&](size_t a,size_t b){return later_keys(a,b,true);});
    if(!options_.route_order_mode) return order;

    const auto started=std::chrono::steady_clock::now();
    const auto& grid=graph_.grid();
    // A net is as hard as its worst terminal: fanout already has its own key,
    // and one blocked pin is enough to strand a net. A cross-die net also has to
    // leave its assigned TSV landing, which build_tree routes as a terminal.
    std::vector<double> difficulty(nets.size());
    const auto charge=[&](size_t net,int col,int row,int layer,int low,int high) {
        if(col<0 || row<0) return;
        const auto terminal=route_order_terminal(net,col,row,layer,low,high);
        if(!terminal.straight_full) return;
        ++counters_.route_order_blocked_terminals;
        // Blocked terminals share the few open columns nearby, so all of them
        // compete; those with the fewest columns and the most full cells go first.
        difficulty[net]=std::max(difficulty[net],double(terminal.straight_full)/(1.0+terminal.usable_columns));
    };
    for(size_t i=0;i<nets.size();++i) {
        const int slot=i<assignment.size()?assignment[i]:-1;
        for(const auto& pin:nets[i].pins) {
            const bool upper=pin.layer_idx>10;
            charge(i,grid.get_col(pin.coord.x),grid.get_row(pin.coord.y),pin.layer_idx,upper?11:1,upper?20:(slot>=0?9:10));
        }
        if(slot>=0) {
            const Point landing=policy_.slots().at(size_t(slot)).point;
            const int col=grid.get_col(landing.x),row=grid.get_row(landing.y);
            charge(i,col,row,10,1,9);
            charge(i,col,row,11,11,20);
        }
    }
    const auto baseline=order;
    const bool cross_first=options_.route_order_mode==1;
    std::stable_sort(order.begin(),order.end(),[&](size_t a,size_t b) {
        if(difficulty[a]!=difficulty[b]) return difficulty[a]>difficulty[b];
        return later_keys(a,b,cross_first);
    });
    std::vector<size_t> old_position(nets.size());
    for(size_t k=0;k<baseline.size();++k) old_position[baseline[k]]=k;
    for(size_t k=0;k<order.size();++k) {
        counters_.route_order_hard_nets+=difficulty[order[k]]>0;
        counters_.route_order_promoted_nets+=difficulty[order[k]]>0 && k<old_position[order[k]];
    }
    counters_.route_order_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
    std::cout<<"[ROUTE-ORDER] mode="<<options_.route_order_mode<<" nets="<<nets.size()
             <<" hard_nets="<<counters_.route_order_hard_nets<<" promoted_nets="<<counters_.route_order_promoted_nets
             <<" blocked_terminals="<<counters_.route_order_blocked_terminals
             <<" seconds="<<counters_.route_order_seconds<<std::endl;
    return order;
}
}
