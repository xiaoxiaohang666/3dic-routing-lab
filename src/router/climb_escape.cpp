#include "router/negotiated_router.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace eda {
// A bounded window around a terminal, reached at live search costs. Each cell
// reached on a layer is a place the net could start or finish its climb, so
// the wire leaves a full column instead of stacking vias straight through it.
bool NegotiatedRouter::climb_column_congested(int root,const NetRoute& own,int low,int high) const {
    const int c=graph_.col(root),r=graph_.row(root),die=(graph_.layer(root)-1)/10;
    for(int layer=low;layer<=high;++layer) {
        if(layer==graph_.layer(root) || (layer-1)/10!=die) continue;
        const int node=graph_.node(c,r,layer);
        if(node<0 || own.nodes.count(node) || pin_exempt(own.net_index,node)) continue;
        if(graph_.official_obstacle_demand(node)+search_cell_demand(node)>=graph_.official_capacity(node)) return true;
    }
    return false;
}
ClimbEscapeField NegotiatedRouter::climb_escape_field(int root,const NetRoute& own,int low,int high) const {
    const int radius=options_.climb_escape_radius,root_layer=graph_.layer(root),die=(root_layer-1)/10;
    const int open_low=std::max(low,die*10+1),open_high=std::min(high,die*10+10);
    ClimbEscapeField field;
    field.root=root;field.radius=radius;field.c0=graph_.col(root);field.r0=graph_.row(root);
    // A terminal can sit just outside the layers open to this search: a TSV
    // landing on M10 while the lower die routes up to M9. Keep its cell in the
    // field, but as in A* no other cell of that layer can be entered.
    field.low=std::min(open_low,root_layer);field.high=std::max(open_high,root_layer);
    field.distance.assign(field.size(),std::numeric_limits<double>::infinity());
    field.parent.assign(field.size(),-1);
    using Item=std::pair<double,int>;
    std::priority_queue<Item,std::vector<Item>,std::greater<Item>> todo;
    field.distance[size_t(field.index(root,graph_))]=0;todo.emplace(0.0,root);
    while(!todo.empty()) {
        const auto [cost,n]=todo.top();todo.pop();
        if(cost>field.distance[size_t(field.index(n,graph_))]) continue;
        for(int next:graph_.neighbors(n)) {
            const int at=field.index(next,graph_);
            if(at<0) continue;
            if(const int layer=graph_.layer(next);layer<open_low || layer>open_high) continue;
            const double through=cost+edge_cost(n,next,own);
            if(through<field.distance[size_t(at)]) {
                field.distance[size_t(at)]=through;field.parent[size_t(at)]=n;todo.emplace(through,next);
            }
        }
    }
    return field;
}
// Cheapest place on `layer` to leave the window for `other`. A tie keeps the
// cell nearest the terminal, so a free direct climb is never shifted.
int NegotiatedRouter::climb_escape_exit(const ClimbEscapeField& field,int layer,int other) const {
    if(layer<field.low || layer>field.high) return -1;
    const double unit=official_search_scale()/graph_.ell0();
    const Point goal=graph_.point(other);
    int best=-1,best_shift=0;double best_cost=std::numeric_limits<double>::infinity();
    for(int dr=-field.radius;dr<=field.radius;++dr) for(int dc=-field.radius;dc<=field.radius;++dc) {
        const int n=graph_.node(field.c0+dc,field.r0+dr,layer);
        if(n<0) continue;
        const double reach=field.distance[size_t(field.index(n,graph_))];
        if(!std::isfinite(reach)) continue;
        const double total=reach+unit*graph_.point(n).distance_to(goal);
        const int shift=std::abs(dc)+std::abs(dr);
        if(total<best_cost || (total==best_cost && shift<best_shift)) {best_cost=total;best=n;best_shift=shift;}
    }
    return best;
}
std::vector<int> ClimbEscapeField::path_to(int node,const RoutingGraph& graph) const {
    std::vector<int> path;
    for(int n=node;n!=-1;n=parent[size_t(index(n,graph))]) path.push_back(n);
    std::reverse(path.begin(),path.end());
    return path;
}
// Whether the way to `node` steps off the terminal's column. An exit in that
// column can still have stepped aside: leave a full cell by the next column,
// then return above it. Only a straight climb equals the template.
bool ClimbEscapeField::leaves_column(int node,const RoutingGraph& graph) const {
    for(int n=node;n!=-1;n=parent[size_t(index(n,graph))])
        if(graph.col(n)!=c0 || graph.row(n)!=r0) return true;
    return false;
}
}
