#include "router/negotiated_router.hpp"
#include "objectives/net_cell_contribution.hpp"
#include <cmath>
#include <iostream>
#include <unordered_map>

namespace eda {
namespace {
// Attachment proxy: a die side joins the TSV at its nearest terminal; wide
// nets use the distance to their terminal bounding box.
double attachment(const std::vector<Point>& pins,Point p) {
    if(pins.empty()) return 0;
    if(pins.size()>32) {
        double x1=pins[0].x,x2=x1,y1=pins[0].y,y2=y1;
        for(const auto& q:pins) {x1=std::min(x1,q.x);x2=std::max(x2,q.x);y1=std::min(y1,q.y);y2=std::max(y2,q.y);}
        return std::max({0.0,x1-p.x,p.x-x2})+std::max({0.0,y1-p.y,p.y-y2});
    }
    double best=std::numeric_limits<double>::infinity();
    for(const auto& q:pins) best=std::min(best,std::abs(q.x-p.x)+std::abs(q.y-p.y));
    return best;
}
} // namespace

// Decide shared TSV hubs before routing. Each candidate group size is scored
// by a deliberately conservative estimate of the complete-score change:
//   T: exact count-based Pearson of the landing layout (TSVs fixed afterwards);
//   W: attachment-length change x2, priced at L = 1.3 R (no routed length yet);
//   C: the unique layout's C upper bound from fixed demand plus its TSV
//      interface demand, against the stacked layout's bound (the loss can only
//      be smaller once routing adds demand to the unique layout).
// Hubs are adopted only when the best estimate exceeds 0.5 weighted points.
std::vector<int> NegotiatedRouter::choose_shared_landings(const std::vector<int>& unique) {
    shared_auto_candidates_.clear();shared_auto_adopted_=1;shared_auto_rho_unique_=0;
    policy_.clear_shared_statistics();
    if(!options_.official_model || unique.size()!=def_.nets().size())
        throw std::logic_error("automatic shared TSV cells need an official complete unique assignment");
    if(official_wirelength_!=0 || official_reference_!=0)
        throw std::logic_error("automatic shared TSV cells must be decided before routing");
    const auto& grid=graph_.grid();
    const size_t cells=size_t(grid.cols)*size_t(grid.rows);
    std::vector<double> heat(cells);
    for(int row=0;row<grid.rows;++row) for(int col=0;col<grid.cols;++col)
        heat[size_t(row)*grid.cols+col]=std::max(0.0,thermal_.get_temperature(col,row)-thermal_.ambient_temperature());
    const auto rho_of=[&](const std::vector<int>& layout) {
        std::vector<int> counts(cells,0);
        for(int slot:layout) if(slot>=0) {
            const auto& s=policy_.slots().at(size_t(slot));
            ++counts[size_t(graph_.row(s.lower_node))*grid.cols+graph_.col(s.lower_node)];
        }
        return PearsonState(heat,std::move(counts)).correlation();
    };
    const auto& metrics=options_.official_metrics;
    static const std::vector<int> no_pins;
    const auto own_pins=[&](size_t i) -> const std::vector<int>& {return i<own_pin_nodes_.size()?own_pin_nodes_[i]:no_pins;};
    const auto interface_q=[&](const std::vector<int>& layout) {
        std::unordered_map<int,long long> demand;
        for(size_t i=0;i<layout.size();++i) if(layout[i]>=0)
            for(const auto& [node,amount]:mandatory_tsv_contribution(graph_,policy_.slots().at(size_t(layout[i])).point,
                    metrics.tsv_congestion_ratio,metrics.tsv_track_demand,own_pins(i),metrics.a150_rules))
                demand[node]+=amount;
        long double q=0;
        for(const auto& [node,amount]:demand) {
            if(amount>std::numeric_limits<int>::max()) throw std::overflow_error("stacked TSV demand overflow");
            q+=official_resource_cell_contribution(graph_,node,int(amount)).squared-
               official_resource_cell_contribution(graph_,node,0).squared;
        }
        return q;
    };
    std::vector<std::vector<Point>> bottom(unique.size()),top(unique.size());
    for(size_t i=0;i<unique.size();++i) if(unique[i]>=0)
        for(const auto& pin:def_.nets()[i].pins) (pin.layer_idx>=11?top:bottom)[i].push_back(pin.coord);
    const auto attach=[&](size_t i,int slot) {
        const auto p=policy_.slots().at(size_t(slot)).point;
        return attachment(bottom[i],p)+attachment(top[i],p);
    };
    const double gain=metrics.thermal_gain;
    const auto thermal_score=[&](double rho) {return 50*(1+std::tanh(gain*rho));};
    const long double fixed_q=official_squared_overflow_;
    const double reference=double(full_reference_);
    const double rho_unique=rho_of(unique);shared_auto_rho_unique_=rho_unique;
    const double c_unique=double(100/std::sqrt(1+fixed_q+interface_q(unique)));
    constexpr double detour=2.0,length_ratio=1.3,adopt_margin=0.5;
    std::vector<int> best_layout;double best_estimate=adopt_margin;size_t best_limit=1;
    for(size_t limit:{size_t(4),size_t(8),size_t(16),size_t(24),size_t(32)}) {
        if(limit>policy_.shared_cell_limit()) continue;
        auto layout=policy_.group_shared(def_.nets(),thermal_,unique,limit,options_.tsv_policy.shared_hot_radius);
        double added=0;
        for(size_t i=0;i<unique.size();++i) if(unique[i]>=0 && layout[i]!=unique[i])
            added+=detour*(attach(i,layout[i])-attach(i,unique[i]));
        SharedAutoCandidate c;c.limit=limit;c.rho=rho_of(layout);
        c.delta_t=.25*(thermal_score(c.rho)-thermal_score(rho_unique));
        c.delta_w=reference>0?-45*added/(length_ratio*length_ratio*reference):0;
        c.delta_c=.30*(double(100/std::sqrt(1+fixed_q+interface_q(layout)))-c_unique);
        c.estimate=c.delta_t+c.delta_w+c.delta_c;
        shared_auto_candidates_.push_back(c);
        if(c.estimate>best_estimate) {best_estimate=c.estimate;best_limit=limit;best_layout=std::move(layout);}
    }
    policy_.clear_shared_statistics();
    std::cout<<"[SHARED-TSV] unique_rho="<<rho_unique<<" unique_C_bound="<<c_unique;
    for(const auto& c:shared_auto_candidates_) std::cout<<" k"<<c.limit<<"="<<c.estimate;
    if(best_limit==1) {std::cout<<" decision=unique\n";return unique;}
    std::cout<<" decision=k"<<best_limit<<'\n';
    shared_auto_adopted_=best_limit;
    // Re-run the chosen grouping so the policy statistics describe the result.
    auto adopted=policy_.group_shared(def_.nets(),thermal_,unique,best_limit,options_.tsv_policy.shared_hot_radius);
    if(adopted!=best_layout) throw std::logic_error("shared TSV grouping is not deterministic");
    return adopted;
}
} // namespace eda
