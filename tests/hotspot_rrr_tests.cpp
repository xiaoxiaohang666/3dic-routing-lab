#include "router/negotiated_router.hpp"
#include "scoring/compat_validation.hpp"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
using namespace eda;
namespace fs=std::filesystem;
namespace {
size_t checks=0;
void require(bool ok,const std::string& message) {++checks;if(!ok) throw std::runtime_error(message);}
void near(double a,double b,const std::string& message) {
    require(std::abs(a-b)<=1e-10*std::max({1.0,std::abs(a),std::abs(b)}),message);
}
std::string write(const fs::path& path,const std::string& contents) {
    std::ofstream out(path);out<<contents;out.close();require(bool(out),"write fixture");return path.string();
}
std::string geometry(const std::vector<NetRoute>& routes) {
    std::ostringstream out;
    for(const auto& r:routes) {
        out<<r.slot<<';';
        for(const auto& g:r.guides) out<<g.layer<<','<<g.x1<<','<<g.y1<<','<<g.x2<<','<<g.y2<<';';
        out<<'\n';
    }
    return out.str();
}
}
namespace eda {
struct HotspotRrrTestAccess {
    static void exact(NegotiatedRouter& r) {
        r.complete_validation_cached_=false;r.validate_complete();
        std::vector<NetGuideMetrics> measured;measured.reserve(r.routes_.size());
        std::vector<int> demand(r.graph_.size());GuideMap guides;
        for(size_t i=0;i<r.routes_.size();++i) {
            const auto& route=r.routes_[i];guides[r.def_.nets()[i].name]=route.guides;
            measured.push_back(measure_official_net_guides(r.def_.nets()[i],route.guides,r.graph_,r.lef_,
                r.options_.official_metrics,r.reference_lengths_[i]));
            require(measured.back().demand==route.measured.demand,"fresh geometry equals cached net demand");
            for(auto [node,amount]:measured.back().demand) demand[node]+=amount;
        }
        require(demand==r.demand_,"fresh guide demand equals transactional ledger");
        require(compat::validate(r.def_,r.lef_,guides).error_count==0,"encoded complete solution is legal");
        std::vector<const NetGuideMetrics*> views;for(const auto& net:measured) views.push_back(&net);
        const auto fresh=aggregate_official_metrics(views,demand,r.graph_,r.thermal_,r.options_.official_metrics,false);
        const auto cached=r.measure();r.verify_official_ledger(cached);
        near(fresh.guide.quality,cached.guide.quality,"fresh score equals live score");
        near(cached.guide.quality,r.best_metrics_.guide.quality,"incumbent score equals live score");
    }
    static void rejected_account_is_atomic(NegotiatedRouter& r) {
        const auto demand=r.demand_;
        const auto usage=r.usage_;
        const auto edge_owners=r.edge_nets_;
        const auto cell_owners=r.cell_nets_;
        const auto before=r.measure();
        const auto wirelength=r.official_wirelength_,reference=r.official_reference_,q=r.official_squared_overflow_;
        const auto pending_q=r.future_tsv_squared_delta_;
        const auto overflow=r.official_overflow_;
        const auto zeros=r.official_zero_capacity_cells_;
        const auto generation=r.state_generation_;
        const auto validation_cached=r.complete_validation_cached_;
        bool rejected=false;
        try {r.account(0,r.routes_.front(),1);}catch(const std::exception&){rejected=true;}
        require(rejected,"duplicate account is rejected before any ownership mutation");
        require(r.demand_==demand&&r.usage_==usage&&r.edge_nets_==edge_owners&&r.cell_nets_==cell_owners,
                "rejected account preserves all real demands and reverse owners");
        require(r.official_wirelength_==wirelength&&r.official_reference_==reference&&
                r.official_squared_overflow_==q&&r.future_tsv_squared_delta_==pending_q&&
                r.official_overflow_==overflow&&r.official_zero_capacity_cells_==zeros,
                "rejected account preserves W/reference/Q/pending Q/overflow caches");
        require(r.state_generation_==generation&&r.complete_validation_cached_==validation_cached,
                "rejected account preserves transaction generation and validation cache");
        const auto after=r.measure();
        near(before.guide.quality,after.guide.quality,"rejected account preserves complete quality");
        near(before.guide.correlation,after.guide.correlation,"rejected account preserves actual TSV correlation");
        r.verify_official_ledger(after);
    }
    static void wasteful(NegotiatedRouter& r) {
        // The two same-die nets share a removable branch on low-capacity M9.
        // Their terminals and TSV reservations stay fixed, so any improvement
        // must come from real encoded route replacement.
        for(size_t i:{3U,4U}) {
            r.account(i,r.routes_[i],-1);
            auto trial=r.build_candidate(i,-1,0);
            int previous=r.graph_.node(0,0,1);trial.nodes.insert(previous);
            for(int layer=2;layer<=9;++layer) {
                const int node=r.graph_.node(0,0,layer);trial.nodes.insert(node);
                trial.edges.insert(route_edge(previous,node));previous=node;
            }
            for(int c=1;c<6;++c) {
                const int node=r.graph_.node(c,0,9);trial.nodes.insert(node);
                trial.edges.insert(route_edge(previous,node));previous=node;
            }
            r.encode(i,trial);r.routes_[i]=std::move(trial);r.account(i,r.routes_[i],1);
        }
        r.best_metrics_=r.measure();exact(r);
        require(r.official_movable_priority(3)>0 && r.official_movable_priority(4)>0,
                "fixture has real shared removable congestion");
    }
    static void cleanup(NegotiatedRouter& r) {
        require(r.route_all(),"cleanup fixture initial route");
        r.options_.local_repair=false;r.options_.pin_local=false;
        r.options_.rrr_score_aware=true;r.options_.v7_rrr=true;
        wasteful(r);
        const auto before=r.best_metrics_;const auto deadline=r.deadline_;
        const auto present=r.present_;const auto history=r.history_scale_;
        const auto maze=r.options_.v7_rrr_astar_net_limit;const auto scope=r.v7_scope_limit_;
        r.options_.overflow_cleanup_rounds=2;
        r.run_overflow_cleanup();
        require(r.counters_.overflow_cleanup_rounds>0 && r.counters_.overflow_cleanup_single>0,
                "cleanup visits the nets in overflowing cells");
        require(r.best_metrics_.guide.quality+1e-10>=before.guide.quality,"cleanup never lowers exact quality");
        require(r.deadline_==deadline && r.present_==present && r.history_scale_==history &&
                r.options_.v7_rrr_astar_net_limit==maze && r.v7_scope_limit_==scope && !r.v7_refinement_active_ &&
                !r.v9_rrr_checkpoint_active_,"cleanup restores search settings and ends its checkpoint");
        exact(r);
        // Blockers never include the net itself and respect the group cap.
        wasteful(r);
        r.options_.overflow_cleanup_group=1;
        for(size_t i:{3U,4U}) {
            const auto b=r.overflow_blockers(i);
            require(b.size()<=1 && std::find(b.begin(),b.end(),i)==b.end(),"blockers exclude the net and respect the cap");
        }
        r.options_.overflow_cleanup_group=8;
        // A joint transaction that cannot route restores every route and ledger.
        const auto checkpoint=geometry(r.routes_);const auto demand=r.demand_;
        const auto cells=r.cell_nets_;const auto edges=r.edge_nets_;const auto stable=r.best_metrics_;
        const auto saved_deadline=r.deadline_;r.deadline_=std::chrono::steady_clock::now()-std::chrono::seconds(1);
        require(!r.joint_rrr_round({3,4}),"joint transaction past the deadline is rejected after lifting");
        require(geometry(r.routes_)==checkpoint && r.demand_==demand && r.cell_nets_==cells && r.edge_nets_==edges,
                "rejected joint transaction restores routes, demand and owners");
        near(r.best_metrics_.guide.quality,stable.guide.quality,"rejected joint transaction keeps the incumbent score");
        r.deadline_=saved_deadline;exact(r);
        // With search, the joint move is kept only when it scores higher; either way the ledger stays exact.
        const bool kept=r.joint_rrr_round({3,4});
        if(kept) require(r.best_metrics_.guide.quality>stable.guide.quality+1e-10,"kept joint transaction improves exact quality");
        else require(geometry(r.routes_)==checkpoint && r.demand_==demand,"refused joint transaction restores geometry");
        exact(r);
        // The whole pass with blockers on keeps the same guarantees.
        wasteful(r);const auto again=r.best_metrics_;
        r.options_.overflow_cleanup_blockers=true;
        r.run_overflow_cleanup();
        require(r.best_metrics_.guide.quality+1e-10>=again.guide.quality,"cleanup with blockers never lowers exact quality");
        require(!r.v9_rrr_checkpoint_active_ && !r.v7_refinement_active_,"cleanup with blockers ends cleanly");
        exact(r);
    }
    static void polish(NegotiatedRouter& r) {
        require(r.route_all(),"polish fixture initial route");
        r.options_.local_repair=false;r.options_.pin_local=false;
        r.options_.rrr_score_aware=true;r.options_.v7_rrr=true;
        wasteful(r);
        const auto before=r.best_metrics_;const auto deadline=r.deadline_;
        const auto present=r.present_;const auto history=r.history_scale_;
        const auto maze=r.options_.v7_rrr_astar_net_limit;const auto scope=r.v7_scope_limit_;
        std::vector<std::vector<int>> tsv;for(const auto& route:r.routes_) tsv.push_back(route.measured.tsv_cells);
        // Past the deadline the pass changes nothing.
        const auto checkpoint=geometry(r.routes_);
        r.deadline_=std::chrono::steady_clock::now()-std::chrono::seconds(1);
        r.run_wirelength_polish();
        require(geometry(r.routes_)==checkpoint && r.counters_.wirelength_polish_attempted==0,"polish past the deadline changes nothing");
        r.deadline_=deadline;
        r.options_.wirelength_polish_rounds=2;
        r.run_wirelength_polish();
        require(r.counters_.wirelength_polish_rounds>0 && r.counters_.wirelength_polish_attempted>0,"polish visits nets above their reference");
        require(r.best_metrics_.guide.quality+1e-10>=before.guide.quality,"polish never lowers exact quality");
        require(r.counters_.wirelength_polish_accepted>0 && r.best_metrics_.guide.wirelength<before.guide.wirelength,
                "polish shortens the wasteful detours");
        for(size_t i=0;i<r.routes_.size();++i) require(r.routes_[i].measured.tsv_cells==tsv[i],"polish keeps every TSV cell");
        require(r.deadline_==deadline && r.present_==present && r.history_scale_==history &&
                r.options_.v7_rrr_astar_net_limit==maze && r.v7_scope_limit_==scope && !r.v7_refinement_active_ &&
                !r.force_rrr_maze_ && !r.v9_rrr_checkpoint_active_,"polish restores search settings and ends its checkpoint");
        exact(r);
    }
    static void initial_maze(NegotiatedRouter& r) {
        const auto expansions=r.options_.v7_rrr_astar_expansions;
        require(r.route_all(),"initial maze route");
        require(r.counters_.initial_maze_nets==r.routes_.size(),"every initial net gets the maze budget");
        require(!r.v7_refinement_active_ && r.options_.v7_rrr_astar_expansions==expansions,
                "initial maze puts the RRR budget back and disarms the search");
        exact(r);
    }
    static void fixed_work(NegotiatedRouter& full,NegotiatedRouter& incremental) {
        for(auto* r:{&full,&incremental}) {
            require(r->route_all(),"checkpoint comparison initial route");
            r->options_.local_repair=false;r->options_.pin_local=false;
            r->options_.rrr_score_aware=true;r->options_.v7_rrr=true;
            r->options_.v7_rrr_astar_net_limit=0;
            wasteful(*r);
        }
        incremental.options_.v9_incremental_rrr=true;
        incremental.options_.v9_checkpoint_groups=32;
        incremental.options_.v9_checkpoint_seconds=60;
        incremental.begin_rrr_checkpoint();
        const auto initial_full_scans=incremental.complete_validations_;
        for(const std::vector<size_t>& scope:std::vector<std::vector<size_t>>{{3},{4},{3,4},{0}}) {
            const bool lhs=full.rrr_round(scope),rhs=incremental.rrr_round(scope);
            require(lhs==rhs,"fixed group sequence accepts the same transactions");
            require(geometry(full.routes_)==geometry(incremental.routes_),"incremental group sequence preserves exact encoded routes");
            require(full.demand_==incremental.demand_,"fixed work produces identical demand");
            near(full.best_metrics_.guide.quality,incremental.best_metrics_.guide.quality,"fixed-work score agrees");
        }
        require(incremental.counters_.v9_local_validations>0,"successful groups execute local proofs");
        require(incremental.complete_validations_==initial_full_scans,"local groups do not scan the complete design");
        require(incremental.finish_rrr_checkpoint(true),"periodic full checkpoint agrees with all local proofs");
        incremental.v9_rrr_checkpoint_active_=false;exact(full);exact(incremental);

        // Corrupt only the aggregate after a committed local improvement.
        // The complete proof must detect drift, recover the previous geometry,
        // and rebuild all demand/owner/score ledgers rather than trust bad deltas.
        wasteful(incremental);
        const auto checkpoint=geometry(incremental.routes_);const auto demand=incremental.demand_;
        const auto expected=incremental.best_metrics_;const auto restores=incremental.counters_.v9_checkpoint_restores;
        incremental.begin_rrr_checkpoint();
        require(incremental.rrr_round({3,4}),"fault fixture has a locally accepted transaction");
        require(!incremental.v9_rrr_journal_.empty(),"checkpoint keeps earliest routes across accepted groups");
        incremental.official_wirelength_+=17;
        require(!incremental.finish_rrr_checkpoint(true),"full checkpoint rejects injected score-ledger drift");
        require(incremental.counters_.v9_checkpoint_restores==restores+1,"checkpoint rollback is observable");
        require(geometry(incremental.routes_)==checkpoint && incremental.demand_==demand,
                "failed complete proof restores the earlier complete solution");
        near(incremental.best_metrics_.guide.quality,expected.guide.quality,"rollback restores checkpoint quality");
        require(!incremental.v9_rrr_checkpoint_active_,"failed checkpoint ends the speculative session");exact(incremental);

        // A bad reverse owner must fail local conservation even if the Guide
        // itself is legal; recovery must repair the owner table too.
        incremental.begin_rrr_checkpoint();incremental.journal_rrr_scope({3});
        const auto old=incremental.routes_[3];
        require(!old.measured.demand.empty(),"ownership fault fixture has dynamic cells");
        const int node=old.measured.demand.front().first;
        incremental.cell_nets_[node].push_back(int(incremental.routes_.size()));
        bool rejected=false;
        try {incremental.validate_rrr_group({{3,old}});} catch(const std::exception&) {rejected=true;}
        require(rejected,"local proof rejects corrupt reverse ownership");
        incremental.restore_rrr_checkpoint("injected owner fault");exact(incremental);

        // A failed account(-1), unlike a failed path search, cannot safely use
        // the ordinary inverse-delta rollback. The earliest route journal must
        // rebuild all ledgers and terminate the speculative session instead.
        incremental.begin_rrr_checkpoint();
        const auto before_account_failure=geometry(incremental.routes_);
        const auto restores_before_account=incremental.counters_.v9_checkpoint_restores;
        incremental.demand_[node]=0;
        require(!incremental.rrr_round({3}),"failed ledger transaction rejects the whole speculative batch");
        require(!incremental.v9_rrr_checkpoint_active_ &&
                incremental.counters_.v9_checkpoint_restores==restores_before_account+1,
                "account failure exits checkpoint session after rebuilding ledgers");
        require(geometry(incremental.routes_)==before_account_failure,"account failure keeps checkpoint geometry");
        exact(incremental);

    }
    static void checkpoint_reuse(NegotiatedRouter& r) {
        const auto original_options=r.options_;
        r.options_.checkpoint_reuse=true;
        exact(r);
        const auto initial_scans=r.complete_validations_;
        const auto initial_hits=r.complete_validation_cache_hits_;
        r.begin_rrr_checkpoint();
        require(r.complete_validations_==initial_scans &&
                r.complete_validation_cache_hits_>initial_hits,
                "checkpoint entry reuses an already verified unchanged generation");
        require(r.finish_rrr_checkpoint(true),"unchanged checkpoint still receives an exit audit");
        require(r.complete_validations_==initial_scans+1,
                "forced checkpoint exit scans even the same unchanged generation");
        r.v9_rrr_checkpoint_active_=false;

        // Restoring the same route is still a ledger mutation: the generation
        // changes, so entry must establish a new complete proof.
        const auto old=r.routes_.front();
        const auto previous_generation=r.state_generation_;
        r.account(0,old,-1);r.account(0,old,1);
        require(r.state_generation_!=previous_generation && !r.complete_validation_cached_,
                "successful ledger mutations invalidate the complete proof");
        const auto dirty_scans=r.complete_validations_;
        r.begin_rrr_checkpoint();
        require(r.complete_validations_==dirty_scans+1,
                "changed generation cannot reuse checkpoint entry validation");
        require(r.finish_rrr_checkpoint(true),"changed generation passes a full checkpoint");
        r.v9_rrr_checkpoint_active_=false;

        // Fault injection deliberately bypasses account(). The independent
        // forced audit must still find corruption that a generation cache
        // cannot describe, then rebuild owners from the verified routes.
        r.begin_rrr_checkpoint();
        const auto geometry_before=geometry(r.routes_);const auto demand_before=r.demand_;
        const auto owners_before=r.cell_nets_;const auto restores=r.counters_.v9_checkpoint_restores;
        require(!r.routes_.front().measured.demand.empty(),"same-generation fault has an owned cell");
        const int node=r.routes_.front().measured.demand.front().first;
        r.cell_nets_[node].push_back(int(r.routes_.size()));
        require(r.complete_validation_cached_ && r.validated_generation_==r.state_generation_,
                "fault injection leaves the generation proof apparently current");
        require(!r.finish_rrr_checkpoint(true),"forced audit detects owner corruption despite cache hit eligibility");
        require(r.counters_.v9_checkpoint_restores==restores+1 && !r.v9_rrr_checkpoint_active_,
                "same-generation owner fault terminates and restores the checkpoint");
        require(geometry(r.routes_)==geometry_before && r.demand_==demand_before && r.cell_nets_==owners_before,
                "owner fault recovery restores complete geometry, demand and owners");
        exact(r);r.options_=original_options;
    }
    static void checkpoint_amortization(NegotiatedRouter& r) {
        using Clock=std::chrono::steady_clock;
        const auto original_options=r.options_;
        r.options_.checkpoint_reuse=false;
        r.options_.v9_checkpoint_groups=32;r.options_.v9_checkpoint_seconds=5;
        exact(r);

        // The original five-second timer stays effective when the new
        // experiment is disabled, irrespective of measured audit cost.
        r.options_.checkpoint_amortization=false;r.begin_rrr_checkpoint();
        r.last_complete_validation_seconds_=10;
        r.v9_rrr_checkpoint_time_=Clock::now()-std::chrono::seconds(6);
        r.v9_rrr_pending_groups_=1;
        auto scans=r.complete_validations_;
        require(r.finish_rrr_checkpoint(false),"legacy checkpoint time trigger still passes");
        require(r.complete_validations_==scans+1,"disabled amortization retains the configured time trigger");
        r.v9_rrr_checkpoint_active_=false;

        r.options_.checkpoint_amortization=true;r.begin_rrr_checkpoint();
        r.last_complete_validation_seconds_=10;
        const auto recorded_cost=r.last_complete_validation_seconds_;
        scans=r.complete_validations_;r.validate_complete();
        require(r.complete_validations_==scans && r.last_complete_validation_seconds_==recorded_cost,
                "validation cache hits preserve the most recent actual audit cost");
        r.v9_rrr_checkpoint_time_=Clock::now()-std::chrono::seconds(6);
        r.v9_rrr_pending_groups_=1;
        require(r.finish_rrr_checkpoint(false),"amortized checkpoint waits below its cost-aware time trigger");
        require(r.complete_validations_==scans && r.v9_rrr_pending_groups_==1,
                "six seconds does not cause another ten-second audit below the group cap");
        r.v9_rrr_pending_groups_=32;
        require(r.finish_rrr_checkpoint(false),"accepted-group cap still triggers a full audit");
        require(r.complete_validations_==scans+1 && r.v9_rrr_pending_groups_==0,
                "amortization cannot defer the accepted-group cap");

        r.last_complete_validation_seconds_=10;
        r.v9_rrr_checkpoint_time_=Clock::now()-std::chrono::seconds(21);
        r.v9_rrr_pending_groups_=1;scans=r.complete_validations_;
        require(r.finish_rrr_checkpoint(false),"cost-aware time trigger passes a complete audit");
        require(r.complete_validations_==scans+1,"twenty-one seconds exceeds twice the measured audit cost");

        // The additional cost-aware extension is capped at sixty seconds.
        r.last_complete_validation_seconds_=100;
        r.v9_rrr_checkpoint_time_=Clock::now()-std::chrono::seconds(61);
        r.v9_rrr_pending_groups_=1;scans=r.complete_validations_;
        require(r.finish_rrr_checkpoint(false),"bounded cost-aware interval still triggers an audit");
        require(r.complete_validations_==scans+1,"large audit cost cannot extend the added interval past sixty seconds");

        r.last_complete_validation_seconds_=10;
        r.v9_rrr_checkpoint_time_=Clock::now();r.v9_rrr_pending_groups_=0;
        scans=r.complete_validations_;
        require(r.finish_rrr_checkpoint(true),"forced final audit passes below all periodic triggers");
        require(r.complete_validations_==scans+1,"force bypasses every amortization delay");
        r.v9_rrr_checkpoint_active_=false;

        // Faults may wait for the scheduled audit, but a forced phase exit
        // must detect them and rebuild the verified state atomically.
        r.begin_rrr_checkpoint();r.last_complete_validation_seconds_=10;
        r.v9_rrr_checkpoint_time_=Clock::now()-std::chrono::seconds(6);
        r.v9_rrr_pending_groups_=1;
        const auto old_geometry=geometry(r.routes_);const auto old_demand=r.demand_;
        const auto old_owners=r.cell_nets_;const auto restores=r.counters_.v9_checkpoint_restores;
        require(!r.routes_.front().measured.demand.empty(),"amortized fault fixture has an owned resource");
        const int node=r.routes_.front().measured.demand.front().first;
        r.cell_nets_[node].push_back(int(r.routes_.size()));
        require(!r.finish_rrr_checkpoint(true),"forced amortized audit detects injected owner corruption");
        require(r.counters_.v9_checkpoint_restores==restores+1 && !r.v9_rrr_checkpoint_active_,
                "amortized fault recovery exits the speculative checkpoint session");
        require(geometry(r.routes_)==old_geometry && r.demand_==old_demand && r.cell_nets_==old_owners,
                "amortized audit fault restores complete geometry, demand and owners");
        exact(r);r.options_=original_options;
    }
    static void run(NegotiatedRouter& r) {
        require(r.route_all(),"fixture routes completely");exact(r);
        rejected_account_is_atomic(r);
        r.options_.local_repair=false;r.options_.pin_local=false;
        r.options_.rrr_score_aware=true;r.options_.v8_rrr_group_size=1;
        r.options_.rrr_net_limit=16;r.options_.v7_rrr_net_limit=16;
        r.options_.v7_rrr_astar_net_limit=0;
        wasteful(r);
        const auto before=r.best_metrics_;
        const auto other_routes=geometry({r.routes_[0],r.routes_[1],r.routes_[2]});
        if(r.options_.hotspot_cell_limit) {
            const auto old_geometry=geometry(r.routes_);const auto old_demand=r.demand_;
            const auto selected=r.select_hotspot_cells(r.options_.hotspot_cell_limit);
            require(selected.size()==2,"concentrated fixture selects exactly its requested two cells");
            require(selected==r.select_hotspot_cells(r.options_.hotspot_cell_limit),"equal cell relief has deterministic ordering");
            for(int node:selected)require(r.graph_.layer(node)==9&&r.graph_.row(node)==0,
                "focus picks the real shared removable branch, not fixed TSV or terminal occupancy");
            require(geometry(r.routes_)==old_geometry&&r.demand_==old_demand,
                "optimistic hotspot ranking never changes real geometry or demand");
        }
        const auto outer_options=r.options_;
        const auto outer_scope=r.v7_scope_limit_;
        const auto outer_topology_budget=r.v5_topology_trial_;
        const auto deadline=r.deadline_;
        r.run_hotspot_rrr(2);
        require(r.counters_.v8_rrr_groups>0,"hotspot scheduler creates real transactions");
        require(r.counters_.v8_rrr_groups_accepted>0,"a local transaction is retained");
        require(r.best_metrics_.guide.quality>before.guide.quality+1e-10,"group replacement improves exact quality");
        if(r.options_.hotspot_cell_limit) {
            require(r.counters_.hotspot_focus_builds>0&&r.counters_.hotspot_focus_owner_nets>0,
                "focused phase records candidate construction and owner work");
            require(geometry({r.routes_[0],r.routes_[1],r.routes_[2]})==other_routes,
                "bounded hotspot transactions leave unrelated net geometry intact");
        } else require(r.counters_.hotspot_focus_builds==0,"disabled focus retains the existing scheduler");
        near(r.best_metrics_.official.thermal_score,before.official.thermal_score,"fixed-slot groups retain thermal score");
        require(r.options_.v7_rrr==outer_options.v7_rrr &&
                r.options_.v7_rrr_astar_net_limit==outer_options.v7_rrr_astar_net_limit &&
                r.options_.v7_rrr_astar_seconds==outer_options.v7_rrr_astar_seconds &&
                r.v7_scope_limit_==outer_scope && !r.v7_refinement_active_ &&
                r.v5_topology_trial_==outer_topology_budget,"group phase restores outer search settings and topology budget flag");
        require(r.deadline_==deadline,"group phase does not extend global deadline");exact(r);

        // Independent failed groups must restore all demand and ownership,
        // leaving the previously accepted complete checkpoint available.
        wasteful(r);const auto checkpoint=geometry(r.routes_);const auto demand=r.demand_;
        const auto cells=r.cell_nets_;const auto edges=r.edge_nets_;const auto stable=r.best_metrics_;
        const auto rejected=r.counters_.v8_rrr_groups_rejected;
        const auto limit=r.options_.search_limit;r.options_.search_limit=0;
        r.run_hotspot_rrr(1);
        require(r.counters_.v8_rrr_groups_rejected>rejected,"failed local groups are independently rejected");
        require(geometry(r.routes_)==checkpoint && r.demand_==demand && r.cell_nets_==cells && r.edge_nets_==edges,
                "failed groups restore routes and every demand/ownership ledger");
        near(r.best_metrics_.guide.quality,stable.guide.quality,"failed groups retain incumbent score");
        r.options_.search_limit=limit;exact(r);
        // Nested callers may already have a topology budget active. A failed
        // group must preserve that active flag just as it restores a false one.
        r.v5_topology_trial_=true;r.options_.search_limit=0;
        r.run_hotspot_rrr(1);
        require(r.v5_topology_trial_,"phase restores an already active topology budget flag");
        require(geometry(r.routes_)==checkpoint && r.demand_==demand,"nested failed groups preserve complete geometry");
        r.v5_topology_trial_=outer_topology_budget;r.options_.search_limit=limit;exact(r);

        // No work is permitted after the shared cutoff, even if the phase has
        // rounds and candidate nets remaining.
        const auto attempted=r.counters_.v8_rrr_groups;
        r.deadline_=std::chrono::steady_clock::now()-std::chrono::seconds(1);
        r.run_hotspot_rrr(3);
        require(r.counters_.v8_rrr_groups==attempted,"expired global deadline prevents new groups");
        require(geometry(r.routes_)==checkpoint && r.demand_==demand,"expired phase preserves complete checkpoint");
        r.deadline_=deadline;exact(r);
    }
};
}
int main(int argc,char** argv) {
    try {
        if(argc!=3) throw std::runtime_error("hotspot_rrr_tests OUTPUT_DIRECTORY OFFICIAL_RUNTIME");
        const fs::path directory=argv[1];fs::create_directories(directory);
        std::string tech,def="UNITS DISTANCE MICRONS 10 ; DIEAREA ( 0 0 ) ( 600 600 ) ; GCELLGRID X 0 DO 7 STEP 100 ; GCELLGRID Y 0 DO 7 STEP 100 ; ";
        for(int layer=1;layer<=20;++layer) {
            const auto name=get_layer_name(layer);
            tech+="LAYER "+name+" TYPE ROUTING ; DIRECTION "+(layer%2?"HORIZONTAL":"VERTICAL")+" ; WIDTH 0.2 ; END "+name+"\n";
            const std::string track=layer==9?"5 DO 6 STEP 100":"5 DO 60 STEP 10";
            def+="TRACKS X "+track+" LAYER "+name+" ; TRACKS Y "+track+" LAYER "+name+" ; ";
        }
        def+="COMPONENTS 0 ; END COMPONENTS PINS 13 ; ";
        auto pin=[&](std::string name,int layer,int x,int y) {
            def+="- "+name+" + LAYER metal"+std::to_string(layer)+" ( -1 -1 ) ( 1 1 ) + FIXED ( "+std::to_string(x)+" "+std::to_string(y)+" ) N ; ";
        };
        for(int i=0;i<3;++i) {pin("a"+std::to_string(i),1,50+i*100,250);pin("b"+std::to_string(i),1,450,350+i*100);pin("c"+std::to_string(i),12,150+i*100,450);}
        pin("local_a",1,50,50);pin("local_b",1,550,550);pin("local_c",1,50,50);pin("local_d",1,550,550);
        def+="END PINS NETS 5 ; ";
        for(int i=0;i<3;++i) def+="- cross"+std::to_string(i)+" ( PIN a"+std::to_string(i)+" ) ( PIN b"+std::to_string(i)+" ) ( PIN c"+std::to_string(i)+" ) ; ";
        def+="- local ( PIN local_a ) ( PIN local_b ) ; - duplicate ( PIN local_c ) ( PIN local_d ) ; END NETS END DESIGN";
        LefParser lef;require(lef.parse(write(directory/"fixture.lef",tech)),"parse LEF");
        DefParser parsed;require(parsed.parse(write(directory/"fixture.def",def),lef,DefGridMapping::OFFICIAL_FULL),"parse DEF");
        std::ostringstream heat;heat<<"{\"meta\":{\"ambientTemp\":25},\"nodes\":[";
        for(int y=0;y<6;++y) for(int x=0;x<6;++x) {
            if(x||y) heat<<',';
            heat<<'['<<5+x*10<<','<<5+y*10<<",0,"<<26+x+6*y<<']';
        }
        heat<<"]}";ThermalParser thermal;
        require(thermal.parse_json(write(directory/"heat.json",heat.str()),parsed.gcell_grid(),parsed.dbu_per_micron(),
                parsed.die_area(),ThermalMapping::OFFICIAL_PREVIOUS),"parse heat");
        RouterOptions options;options.official_model=true;options.official_runtime=argv[2];
        options.graph_options={RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true};
        options.official_metrics.score_20261020=true;options.official_metrics.thermal_gain=3;
        options.official_metrics.gcell_width_dbu=parsed.gcell_pitch_x();options.official_metrics.gcell_height_dbu=parsed.gcell_pitch_y();
        options.tsv_policy.resource_mode=TsvResourceMode::GLOBAL_GRID;
        options.rrr_rounds=options.tsv_rounds=options.wirelength_rounds=0;options.tsv_local=false;
        options.v7_cell_cost=true;options.v8_rrr=true;options.time_limit=120;
        NegotiatedRouter router(parsed,lef,thermal,options);HotspotRrrTestAccess::run(router);
        options.v9_incremental_rrr=true;
        NegotiatedRouter checkpointed(parsed,lef,thermal,options);HotspotRrrTestAccess::run(checkpointed);
        NegotiatedRouter full(parsed,lef,thermal,options),incremental(parsed,lef,thermal,options);
        HotspotRrrTestAccess::fixed_work(full,incremental);
        HotspotRrrTestAccess::checkpoint_reuse(incremental);
        HotspotRrrTestAccess::checkpoint_amortization(incremental);
        {
            auto plain=options;plain.v9_incremental_rrr=false;
            NegotiatedRouter cleanup_plain(parsed,lef,thermal,plain);HotspotRrrTestAccess::cleanup(cleanup_plain);
            NegotiatedRouter cleanup_checkpointed(parsed,lef,thermal,options);HotspotRrrTestAccess::cleanup(cleanup_checkpointed);
            NegotiatedRouter polish_plain(parsed,lef,thermal,plain);HotspotRrrTestAccess::polish(polish_plain);
            NegotiatedRouter polish_checkpointed(parsed,lef,thermal,options);HotspotRrrTestAccess::polish(polish_checkpointed);
            auto maze=options;maze.initial_maze_expansions=20000;maze.initial_maze_seconds=0.5;
            NegotiatedRouter initial_maze(parsed,lef,thermal,maze);HotspotRrrTestAccess::initial_maze(initial_maze);
        }
        // The opt-in candidate is checked under the new scalar/via objective,
        // with and without deferred checkpoints. This is a synthetic contract
        // test; it does not authorize public-case quality experiments.
        options.hotspot_cell_limit=2;
        options.graph_options.congestion_overflow_scale=1;
        options.official_metrics.all_adjacent_overlaps=true;
        options.official_metrics.snap_reference_coordinates=true;
        options.v9_incremental_rrr=false;
        NegotiatedRouter focused(parsed,lef,thermal,options);HotspotRrrTestAccess::run(focused);
        options.v9_incremental_rrr=true;
        NegotiatedRouter focused_checkpointed(parsed,lef,thermal,options);HotspotRrrTestAccess::run(focused_checkpointed);
        std::cout<<"hotspot_rrr_tests: "<<checks<<" checks PASS\n";return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
