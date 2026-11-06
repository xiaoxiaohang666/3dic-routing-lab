#include "parser/lef_parser.hpp"
#include "parser/def_parser.hpp"
#include "parser/thermal_parser.hpp"
#include "router/baseline_router.hpp"
#include "router/negotiated_router.hpp"
#include <filesystem>
#include <iostream>
#include <memory>

namespace {
void usage(const char* p) {
    std::cout<<"3D IC Global Router v9 (documented-rules-v1)\nUsage: "<<p<<" --def FILE --lef FILE --thermal-report FILE --output FILE\n"
        <<"Input aliases: --input-def, --lefs (repeatable), --thermal-json, -o\n"
        <<"--router negotiated|baseline  (default negotiated)\n"
        <<"--rrr-rounds N  --tsv-rounds N  (defaults 30, 10)\n"
        <<"--rrr-score-aware 0|1 (default 0; official movable-demand congestion priority)\n"
        <<"--rrr-net-limit N (default 128; 1..8192, requires rrr-score-aware 1)\n"
        <<"--v9-incremental-rrr 0|1 --v9-checkpoint-groups N (1..256) --v9-checkpoint-seconds S (>0..60)\n"
        <<"--v9-subtree-recovery 0|1 --v9-subtree-branches N (1..16) --v9-subtree-boundary N (2..64)\n"
        <<"--v9-access-rematch 0|1 --v9-rematch-edges N (2..32) --v9-rematch-access-seconds S (>0..1)\n"
        <<"--v9-tsv-coverage 0|1 --v9-tsv-stale-sweeps N (1..8) --v9-thermal-regions 0|1\n"
        <<"--v9-adaptive-budget 0|1 (stage allocation by measured gain/time, unified process clock)\n"
        <<"--v8-rrr 0|1 --v8-rrr-group-size N (1..256, default 48; hotspot transactions)\n"
        <<"--hotspot-cell-limit N (0..4096; default 0; opt-in concentrated hotspot selection)\n"
        <<"--overflow-cleanup-seconds S (0..7200; default 800 under --model official with the a150-20261029 rules and fast quality, otherwise 0=off; final pass rerouting each overflowing net, time taken from the same deadline)\n"
        <<"--overflow-cleanup-rounds N (1..64, default 4) --overflow-cleanup-blockers 0|1 (default 1 where the cleanup defaults on, otherwise 0; also lift nets filling full cells next to it)\n"
        <<"--overflow-cleanup-group N (1..32, default 8; most extra nets lifted with one overflowing net)\n"
        <<"--wirelength-polish-seconds S (0..36000; default 0=off; after the overflow cleanup, reroute every net alone with the maze search, longest excess first, keep only complete-score gains)\n"
        <<"--wirelength-polish-rounds N (1..64, default 2)\n"
        <<"--initial-maze-expansions N (0..10000000; default 0=off) --initial-maze-seconds S (0..60, per net; both nonzero turn it on: every initial net also gets the bounded maze search, templates stay the incumbent)\n"
        <<"--via-stack-radius N (0..3; default 0; local ordinary-via relocation during V8 RRR)\n"
        <<"--climb-escape-radius N (0..8; default 3 under --model official with the a150-20261029 rules and fast quality, otherwise 0; offer a sideways step before a via climb through full cells)\n"
        <<"--recovery-global-wirelength 0|1 (global-W recovery shortlist; default 0)\n"
        <<"--initial-short-first 0|1 (default 0; initial routing takes same-die nets shortest pin span first)\n"
        <<"--layer-swap 0|1 (default 0; adaptive-schedule stage: a climbing same-die net takes its cheapest path and\n"
        <<"  the nets filling it are rerouted; kept only if the score improves without adding wirelength)\n"
        <<"--layer-swap-blockers N (1..16, default 4; nets rerouted with each layer swap)\n"
        <<"--v8-tsv-rematch 0|1 --v8-tsv-rematch-group-size N (2..8, default 6)\n"
        <<"--v8-tsv-rematch-nets N (1..32768, default 256) --v8-tsv-rematch-seconds N (>0..3600)\n"
        <<"--v8-tsv-dynamic 0|1 --v8-tsv-probe-candidates N (1..8, default 2)\n"
        <<"--v8-recovery 0|1 (migration excess priority and layer/trunk candidates; requires large-recovery)\n"
        <<"--unified-budget 0|1 (all optional stages share routing cutoff; default 0)\n"
        <<"--congestion-price-cap N (>0..1e9, default 10000) --score-unit-cost 0|1 (finite Q loss)\n"
        <<"--fixed-resource-model production|observed-20261104 (default production; see docs/rules/fixed-resource-model-20261105.md)\n"
        <<"--congestion-target-q Q (>=0; default off; price and accept overflow at the C slope of this Q)\n"
        <<"--v7-rrr 0|1 (default 0; adaptive RRR and bounded congestion search)\n"
        <<"--v7-rrr-net-limit N (1..32768, default 4096)\n"
        <<"--v7-rrr-astar-net-limit N (0..4096, default 64; zero disables maze trials)\n"
        <<"--v7-rrr-astar-expansions N (1..1000000, default 20000, per rerouted net)\n"
        <<"--v7-rrr-astar-seconds N (>0..10, default 0.1, per rerouted net)\n"
        <<"--v7-tsv 0|1 (default 0; post-route TSV/thermal joint refinement)\n"
        <<"--v7-tsv-nets N (1..32768, default 256) --v7-tsv-candidates N (1..64, default 4)\n"
        <<"--v7-tsv-seconds N (>0..3600, default 30; within total routing budget)\n"
        <<"--v7-topology 0|1 (default 0; compare shared pin-rooted trees during initial/refinement routing)\n"
        <<"--v7-cell-cost 0|1 (default 0; incremental own-tree cell reuse)\n"
        <<"--rules-profile a150-20261029|a150|historical (official default a150-20261029; older profiles replay)\n"
        <<"--tsv-obstacle-scope interface|all-layers (a150 default interface: official Q&A checks only M10/M11)\n"
        <<"--resource-profile documented|20261020 (production uses documented resources)\n"
        <<"--future-tsv-reservation 0|1 (search-only unbuilt TSV promises)\n"
        <<"--resource-local-rrr 0|1 (global hotspots plus bounded landing repair)\n"
        <<"--joint-layer-recovery 0|1 (shared-tree joint layer/via recovery)\n"
        <<"--tsv-index-reuse 0|1 (reuse proposal index while assignments agree)\n"
        <<"--checkpoint-amortization 0|1 (bound checkpoint work using observed validation cost)\n"
        <<"--checkpoint-reuse 0|1 (reuse unchanged validated RRR state)\n"
        <<"--tsv-tree-move 0|1 (opt-in A150 v11 tree-preserving TSV candidate)\n"
        <<"--tsv-tree-move-seconds S (>0..1) --tsv-tree-move-expansions N (1..1000000)\n"
        <<"--low-layer-mask 0|1|2|3 (opt-in bottom/top-die low-layer chain lifting)\n"
        <<"--low-layer-seconds S (>0..3600) --low-layer-max-attempts N (1..1000000)\n"
        <<"--rrr-reserved-seconds S (0..3600; one RRR phase inside the total budget)\n"
        <<"--resource-candidate-pool 0|1 (resource-aware TSV retrieval before truncation)\n"
        <<"--thermal-coverage 0|1 (independent thermal exploration coverage)\n"
        <<"--tsv-track-demand 0|1 (5um landing-cell track union; default 0)\n"
        <<"--tsv-landing-pricing 0|1 (撤账后TSV候选报价; requires track demand and dynamic TSV)\n"
        <<"--tsv-finite-quote 0|1 (default 0; exact C change on the common other-net ledger; requires landing pricing)\n"
        <<"--tsv-landing-rrr 0|1 (fixed-TSV M11 movable-Guide repair; requires track demand and hotspot RRR)\n"
        <<"--large-recovery 0|1 (default 0; bounded fixed-TSV Guide wirelength recovery, official only)\n"
        <<"--large-recovery-nets N (1..4096, default 1024) --large-recovery-rounds N (1..2, default 1)\n"
        <<"--large-recovery-seconds N (0..3600 exclusive of zero, default 300; capped by total with unified-budget 1)\n"
        <<"--large-recovery-branches N (0..8, default 8; zero disables shared branch candidates)\n"
        <<"--recovery-rotate 0|1 (default 0; later recovery slices skip nets already attempted until\n"
        <<"  every qualifying net had one attempt, then start a new sweep; requires large-recovery)\n"
        <<"--search astar|dijkstra  --search-limit N (per search, default 250000)\n"
        <<"--time-limit SECONDS (default 120; scales with large net count)  --local-repair 0|1 (default 1)\n"
        <<"--wirelength-rounds N (default 3)  --pin-local 0|1 (default 1)\n"
        <<"--tsv-local 0|1 (default 1)  Fixed-TSV terminal path recovery\n"
        <<"--tsv-resource-mode auto|paired|projected|global-grid (default auto) --tsv-candidates N --tsv-max-candidates N\n"
        <<"--cell-pin-access 0|1 --fast-initial 0|1 (defaults 1)\n"
        <<"--fast-quality 0|1 (default 1; bounded official fast tree/Z improvements)\n"
        <<"--fast-layer-pruning 0|1 (default 1; equivalent pruning of dominated L candidates)\n"
        <<"--model physical|official (default physical; official uses documented October 20 rules)\n"
        <<"--score-profile 20261020 (selected automatically for official; legacy is diagnostic only)\n"
        <<"--official-score-pricing 0|1 (default 0; derivative-scaled W/C congestion prices)\n"
        <<"--official-runtime DIR --nonpreferred 0|1 (official Guide model)\n"
        <<"--objective-profile documented|site-20261029 (explicit G; all real overlaps; resource alignment pending)\n"
        <<"--thermal-gain G (>0; default 20 is a uniform proxy, never inferred from filename)\n"
        <<"--thermal-gain-estimate hottest-k (no official G: use 1.5 / hottest-K Pearson bound as an optimisation proxy;\n"
        <<"    the later of --thermal-gain and --thermal-gain-estimate wins)\n"
        <<"--tsv-assignment-polish 0|1 (default 0; projected resource move/swap proxy optimization)\n"
        <<"--tsv-polish-thermal weighted|pareto (default weighted; pareto requires official model)\n"
        <<"--tsv-polish-heat-weight NUMBER (default 2; experimental proxy, requires polish 1)\n"
        <<"--tsv-polish-candidates N (default 0: inherit tsv-candidates; max 65536)\n"
        <<"--tsv-polish-passes N (default 2; range 1..64) --tsv-polish-bidirectional 0|1 (default 0)\n"
        <<"--tsv-polish-thermal-candidates 0|1 (default 0; thermal-aware free-slot KD candidates)\n"
        <<"--tsv-polish-score-aware 0|1 (default 0; judge polish moves by predicted single-case score change:\n"
        <<"    exact Pearson for T, terminal MST change for W; adds thermal candidates; official model, weighted polish)\n"
        <<"--tsv-polish-length-ratio R (default 1.4; assumed final length / FLUTE reference for the W rate, 1..10)\n"
        <<"--tsv-shared-cell-limit N (default 1; experimental multi-net same-cell TSV hubs of at most N nets, global-grid only;\n"
        <<"    TSV move/rematch phases are skipped while N>1) --tsv-shared-hot-radius R (default 0; hottest hub within R rings)\n"
        <<"--tsv-shared-auto 0|1 (default 0; choose hub size 4..32 or unique before routing from a conservative score estimate)\n"
        <<"--tsv-block-refine 0|1 (default 0; occupied-slot cost-improving permutations, official only)\n"
        <<"--tsv-block-size N (3..64, default 16) --tsv-block-passes N (1..16, default 2)\n"
        <<"--tsv-block-max-groups N (1..65536) --tsv-block-max-evals N (1..100000000)\n"
        <<"--tsv-block-seconds N (>0..600, default 60)\n"
        <<"--v5-pin-rooted 0|1 (default 0; fixed-TSV pin-rooted shared tree candidates, official only)\n"
        <<"--v5-topology-max-pins N (3..128, default 32)\n"
        <<"--stats FILE  Write metrics, TSV policy, assignment and search counters\n";
}
int integer(const std::string& value,bool positive=false) {
    size_t end=0;long n=std::stol(value,&end);
    if(end!=value.size()||n<(positive?1:0)||n>100000000) throw std::runtime_error("invalid integer option: "+value);
    return int(n);
}
bool same_file(const std::string& a,const std::string& b) {
    return std::filesystem::weakly_canonical(a)==std::filesystem::weakly_canonical(b)||
        (std::filesystem::exists(a)&&std::filesystem::exists(b)&&std::filesystem::equivalent(a,b));
}
}
int main(int argc,char** argv) {
    try {
        if(argc<2) {usage(argv[0]);return 1;}
        std::string def_file,thermal_file,output="output/result.guide",stats,kind="negotiated";
        std::vector<std::string> lefs;eda::RouterOptions options;bool explicit_time=false,gain_estimate=false,explicit_polish_heat=false,explicit_polish_thermal=false,explicit_polish_budget=false,explicit_rrr_limit=false,explicit_recovery_budget=false;
        bool explicit_block_budget=false,explicit_topology_budget=false,explicit_nonpreferred=false,explicit_v7_budget=false,explicit_v7_tsv_budget=false;
        options.process_started=std::chrono::steady_clock::now();
        std::set<std::string> explicit_v8,explicit_v9;
        std::string rules_profile="a150-20261029",obstacle_scope="interface",objective_profile="documented";
        bool explicit_rules=false,explicit_scope=false,explicit_track_demand=false,explicit_climb_radius=false,explicit_cleanup_seconds=false,explicit_cleanup_blockers=false;
        for(int i=1;i<argc;++i) {
            std::string arg=argv[i];if(arg=="--help"||arg=="-h") {usage(argv[0]);return 0;}
            const std::set<std::string> recognized={"--def","--input-def","--lef","--lefs","--thermal-report","--thermal-json",
                "--rules-profile","--tsv-obstacle-scope","--output","-o","--stats","--router","--rrr-rounds","--rrr-score-aware","--rrr-net-limit","--tsv-rounds","--search","--time-limit","--search-limit","--local-repair","--wirelength-rounds","--pin-local","--tsv-local","--tsv-resource-mode","--tsv-candidates","--tsv-max-candidates","--cell-pin-access","--fast-initial","--fast-quality","--fast-layer-pruning","--model","--official-runtime","--thermal-gain","--official-score-pricing","--nonpreferred","--tsv-assignment-polish","--tsv-polish-heat-weight","--tsv-polish-thermal","--tsv-polish-candidates","--tsv-polish-passes","--tsv-polish-bidirectional","--tsv-polish-thermal-candidates","--tsv-polish-score-aware","--tsv-polish-length-ratio","--tsv-shared-cell-limit","--tsv-shared-hot-radius","--tsv-shared-auto","--thermal-gain-estimate","--congestion-target-q","--fixed-resource-model"};
            const std::set<std::string> recovery_options={"--large-recovery","--large-recovery-nets","--large-recovery-rounds","--large-recovery-seconds","--large-recovery-branches","--recovery-rotate"};
            const std::set<std::string> v5_options={"--tsv-block-refine","--tsv-block-size","--tsv-block-passes","--tsv-block-max-groups","--tsv-block-max-evals","--tsv-block-seconds","--v5-pin-rooted","--v5-topology-max-pins"};
            const std::set<std::string> v7_options={"--v7-topology","--v7-rrr","--v7-rrr-net-limit","--v7-rrr-astar-net-limit","--v7-rrr-astar-expansions","--v7-rrr-astar-seconds","--v7-cell-cost","--resource-profile","--v7-tsv","--v7-tsv-nets","--v7-tsv-candidates","--v7-tsv-seconds"};
            const std::set<std::string> v8_options={"--v8-rrr","--v8-rrr-group-size","--v8-tsv-rematch","--v8-tsv-rematch-group-size","--v8-tsv-rematch-nets","--v8-tsv-rematch-seconds","--v8-tsv-dynamic","--v8-tsv-probe-candidates","--v8-recovery","--unified-budget","--congestion-price-cap","--score-unit-cost"};
            const std::set<std::string> v9_options={"--v9-incremental-rrr","--v9-checkpoint-groups","--v9-checkpoint-seconds","--v9-subtree-recovery","--v9-subtree-branches","--v9-subtree-boundary","--v9-access-rematch","--v9-rematch-edges","--v9-rematch-access-seconds","--v9-tsv-coverage","--v9-tsv-stale-sweeps","--v9-thermal-regions","--v9-adaptive-budget"};
            const std::set<std::string> landing_options={"--tsv-track-demand","--tsv-landing-pricing","--tsv-finite-quote","--tsv-landing-rrr",
                "--future-tsv-reservation","--resource-local-rrr","--joint-layer-recovery","--resource-candidate-pool","--thermal-coverage","--tsv-index-reuse","--checkpoint-reuse","--checkpoint-amortization"};
            const std::set<std::string> low_layer_options={"--low-layer-mask","--low-layer-seconds","--low-layer-max-attempts","--rrr-reserved-seconds"};
            const std::set<std::string> tree_move_options={"--tsv-tree-move","--tsv-tree-move-seconds","--tsv-tree-move-expansions"};
            if(arg!="--recovery-global-wirelength"&&arg!="--initial-short-first"&&arg!="--layer-swap"&&arg!="--layer-swap-blockers"&&arg!="--via-stack-radius"&&arg!="--climb-escape-radius"&&arg!="--hotspot-cell-limit"&&arg!="--overflow-cleanup-seconds"&&arg!="--overflow-cleanup-rounds"&&arg!="--overflow-cleanup-blockers"&&arg!="--overflow-cleanup-group"&&arg!="--wirelength-polish-seconds"&&arg!="--wirelength-polish-rounds"&&arg!="--initial-maze-expansions"&&arg!="--initial-maze-seconds"&&arg!="--objective-profile"&&!tree_move_options.count(arg)&&!low_layer_options.count(arg)&&!landing_options.count(arg)&&!v9_options.count(arg)&&!v8_options.count(arg)&&!recognized.count(arg)&&!recovery_options.count(arg)&&!v5_options.count(arg)&&!v7_options.count(arg)&&arg!="--score-profile") throw std::runtime_error("unknown option: "+arg);
            if(i+1>=argc) throw std::runtime_error("missing option value: "+arg);
            const std::string value=argv[++i];if(value.empty()||value[0]=='-') throw std::runtime_error("invalid option value: "+arg);
            if(arg=="--recovery-global-wirelength") {
                const int enabled=integer(value);if(enabled>1)throw std::runtime_error("global wirelength recovery must be 0 or 1");
                options.recovery_global_wirelength=enabled;
            } else if(arg=="--initial-short-first") {
                const int enabled=integer(value);if(enabled>1)throw std::runtime_error("initial-short-first must be 0 or 1");
                options.initial_short_first=enabled;
            } else if(arg=="--layer-swap") {
                const int enabled=integer(value);if(enabled>1)throw std::runtime_error("layer-swap must be 0 or 1");
                options.layer_swap=enabled;
            } else if(arg=="--layer-swap-blockers") {
                const int blockers=integer(value,true);if(blockers>16)throw std::runtime_error("layer-swap-blockers out of range");
                options.layer_swap_blockers=size_t(blockers);
            } else if(arg=="--via-stack-radius") {
                const int radius=integer(value);if(radius>3) throw std::runtime_error("via stack radius out of range");
                options.via_stack_radius=radius;
            } else if(arg=="--climb-escape-radius") {
                const int radius=integer(value);if(radius>8) throw std::runtime_error("climb escape radius out of range");
                options.climb_escape_radius=radius;explicit_climb_radius=true;
            } else if(arg=="--overflow-cleanup-seconds") {
                size_t end=0;const double v=std::stod(value,&end);
                if(end!=value.size()||!std::isfinite(v)||v<0||v>7200) throw std::runtime_error("overflow-cleanup-seconds must be 0..7200");
                options.overflow_cleanup_seconds=v;explicit_cleanup_seconds=true;
            } else if(arg=="--overflow-cleanup-rounds") {
                const int v=integer(value);if(v<1||v>64) throw std::runtime_error("overflow-cleanup-rounds must be 1..64");
                options.overflow_cleanup_rounds=v;
            } else if(arg=="--overflow-cleanup-blockers") {
                const int v=integer(value);if(v>1) throw std::runtime_error("overflow-cleanup-blockers must be 0 or 1");
                options.overflow_cleanup_blockers=v;explicit_cleanup_blockers=true;
            } else if(arg=="--overflow-cleanup-group") {
                const int v=integer(value);if(v<1||v>32) throw std::runtime_error("overflow-cleanup-group must be 1..32");
                options.overflow_cleanup_group=size_t(v);
            } else if(arg=="--wirelength-polish-seconds") {
                size_t end=0;const double v=std::stod(value,&end);
                if(end!=value.size()||!std::isfinite(v)||v<0||v>36000) throw std::runtime_error("wirelength-polish-seconds must be 0..36000");
                options.wirelength_polish_seconds=v;
            } else if(arg=="--initial-maze-expansions") {
                size_t end=0;const long long v=std::stoll(value,&end);
                if(end!=value.size()||v<0||v>10000000) throw std::runtime_error("initial-maze-expansions must be 0..10000000");
                options.initial_maze_expansions=size_t(v);
            } else if(arg=="--initial-maze-seconds") {
                size_t end=0;const double v=std::stod(value,&end);
                if(end!=value.size()||!std::isfinite(v)||v<0||v>60) throw std::runtime_error("initial-maze-seconds must be 0..60");
                options.initial_maze_seconds=v;
            } else if(arg=="--wirelength-polish-rounds") {
                const int v=integer(value);if(v<1||v>64) throw std::runtime_error("wirelength-polish-rounds must be 1..64");
                options.wirelength_polish_rounds=v;
            } else if(arg=="--hotspot-cell-limit") {
                const int count=integer(value);if(count>4096) throw std::runtime_error("hotspot cell limit out of range");
                options.hotspot_cell_limit=size_t(count);
            } else if(arg=="--objective-profile") {
                if(value!="documented"&&value!="site-20261029-scalars"&&value!="site-20261029") throw std::runtime_error("invalid objective profile");
                objective_profile=value;
            } else if(arg=="--rules-profile") {
                if(value!="a150-20261029"&&value!="a150"&&value!="historical") throw std::runtime_error("invalid rules profile");
                rules_profile=value;explicit_rules=true;
            } else if(arg=="--tsv-obstacle-scope") {
                if(value!="all-layers"&&value!="interface") throw std::runtime_error("invalid TSV obstacle scope");
                obstacle_scope=value;explicit_scope=true;
            } else if(arg=="--def"||arg=="--input-def") def_file=value;
            else if(arg=="--lef"||arg=="--lefs") lefs.push_back(value);
            else if(arg=="--thermal-report"||arg=="--thermal-json") thermal_file=value;
            else if(arg=="--output"||arg=="-o") output=value;
            else if(arg=="--stats") stats=value;
            else if(arg=="--thermal-gain") {
                size_t end=0;const double gain=std::stod(value,&end);
                if(end!=value.size()||!std::isfinite(gain)||gain<=0||gain>1e6) throw std::runtime_error("thermal gain out of range");
                options.official_metrics.thermal_gain=gain;options.explicit_thermal_gain=true;
                gain_estimate=false; // the later of --thermal-gain / --thermal-gain-estimate wins
            }
            else if(arg=="--thermal-gain-estimate") {
                if(value!="hottest-k") throw std::runtime_error("thermal-gain-estimate must be hottest-k");
                gain_estimate=true;options.explicit_thermal_gain=false;
            }
            else if(arg=="--router") {if(value!="baseline"&&value!="negotiated") throw std::runtime_error("invalid router");kind=value;}
            else if(arg=="--rrr-rounds") options.rrr_rounds=integer(value);
            else if(tree_move_options.count(arg)) {
                if(arg=="--tsv-tree-move") {
                    const int v=integer(value);if(v>1) throw std::runtime_error("TSV tree move must be 0 or 1");
                    options.tsv_tree_move=v;
                } else if(arg=="--tsv-tree-move-expansions") {
                    const int v=integer(value,true);if(v>1000000) throw std::runtime_error("TSV tree expansions out of range");
                    options.tsv_tree_move_expansions=size_t(v);
                } else {
                    size_t end=0;const double v=std::stod(value,&end);
                    if(end!=value.size()||!std::isfinite(v)||v<=0||v>1) throw std::runtime_error("TSV tree seconds out of range");
                    options.tsv_tree_move_seconds=v;
                }
            }
            else if(low_layer_options.count(arg)) {
                if(arg=="--low-layer-mask") {
                    const int v=integer(value);if(v>3) throw std::runtime_error("low-layer mask must be 0..3");
                    options.low_layer_mask=v;
                } else if(arg=="--low-layer-max-attempts") {
                    const int v=integer(value,true);if(v>1000000) throw std::runtime_error("low-layer attempt budget out of range");
                    options.low_layer_max_attempts=size_t(v);
                } else {
                    size_t end=0;const double v=std::stod(value,&end);
                    if(end!=value.size()||!std::isfinite(v)||v<0||v>3600||(arg=="--low-layer-seconds"&&v==0))
                        throw std::runtime_error("low-layer/RRR seconds out of range");
                    if(arg=="--low-layer-seconds") options.low_layer_seconds=v;
                    else options.rrr_reserved_seconds=v;
                }
            }
            else if(landing_options.count(arg)) {
                const int v=integer(value);if(v>1) throw std::runtime_error("TSV landing boolean must be 0 or 1");
                if(arg=="--tsv-track-demand") { options.official_metrics.tsv_track_demand=v;explicit_track_demand=true; }
                else if(arg=="--tsv-landing-pricing") options.tsv_landing_pricing=v;
                else if(arg=="--tsv-finite-quote") options.tsv_finite_quote=v;
                else if(arg=="--tsv-landing-rrr") options.tsv_landing_rrr=v;
                else if(arg=="--future-tsv-reservation") options.future_tsv_reservation=v;
                else if(arg=="--resource-local-rrr") options.resource_local_rrr=v;
                else if(arg=="--joint-layer-recovery") options.joint_layer_recovery=v;
                else if(arg=="--resource-candidate-pool") options.resource_candidate_pool=v;
                else if(arg=="--thermal-coverage") options.thermal_coverage=v;
                else if(arg=="--tsv-index-reuse") options.tsv_index_reuse=v;
                else if(arg=="--checkpoint-reuse") options.checkpoint_reuse=v;
                else options.checkpoint_amortization=v;
            }
            else if(v9_options.count(arg)) {
                explicit_v9.insert(arg);
                if(arg=="--v9-rematch-access-seconds") {
                    size_t end=0;const double v=std::stod(value,&end);
                    if(end!=value.size()||!std::isfinite(v)||v<=0||v>1) throw std::runtime_error("v9 access seconds out of range");
                    options.v9_rematch_access_seconds=v;
                } else if(arg=="--v9-checkpoint-seconds") {
                    size_t end=0;const double v=std::stod(value,&end);
                    if(end!=value.size()||!std::isfinite(v)||v<=0||v>60) throw std::runtime_error("v9 checkpoint seconds out of range");
                    options.v9_checkpoint_seconds=v;
                } else if(arg=="--v9-checkpoint-groups"||arg=="--v9-subtree-branches"||arg=="--v9-subtree-boundary"||arg=="--v9-rematch-edges"||arg=="--v9-tsv-stale-sweeps") {
                    const size_t v=size_t(integer(value,true));
                    const size_t lo=(arg=="--v9-subtree-boundary"||arg=="--v9-rematch-edges")?2:1;
                    const size_t hi=arg=="--v9-checkpoint-groups"?256:arg=="--v9-subtree-branches"?16:arg=="--v9-subtree-boundary"?64:arg=="--v9-rematch-edges"?32:8;
                    if(v<lo||v>hi) throw std::runtime_error("v9 integer budget out of range");
                    if(arg=="--v9-checkpoint-groups") options.v9_checkpoint_groups=v;
                    else if(arg=="--v9-subtree-branches") options.v9_subtree_branches=v;
                    else if(arg=="--v9-subtree-boundary") options.v9_subtree_boundary=v;
                    else if(arg=="--v9-rematch-edges") options.v9_rematch_edges=v;
                    else options.v9_tsv_stale_sweeps=v;
                } else {
                    const int v=integer(value);if(v>1) throw std::runtime_error("v9 boolean must be 0 or 1");
                    if(arg=="--v9-incremental-rrr") options.v9_incremental_rrr=v;
                    else if(arg=="--v9-subtree-recovery") options.v9_subtree_recovery=v;
                    else if(arg=="--v9-access-rematch") options.v9_access_rematch=v;
                    else if(arg=="--v9-tsv-coverage") options.v9_tsv_coverage=v;
                    else if(arg=="--v9-thermal-regions") options.v9_thermal_regions=v;
                    else options.v9_adaptive_budget=v;
                }
            }
            else if(v8_options.count(arg)) {
                explicit_v8.insert(arg);
                if(arg=="--congestion-price-cap"||arg=="--v8-tsv-rematch-seconds") {
                    size_t end=0;const double v=std::stod(value,&end);
                    const double upper=arg=="--congestion-price-cap"?1e9:3600;
                    if(end!=value.size()||!std::isfinite(v)||v<=0||v>upper) throw std::runtime_error(arg+" out of range");
                    if(arg=="--congestion-price-cap") options.congestion_price_cap=v;else options.v8_tsv_rematch_seconds=v;
                } else if(arg=="--v8-rrr-group-size"||arg=="--v8-tsv-rematch-group-size"||arg=="--v8-tsv-rematch-nets"||arg=="--v8-tsv-probe-candidates") {
                    const size_t v=size_t(integer(value,true));
                    if(arg=="--v8-rrr-group-size") {if(v>256) throw std::runtime_error("v8 RRR group size must be 1..256");options.v8_rrr_group_size=v;}
                    else if(arg=="--v8-tsv-rematch-group-size") {if(v<2||v>8) throw std::runtime_error("v8 rematch group size must be 2..8");options.v8_tsv_rematch_group_size=v;}
                    else if(arg=="--v8-tsv-rematch-nets") {if(v>32768) throw std::runtime_error("v8 rematch nets must be 1..32768");options.v8_tsv_rematch_nets=v;}
                    else {if(v>8) throw std::runtime_error("v8 probe candidates must be 1..8");options.v8_tsv_probe_candidates=v;}
                } else {
                    const int v=integer(value);if(v>1) throw std::runtime_error("v8 boolean option must be 0 or 1");
                    if(arg=="--v8-rrr") options.v8_rrr=v;
                    else if(arg=="--v8-tsv-rematch") options.v8_tsv_rematch=v;
                    else if(arg=="--v8-tsv-dynamic") options.v8_tsv_dynamic=v;
                    else if(arg=="--v8-recovery") options.v8_recovery=v;
                    else if(arg=="--unified-budget") options.unified_budget=v;
                    else options.score_unit_cost=v;
                }
            }
            else if(v7_options.count(arg)) {
                if(arg=="--resource-profile") {
                    if(value!="documented"&&value!="20261020"&&value!="platform-20261024") throw std::runtime_error("invalid resource profile");
                    options.graph_options.platform_20261024=value=="platform-20261024";
                } else if(arg=="--v7-tsv-seconds") {
                    size_t end=0;const double v=std::stod(value,&end);
                    if(end!=value.size()||!std::isfinite(v)||v<=0||v>3600) throw std::runtime_error("v7-tsv-seconds must be >0 and <=3600");
                    options.v7_tsv_seconds=v;explicit_v7_tsv_budget=true;
                } else if(arg=="--v7-tsv-nets"||arg=="--v7-tsv-candidates") {
                    const int v=integer(value,true);explicit_v7_tsv_budget=true;
                    if(arg=="--v7-tsv-nets") {
                        if(v>32768) throw std::runtime_error("v7-tsv-nets must be 1..32768");
                        options.v7_tsv_nets=size_t(v);
                    } else {
                        if(v>64) throw std::runtime_error("v7-tsv-candidates must be 1..64");
                        options.v7_tsv_candidates=size_t(v);
                    }
                } else if(arg=="--v7-tsv") {
                    const int v=integer(value);if(v>1) throw std::runtime_error("v7-tsv must be 0 or 1");
                    options.v7_tsv=v;
                } else if(arg=="--v7-rrr-astar-seconds") {
                    size_t end=0;const double v=std::stod(value,&end);
                    if(end!=value.size()||!std::isfinite(v)||v<=0||v>10) throw std::runtime_error("v7-rrr-astar-seconds must be >0 and <=10");
                    options.v7_rrr_astar_seconds=v;explicit_v7_budget=true;
                } else {
                    const int v=integer(value);
                    if(arg=="--v7-rrr"||arg=="--v7-cell-cost"||arg=="--v7-topology") {
                        if(v>1) throw std::runtime_error("v7 boolean option must be 0 or 1");
                        if(arg=="--v7-rrr") options.v7_rrr=v;else if(arg=="--v7-topology") options.v7_topology=v;else options.v7_cell_cost=v;
                    } else {
                        explicit_v7_budget=true;
                        if(arg=="--v7-rrr-net-limit") {
                            if(v<1||v>32768) throw std::runtime_error("v7-rrr-net-limit must be 1..32768");
                            options.v7_rrr_net_limit=size_t(v);
                        } else if(arg=="--v7-rrr-astar-net-limit") {
                            if(v>4096) throw std::runtime_error("v7-rrr-astar-net-limit must be 0..4096");
                            options.v7_rrr_astar_net_limit=size_t(v);
                        } else {
                            if(v<1||v>1000000) throw std::runtime_error("v7-rrr-astar-expansions must be 1..1000000");
                            options.v7_rrr_astar_expansions=size_t(v);
                        }
                    }
                }
            }
            else if(v5_options.count(arg)) {
                if(arg=="--tsv-block-seconds") {
                    size_t end=0;const double v=std::stod(value,&end);
                    if(end!=value.size()||!std::isfinite(v)||v<=0||v>600)
                        throw std::runtime_error("tsv-block-seconds must be >0 and <=600");
                    options.tsv_policy.block_max_seconds=v;explicit_block_budget=true;
                } else {
                    const int v=integer(value);
                    if(arg=="--tsv-block-refine"||arg=="--v5-pin-rooted") {
                        if(v>1) throw std::runtime_error("v5 boolean option must be 0 or 1");
                        if(arg=="--tsv-block-refine") options.tsv_policy.assignment_block_refine=v;
                        else options.v5_pin_rooted=v;
                    } else if(arg=="--v5-topology-max-pins") {
                        if(v<3||v>128) throw std::runtime_error("v5-topology-max-pins must be 3..128");
                        options.v5_topology_max_pins=size_t(v);explicit_topology_budget=true;
                    } else {
                        explicit_block_budget=true;
                        if(arg=="--tsv-block-size") {
                            if(v<3||v>64) throw std::runtime_error("tsv-block-size must be 3..64");
                            options.tsv_policy.block_size=size_t(v);
                        } else if(arg=="--tsv-block-passes") {
                            if(v<1||v>16) throw std::runtime_error("tsv-block-passes must be 1..16");
                            options.tsv_policy.block_passes=size_t(v);
                        } else if(arg=="--tsv-block-max-groups") {
                            if(v<1||v>65536) throw std::runtime_error("tsv-block-max-groups must be 1..65536");
                            options.tsv_policy.block_max_groups=size_t(v);
                        } else {
                            if(v<1) throw std::runtime_error("tsv-block-max-evals must be positive");
                            options.tsv_policy.block_max_cost_evaluations=size_t(v);
                        }
                    }
                }
            }
            else if(recovery_options.count(arg)) {
                if(arg=="--large-recovery-seconds") {
                    size_t end=0;const double v=std::stod(value,&end);
                    if(end!=value.size()||!std::isfinite(v)||v<=0||v>3600) throw std::runtime_error("large-recovery-seconds must be >0 and <=3600");
                    options.large_recovery_seconds=v;explicit_recovery_budget=true;
                } else {
                    const int v=integer(value);
                    if(arg=="--large-recovery") {if(v>1) throw std::runtime_error("large-recovery must be 0 or 1");options.large_recovery=v;}
                    else if(arg=="--recovery-rotate") {if(v>1) throw std::runtime_error("recovery-rotate must be 0 or 1");options.recovery_rotate=v;}
                    else {
                        explicit_recovery_budget=true;
                        if(arg=="--large-recovery-nets") {if(v<1||v>4096) throw std::runtime_error("large-recovery-nets must be 1..4096");options.large_recovery_nets=size_t(v);}
                        else if(arg=="--large-recovery-rounds") {if(v<1||v>2) throw std::runtime_error("large-recovery-rounds must be 1..2");options.large_recovery_rounds=v;}
                        else {if(v>8) throw std::runtime_error("large-recovery-branches must be 0..8");options.large_recovery_branches=v;}
                    }
                }
            }
            else if(arg=="--rrr-score-aware") {
                const int v=integer(value);if(v>1) throw std::runtime_error("rrr-score-aware must be 0 or 1");
                options.rrr_score_aware=v;
            }
            else if(arg=="--rrr-net-limit") {
                explicit_rrr_limit=true;const int v=integer(value,true);
                if(v>8192) throw std::runtime_error("rrr-net-limit must be 1..8192");
                options.rrr_net_limit=size_t(v);
            }
            else if(arg=="--tsv-rounds") options.tsv_rounds=integer(value);
            else if(arg=="--wirelength-rounds") options.wirelength_rounds=integer(value);
            else if(arg=="--pin-local") {int v=integer(value);if(v>1) throw std::runtime_error("pin-local must be 0 or 1");options.pin_local=v;}
            else if(arg=="--tsv-local") {int v=integer(value);if(v>1) throw std::runtime_error("tsv-local must be 0 or 1");options.tsv_local=v;}
            else if(arg=="--tsv-resource-mode") {
                if(value=="auto") options.tsv_policy.resource_mode=eda::TsvResourceMode::AUTO;
                else if(value=="paired") options.tsv_policy.resource_mode=eda::TsvResourceMode::PAIRED;
                else if(value=="projected") options.tsv_policy.resource_mode=eda::TsvResourceMode::PROJECTED;
                else if(value=="global-grid") options.tsv_policy.resource_mode=eda::TsvResourceMode::GLOBAL_GRID;
                else throw std::runtime_error("invalid TSV resource mode");
            }
            else if(arg=="--tsv-polish-thermal") {
                explicit_polish_thermal=true;
                if(value!="weighted" && value!="pareto") throw std::runtime_error("invalid TSV polish thermal rule");
                options.tsv_policy.polish_thermal_pareto=value=="pareto";
            }
            else if(arg=="--tsv-polish-heat-weight") {explicit_polish_heat=true;size_t end=0;double v=std::stod(value,&end);if(end!=value.size()||!std::isfinite(v)||v<0||v>1000000) throw std::runtime_error("invalid TSV polish heat weight");options.tsv_policy.polish_heat_weight=v;}
            else if(arg=="--tsv-assignment-polish") {int v=integer(value);if(v>1) throw std::runtime_error("tsv-assignment-polish must be 0 or 1");options.tsv_policy.assignment_polish=v;}
            else if(arg=="--tsv-polish-candidates") {
                explicit_polish_budget=true;int v=integer(value);
                if(v>65536) throw std::runtime_error("tsv-polish-candidates must be 0..65536");
                options.tsv_policy.polish_candidates=size_t(v);
            }
            else if(arg=="--tsv-polish-passes") {
                explicit_polish_budget=true;int v=integer(value,true);
                if(v>64) throw std::runtime_error("tsv-polish-passes must be 1..64");
                options.tsv_policy.polish_passes=v;
            }
            else if(arg=="--tsv-polish-bidirectional") {
                explicit_polish_budget=true;int v=integer(value);
                if(v>1) throw std::runtime_error("tsv-polish-bidirectional must be 0 or 1");
                options.tsv_policy.polish_bidirectional=v;
            }
            else if(arg=="--tsv-shared-cell-limit") {
                const int v=integer(value,true);
                if(v>64) throw std::runtime_error("tsv-shared-cell-limit must be 1..64");
                options.tsv_policy.shared_cell_limit=size_t(v);
            }
            else if(arg=="--tsv-shared-auto") {
                const int v=integer(value);
                if(v>1) throw std::runtime_error("tsv-shared-auto must be 0 or 1");
                options.tsv_shared_auto=options.tsv_policy.shared_cells_deferred=v;
            }
            else if(arg=="--tsv-shared-hot-radius") {
                const int v=integer(value);
                if(v>64) throw std::runtime_error("tsv-shared-hot-radius must be 0..64");
                options.tsv_policy.shared_hot_radius=v;
            }
            else if(arg=="--tsv-polish-thermal-candidates") {
                explicit_polish_budget=true;const int v=integer(value);
                if(v>1) throw std::runtime_error("tsv-polish-thermal-candidates must be 0 or 1");
                options.tsv_policy.polish_thermal_candidates=v;
            }
            else if(arg=="--tsv-polish-score-aware") {
                const int v=integer(value);
                if(v>1) throw std::runtime_error("tsv-polish-score-aware must be 0 or 1");
                options.tsv_policy.polish_score_aware=v;
            }
            else if(arg=="--tsv-polish-length-ratio") {
                size_t end=0;const double v=std::stod(value,&end);
                if(end!=value.size()||!std::isfinite(v)||v<1||v>10) throw std::runtime_error("tsv-polish-length-ratio must be 1..10");
                options.tsv_policy.polish_length_ratio=v;
            }
            else if(arg=="--tsv-candidates") options.tsv_policy.nearest_candidates=size_t(integer(value,true));
            else if(arg=="--tsv-max-candidates") options.tsv_policy.max_candidates=size_t(integer(value,true));
            else if(arg=="--cell-pin-access"||arg=="--fast-initial"||arg=="--fast-quality"||arg=="--fast-layer-pruning") {
                int v=integer(value);if(v>1) throw std::runtime_error("boolean option must be 0 or 1");
                if(arg=="--cell-pin-access") options.cell_pin_access=v;
                else if(arg=="--fast-initial") options.fast_initial=v;
                else if(arg=="--fast-quality") options.fast_quality=v;else options.fast_layer_pruning=v;
            }
            else if(arg=="--model") {
                if(value!="physical" && value!="official") throw std::runtime_error("invalid model");
                options.official_model=value=="official";
            }
            else if(arg=="--official-runtime") options.official_runtime=value;
            else if(arg=="--score-profile") {
                if(value!="legacy-20261011" && value!="20261020") throw std::runtime_error("invalid score profile");
                if(value=="legacy-20261011") throw std::runtime_error("legacy score profile is diagnostic only; production routing uses 20261020");
                options.official_metrics.score_20261020=true;
                options.graph_options.score_20261020=options.official_metrics.score_20261020;
            }
            else if(arg=="--fixed-resource-model") {
                if(value!="production"&&value!="observed-20261104") throw std::runtime_error("fixed-resource-model must be production or observed-20261104");
                options.graph_options.observed_fixed_20261104=value=="observed-20261104";
            }
            else if(arg=="--congestion-target-q") {
                size_t end=0;const double v=std::stod(value,&end);
                if(end!=value.size()||!std::isfinite(v)||v<0) throw std::runtime_error("congestion-target-q must be nonnegative");
                options.official_metrics.congestion_target_q=v;
            }
            else if(arg=="--official-score-pricing") {
                const int v=integer(value);if(v>1) throw std::runtime_error("official-score-pricing must be 0 or 1");
                options.official_score_pricing=v;
            }
            else if(arg=="--nonpreferred") {int v=integer(value);if(v>1) throw std::runtime_error("nonpreferred must be 0 or 1");options.graph_options.allow_nonpreferred=v;explicit_nonpreferred=true;}
            else if(arg=="--search-limit") options.search_limit=size_t(integer(value,true));
            else if(arg=="--local-repair") {int v=integer(value);if(v>1) throw std::runtime_error("local-repair must be 0 or 1");options.local_repair=v;}
            else if(arg=="--search") {if(value!="astar"&&value!="dijkstra") throw std::runtime_error("invalid search");options.dijkstra=value=="dijkstra";}
            else if(arg=="--time-limit") {explicit_time=true;size_t end=0;options.time_limit=std::stod(value,&end);
                if(end!=value.size()||!std::isfinite(options.time_limit)||options.time_limit<=0) throw std::runtime_error("invalid time limit");}
        }
        if((explicit_scope||(explicit_rules&&rules_profile!="historical"))&&!options.official_model)
            throw std::runtime_error("A150/rules scope requires --model official");
        if(rules_profile=="historical"&&explicit_scope)
            throw std::runtime_error("historical replay cannot select an A150 obstacle scope");
        if(options.official_model&&rules_profile!="historical") {
            if(options.tsv_policy.resource_mode==eda::TsvResourceMode::AUTO)
                options.tsv_policy.resource_mode=eda::TsvResourceMode::GLOBAL_GRID;
            if(options.tsv_policy.resource_mode!=eda::TsvResourceMode::GLOBAL_GRID)
                throw std::runtime_error("A150 requires --tsv-resource-mode global-grid");
            if(explicit_track_demand&&!options.official_metrics.tsv_track_demand)
                throw std::runtime_error("A150 requires --tsv-track-demand 1");
            options.official_metrics.a150_rules=true;
            options.graph_options.pin_zero_track_floor=rules_profile=="a150-20261029";
            options.official_metrics.tsv_track_demand=true;
            options.tsv_policy.tsv_obstacle_scope=obstacle_scope=="all-layers"?
                eda::TsvObstacleScope::ALL_LAYERS:eda::TsvObstacleScope::INTERFACE;
        }
        if(options.official_model) {
            options.official_metrics.score_20261020=true;
            options.graph_options.score_20261020=true;
            // The struct default stays 0: other modes refuse a nonzero radius and historical replays must not change.
            if(!explicit_climb_radius&&options.fast_quality&&rules_profile=="a150-20261029") options.climb_escape_radius=3;
            // Same scope for the end-of-run overflow cleanup: 800 s from the same
            // deadline, with blocking nets lifted together. Explicit flags win.
            if(options.fast_quality&&rules_profile=="a150-20261029") {
                if(!explicit_cleanup_seconds) options.overflow_cleanup_seconds=800;
                if(!explicit_cleanup_blockers) options.overflow_cleanup_blockers=true;
            }
        }
        if(options.official_metrics.score_20261020) {
            if(!options.official_model) throw std::runtime_error("20261020 score profile requires --model official");
            if(explicit_nonpreferred && options.graph_options.allow_nonpreferred)
                throw std::runtime_error("20261020 score profile forbids --nonpreferred 1");
            options.graph_options.allow_nonpreferred=false;
        }
        if(options.official_metrics.tsv_track_demand && (!options.official_model || options.tsv_policy.resource_mode!=eda::TsvResourceMode::GLOBAL_GRID))
            throw std::runtime_error("TSV track demand requires --model official --tsv-resource-mode global-grid");
        if(options.future_tsv_reservation && !options.official_metrics.tsv_track_demand)
            throw std::runtime_error("Future TSV reservation requires --tsv-track-demand 1");
        if(options.resource_local_rrr && (!options.official_metrics.tsv_track_demand || !options.v8_rrr))
            throw std::runtime_error("Resource local RRR requires --tsv-track-demand 1 --v8-rrr 1");
        if(options.joint_layer_recovery && (!options.official_model || !options.large_recovery))
            throw std::runtime_error("Joint layer recovery requires official budgeted recovery");
        if((options.resource_candidate_pool || options.thermal_coverage) && (!options.official_metrics.tsv_track_demand || !options.v8_tsv_dynamic))
            throw std::runtime_error("Resource thermal search requires track demand and dynamic TSV");
        if(options.tsv_landing_pricing && (!options.official_metrics.tsv_track_demand || !options.v8_tsv_dynamic))
            throw std::runtime_error("TSV landing pricing requires --tsv-track-demand 1 --v8-tsv-dynamic 1");
        if(options.tsv_finite_quote && !options.tsv_landing_pricing)
            throw std::runtime_error("--tsv-finite-quote 1 requires --tsv-landing-pricing 1");
        if(options.tsv_shared_auto) {
            if(options.tsv_policy.shared_cell_limit==1) options.tsv_policy.shared_cell_limit=32;
            if(options.tsv_policy.shared_cell_limit<4)
                throw std::runtime_error("--tsv-shared-auto 1 needs --tsv-shared-cell-limit of at least 4 (default 32)");
            if(!options.official_model || options.tsv_policy.resource_mode!=eda::TsvResourceMode::GLOBAL_GRID)
                throw std::runtime_error("--tsv-shared-auto 1 requires --model official --tsv-resource-mode global-grid");
        }
        if(options.tsv_landing_rrr && (!options.official_metrics.tsv_track_demand || !options.v8_rrr))
            throw std::runtime_error("TSV landing RRR requires --tsv-track-demand 1 --v8-rrr 1");
        if(gain_estimate && !options.official_model)
            throw std::runtime_error("--thermal-gain-estimate needs the official model");
        if(objective_profile!="documented") {
            if(!options.official_model || !(options.explicit_thermal_gain || gain_estimate) || options.graph_options.platform_20261024)
                throw std::runtime_error("site scalar objective requires official documented resources and explicit --thermal-gain or --thermal-gain-estimate");
            options.graph_options.congestion_overflow_scale=1.0;
            options.official_metrics.all_adjacent_overlaps=true;
            options.official_metrics.snap_reference_coordinates=true;
        }
        if(options.tsv_tree_move && (!options.official_model || !options.unified_budget ||
           !options.v8_tsv_dynamic || !options.official_metrics.a150_rules ||
           !options.graph_options.pin_zero_track_floor || options.graph_options.platform_20261024))
            throw std::runtime_error("TSV tree move requires A150 October 29 documented resources, dynamic TSV and unified budget");
        const bool any_v9=options.v9_incremental_rrr||options.v9_subtree_recovery||options.v9_access_rematch||options.v9_tsv_coverage||options.v9_thermal_regions||options.v9_adaptive_budget;
        if((options.low_layer_mask||options.rrr_reserved_seconds>0)&&
           (!options.official_model||!options.unified_budget||options.graph_options.platform_20261024))
            throw std::runtime_error("low-layer experiments require official documented resources and unified budget");
        if(options.rrr_reserved_seconds>0&&(!options.v9_adaptive_budget||!options.v8_rrr||options.rrr_rounds<=0))
            throw std::runtime_error("reserved RRR requires adaptive budget, hotspot RRR and positive rounds");
        if(any_v9&&(!options.official_model||!options.unified_budget)) throw std::runtime_error("v9 requires --model official and --unified-budget 1");
        if(options.v9_incremental_rrr&&!options.v8_rrr) throw std::runtime_error("v9 incremental RRR requires --v8-rrr 1");
        if(options.v9_subtree_recovery&&!options.large_recovery) throw std::runtime_error("v9 subtree recovery requires --large-recovery 1");
        if(options.v9_access_rematch&&!options.v8_tsv_rematch) throw std::runtime_error("v9 access rematch requires --v8-tsv-rematch 1");
        if((options.v9_tsv_coverage||options.v9_thermal_regions)&&!options.v8_tsv_dynamic) throw std::runtime_error("v9 TSV requires --v8-tsv-dynamic 1");
        for(const auto& flag:explicit_v9) {
            if(flag.find("--v9-checkpoint-")==0&&!options.v9_incremental_rrr) throw std::runtime_error("v9 checkpoint budget requires incremental RRR");
            if((flag=="--v9-subtree-branches"||flag=="--v9-subtree-boundary")&&!options.v9_subtree_recovery) throw std::runtime_error("v9 subtree budgets require subtree recovery");
            if((flag=="--v9-rematch-edges"||flag=="--v9-rematch-access-seconds")&&!options.v9_access_rematch) throw std::runtime_error("v9 rematch budget requires access rematch");
            if(flag=="--v9-tsv-stale-sweeps"&&!options.v9_tsv_coverage) throw std::runtime_error("v9 sweep budget requires TSV coverage");
        }
        if((options.v8_rrr||options.v8_tsv_dynamic||options.v8_tsv_rematch||options.v8_recovery||options.score_unit_cost) &&
           (!options.official_model||!options.official_metrics.score_20261020))
            throw std::runtime_error("v8 routing requires --model official --score-profile 20261020");
        if(options.v8_rrr&&!options.rrr_score_aware) throw std::runtime_error("v8 RRR requires --rrr-score-aware 1");
        if(options.hotspot_cell_limit && !options.v8_rrr) throw std::runtime_error("hotspot cell limit requires --v8-rrr 1");
        if(options.via_stack_radius && !options.v8_rrr) throw std::runtime_error("via stack radius requires --v8-rrr 1");
        if(options.recovery_global_wirelength && !options.large_recovery) throw std::runtime_error("global wirelength recovery requires --large-recovery 1");
        if(options.recovery_rotate && !options.large_recovery) throw std::runtime_error("recovery rotation requires --large-recovery 1");
        if(options.layer_swap && (!options.official_model || !options.v9_adaptive_budget))
            throw std::runtime_error("layer swap requires --model official and --v9-adaptive-budget 1");
        if((options.v8_tsv_rematch||options.v8_tsv_dynamic)&&options.tsv_policy.resource_mode!=eda::TsvResourceMode::GLOBAL_GRID)
            throw std::runtime_error("v8 TSV requires --tsv-resource-mode global-grid");
        if(options.v8_tsv_dynamic&&!options.v7_tsv) throw std::runtime_error("dynamic TSV requires --v7-tsv 1");
        if(options.v8_recovery&&!options.large_recovery) throw std::runtime_error("v8 recovery requires --large-recovery 1");
        if((options.score_unit_cost||explicit_v8.count("--congestion-price-cap"))&&!options.official_score_pricing)
            throw std::runtime_error("search price options require --official-score-pricing 1");
        for(const auto& flag:explicit_v8) {
            if(flag.find("--v8-tsv-rematch-")==0&&!options.v8_tsv_rematch) throw std::runtime_error("rematch budgets require --v8-tsv-rematch 1");
            if(flag=="--v8-rrr-group-size"&&!options.v8_rrr) throw std::runtime_error("group budget requires --v8-rrr 1");
            if(flag=="--v8-tsv-probe-candidates"&&!options.v8_tsv_dynamic) throw std::runtime_error("probe budget requires --v8-tsv-dynamic 1");
        }
        if((options.v7_topology||options.v7_rrr||options.v7_cell_cost||options.v7_tsv||options.graph_options.platform_20261024) &&
           (!options.official_model||!options.official_metrics.score_20261020))
            throw std::runtime_error("v7 routing and platform resources require --model official --score-profile 20261020");
        if(explicit_v7_tsv_budget&&!options.v7_tsv)
            throw std::runtime_error("v7 TSV budgets require --v7-tsv 1");
        if(options.v7_tsv&&options.tsv_policy.resource_mode!=eda::TsvResourceMode::GLOBAL_GRID)
            throw std::runtime_error("v7 TSV optimization requires --tsv-resource-mode global-grid");
        if(explicit_v7_budget&&!options.v7_rrr&&!options.v8_rrr)
            throw std::runtime_error("v7 RRR budgets require --v7-rrr 1");
        if(options.v7_rrr&&!options.rrr_score_aware)
            throw std::runtime_error("v7 RRR requires --rrr-score-aware 1");
        if(explicit_polish_heat&&!options.tsv_policy.assignment_polish)
            throw std::runtime_error("--tsv-polish-heat-weight requires --tsv-assignment-polish 1");
        if(explicit_polish_thermal&&!options.tsv_policy.assignment_polish)
            throw std::runtime_error("--tsv-polish-thermal requires --tsv-assignment-polish 1");
        if(explicit_polish_budget&&!options.tsv_policy.assignment_polish)
            throw std::runtime_error("TSV polish budgets require --tsv-assignment-polish 1");
        if(options.tsv_policy.polish_thermal_pareto&&!options.official_model)
            throw std::runtime_error("Pareto thermal rule requires --model official");
        if(explicit_polish_heat&&options.tsv_policy.polish_thermal_pareto)
            throw std::runtime_error("--tsv-polish-heat-weight requires weighted thermal rule");
        if(options.tsv_policy.polish_thermal_candidates && options.tsv_policy.polish_thermal_pareto)
            throw std::runtime_error("thermal candidates require weighted thermal rule");
        if(options.tsv_policy.polish_score_aware && (options.tsv_policy.polish_thermal_pareto ||
           !options.tsv_policy.assignment_polish || !options.official_model))
            throw std::runtime_error("--tsv-polish-score-aware requires --model official, --tsv-assignment-polish 1 and weighted thermal rule");
        if(options.rrr_score_aware && !options.official_model)
            throw std::runtime_error("score-aware RRR requires --model official");
        if(options.official_score_pricing && !options.official_model)
            throw std::runtime_error("score pricing requires --model official");
        if(explicit_rrr_limit && !options.rrr_score_aware)
            throw std::runtime_error("--rrr-net-limit requires --rrr-score-aware 1");
        if(options.large_recovery && !options.official_model)
            throw std::runtime_error("large recovery requires --model official");
        if(explicit_recovery_budget && !options.large_recovery)
            throw std::runtime_error("large recovery budgets require --large-recovery 1");
        if(options.tsv_policy.resource_mode==eda::TsvResourceMode::GLOBAL_GRID&&!options.official_model)
            throw std::runtime_error("global-grid TSV resources require --model official");
        if((options.tsv_policy.assignment_block_refine||options.v5_pin_rooted)&&!options.official_model)
            throw std::runtime_error("v5 assignment/topology optimization requires --model official");
        if(explicit_block_budget&&!options.tsv_policy.assignment_block_refine)
            throw std::runtime_error("TSV block budgets require --tsv-block-refine 1");
        if(explicit_topology_budget&&!options.v5_pin_rooted)
            throw std::runtime_error("v5 topology budget requires --v5-pin-rooted 1");
        if(options.graph_options.platform_20261024)
            throw std::runtime_error("platform-20261024 resources are diagnostic only; use --resource-profile documented");
        if(def_file.empty()||lefs.empty()||thermal_file.empty()) throw std::runtime_error("required: --def --lef --thermal-report");
        std::vector<std::string> inputs=lefs;inputs.push_back(def_file);inputs.push_back(thermal_file);
        for(const auto& input:inputs) if(same_file(input,output)||(!stats.empty()&&same_file(input,stats)))
            throw std::runtime_error("output/stats cannot overwrite input: "+input);
        if(!stats.empty()&&same_file(output,stats)) throw std::runtime_error("stats and guide output must differ");
        if(kind=="baseline"&&!stats.empty()) throw std::runtime_error("--stats requires negotiated router");
        eda::LefParser lef;for(const auto& p:lefs) if(!lef.parse(p)) return 1;
        eda::DefParser def;if(!def.parse(def_file,lef,options.official_model?eda::DefGridMapping::OFFICIAL_FULL:eda::DefGridMapping::CLIPPED_PHYSICAL)) return 1;
        if(!explicit_time && def.nets().size()>1000) options.time_limit=std::clamp(def.nets().size()*0.03,120.0,7200.0);
        eda::ThermalParser thermal;if(!thermal.parse_json(thermal_file,def.gcell_grid(),def.dbu_per_micron(),def.die_area(),options.official_model?eda::ThermalMapping::OFFICIAL_PREVIOUS:eda::ThermalMapping::LEGACY_BILINEAR)) return 1;
        if(thermal.interpolated_cells())
            std::cerr<<"[WARN] Missing GCell temperatures were locally interpolated; the fixed evaluator does not implement this mapping.\n";
        if(options.official_model) {
            if(kind!="negotiated") throw std::runtime_error("official model requires negotiated router");
            options.graph_options.geometry=eda::RoutingGeometryMode::GLOBAL_GUIDE;
            if(gain_estimate) {
                size_t cross=0;for(const auto& net:def.nets()) cross+=net.is_cross_die;
                if(const auto estimate=eda::hottest_k_thermal_gain(thermal,def.gcell_grid(),cross)) {
                    options.official_metrics.thermal_gain=*estimate;
                    options.explicit_thermal_gain=options.estimated_thermal_gain=true;
                    std::cerr<<"[MODEL] Thermal gain estimate "<<*estimate<<" = 1.5 / hottest-K Pearson bound (K="<<cross
                             <<"); an optimisation proxy, not the official per-case G.\n";
                } else std::cerr<<"[MODEL] Degenerate heat/TSV count: thermal score is constant; keeping G 20.\n";
            }
            if(!options.explicit_thermal_gain)
                std::cerr<<"[MODEL] Thermal gain 20 is a uniform proxy; official G_case is not specified by the text.\n";
            options.official_metrics.gcell_width_dbu=def.gcell_pitch_x();
            options.official_metrics.gcell_height_dbu=def.gcell_pitch_y();
            if(options.official_runtime.empty()) options.official_runtime="output/runtime/official-flute";
        }
        if(kind=="baseline") {
            eda::BaselineRouter router(def,lef,thermal);if(!router.route_all()||!router.export_guide(output)) return 1;
        } else {
            // Resolve the TSV policy before allocating a potentially huge graph.
            eda::TsvPolicy::preflight(def,lef,options.tsv_policy);
            eda::NegotiatedRouter router(def,lef,thermal,options);
            if(!router.route_all()||!router.export_guide(output)||(!stats.empty()&&!router.export_stats(stats))) return 1;
        }
        std::cout<<"[SUCCESS] Guide exported: "<<output<<"\n";return 0;
    } catch(const std::exception& e) {std::cerr<<"[ERROR] "<<e.what()<<"\n";return 1;}
}
