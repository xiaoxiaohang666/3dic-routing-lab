#include "router/guide_subtree_reconnector.hpp"
#include "validation/gcell_encoding.hpp"
#include "objectives/net_cell_contribution.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

using namespace eda;
namespace fs=std::filesystem;
void require(bool value,const char* message) {if(!value)throw std::runtime_error(message);}
bool same(const std::vector<GuideRect>& a,const std::vector<GuideRect>& b) {
    if(a.size()!=b.size()) return false;
    for(size_t i=0;i<a.size();++i) if(a[i].x1!=b[i].x1||a[i].x2!=b[i].x2||a[i].y1!=b[i].y1||a[i].y2!=b[i].y2||a[i].layer!=b[i].layer) return false;
    return true;
}
int main(int argc,char** argv) {try {
    require(argc==2,"new artifact path required");const fs::path parent=argv[1];fs::create_directories(parent);const fs::path root=parent/("run-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));require(!fs::exists(root),"preserve artifacts");fs::create_directory(root);
    {
        std::ofstream lef(root/"in.lef");
        for(int l=1;l<=20;++l) lef<<"LAYER metal"<<l<<" TYPE ROUTING ; DIRECTION "<<(l%2?"HORIZONTAL":"VERTICAL")<<" ; WIDTH .1 ; END metal"<<l<<'\n';
        lef<<"MACRO low SIZE 10 BY 10 ; PIN A PORT LAYER metal1 ; RECT 1 1 9 9 ; END END A END low\n"
              "MACRO high SIZE 10 BY 10 ; PIN A PORT LAYER metal11 ; RECT 1 1 9 9 ; END END A END high\n"
              "MACRO mid SIZE 10 BY 10 ; PIN A PORT LAYER metal7 ; RECT 1 1 9 9 ; END END A END mid\n"
              "MACRO floor SIZE 10 BY 10 ; PIN A PORT LAYER metal4 ; RECT 1 1 9 9 ; END END A END floor\n";
        std::ofstream def(root/"in.def");def<<"UNITS DISTANCE MICRONS 1 ; DIEAREA ( 0 0 ) ( 1000 1000 ) ; "
            "GCELLGRID X 0 DO 11 STEP 100 ; GCELLGRID Y 0 DO 11 STEP 100 ; ";
        for(int l=1;l<=20;++l) def<<"TRACKS "<<(l%2?'Y':'X')<<" 10 DO 50 STEP 20 LAYER metal"<<l<<" ; ";
        def<<"COMPONENTS 4 ; - a low + PLACED ( 100 100 ) N ; - b low + PLACED ( 500 100 ) N ; - c high + PLACED ( 600 300 ) N ; - d mid + PLACED ( 500 100 ) N ; END COMPONENTS "
              "PINS 0 ; END PINS NETS 3 ; - straight ( a A ) ( b A ) ; - cross ( a A ) ( c A ) ; - dog ( a A ) ( d A ) ; END NETS END DESIGN\n";
    }
    LefParser lef;require(lef.parse((root/"in.lef").string()),"parse lef");DefParser def;
    require(def.parse((root/"in.def").string(),lef,DefGridMapping::OFFICIAL_FULL),"parse def");
    RoutingGraph graph(def,lef,{RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true,true,1});
    OfficialMetricOptions mo;mo.score_20261020=true;mo.tsv_track_demand=true;mo.a150_rules=true;mo.all_adjacent_overlaps=true;mo.snap_reference_coordinates=true;mo.gcell_width_dbu=100;mo.gcell_height_dbu=100;
    const auto net=[&](std::string name)->const Net& {for(const auto& n:def.nets())if(n.name==name)return n;throw std::runtime_error("missing net");};


    size_t checks=0;
    Net tri=net("dog");tri.name="tri";tri.pins.push_back(net("straight").pins.back());
    std::vector<GuideRect> tree{{100,100,600,200,"metal1"},{300,100,600,200,"metal7"}};
    for(int l=2;l<7;++l)tree.emplace_back(300,100,400,200,get_layer_name(l));
    const auto original=measure_official_net_guides(tri,tree,graph,lef,mo,0);
    std::vector<int> demand(graph.size());for(auto [n,d]:original.demand)demand[n]=d;const int hot=graph.node(3,1,1);demand[hot]+=20;const auto saved=demand;
    GuideSubtreeOptions opt;opt.deadline=std::chrono::steady_clock::now()+std::chrono::seconds(20);opt.net_seconds=10;opt.max_expansions=10000;opt.max_windows=2;opt.radius=1;opt.total_L=1e9;opt.reference=8e8;opt.max_total_L=1.01e9;
    for(size_t n=0;n<graph.size();++n)opt.total_Q+=double(official_resource_cell_contribution(graph,int(n),demand[n]).squared);
    auto priority=[&](int n){return n==hot?100.:0.;};
    const auto result=reconnect_guide_subtree(tri,tree,graph,lef,mo,opt,demand,priority);
    require(!result.pareto.empty()&&!result.removed.empty(),"three-terminal local Steiner reconnect finds a useful replacement");++checks;
    require(result.multi_terminal_candidates>0&&result.astar_expansions>0,"real multi-source multi-target A* reconnects three retained subtrees");++checks;
    std::string error;require(validate_net_guides(tri,def.die_area(),lef,result.guides,error,100,100),"all three terminal branches connected");++checks;
    const auto measured=measure_official_net_guides(tri,result.guides,graph,lef,mo,0);
    require(measured.wirelength-original.wirelength==result.delta_L,"full submitted-Guide length exactly charged");++checks;
    auto patched=demand;for(auto [n,d]:original.demand)patched[n]-=d;for(auto [n,d]:measured.demand)patched[n]+=d;long double full_Q=0;
    for(size_t n=0;n<graph.size();++n)full_Q+=official_resource_cell_contribution(graph,int(n),patched[n]).squared;
    require(std::abs(double(full_Q)-opt.total_Q-result.delta_Q)<1e-10&&result.delta_Q<0,"incremental Q agrees with independent whole-grid reconstruction");++checks;
    require(demand==saved&&tree.size()==7,"input Guide and global demand never mutated");++checks;
    for(const auto& candidate:result.pareto){const auto m=measure_official_net_guides(tri,candidate.guides,graph,lef,mo,0);require(static_cast<long long>(m.ordinary_connections)-static_cast<long long>(original.ordinary_connections)==candidate.delta_vias,"all adjacent overlap vias charged for each Pareto candidate");}++checks;
    for(size_t i=0;i<result.pareto.size();++i)for(size_t j=0;j<result.pareto.size();++j)if(i!=j){const auto& a=result.pareto[i];const auto& b=result.pareto[j];require(!(a.delta_L<=b.delta_L+1e-8&&a.delta_Q<=b.delta_Q+1e-12&&a.delta_vias<=b.delta_vias),"retained objective vectors are mutually nondominated");}++checks;
    auto expired=opt;expired.deadline=std::chrono::steady_clock::now()-std::chrono::seconds(1);auto timed=reconnect_guide_subtree(tri,tree,graph,lef,mo,expired,demand,priority);require(timed.timed_out&&same(timed.guides,tree),"expired search preserves original");++checks;
    auto reject=opt;reject.reject_all=true;auto rejected=reconnect_guide_subtree(tri,tree,graph,lef,mo,reject,demand,priority);require(!rejected.pareto.empty()&&rejected.removed.empty()&&same(rejected.guides,tree)&&demand==saved,"reject-all leaves all state and saved Guide unchanged");++checks;
    auto bounded=opt;bounded.max_expansions=0;auto none=reconnect_guide_subtree(tri,tree,graph,lef,mo,bounded,demand,priority);require(none.removed.empty()&&none.astar_expansions==0,"A* expansion bound is hard");++checks;
    auto cap=opt;cap.max_total_L=0;auto capped=reconnect_guide_subtree(tri,tree,graph,lef,mo,cap,demand,priority);require(capped.removed.empty(),"length cap applies to all objectives");++checks;
    auto duplicate=tree;duplicate.emplace_back(300,100,400,200,"metal3");auto fake=reconnect_guide_subtree(tri,duplicate,graph,lef,mo,opt,demand,priority);require(fake.canonical_rejections==1&&same(fake.guides,duplicate),"normalization cannot create false via benefit");++checks;
    auto pin_seed=reconnect_guide_subtree(tri,tree,graph,lef,mo,opt,demand,[&](int n){return n==graph.node(1,1,1)?100.:0.;});require(pin_seed.windows==0&&pin_seed.removed.empty(),"PIN access cannot be extracted");++checks;
    std::vector<GuideRect> cross{{100,100,400,200,"metal1"}};for(int l=2;l<=7;++l)cross.emplace_back(300,100,400,200,get_layer_name(l));cross.emplace_back(300,100,400,400,"metal8");for(int l=9;l<=10;++l)cross.emplace_back(300,300,400,400,get_layer_name(l));cross.emplace_back(300,300,700,400,"metal11");
    const auto cm=measure_official_net_guides(net("cross"),cross,graph,lef,mo,0);std::vector<int> cd(graph.size());for(auto [n,d]:cm.demand)cd[n]=d;const int ch=graph.node(3,1,4);cd[ch]+=20;auto co=opt;co.total_Q=0;for(size_t n=0;n<graph.size();++n)co.total_Q+=double(official_resource_cell_contribution(graph,int(n),cd[n]).squared);
    auto cr=reconnect_guide_subtree(net("cross"),cross,graph,lef,mo,co,cd,[&](int n){return n==ch?100.:0.;});require(!cr.removed.empty(),"ordinary subtree of a cross-die net can reconnect");++checks;
    const auto cn=measure_official_net_guides(net("cross"),cr.guides,graph,lef,mo,0);require(cn.tsv_cells==cm.tsv_cells&&cn.tsv_points.size()==cm.tsv_points.size()&&cn.tsv_points[0].distance_to(cm.tsv_points[0])==0,"one TSV per net preserved without cross-die search");++checks;
    Net vertical=tri;vertical.name="rotated";for(auto& pin:vertical.pins){std::swap(pin.coord.x,pin.coord.y);++pin.layer_idx;pin.layer=get_layer_name(pin.layer_idx);for(auto& shape:pin.port_rects){++shape.layer_idx;std::swap(shape.rect.x1,shape.rect.y1);std::swap(shape.rect.x2,shape.rect.y2);}}
    auto vg=tree;for(auto& g:vg){std::swap(g.x1,g.y1);std::swap(g.x2,g.y2);g.layer=get_layer_name(get_layer_index(g.layer)+1);}auto vm=measure_official_net_guides(vertical,vg,graph,lef,mo,0);std::vector<int> vd(graph.size());for(auto [n,d]:vm.demand)vd[n]=d;const int vh=graph.node(1,3,2);vd[vh]+=20;auto vo=opt;vo.total_Q=0;for(size_t n=0;n<graph.size();++n)vo.total_Q+=double(official_resource_cell_contribution(graph,int(n),vd[n]).squared);auto vr=reconnect_guide_subtree(vertical,vg,graph,lef,mo,vo,vd,[&](int n){return n==vh?100.:0.;});require(!vr.removed.empty(),"layer directions are obtained from LEF for rotated topology");++checks;

    {
        std::ifstream in(root/"in.def");std::string text((std::istreambuf_iterator<char>(in)),{});text.replace(text.find("COMPONENTS 4"),12,"COMPONENTS 5");text.insert(text.find("END COMPONENTS"),"- e floor + PLACED ( 200 200 ) N ; ");std::ofstream out(root/"foreign-pin.def");out<<text;out.close();
        DefParser blocked;require(blocked.parse((root/"foreign-pin.def").string(),lef,DefGridMapping::OFFICIAL_FULL),"foreign PIN fixture parse");RoutingGraph bg(blocked,lef,{RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true,true,1});require(bg.tsv_obstacle_layers(2,2,TSV_MACRO_PIN|TSV_IO_PIN)!=0,"fixture really contains unconnected PIN geometry");
        auto blocked_result=reconnect_guide_subtree(tri,tree,bg,lef,mo,opt,demand,priority);require(!blocked_result.pareto.empty(),"foreign PIN does not disable all search");
        for(const auto& cand:blocked_result.pareto)for(int n:cand.added)require(!(bg.tsv_obstacle_layers(bg.col(n),bg.row(n),TSV_MACRO_PIN|TSV_IO_PIN)&(1u<<(bg.layer(n)-1))),"new demand must not enter any PIN geometry");++checks;
    }
    auto component_cap=opt;component_cap.max_components=2;auto capped_tree=reconnect_guide_subtree(tri,tree,graph,lef,mo,component_cap,demand,priority);require(capped_tree.removed.empty()&&capped_tree.extraction_rejections>0,"component bound preserves all retained branches");++checks;
    std::cout<<"PASS "<<checks<<" subtree checks; Pareto vectors="<<result.pareto.size()<<" components>2 candidates="<<result.multi_terminal_candidates<<"\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
