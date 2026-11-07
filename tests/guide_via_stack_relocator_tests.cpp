#include "router/guide_via_stack_relocator.hpp"
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
    require(argc==2,"new artifact path required");const fs::path root=argv[1];require(!fs::exists(root),"preserve artifacts");fs::create_directories(root);
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
    const auto opts=[&](){GuideViaStackOptions o;o.deadline=std::chrono::steady_clock::now()+std::chrono::seconds(10);o.radius=1;o.total_L=1e9;o.reference=8e8;o.max_total_L=o.total_L*1.001;return o;};
    std::vector<GuideRect> ladder{{100,100,400,200,"metal1"},{300,100,600,200,"metal7"}};
    for(int l=2;l<7;++l)ladder.emplace_back(300,100,400,200,get_layer_name(l));
    const auto original=measure_official_net_guides(net("dog"),ladder,graph,lef,mo,0);
    std::vector<int> demand(graph.size());for(auto [n,d]:original.demand)demand[n]=d;const int hot=graph.node(3,1,4);demand[hot]+=20;
    auto opt=opts();for(size_t n=0;n<graph.size();++n)opt.total_Q+=double(official_resource_cell_contribution(graph,int(n),demand[n]).squared);
    const auto priority=[&](int n){return n==hot?100.:0.;};const auto saved=demand;
    auto moved=relocate_guide_via_stack(net("dog"),ladder,graph,lef,mo,opt,demand,priority);
    require(!moved.removed.empty()&&moved.delta_Q<0,"move a real ordinary via ladder away from congestion");++checks;
    auto measured=measure_official_net_guides(net("dog"),moved.guides,graph,lef,mo,0);
    require(original.ordinary_connections==6&&measured.ordinary_connections==6,"charge all six vias after relocation");++checks;
    require(measured.wirelength-original.wirelength==moved.delta_L&&measured.wirelength>original.wirelength,"actual longer endpoint attachment is charged");++checks;
    require(demand==saved,"candidate generator leaves global demand untouched");++checks;
    std::string error;require(validate_net_guides(net("dog"),def.die_area(),lef,moved.guides,error,100,100),"all PINs and endpoint subtrees remain connected");++checks;
    auto expired=opt;expired.deadline=std::chrono::steady_clock::now()-std::chrono::seconds(1);
    auto no_time=relocate_guide_via_stack(net("dog"),ladder,graph,lef,mo,expired,demand,priority);
    require(no_time.timed_out&&same(no_time.guides,ladder)&&no_time.removed.empty(),"timeout preserves input");++checks;
    auto capped=opt;capped.max_total_L=capped.total_L;
    auto no_length=relocate_guide_via_stack(net("dog"),ladder,graph,lef,mo,capped,demand,priority);
    require(no_length.removed.empty()&&same(no_length.guides,ladder),"global length cap blocks growth");++checks;
    auto reject=opt;reject.reject_all=true;
    auto rejected=relocate_guide_via_stack(net("dog"),ladder,graph,lef,mo,reject,demand,priority);
    require(rejected.trials>0&&rejected.removed.empty()&&same(rejected.guides,ladder)&&demand==saved,"injected rejection leaves all state untouched");++checks;
    auto trial_limit=opt;trial_limit.max_trials=1;
    auto limited=relocate_guide_via_stack(net("dog"),ladder,graph,lef,mo,trial_limit,demand,priority);
    require(limited.trials==1,"per-net work bounded");++checks;
    auto quiet=relocate_guide_via_stack(net("dog"),ladder,graph,lef,mo,opt,demand,[](int){return 0.;});
    require(quiet.trials==0&&quiet.removed.empty(),"no hotspot creates no candidate");++checks;
    auto duplicate=ladder;duplicate.emplace_back(300,100,400,200,"metal3");
    auto representation=relocate_guide_via_stack(net("dog"),duplicate,graph,lef,mo,opt,demand,priority);
    require(representation.canonical_rejections==1&&same(representation.guides,duplicate),"merging cannot impersonate via savings");++checks;
    {
        std::ifstream in(root/"in.def");std::string text((std::istreambuf_iterator<char>(in)),{});
        text.replace(text.find("COMPONENTS 4"),12,"COMPONENTS 8");text.insert(text.find("END COMPONENTS"),"- e floor + PLACED ( 200 100 ) N ; - f floor + PLACED ( 400 100 ) N ; - g floor + PLACED ( 300 0 ) N ; - h floor + PLACED ( 300 200 ) N ; ");
        std::ofstream out(root/"blocked.def");out<<text;out.close();DefParser blocked;require(blocked.parse((root/"blocked.def").string(),lef,DefGridMapping::OFFICIAL_FULL),"blocked fixture parse");
        RoutingGraph bg(blocked,lef,{RoutingGeometryMode::GLOBAL_GUIDE,false,true,false,true,true,1});
        auto hit=relocate_guide_via_stack(net("dog"),ladder,bg,lef,mo,opt,demand,priority);
        require(hit.removed.empty()&&hit.pin_rejections>0,"unconnected PIN geometry cannot enter changed demand");++checks;
    }
    std::vector<GuideRect> cross{{100,100,400,200,"metal1"}};
    for(int l=2;l<=7;++l)cross.emplace_back(300,100,400,200,get_layer_name(l));
    cross.emplace_back(300,100,400,400,"metal8");for(int l=9;l<=10;++l)cross.emplace_back(300,300,400,400,get_layer_name(l));cross.emplace_back(300,300,700,400,"metal11");
    const auto cm=measure_official_net_guides(net("cross"),cross,graph,lef,mo,0);std::vector<int> cd(graph.size());for(auto [n,d]:cm.demand)cd[n]=d;cd[hot]+=20;auto co=opts();for(size_t n=0;n<graph.size();++n)co.total_Q+=double(official_resource_cell_contribution(graph,int(n),cd[n]).squared);
    auto cr=relocate_guide_via_stack(net("cross"),cross,graph,lef,mo,co,cd,priority);require(!cr.removed.empty(),"ordinary stack can move within cross-die net");++checks;
    const auto cn=measure_official_net_guides(net("cross"),cr.guides,graph,lef,mo,0);
    require(cm.tsv_connections==1&&cn.tsv_cells==cm.tsv_cells&&cn.tsv_points.size()==1&&cn.tsv_points[0].distance_to(cm.tsv_points[0])==0,"per-net TSV location and count fixed");++checks;
    auto branch=ladder;branch.emplace_back(300,100,400,400,"metal4");
    auto br=relocate_guide_via_stack(net("dog"),branch,graph,lef,mo,opt,demand,priority);
    require(br.removed.empty(),"hot branch anchor is not removed as a ladder interior");++checks;

    Net vertical=net("dog");vertical.name="vertical";
    for(auto& pin:vertical.pins){std::swap(pin.coord.x,pin.coord.y);++pin.layer_idx;pin.layer=get_layer_name(pin.layer_idx);for(auto& shape:pin.port_rects){++shape.layer_idx;std::swap(shape.rect.x1,shape.rect.y1);std::swap(shape.rect.x2,shape.rect.y2);}}
    auto vg=ladder;for(auto& g:vg){std::swap(g.x1,g.y1);std::swap(g.x2,g.y2);g.layer=get_layer_name(get_layer_index(g.layer)+1);}
    const auto vm=measure_official_net_guides(vertical,vg,graph,lef,mo,0);std::vector<int> vd(graph.size());for(auto [n,d]:vm.demand)vd[n]=d;const int vh=graph.node(1,3,5);vd[vh]+=20;auto vo=opts();for(size_t n=0;n<graph.size();++n)vo.total_Q+=double(official_resource_cell_contribution(graph,int(n),vd[n]).squared);
    const auto vr=relocate_guide_via_stack(vertical,vg,graph,lef,mo,vo,vd,[&](int n){return n==vh?100.:0.;});require(!vr.removed.empty(),"vertical endpoint direction moves stack along Y");++checks;
    require(ladder.size()==7&&demand==saved,"caller Guide and global ledger still unchanged after all checks");++checks;
    std::cout<<"PASS "<<checks<<" via-stack checks\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
