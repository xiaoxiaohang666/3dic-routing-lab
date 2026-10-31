#include "scoring/site_wirelength.hpp"
#include <queue>
#include <tuple>
#include <limits>
#include <stdexcept>
#include <algorithm>
namespace eda {
double site_rectangle_mst_diagnostic(const std::vector<GuideRect>& g,const LefParser& lef,double pitch) {
 if(g.empty()||pitch<=0)throw std::invalid_argument("invalid MST diagnostic inputs");
 auto connects=[&](size_t a,size_t b) {
  const auto& x=g[a];const auto& y=g[b];int l=get_layer_index(x.layer),k=get_layer_index(y.layer);
  double ox=std::min(x.x2,y.x2)-std::max(x.x1,y.x1),oy=std::min(x.y2,y.y2)-std::max(x.y1,y.y1);
  if(l==k)return lef.get_layer_direction(x.layer)==Direction::HORIZONTAL?ox>=0&&oy>0:ox>0&&oy>=0;
  return std::abs(l-k)==1&&ox>0&&oy>0;
 };
 using Item=std::tuple<double,int,int>;std::priority_queue<Item,std::vector<Item>,std::greater<Item>> q;q.emplace(0,0,-1);
 std::vector<bool> seen(g.size());size_t count=0;double result=0;
 while(!q.empty()) {
  auto [cost,u,p]=q.top();q.pop();if(seen[u])continue;seen[u]=true;++count;
  if(p>=0) {
   const auto& a=g[p];const auto& b=g[u];
   if(a.layer!=b.layer)result+=cost;
   else if(lef.get_layer_direction(a.layer)==Direction::HORIZONTAL)result+=std::max(a.x2,b.x2)-std::min(a.x1,b.x1);
   else result+=std::max(a.y2,b.y2)-std::min(a.y1,b.y1);
  }
  for(size_t v=0;v<g.size();++v)if(!seen[v]&&connects(u,v)) {
   int l=get_layer_index(g[u].layer),k=get_layer_index(g[v].layer);double cost=g[u].center().distance_to(g[v].center());
   if(l!=k)cost+=((l<=10)!=(k<=10)?6:3)*pitch;
   q.emplace(cost,int(v),u);
  }
 }
 if(count!=g.size())throw std::runtime_error("strict diagnostic MST disconnected");return result;
}
}
