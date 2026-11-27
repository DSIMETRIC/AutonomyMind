#pragma once
#include "autonomymind/types.hpp"
namespace autonomymind {
inline int trace_add(BehaviorTrace& trace, BehaviorTreeId tree_id, int node_id,int parent_id,int depth,BehaviorNodeStatus status){if(trace.count>=FM_BEHAVIOR_TRACE_NODES){trace.overflow=true;return -1;}const int index=trace.count++;auto& node=trace.nodes[index];node.tree_id=tree_id;node.node_id=node_id;node.parent_id=parent_id;node.depth=depth;node.status=status;node.tick_order=index;return index;}
inline void trace_merge(BehaviorTrace& dst,const BehaviorTrace& src,int depth_offset,int parent_override){const int base=dst.count;for(int i=0;i<src.count;++i){if(dst.count>=FM_BEHAVIOR_TRACE_NODES){dst.overflow=true;break;}auto n=src.nodes[i];n.depth+=depth_offset;n.parent_id=(n.parent_id<0)?parent_override:(base+n.parent_id);n.tick_order=dst.count;dst.nodes[dst.count++]=n;}dst.overflow=dst.overflow||src.overflow||src.count>(FM_BEHAVIOR_TRACE_NODES-base);}
}