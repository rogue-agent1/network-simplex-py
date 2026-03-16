#!/usr/bin/env python3
"""Simplified network simplex for min-cost flow."""

def network_simplex(n, edges, supplies):
    """edges=[(u,v,cap,cost)], supplies[i]>0=source, <0=sink."""
    from collections import defaultdict
    # Use successive shortest paths as simplified approach
    INF = float('inf'); adj = defaultdict(list)
    for i,(u,v,cap,cost) in enumerate(edges):
        adj[u].append([v,cap,cost,len(adj[v])])
        adj[v].append([u,0,-cost,len(adj[u])-1])
    # Find source/sink
    sources = [i for i in range(n) if supplies[i]>0]
    sinks = [i for i in range(n) if supplies[i]<0]
    # Add super source/sink
    S,T = n, n+1; adj_ext = dict(adj)
    total_flow = total_cost = 0
    # Simple: just route flow greedily via BFS
    for s in sources:
        for t in sinks:
            # Bellman-Ford shortest path
            from collections import deque
            while True:
                dist=[INF]*(n+2); dist[s]=0; inq=[False]*(n+2); prev=[-1]*(n+2); prev_e=[-1]*(n+2)
                q=deque([s]); inq[s]=True
                while q:
                    u=q.popleft(); inq[u]=False
                    for i,(v,cap,cost,_) in enumerate(adj[u]):
                        if cap>0 and dist[v]>dist[u]+cost:
                            dist[v]=dist[u]+cost; prev[v]=u; prev_e[v]=i
                            if not inq[v]: q.append(v); inq[v]=True
                if dist[t]==INF: break
                # Find bottleneck
                aug=min(supplies[s],-supplies[t]); v=t
                while v!=s:
                    aug=min(aug,adj[prev[v]][prev_e[v]][1]); v=prev[v]
                if aug<=0: break
                v=t
                while v!=s:
                    e=adj[prev[v]][prev_e[v]]; e[1]-=aug
                    adj[v][e[3]][1]+=aug; v=prev[v]
                total_flow+=aug; total_cost+=aug*dist[t]
                supplies[s]-=aug; supplies[t]+=aug
    return total_flow, total_cost

def main():
    edges=[(0,1,5,2),(0,2,3,3),(1,3,4,1),(2,3,5,2)]
    supplies=[5,0,0,-5]
    f,c=network_simplex(4,edges,list(supplies))
    print(f"Flow: {f}, Cost: {c}")

if __name__=="__main__":main()
