"""MAPF planners. Coordinates are (row, column); time starts at zero."""
from collections import deque
from itertools import count
import heapq
import random
import time

class Limit(Exception):
    pass

def check(deadline):
    if time.perf_counter() >= deadline:
        raise Limit()

def cells(grid):
    return [(r,c) for r,row in enumerate(grid) for c,x in enumerate(row) if x != '#']

def neighbors(grid,p,wait=True):
    r,c=p
    for dr,dc in ((0,0),(-1,0),(1,0),(0,-1),(0,1)):
        if not wait and dr==dc==0:
            continue
        q=(r+dr,c+dc)
        if 0<=q[0]<len(grid) and 0<=q[1]<len(grid[0]) and grid[q[0]][q[1]]!='#':
            yield q

def distances(grid,goal):
    d={goal:0}; queue=deque([goal])
    while queue:
        p=queue.popleft()
        for q in neighbors(grid,p,False):
            if q not in d:
                d[q]=d[p]+1; queue.append(q)
    return d

def reconstruct(parent,node):
    out=[]
    while node is not None:
        out.append(node); node=parent[node]
    return out[::-1]

def independent(instance,deadline):
    paths=[]
    for s,g in zip(instance['starts'],instance['goals']):
        queue=deque([s]); parent={s:None}
        while queue and g not in parent:
            check(deadline); p=queue.popleft()
            for q in neighbors(instance['grid'],p,False):
                if q not in parent:
                    parent[q]=p; queue.append(q)
        if g not in parent:
            return None,'unreachable'
        paths.append(reconstruct(parent,g))
    return paths,'paths_found'

def at(path,t):
    return path[min(t,len(path)-1)]

def validate(instance,paths):
    """Independent of planner internals; count conflicts per robot pair/time."""
    errors=[]; vc=[]; ec=[]; costs=[]
    grid=instance['grid']; starts=instance['starts']; goals=instance['goals']
    if not isinstance(paths,list) or len(paths)!=len(starts) or any(not p for p in paths):
        return dict(valid=False,errors=['missing paths'],vertex_conflicts=0,swap_conflicts=0,sum_of_costs=None,events=[])
    for i,p in enumerate(paths):
        if p[0]!=starts[i]: errors.append(f'robot {i}: wrong start')
        if p[-1]!=goals[i]: errors.append(f'robot {i}: wrong goal')
        for t,q in enumerate(p):
            if q not in set(cells(grid)): errors.append(f'robot {i}: blocked/outside at {t}')
            if t and abs(q[0]-p[t-1][0])+abs(q[1]-p[t-1][1])>1:
                errors.append(f'robot {i}: illegal move at {t}')
        # Final arrival, ignoring trailing waits at goal, including prior departures.
        last_off=max((t for t,q in enumerate(p) if q!=goals[i]),default=-1)
        costs.append(last_off+1 if p[-1]==goals[i] else None)
    for t in range(max(map(len,paths))):
        for i in range(len(paths)):
            for j in range(i):
                a,b=at(paths[i],t),at(paths[j],t)
                if a==b: vc.append((t,i,j,a))
                if t and a!=b and at(paths[i],t-1)==b and at(paths[j],t-1)==a:
                    ec.append((t,i,j,a,b))
    return dict(valid=not errors and not vc and not ec,errors=errors,vertex_conflicts=len(vc),swap_conflicts=len(ec),sum_of_costs=sum(costs) if None not in costs else None,events=vc+ec)

def single_reserved(grid,start,goal,previous,deadline):
    d=distances(grid,goal)
    if start not in d: return None,'unreachable'
    # Once every previous robot is parked, any remaining route is static.
    # Thus max reservation time + number of free cells is a sufficient horizon.
    horizon=max((len(p)-1 for p in previous),default=0)+len(cells(grid))
    if any(at(p,0)==start for p in previous): return None,'blocked'
    serial=count(); root=(start,0); parent={root:None}; heap=[(d[start],next(serial),root)]
    while heap:
        check(deadline); _,_,node=heapq.heappop(heap); p,t=node
        if p==goal and all(goal not in path[min(t,len(path)-1):] for path in previous):
            return [q for q,_ in reconstruct(parent,node)],'solved'
        if t>=horizon: continue
        for q in neighbors(grid,p):
            if q not in d: continue
            if any(at(path,t+1)==q or (at(path,t)==q and at(path,t+1)==p) for path in previous): continue
            nxt=(q,t+1)
            if nxt not in parent:
                parent[nxt]=node; heapq.heappush(heap,(t+1+d[q],next(serial),nxt))
    return None,'blocked'

def pp(instance,deadline,order):
    planned=[]; result=[None]*len(order)
    for i in order:
        path,status=single_reserved(instance['grid'],instance['starts'][i],instance['goals'][i],planned,deadline)
        if path is None: return None,status,i
        planned.append(path); result[i]=path
    return result,'solved',None

def joint(instance,deadline):
    """Optimal sum-of-final-arrival-times A*, with explicit irreversible retirement.

    At goal a robot may retire (zero further cost) or remain active (unit cost,
    including waits). Active robots can leave a goal. This avoids the incorrect
    shortcut of making *every* wait at a goal free before a later departure.
    """
    grid=instance['grid']; starts=tuple(instance['starts']); goals=tuple(instance['goals']); n=len(starts)
    if len(set(starts))!=n or len(set(goals))!=n: return None,'invalid_instance'
    ds=[distances(grid,g) for g in goals]
    if any(s not in d for s,d in zip(starts,ds)): return None,'unreachable'
    root=(starts,0); best={root:0}; parent={root:None}; serial=count()
    heap=[(sum(d[s] for d,s in zip(ds,starts)),0,next(serial),root)]
    while heap:
        check(deadline); _,cost,_,state=heapq.heappop(heap)
        if best.get(state)!=cost: continue
        pos,mask=state
        if pos==goals:
            chain=reconstruct(parent,state)
            paths=[[s[0][i] for s in chain] for i in range(n)]
            for p,g in zip(paths,goals):
                while len(p)>1 and p[-2]==g: p.pop()
            return paths,'optimal'
        chosen=[]
        def expand(i,newmask,stepcost):
            check(deadline)
            if i==n:
                nxt=(tuple(chosen),newmask); value=cost+stepcost
                if value<best.get(nxt,float('inf')):
                    best[nxt]=value; parent[nxt]=state
                    h=sum(ds[j][chosen[j]] for j in range(n) if not (newmask>>j)&1)
                    heapq.heappush(heap,(value+h,value,next(serial),nxt))
                return
            if (mask>>i)&1:
                options=[(pos[i],True)]
            else:
                options=[(q,False) for q in neighbors(grid,pos[i]) if q in ds[i]]
                if pos[i]==goals[i]: options.insert(0,(pos[i],True))
            for q,retire in options:
                if q in chosen: continue
                if any(q==pos[j] and chosen[j]==pos[i] for j in range(i)): continue
                chosen.append(q)
                expand(i+1,newmask | ((1<<i) if retire else 0),stepcost+(not retire))
                chosen.pop()
        expand(0,mask,0)
    return None,'no_solution'

def solve(instance,time_limit=10,method='pp',seed=0,max_attempts=32):
    begun=time.perf_counter(); deadline=begun+time_limit; paths=None; attempts=1; order=list(range(len(instance['starts'])))
    try:
        if method=='independent': paths,status=independent(instance,deadline)
        elif method=='joint_astar': paths,status=joint(instance,deadline)
        elif method=='pp': paths,status,_=pp(instance,deadline,order)
        elif method in ('pp_promote','pp_shuffle'):
            rng=random.Random(seed); seen=set(); status='attempt_limit'
            for attempts in range(1,max_attempts+1):
                check(deadline)
                key=tuple(order)
                if key in seen:
                    rng.shuffle(order); key=tuple(order)
                seen.add(key)
                paths,status,failed=pp(instance,deadline,order)
                if paths is not None: break
                if method=='pp_promote': order.remove(failed); order.insert(0,failed)
                else: rng.shuffle(order)
            if paths is None: status='attempt_limit'
        else: raise ValueError(method)
    except Limit:
        status='timeout'; paths=None
    elapsed=time.perf_counter()-begun
    return dict(success=paths is not None,paths=paths,runtime=elapsed,status=status,attempts=attempts,order=order)

def make_grid():
    """12x20 shelf grid with one-cell aisles and recessed endpoint bays.

    Four horizontal aisles are joined by two boundary spines. Station bays
    are leaves of this graph, so parked robots do not obstruct through routes.
    No traversable 2x2 square exists, even when station cells are included.
    """
    grid=[['#']*20 for _ in range(12)]
    for r in (1,4,7,10):
        for c in range(20): grid[r][c]='.'
    for r in range(1,11):
        for c in (0,19): grid[r][c]='.'
    # Boundary stations and staggered recessed pickup/station bays.
    for r in (0,2,5,8,11):
        for c in (2,5,8,11,14,17): grid[r][c]='S'
    for r in (3,6,9):
        for c in (3,6,9,12,15): grid[r][c]='S'
    return [''.join(row) for row in grid]

def generate(layout,k,seed):
    grid=make_grid(); rng=random.Random(seed)
    pool=[p for p in cells(grid) if grid[p[0]][p[1]]=='S'] if layout=='warehouse' else [p for p in cells(grid) if grid[p[0]][p[1]]=='.']
    selected=rng.sample(pool,2*k)
    return dict(grid=grid,starts=selected[:k],goals=selected[k:])
