"""Why did fixed-order PP fail on the benchmark map?

For every fixed-order PP run in scale/results.csv that did not succeed, PP is
re-run robot by robot (60 s limit) to find the robot it could not plan. Then a
static check: with the robots planned before it parked on their goals and treated
as walls, can that robot still get from its start to its goal at all?
"No" means earlier robots parked in a way that cuts the robot off once they are all
in place (it could only succeed by arriving before the wall closes).

Run from the repository root: python scale/diagnose_failures.py
Output of the run used in the README: scale/failure_diagnosis.txt
"""
import sys,csv,json,time
sys.path.insert(0,'scale'); sys.path.insert(0,'.')
import fast_pp, mapf
from maps import load_map
from collections import deque
g=load_map('warehouse-10-20-10-2-1'); graph=fast_pp.Graph(g)
I={d['id']:d for d in json.load(open('scale/instances.json'))['instances']}
R=[r for r in csv.DictReader(open('scale/results.csv')) if r['solver']=='pp' and r['status']!='solved']
for r in R:
    d=I[r['instance_id']]; S=[tuple(x) for x in d['starts']]; G=[tuple(x) for x in d['goals']]
    res=fast_pp.Reservations(graph.n_cells); dl=time.perf_counter()+60; who=None; st=None
    for i in range(len(S)):
        try:
            p,st=fast_pp.single_reserved(graph,S[i],G[i],res,dl)
        except mapf.Limit:
            st='timeout'; p=None
        if p is None: who=i; break
        res.add(p)
    # static check: after earlier robots park, can robot `who` reach its goal from anywhere it could be?
    blocked=set(res.parked)  # parked goals of earlier robots
    def reach(a,b):
        if a in blocked: return False
        seen={a};q=deque([a])
        while q:
            x=q.popleft()
            if x==b: return True
            for y in graph.steps[x]:
                if y not in seen and y not in blocked: seen.add(y);q.append(y)
        return False
    print(r['instance_id'], r['status'], '-> robot', who, 'of', len(S), st,
          '| goal parked-on:', G[who] in blocked, '| start->goal reachable with parked robots as walls:', reach(S[who],G[who]))
