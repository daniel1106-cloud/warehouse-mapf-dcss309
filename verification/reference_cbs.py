"""창고 격자 다중 로봇 경로계획(MAPF) 예비 실험용 구현.

들어 있는 것
- 지도 생성: 선반 블록 + 1칸 통로 + 작업 스테이션 칸
- 시공간 A* (한 대 계획, 예약 테이블/제약 지원)
- Prioritized Planning (업계형 기준선: 한 대씩 순서대로 계획)
- 순서 재조정 전략 (제안): 긴 거리 우선 / 무작위 재시도 / 실패 로봇 앞으로 / 여러 순서 중 최선
- CBS (최적, 참고용 자)
- Joint-state A* (최적, 전수에 가까운 순진한 방법, CBS 교차 검증용)
- 독립 검증기: 계획기와 무관하게 충돌 여부와 비용을 다시 계산
"""
import heapq
import random
import time
from collections import deque

# ----------------------------------------------------------------------------
# 지도
# ----------------------------------------------------------------------------

class Warehouse:
    """세로 2칸 x 가로 blk_w칸 선반 블록을 가로 1칸 통로로 구분한 창고.

    y=0 과 y=H-1 은 스테이션 줄이다. 스테이션 칸(bay)만 열려 있고 나머지는 벽이다.
    cell = y * W + x
    """

    def __init__(self, blk_cols=3, blk_rows=3, blk_w=5):
        self.W = 1 + blk_cols * (blk_w + 1)
        self.H = 2 + 1 + blk_rows * 3            # 스테이션 2줄 + 통로 + (선반2+통로1)*rows
        W, H = self.W, self.H
        self.aisle = set()
        self.shelf = set()
        self.station = set()
        for y in range(H):
            for x in range(W):
                c = y * W + x
                if y == 0 or y == H - 1:
                    if x % 2 == 1:
                        self.station.add(c)
                    continue
                yy = y - 1                       # 통로/선반 영역 안에서의 줄 번호
                if yy % 3 == 0 or x % (blk_w + 1) == 0:
                    self.aisle.add(c)
                else:
                    self.shelf.add(c)
        self.nbrs = {}
        for y in range(H):
            for x in range(W):
                c = y * W + x
                ns = []
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < W and 0 <= ny < H:
                        ns.append(ny * W + nx)
                self.nbrs[c] = ns

    def xy(self, c):
        return c % self.W, c // self.W

    def render(self, marks=None):
        marks = marks or {}
        out = []
        for y in range(self.H):
            row = ''
            for x in range(self.W):
                c = y * self.W + x
                if c in marks:
                    row += marks[c]
                elif c in self.aisle:
                    row += '.'
                elif c in self.shelf:
                    row += '#'
                elif c in self.station:
                    row += 'S'
                else:
                    row += ' '
            out.append(row)
        return '\n'.join(out)


def bfs_dist(wh, goal, passable):
    """goal 에서 각 칸까지의 실제 최단 거리 (A* 의 휴리스틱으로 쓴다)."""
    dist = {goal: 0}
    dq = deque([goal])
    while dq:
        u = dq.popleft()
        for v in wh.nbrs[u]:
            if v in passable and v not in dist:
                dist[v] = dist[u] + 1
                dq.append(v)
    return dist


class Instance:
    """로봇 k대의 출발/목표와, 로봇별로 지나갈 수 있는 칸 집합."""

    def __init__(self, wh, starts, goals, passables):
        self.wh = wh
        self.starts = starts
        self.goals = goals
        self.passables = passables               # 로봇별 set
        self.k = len(starts)
        self.h = [bfs_dist(wh, goals[i], passables[i]) for i in range(self.k)]
        self.nfree = max(len(p) for p in passables)


def make_aisle_instance(wh, k, rng):
    """F1 '통로 배치': 출발과 목표가 모두 통로 칸. 교과서형 one-shot MAPF.
    도착한 로봇이 통로 한가운데 계속 서 있으므로 길을 막을 수 있다."""
    cells = sorted(wh.aisle)
    starts = rng.sample(cells, k)
    goals = rng.sample(cells, k)
    pas = set(wh.aisle)
    return Instance(wh, starts, goals, [pas] * k)


def make_wellformed_instance(wh, k, rng):
    """F2 '창고형 배치': 출발/목표가 선반 밑 칸 또는 스테이션 칸.
    남의 선반 밑과 남의 스테이션은 지나갈 수 없다(선반을 든 로봇은 다른 선반 밑을 못 지난다).
    따라서 도착해서 서 있는 로봇이 통로를 막지 않는다."""
    shelves = rng.sample(sorted(wh.shelf), k)
    stations = rng.sample(sorted(wh.station), k)
    starts, goals, pas = [], [], []
    for i in range(k):
        if rng.random() < 0.5:
            s, g = shelves[i], stations[i]       # 선반을 들고 스테이션으로
        else:
            s, g = stations[i], shelves[i]       # 스테이션에서 보관 칸으로
        starts.append(s)
        goals.append(g)
        pas.append(set(wh.aisle) | {s, g})
    return Instance(wh, starts, goals, pas)


# ----------------------------------------------------------------------------
# 시공간 A*
# ----------------------------------------------------------------------------

def st_astar(wh, start, goal, passable, h, vcons, econs, parked, min_goal_t, tmax,
             deadline=None):
    """한 대의 최단 경로. 상태는 (칸, 시각).

    vcons  : {(칸, t)}        그 시각에 그 칸에 있으면 안 된다
    econs  : {(u, v, t)}      t-1 -> t 사이에 u 에서 v 로 움직이면 안 된다
    parked : {칸: t0}         t0 이후로 계속 누가 서 있는 칸
    min_goal_t : 이 시각 이후에 목표에 있어야 '도착'으로 인정 (그 뒤로 영원히 서 있을 수 있어야 하므로)
    반환   : [p0, p1, ..., pL] 또는 None
    """
    if start not in h:
        return None
    openh = [(h[start], 0, start)]
    parent = {(start, 0): None}
    closed = set()
    nbrs = wh.nbrs
    pops = 0
    while openh:
        f, negg, cell = heapq.heappop(openh)
        t = -negg
        if (cell, t) in closed:
            continue
        closed.add((cell, t))
        pops += 1
        if deadline is not None and (pops & 1023) == 0 and time.perf_counter() > deadline:
            return None
        if cell == goal and t >= min_goal_t:
            path = []
            s = (cell, t)
            while s is not None:
                path.append(s[0])
                s = parent[s]
            path.reverse()
            return path
        if t >= tmax:
            continue
        nt = t + 1
        for nc in nbrs[cell] + [cell]:
            if nc not in passable or nc not in h:
                continue
            if (nc, nt) in vcons or (cell, nc, nt) in econs:
                continue
            p0 = parked.get(nc)
            if p0 is not None and nt >= p0:
                continue
            key = (nc, nt)
            if key in closed or key in parent:
                continue
            parent[key] = (cell, t)
            heapq.heappush(openh, (nt + h[nc], -nt, nc))
    return None


# ----------------------------------------------------------------------------
# Prioritized Planning (기준선)
# ----------------------------------------------------------------------------

def prioritized_planning(inst, order, deadline=None):
    """order 순서대로 한 대씩 계획한다. 앞 로봇의 경로는 움직이는 장애물로 취급한다.
    반환: (paths 또는 None, 실패한 로봇 번호 또는 None)"""
    wh = inst.wh
    vcons, econs, parked, last_visit = set(), set(), {}, {}
    paths = [None] * inst.k
    tp = 0                                        # 앞 로봇들이 모두 멈추는 시각
    for i in order:
        s, g = inst.starts[i], inst.goals[i]
        if g in parked:
            return None, i
        min_goal_t = last_visit.get(g, -1) + 1
        tmax = tp + inst.nfree + 2
        p = st_astar(wh, s, g, inst.passables[i], inst.h[i], vcons, econs, parked,
                     min_goal_t, tmax, deadline)
        if p is None:
            return None, i
        paths[i] = p
        L = len(p) - 1
        for t, c in enumerate(p):
            vcons.add((c, t))
            if last_visit.get(c, -1) < t:
                last_visit[c] = t
        for t in range(1, L + 1):
            econs.add((p[t], p[t - 1], t))        # 맞바꾸기 금지
        parked[g] = L
        tp = max(tp, L)
    return paths, None


def soc(paths):
    """이동 시간 합(sum of costs): 각 로봇이 목표에 '마지막으로 도착한' 시각의 합."""
    return sum(len(p) - 1 for p in paths)


def order_index(inst):
    return list(range(inst.k))


def order_longest_first(inst):
    return sorted(range(inst.k), key=lambda i: -inst.h[i].get(inst.starts[i], 0))


def pp_random_restart(inst, max_tries, rng, deadline=None):
    """실패하면 순서를 무작위로 다시 섞어 재시도."""
    order = order_index(inst)
    for tries in range(1, max_tries + 1):
        paths, _ = prioritized_planning(inst, order, deadline)
        if paths is not None:
            return paths, tries
        if deadline is not None and time.perf_counter() > deadline:
            break
        order = order[:]
        rng.shuffle(order)
    return None, tries


def pp_failed_first(inst, max_tries, rng, deadline=None):
    """실패하면 '길을 못 찾은 그 로봇'을 맨 앞으로 올려 재시도.
    같은 순서가 다시 나오면(돌고 도는 경우) 무작위로 섞는다."""
    order = order_index(inst)
    seen = set()
    for tries in range(1, max_tries + 1):
        seen.add(tuple(order))
        paths, failed = prioritized_planning(inst, order, deadline)
        if paths is not None:
            return paths, tries
        if deadline is not None and time.perf_counter() > deadline:
            break
        order = [failed] + [a for a in order if a != failed]
        if tuple(order) in seen:
            order = order[:]
            rng.shuffle(order)
    return None, tries


def pp_best_of(inst, n_orders, rng, deadline=None):
    """순서 n_orders 개를 시도해 이동 시간 합이 가장 작은 계획을 남긴다."""
    best = None
    order = order_index(inst)
    for t in range(n_orders):
        paths, _ = prioritized_planning(inst, order, deadline)
        if paths is not None and (best is None or soc(paths) < soc(best)):
            best = paths
        if deadline is not None and time.perf_counter() > deadline:
            break
        order = order[:]
        rng.shuffle(order)
    return best


# ----------------------------------------------------------------------------
# 독립 검증기
# ----------------------------------------------------------------------------

def validate(inst, paths):
    """계획기 코드를 쓰지 않고 경로 집합이 규칙을 지키는지 다시 확인한다.
    반환: (True, 이동 시간 합) 또는 (False, 이유)"""
    wh = inst.wh
    if paths is None or len(paths) != inst.k:
        return False, 'no paths'
    T = max(len(p) for p in paths)
    for i, p in enumerate(paths):
        if p[0] != inst.starts[i]:
            return False, f'robot {i} wrong start'
        if p[-1] != inst.goals[i]:
            return False, f'robot {i} wrong goal'
        for t in range(len(p)):
            if p[t] not in inst.passables[i]:
                return False, f'robot {i} in blocked cell at t={t}'
            if t > 0 and p[t] != p[t - 1] and p[t] not in wh.nbrs[p[t - 1]]:
                return False, f'robot {i} jumps at t={t}'
        if len(p) > 1 and p[-2] == p[-1]:
            return False, f'robot {i} path has trailing wait (cost overcounted)'

    def at(i, t):
        p = paths[i]
        return p[t] if t < len(p) else p[-1]

    for t in range(T):
        pos = {}
        for i in range(inst.k):
            c = at(i, t)
            if c in pos:
                return False, f'vertex conflict robots {pos[c]},{i} t={t}'
            pos[c] = i
        if t > 0:
            for i in range(inst.k):
                a, b = at(i, t - 1), at(i, t)
                if a == b:
                    continue
                j = pos.get(a)
                if j is not None and j != i and at(j, t - 1) == b:
                    return False, f'swap conflict robots {i},{j} t={t}'
    return True, sum(len(p) - 1 for p in paths)


def count_conflicts_independent(inst):
    """조율 없이 각자 최단 경로로 갈 때 생기는 충돌 수."""
    paths = []
    for i in range(inst.k):
        p = st_astar(inst.wh, inst.starts[i], inst.goals[i], inst.passables[i], inst.h[i],
                     set(), set(), {}, 0, inst.nfree + 2)
        if p is None:
            return None
        paths.append(p)
    return len(all_conflicts(paths))


# ----------------------------------------------------------------------------
# CBS (최적, 참고용)
# ----------------------------------------------------------------------------

def _at(p, t):
    return p[t] if t < len(p) else p[-1]


def all_conflicts(paths):
    out = []
    k = len(paths)
    T = max(len(p) for p in paths)
    for t in range(T):
        pos = {}
        for i in range(k):
            c = _at(paths[i], t)
            if c in pos:
                out.append(('v', pos[c], i, c, t))
            else:
                pos[c] = i
        if t > 0:
            for i in range(k):
                a, b = _at(paths[i], t - 1), _at(paths[i], t)
                if a == b:
                    continue
                j = pos.get(a)
                if j is not None and j > i and _at(paths[j], t - 1) == b:
                    out.append(('e', i, j, a, b, t))
    return out


def first_conflict(paths):
    k = len(paths)
    T = max(len(p) for p in paths)
    for t in range(T):
        pos = {}
        for i in range(k):
            c = _at(paths[i], t)
            if c in pos:
                return ('v', pos[c], i, c, t)
            pos[c] = i
        if t > 0:
            for i in range(k):
                a, b = _at(paths[i], t - 1), _at(paths[i], t)
                if a == b:
                    continue
                j = pos.get(a)
                if j is not None and j != i and _at(paths[j], t - 1) == b:
                    return ('e', i, j, a, b, t)
    return None


def _low_level(inst, i, vc, ec, deadline):
    mg = 0
    mt = 0
    g = inst.goals[i]
    for (c, t) in vc:
        if c == g and t + 1 > mg:
            mg = t + 1
        if t > mt:
            mt = t
    for (_, _, t) in ec:
        if t > mt:
            mt = t
    return st_astar(inst.wh, inst.starts[i], g, inst.passables[i], inst.h[i], vc, ec, {},
                    mg, mt + inst.nfree + 2, deadline)


def cbs(inst, time_limit):
    """Conflict-Based Search. 이동 시간 합이 최소인 계획을 찾는다.
    반환: (paths 또는 None, 펼친 상위 노드 수)"""
    deadline = time.perf_counter() + time_limit
    k = inst.k
    cons = [(frozenset(), frozenset()) for _ in range(k)]
    paths = []
    for i in range(k):
        p = _low_level(inst, i, cons[i][0], cons[i][1], deadline)
        if p is None:
            return None, 0
        paths.append(p)
    cnt = 0
    openh = [(soc(paths), cnt, cons, paths)]
    expanded = 0
    while openh:
        if time.perf_counter() > deadline:
            return None, expanded
        cost, _, cons, paths = heapq.heappop(openh)
        expanded += 1
        cf = first_conflict(paths)
        if cf is None:
            return paths, expanded
        if cf[0] == 'v':
            _, a, b, c, t = cf
            branches = [(a, ('v', (c, t))), (b, ('v', (c, t)))]
        else:
            _, a, b, u, v, t = cf                 # a: u->v, b: v->u
            branches = [(a, ('e', (u, v, t))), (b, ('e', (v, u, t)))]
        for ag, (kind, con) in branches:
            vc, ec = cons[ag]
            if kind == 'v':
                vc = vc | {con}
            else:
                ec = ec | {con}
            np_ = _low_level(inst, ag, vc, ec, deadline)
            if np_ is None:
                continue
            ncons = list(cons)
            ncons[ag] = (vc, ec)
            npaths = list(paths)
            npaths[ag] = np_
            cnt += 1
            heapq.heappush(openh, (soc(npaths), cnt, ncons, npaths))
    return None, expanded


# ----------------------------------------------------------------------------
# Joint-state A* (순진한 정확 해법)
# ----------------------------------------------------------------------------

def joint_astar(inst, time_limit):
    """모든 로봇의 위치 조합을 하나의 상태로 보고 탐색한다. 한 단계에 로봇마다
    5가지(상하좌우, 대기) + '도착 확정'을 고르므로 가지 수가 로봇 수에 지수적이다.
    반환: (최적 이동 시간 합 또는 None, 펼친 상태 수)"""
    deadline = time.perf_counter() + time_limit
    k = inst.k
    wh = inst.wh
    goals = inst.goals
    h = inst.h
    for i in range(k):
        if inst.starts[i] not in h[i]:
            return None, 0
    start = (tuple(inst.starts), 0)               # (위치들, 도착 확정 비트마스크)
    full = (1 << k) - 1

    def hsum(pos, done):
        return sum(h[i][pos[i]] for i in range(k) if not (done >> i) & 1)

    g0 = {start: 0}
    openh = [(hsum(*start), 0, start)]
    expanded = 0
    while openh:
        f, g, state = heapq.heappop(openh)
        if g > g0.get(state, 1 << 60):
            continue
        pos, done = state
        if done == full:
            return g, expanded
        expanded += 1
        if (expanded & 255) == 0 and time.perf_counter() > deadline:
            return None, expanded
        # 로봇별 선택지
        options = []
        for i in range(k):
            if (done >> i) & 1:
                options.append([(pos[i], True, 0)])
                continue
            opts = []
            if pos[i] == goals[i]:
                opts.append((pos[i], True, 0))    # 여기서 끝내고 계속 서 있기
            for nc in wh.nbrs[pos[i]] + [pos[i]]:
                if nc in inst.passables[i] and nc in h[i]:
                    opts.append((nc, False, 1))
            options.append(opts)

        # 조합을 깊이 우선으로 만들며 충돌하는 가지는 바로 버린다
        newpos = [None] * k

        def rec(i, used, ndone, cost):
            if i == k:
                ns = (tuple(newpos), ndone)
                ng = g + cost
                if ng < g0.get(ns, 1 << 60):
                    g0[ns] = ng
                    heapq.heappush(openh, (ng + hsum(*ns), ng, ns))
                return
            for (nc, dn, c) in options[i]:
                if nc in used:
                    continue
                ok = True
                for j in range(i):                # 맞바꾸기 검사
                    if newpos[j] == pos[i] and nc == pos[j] and nc != pos[i]:
                        ok = False
                        break
                if not ok:
                    continue
                newpos[i] = nc
                used.add(nc)
                rec(i + 1, used, ndone | (1 << i) if dn else ndone, cost + c)
                used.discard(nc)

        rec(0, set(), 0, 0)
    return None, expanded
