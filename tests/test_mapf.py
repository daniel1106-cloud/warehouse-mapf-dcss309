import unittest,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mapf import validate,solve,generate,make_grid,cells,neighbors,distances

class ValidationTests(unittest.TestCase):
    def case(self,paths,goals,expected):
        inst=dict(grid=['....','....'],starts=[p[0] for p in paths],goals=goals)
        v=validate(inst,paths)
        self.assertEqual((v['vertex_conflicts'],v['swap_conflicts'],v['sum_of_costs']),expected)
    def test_A_swap(self): self.case([[(0,0),(0,1)],[(0,1),(0,0)]],[(0,1),(0,0)],(0,1,2))
    def test_B_vertex(self): self.case([[(0,0),(0,1),(0,2)],[(0,2),(0,1),(0,0)]],[(0,2),(0,0)],(1,0,4))
    def test_C_clear(self): self.case([[(0,0),(0,1),(0,2)],[(1,2),(1,1),(1,0)]],[(0,2),(1,0)],(0,0,4))
    def test_D_parked(self): self.case([[(0,0),(0,1)],[(0,3),(0,2),(0,1),(0,0)]],[(0,1),(0,0)],(1,0,4))
    def test_E_return(self): self.case([[(0,0),(0,1),(0,0),(0,1)]],[(0,1)],(0,0,3))
    def test_trailing_wait(self): self.case([[(0,0),(0,1),(0,1)]],[(0,1)],(0,0,1))
    def test_illegal(self):
        inst=dict(grid=['.#.'],starts=[(0,0)],goals=[(0,2)])
        self.assertFalse(validate(inst,[[(0,0),(0,2)]])['valid'])
    def test_optimal_detour(self):
        inst=dict(grid=['...','...'],starts=[(0,0),(0,2)],goals=[(0,2),(0,0)])
        result=solve(inst,5,'joint_astar'); v=validate(inst,result['paths'])
        self.assertEqual(result['status'],'optimal'); self.assertTrue(v['valid']); self.assertEqual(v['sum_of_costs'],6)
    def test_unsolvable(self):
        inst=dict(grid=['..'],starts=[(0,0),(0,1)],goals=[(0,1),(0,0)])
        self.assertEqual(solve(inst,2,'joint_astar')['status'],'no_solution')
    def test_start_goal(self):
        inst=dict(grid=['...','...'],starts=[(0,1),(0,0)],goals=[(0,1),(0,2)])
        result=solve(inst,5,'joint_astar'); v=validate(inst,result['paths']); self.assertTrue(v['valid']); self.assertEqual(v['sum_of_costs'],4)
    def test_goal_must_be_vacated(self):
        inst=dict(grid=['#.#','...'],starts=[(1,1),(1,0)],goals=[(1,1),(1,2)])
        result=solve(inst,5,'joint_astar'); v=validate(inst,result['paths'])
        self.assertTrue(v['valid']); self.assertEqual(v['sum_of_costs'],4)
    def test_one_cell_aisles_and_accessible_bays(self):
        grid=make_grid()
        self.assertEqual((len(grid),{len(row) for row in grid}),(12,{20}))
        free=set(cells(grid))
        for r in range(11):
            for c in range(19):
                self.assertFalse({(r,c),(r+1,c),(r,c+1),(r+1,c+1)}<=free)
        stations=[p for p in free if grid[p[0]][p[1]]=='S']
        self.assertGreaterEqual(len(stations),40)
        self.assertEqual(len(distances(grid,stations[0])),len(free))
        for station in stations:
            adjacent=list(neighbors(grid,station,False))
            self.assertEqual(len(adjacent),1)
            r,c=adjacent[0]
            self.assertEqual(grid[r][c],'.')
        for layout in ('aisle','warehouse'):
            inst=generate(layout,20,20261029)
            self.assertEqual(len(set(inst['starts']+inst['goals'])),40)
    def test_reproducible(self): self.assertEqual(generate('warehouse',20,42),generate('warehouse',20,42))
    def test_solvers(self):
        inst=generate('warehouse',4,19)
        for method in ['pp','pp_promote','pp_shuffle']:
            result=solve(inst,3,method)
            self.assertIsNotNone(result['paths']); self.assertTrue(validate(inst,result['paths'])['valid'])

if __name__=='__main__': unittest.main()
