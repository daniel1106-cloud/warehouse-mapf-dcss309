"""Pipeline regression checks for complete batches and data-driven reports."""
import csv,json,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import run
from report import write_report

class PipelineTests(unittest.TestCase):
    def test_joint_finishes_timeout_size_before_skipping(self):
        calls=[]
        def fake_solve(instance,time_limit,method,seed=0,max_attempts=32):
            calls.append((method,len(instance['starts'])))
            return dict(paths=None,status='timeout' if method=='joint_astar' else 'blocked',runtime=.001,attempts=1,order=[])
        with tempfile.TemporaryDirectory() as tmp:
            with patch.object(sys,'argv',['run.py','--instances','2','--max-robots','4','--output',tmp]), patch.object(run,'solve',side_effect=fake_solve), patch.object(run,'plot'), patch.object(run,'animate'), patch('builtins.print'):
                run.main()
            with (Path(tmp)/'results.csv').open() as f: rows=list(csv.DictReader(f))
            for layout in ['aisle','warehouse']:
                size2=[r for r in rows if r['layout']==layout and r['solver']=='joint_astar' and r['num_robots']=='2']
                size4=[r for r in rows if r['layout']==layout and r['solver']=='joint_astar' and r['num_robots']=='4']
                self.assertEqual([r['status'] for r in size2],['timeout']*2)
                self.assertEqual([r['status'] for r in size4],['skipped_after_timeout']*2)
            self.assertEqual(sum(m=='joint_astar' for m,k in calls),4)
            plans=[json.loads(line) for line in (Path(tmp)/'plans.jsonl').read_text().splitlines()]
            self.assertEqual(len(plans),len(rows))
            self.assertTrue((Path(tmp)/'EXPERIMENT_REPORT.md').exists())
            # Rebuild from altered current data: the report must not use static numbers.
            rows[0]['success']='True'
            with (Path(tmp)/'results.csv').open('w',newline='') as f:
                writer=csv.DictWriter(f,fieldnames=run.FIELDS);writer.writeheader();writer.writerows(rows)
            content=write_report(tmp)
            self.assertIn('| aisle | independent | 4 | 0 | 1 | 0 |',content)
            self.assertIn('not yet been included',content)

if __name__=='__main__': unittest.main()
