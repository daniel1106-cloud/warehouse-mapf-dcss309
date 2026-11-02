"""Hold instances fixed; change ONLY the PP order, including successful orders."""
import argparse,csv,json,random,time
from pathlib import Path
from mapf import pp,validate,Limit
from report import write_report

def main():
    p=argparse.ArgumentParser(); p.add_argument('--output',default='outputs'); p.add_argument('--trials',type=int,default=5); p.add_argument('--time-limit',type=float,default=None); args=p.parse_args(); out=Path(args.output)
    instances=json.loads((out/'instances.json').read_text())
    metadata=json.loads((out/'metadata.json').read_text())
    if args.time_limit is None: args.time_limit=metadata['config']['time_limit']
    if args.trials<1 or args.time_limit<=0: p.error('trials and time limit must be positive')
    with (out/'order_sensitivity.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=['instance_id','layout','num_robots','trial','order','success','status','runtime_sec','sum_of_costs']); writer.writeheader()
        for inst in instances:
            inst['starts']=list(map(tuple,inst['starts'])); inst['goals']=list(map(tuple,inst['goals']))
            rng=random.Random(inst['seed']); order=list(range(len(inst['starts'])))
            for trial in range(args.trials):
                if trial: rng.shuffle(order)
                begun=time.perf_counter(); paths=None
                try: paths,status,_=pp(inst,begun+args.time_limit,order)
                except Limit: status='timeout'
                elapsed=time.perf_counter()-begun; report=validate(inst,paths) if paths else None
                writer.writerow(dict(instance_id=inst['id'],layout=inst['id'].split('-')[0],num_robots=len(order),trial=trial,order=json.dumps(order),success=bool(report and report['valid'] and elapsed<=args.time_limit),status=status,runtime_sec=elapsed,sum_of_costs=report['sum_of_costs'] if report else ''))
    metadata['order_study']=dict(trials_per_instance=args.trials,time_limit=args.time_limit,records=len(instances)*args.trials)
    (out/'metadata.json').write_text(json.dumps(metadata,indent=2))
    write_report(out,include_order=True)
    print('Order study saved:',out/'order_sensitivity.csv')
if __name__=='__main__': main()
