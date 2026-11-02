"""One command: python run.py (full protocol); --quick for a smoke run."""
import argparse,csv,json,platform,statistics,time
from report import write_report
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation,PillowWriter
from mapf import solve,generate,validate,at

METHODS=['independent','joint_astar','pp','pp_promote','pp_shuffle']
FIELDS=['layout','num_robots','instance_id','seed','solver','success','status','runtime_sec','sum_of_costs','vertex_conflicts','swap_conflicts','attempts','order','optimal_cost','gap_pct']

def plot(rows,out):
    fig,axes=plt.subplots(1,2,figsize=(12,4))
    for ax,layout in zip(axes,['aisle','warehouse']):
        for m in METHODS:
            pts=[]
            for k in sorted({int(r['num_robots']) for r in rows}):
                group=[r for r in rows if r['layout']==layout and r['solver']==m and int(r['num_robots'])==k and r['status']!='skipped_after_timeout']
                if group:
                    value=statistics.median(float(r['runtime_sec']) for r in group); pts.append((k,value))
                    if m=='joint_astar': ax.annotate(f'n={len(group)}',(k,value),xytext=(0,8),textcoords='offset points',ha='center',fontsize=8)
            if pts: ax.plot(*zip(*pts),marker='o',label=m)
        ax.set(title=layout,xlabel='Robots',ylabel='Median runtime (seconds)',yscale='log'); ax.grid(alpha=.2); ax.legend(fontsize=7)
    fig.suptitle('Runtime (executed runs, including failures/timeouts)'); fig.tight_layout(); fig.savefig(out/'runtime.png',dpi=160); plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(12,4))
    for ax,layout in zip(axes,['aisle','warehouse']):
        for m in METHODS:
            pts=[]
            for k in sorted({int(r['num_robots']) for r in rows}):
                group=[r for r in rows if r['layout']==layout and r['solver']==m and int(r['num_robots'])==k and r['status']!='skipped_after_timeout']
                if group:
                    value=100*sum(str(r['success']) in ('True','1') for r in group)/len(group); pts.append((k,value))
                    if m=='joint_astar': ax.annotate(f'n={len(group)}',(k,value),xytext=(0,-14),textcoords='offset points',ha='center',fontsize=8)
            if pts: ax.plot(*zip(*pts),marker='o',label=m)
        ax.set(title=layout,xlabel='Robots',ylabel='Collision-free success (%)',ylim=(-3,103)); ax.grid(alpha=.2); ax.legend(fontsize=7)
    fig.suptitle('Success (executed runs; skipped runs excluded)'); fig.tight_layout(); fig.savefig(out/'success.png',dpi=160); plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(12,4))
    for ax,layout in zip(axes,['aisle','warehouse']):
        for m in METHODS:
            if m in ('independent','joint_astar'): continue
            pts=[]
            for k in sorted({int(r['num_robots']) for r in rows}):
                vals=[float(r['gap_pct']) for r in rows if r['layout']==layout and r['solver']==m and int(r['num_robots'])==k and r['gap_pct']!='']
                if vals:
                    pts.append((k,statistics.median(vals)))
                    if m=='pp': ax.annotate(f'n={len(vals)}', (k,statistics.median(vals)),xytext=(0,8),textcoords='offset points',ha='center',fontsize=8)
            if pts: ax.plot(*zip(*pts),marker='o',label=m)
        ax.set(title=layout,xlabel='Robots',ylabel='Median cost above optimum (%)'); ax.grid(alpha=.2)
        if ax.lines: ax.legend(fontsize=7)
        else: ax.text(.5,.5,'No matched optimal solutions',ha='center',transform=ax.transAxes)
    fig.suptitle('Cost gap (paired successful runs with proven Joint A* optimum only)'); fig.tight_layout(); fig.savefig(out/'cost_gap.png',dpi=160); plt.close(fig)

def animate(instance,paths,target):
    report=validate(instance,paths); n=len(paths); grid=instance['grid']; colors=plt.get_cmap('tab20')
    fig,ax=plt.subplots(figsize=(10,6))
    def frame(t):
        ax.clear()
        ax.imshow([[0 if c=='#' else .8 if c=='S' else 1 for c in row] for row in grid],cmap='gray',vmin=0,vmax=1)
        for i,(p,g) in enumerate(zip(paths,instance['goals'])):
            r,c=at(p,t); ax.scatter(c,r,s=150,color=colors(i%20),edgecolor='black',zorder=3); ax.text(c,r,str(i),ha='center',va='center',fontsize=8,zorder=4)
            ax.scatter(g[1],g[0],s=120,marker='x',color=colors(i%20))
        events=[e for e in report['events'] if e[0]==t]
        for e in events:
            for r,c in e[3:]: ax.scatter(c,r,s=350,facecolor='none',edgecolor='red',linewidth=3,zorder=5)
        ax.set(title=f'Independent BFS demonstration | t={t} | conflicts now={len(events)}',xticks=range(len(grid[0])),yticks=range(len(grid)))
        ax.grid(alpha=.15); fig.tight_layout()
    anim=FuncAnimation(fig,frame,frames=max(map(len,paths))+3,interval=350)
    anim.save(target,writer=PillowWriter(fps=3)); plt.close(fig)

def main():
    p=argparse.ArgumentParser(); p.add_argument('--quick',action='store_true'); p.add_argument('--instances',type=int,default=20); p.add_argument('--max-robots',type=int,default=20); p.add_argument('--time-limit',type=float,default=10); p.add_argument('--seed',type=int,default=20261029); p.add_argument('--attempts',type=int,default=32); p.add_argument('--output',default='outputs'); p.add_argument('--plot-only',action='store_true'); args=p.parse_args()
    if args.instances<1 or not 2<=args.max_robots<=20 or args.time_limit<=0 or args.attempts<1:
        p.error('instances and attempts must be positive, max-robots must be between 2 and 20, and time-limit must be positive')
    out=Path(args.output); out.mkdir(parents=True,exist_ok=True)
    if args.plot_only:
        with (out/'results.csv').open(encoding='utf-8') as f: plot(list(csv.DictReader(f)),out)
        existing_meta=json.loads((out/'metadata.json').read_text(encoding='utf-8'))
        write_report(out,include_order='order_study' in existing_meta)
        return
    if args.quick: args.instances=3; args.max_robots=6; args.time_limit=1
    rows=[]; instances=[]; stop_joint={}; demo=None; begun=time.time()
    with (out/'results.csv').open('w',newline='',encoding='utf-8') as f, (out/'plans.jsonl').open('w',encoding='utf-8') as plans:
        writer=csv.DictWriter(f,fieldnames=FIELDS); writer.writeheader()
        for li,layout in enumerate(['aisle','warehouse']):
            for k in range(2,args.max_robots+1,2):
                for j in range(args.instances):
                    seed=args.seed+li*100000+k*100+j; instance=generate(layout,k,seed); iid=f'{layout}-k{k}-{j:03d}'
                    instances.append(dict(id=iid,seed=seed,**instance)); batch=[]; optimum=None
                    for method in METHODS:
                        if method=='joint_astar' and layout in stop_joint and k>stop_joint[layout]['num_robots']:
                            result=dict(paths=None,status='skipped_after_timeout',runtime=0,attempts=0,order=[])
                        else:
                            result=solve(instance,args.time_limit,method,seed,args.attempts)
                        report=validate(instance,result['paths']) if result['paths'] is not None else dict(valid=False,sum_of_costs=None,vertex_conflicts='',swap_conflicts='')
                        success=report['valid'] and result['runtime']<=args.time_limit
                        status=result['status']
                        if result['paths'] is not None and not report['valid']: status='collision' if not report['errors'] else 'invalid_path'
                        if result['paths'] is not None and result['runtime']>args.time_limit: status='timeout'; success=False
                        if method=='joint_astar' and status=='timeout' and layout not in stop_joint:
                            stop_joint[layout]=dict(first_timeout=iid,num_robots=k,policy='finish_current_size_then_skip_larger')
                        plans.write(json.dumps(dict(instance_id=iid,solver=method,status=status,paths=result['paths']))+'\n')
                        row=dict(layout=layout,num_robots=k,instance_id=iid,seed=seed,solver=method,success=success,status=status,runtime_sec=result['runtime'],sum_of_costs=report['sum_of_costs'] if report['sum_of_costs'] is not None else '',vertex_conflicts=report['vertex_conflicts'],swap_conflicts=report['swap_conflicts'],attempts=result['attempts'],order=json.dumps(result['order']),optimal_cost='',gap_pct='')
                        if method=='joint_astar' and success: optimum=report['sum_of_costs']
                        if method=='independent' and report['vertex_conflicts'] and demo is None: demo=(instance,result['paths'])
                        batch.append(row)
                    for row in batch:
                        if optimum is not None:
                            row['optimal_cost']=optimum
                            if row['success'] and optimum>0:
                                assert row['sum_of_costs']>=optimum, 'Planner cost below proven optimum!'
                                row['gap_pct']=100*(row['sum_of_costs']/optimum-1)
                        writer.writerow(row); rows.append(row)
                    f.flush()
                print(f'{layout} k={k} complete ({args.instances} instances per method; Joint A* skips only larger sizes after a timeout)',flush=True)
    (out/'instances.json').write_text(json.dumps(instances,indent=2))
    metadata=dict(config=vars(args),python=platform.python_version(),platform=platform.platform(),seconds=time.time()-begun,joint_stopped_at=stop_joint,notes='After the first Joint A* timeout in each layout, all instances at that robot count are completed; only larger sizes are skipped. Remaining rows are skipped, not failures. Success denominators exclude skipped rows. Same instance/seed for all methods. Timings are machine-dependent.')
    (out/'metadata.json').write_text(json.dumps(metadata,indent=2))
    plot(rows,out)
    if demo is None:
        demo_instance={'grid':['....','....'],'starts':[(0,0),(0,3)],'goals':[(0,3),(0,0)]}; demo=(demo_instance,solve(demo_instance,1,'independent')['paths'])
    animate(*demo,out/'demo.gif')
    summary=[]
    for layout in ['aisle','warehouse']:
        for m in METHODS:
            group=[r for r in rows if r['layout']==layout and r['solver']==m]; executed=[r for r in group if r['status']!='skipped_after_timeout']
            summary.append(dict(layout=layout,solver=m,executed=len(executed),skipped=len(group)-len(executed),successes=sum(r['success'] for r in executed),timeouts=sum(r['status']=='timeout' for r in executed)))
    (out/'summary.json').write_text(json.dumps(summary,indent=2)); print(json.dumps(summary,indent=2))
    write_report(out)

if __name__=='__main__': main()
