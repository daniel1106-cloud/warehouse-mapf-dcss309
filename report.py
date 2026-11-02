"""Generate English reports from current CSVs; no hard-coded result numbers."""
import argparse
import csv
import json
import statistics
from collections import defaultdict
from pathlib import Path


def successful(row):
    return str(row['success']).lower() in ('true','1')


def write_report(output,include_order=False):
    out=Path(output)
    with (out/'results.csv').open(encoding='utf-8',newline='') as f:
        rows=list(csv.DictReader(f))
    meta=json.loads((out/'metadata.json').read_text(encoding='utf-8'))
    config=meta['config']
    lines=['# Experimental Results','',
           'Automatically generated from the current results.csv and metadata.json. This file is refreshed after each main run and after the planning-order study.','',
           f"Configuration: {config['instances']} instances per robot count and layout; robot counts from 2 to {config['max_robots']} in steps of 2; {config['time_limit']} seconds per solver run; base seed {config['seed']}.",'',
           'Map: 12 rows by 20 columns, one-cell-wide connecting aisles, and 45 recessed station/pickup bays. No traversable 2x2 square exists.','',
           f"Distinct instances: {len({r['instance_id'] for r in rows})}. Main-experiment records: {len(rows)}.",'',
           '| Layout | Method | Executed | Skipped | Successful | Timeouts |',
           '| --- | --- | ---: | ---: | ---: | ---: |']
    for layout in dict.fromkeys(r['layout'] for r in rows):
        for method in dict.fromkeys(r['solver'] for r in rows):
            group=[r for r in rows if r['layout']==layout and r['solver']==method]
            executed=[r for r in group if r['status']!='skipped_after_timeout']
            lines.append(f"| {layout} | {method} | {len(executed)} | {len(group)-len(executed)} | {sum(successful(r) for r in executed)} | {sum(r['status']=='timeout' for r in executed)} |")
    lines+=['','## Q1: Runtime and scalability','',
            'Joint A* attempts every instance at the current robot count even if one times out. After completing that size, larger sizes are skipped for that layout. Skipped rows are excluded from runtime medians and success-rate denominators.','',
            '| Layout | Robots | Joint A* executed | Successful | Timeouts | Median seconds |',
            '| --- | ---: | ---: | ---: | ---: | ---: |']
    groups=defaultdict(list)
    for r in rows:
        if r['solver']=='joint_astar' and r['status']!='skipped_after_timeout':
            groups[(r['layout'],int(r['num_robots']))].append(r)
    for (layout,k),group in sorted(groups.items()):
        lines.append(f"| {layout} | {k} | {len(group)} | {sum(successful(r) for r in group)} | {sum(r['status']=='timeout' for r in group)} | {statistics.median(float(r['runtime_sec']) for r in group):.6f} |")
    lines+=['','These measurements characterize this implementation and machine, not every optimal planner or a production warehouse.','',
            '## Q2: Cost relative to a proven optimum','',
            'Only matched successful runs on instances with a proven Joint A* optimum are compared. A zero median gap does not mean every run is optimal. Sample counts may be smaller than the number of executed instances.','',
            '| Layout | Robots | Method | Paired samples | Median gap (%) | Maximum gap (%) |',
            '| --- | ---: | --- | ---: | ---: | ---: |']
    gaps=defaultdict(list)
    for r in rows:
        if r['solver'] in ('pp','pp_promote','pp_shuffle') and successful(r) and r['gap_pct']!='':
            gaps[(r['layout'],int(r['num_robots']),r['solver'])].append(float(r['gap_pct']))
    for (layout,k,method),values in sorted(gaps.items()):
        lines.append(f'| {layout} | {k} | {method} | {len(values)} | {statistics.median(values):.2f} | {max(values):.2f} |')
    if not gaps: lines+=['','No matched optimal solutions are available in this run.']
    lines+=['','## Q3: Aisle versus warehouse endpoints','']
    for layout in ('aisle','warehouse'):
        group=[r for r in rows if r['layout']==layout and r['solver']=='pp' and r['status']!='skipped_after_timeout']
        if group: lines.append(f"- Fixed-order PP on {layout}: {sum(successful(r) for r in group)}/{len(group)} successful ({100*sum(successful(r) for r in group)/len(group):.1f}%).")
    lines+=['','The layouts use different endpoint sampling distributions. Differences do not prove that a single factor alone caused the result, or guarantee success in every warehouse.','',
            '## Q4: Changing the planning order','']
    fixed={r['instance_id']:successful(r) for r in rows if r['solver']=='pp'}
    for method in ('pp_promote','pp_shuffle'):
        rescued=sum(successful(r) and not fixed.get(r['instance_id'],True) for r in rows if r['solver']==method)
        lines.append(f'- {method}: solved {rescued} instances that fixed-order PP failed to solve.')
    if include_order and (out/'order_sensitivity.csv').exists():
        with (out/'order_sensitivity.csv').open(encoding='utf-8',newline='') as f: trials=list(csv.DictReader(f))
        by_instance=defaultdict(list)
        for r in trials: by_instance[r['instance_id']].append(r)
        changing_success=sum(len({successful(r) for r in group})>1 for group in by_instance.values())
        changing_cost=sum(len({float(r['sum_of_costs']) for r in group if successful(r)})>1 for group in by_instance.values())
        lines+=['',f'The separate order study contains {len(trials)} trials across {len(by_instance)} fixed instances.',
                f'- Instances whose success changed with the planning order: {changing_success}.',
                f'- Instances with different costs across successful orders: {changing_cost}.',
                '- Repeated orders may occur; trials are not necessarily distinct permutations.']
    else:
        lines+=['','The separate order study has not yet been included for this main run. Run order_study.py, or use run_project.py to execute the full pipeline. Old order-study files, if present, are not used at this stage.']
    lines+=['','## Interpretation and validation','',
            '- All numbers above are computed from saved experimental records; no expected results are substituted for measurements.',
            '- Solver runtime excludes validation, plotting, and GIF saving. The time limit is cooperative, and an over-budget result does not count as a success.',
            '- Skipped Joint A* rows have a runtime of zero as a placeholder; they are not zero-second solutions.',
            '- Reordering stops when it finds a feasible plan. It does not improve the cost further when the initial order already succeeds.',
            '- plans.jsonl saves the paths returned by each solver for independent review; join records to instances.json using instance_id. Skipped and unsuccessful runs may have null paths.',
            '- The included validator is a reference implementation. A teammate should still perform independent cross-validation.',
            '- CSVs, plots, instance data, execution metadata, and the AI assistance disclosure accompany the code.']
    content='\n'.join(lines)+'\n'
    (out/'EXPERIMENT_REPORT.md').write_text(content,encoding='utf-8')
    # Only the default output refreshes the project-root report.
    if out.resolve()==(Path(__file__).resolve().parent/'outputs').resolve():
        (Path(__file__).resolve().parent/'EXPERIMENT_REPORT.md').write_text(content,encoding='utf-8')
    return content

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',default='outputs')
    parser.add_argument('--include-order',action='store_true')
    args=parser.parse_args()
    write_report(args.output,args.include_order)
