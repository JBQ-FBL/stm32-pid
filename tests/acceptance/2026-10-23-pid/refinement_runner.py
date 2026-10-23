import ast,sys,json,time,csv
from pathlib import Path
sys.dont_write_bytecode=True;sys.path.insert(0,str(Path('../tools').resolve()))
import pid_autotune as t
node=next(n for n in ast.parse(Path('/tmp/run_pid_checked.py').read_text()).body if isinstance(n,ast.ClassDef) and n.name=='CheckedPlant')
exec(compile(ast.Module(body=[node],type_ignores=[]),'checked_plant','exec'))
folder=Path('build/plan1/tuning/refinement');folder.mkdir(parents=True,exist_ok=True)
plant=CheckedPlant('/dev/ttyUSB0',115200);plant.ser.exclusive=True
cases=[t.Candidate(12,9,0,3.95),t.Candidate(10,9,0,3.95),t.Candidate(10,9,0,3.75),t.Candidate(10,7,0,3.75),t.Candidate(12,9,0,3.75)]
results=[];best=None
try:
 plant.ensure_stream()
 for i,cand in enumerate(cases,1):
  print('CASE',i,cand.as_dict(),flush=True);plant.apply_candidate(cand);metrics=[]
  for rpm in (80,200):
   frames=plant.run_step(rpm,3.,1.5)
   assert len(frames)>500 and all(f[1]==rpm for f in frames[-50:]),'target mismatch'
   with (folder/f'case_{i}_{rpm}.csv').open('w',newline='') as f:
    w=csv.writer(f,lineterminator='\n');w.writerow(['time_s','target_rpm','actual_rpm','out_permille']);w.writerows(frames)
   m=t.sp.metrics(frames,rpm,3);m['rpm']=rpm;metrics.append(m)
   status=plant.health()
   for k,v in cand.as_dict().items():assert abs(status[k]-v)<.051,'gain changed'
  low,high=metrics
  qualified=low['rise_ms'] is not None and low['rise_ms']<=50 and all(m['overshoot_pct']<=10 and abs(m['error_pct'])<2 for m in metrics)
  score=low['rise_ms']+.35*high['rise_ms']+2*low['overshoot_pct']+high['overshoot_pct']+5*sum(abs(m['error_pct']) for m in metrics)
  row={'candidate':cand.as_dict(),'metrics':metrics,'qualified':qualified,'score':score};results.append(row)
  print(json.dumps(row),flush=True)
  (folder/'results.json').write_text(json.dumps(results,indent=2))
 good=[r for r in results if r['qualified']]
 if not good:raise RuntimeError('no candidate meets response/overshoot/error criteria')
 best=min(good,key=lambda r:r['score']);(folder/'best.json').write_text(json.dumps(best,indent=2))
 print('BEST',json.dumps(best),flush=True)
finally:
 plant.apply_candidate(t.Candidate(**best['candidate']) if best else t.Candidate(14,5,0,4.15))
 plant.stop();plant._read(.5);print('STOPPED',plant.status(),flush=True);plant.close()
