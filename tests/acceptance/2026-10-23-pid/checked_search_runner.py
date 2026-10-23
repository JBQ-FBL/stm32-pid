import sys,time,json,math
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path('/home/niko/stm32/PID/tools')))
import pid_autotune as t

BASE=t.Candidate(kp=14.,ki=5.,kd=0.,kv=4.15)
ROOT=Path('build/plan1/tuning/checked')/time.strftime('%Y%m%d-%H%M%S')
ROOT.mkdir(parents=True)
class CheckedPlant(t.SerialPlant):
 def apply_candidate(self,cand):
  ok,why=t.validate_candidate(cand)
  if not ok: raise t.TrialAbort(why)
  self.stop();self._read(.6)
  # S0/X latch skips PID calculations, and avoids streaming a burst of gain commands.
  for name,cmd in [('kp','KP'),('ki','KI'),('kd','KD'),('kv','KV')]:
   expected=getattr(cand,name)
   for attempt in range(3):
    self._send(f'{cmd}{expected:.3f}');self._read(.12)
    status=self.health()
    tolerance=.011 if name=='kd' else .051 # status rounds kp/ki/kv to one decimal
    if abs(status[name]-expected)<=tolerance:break
    print('RETRY',cmd,expected,status,flush=True)
   else: raise t.TrialAbort('parameter readback failed: '+name)
  self.applied=cand

plant=CheckedPlant('/dev/ttyUSB0',115200);plant.ser.exclusive=True
rec=t.Recorder(str(ROOT),{'mode':'live_checked','baseline':BASE.as_dict(),'targets_rpm':[80,200],
 'bounds':t.BOUNDS,'policy':'stop before parameter change; individual readback; 8 candidates per parameter; Kd=0',
 'step':{'kv':.2,'kp':2.,'ki':2.},'max_trials':25})
tuner=t.Tuner(plant,rec,[80,200],25,420,smooth_n=3,verbose=True,warmup=False)
best=None;baseline=None
try:
 plant.ensure_stream();plant.stop();plant._read(.5)
 print('RECORD',ROOT,flush=True)
 base,raw=tuner.trial(BASE,'checked baseline');tuner._record(base,raw,'baseline');best=base;baseline=base.metrics
 for name,initial in [('kv',.2),('kp',2.),('ki',2.)]:
  step=initial;used=0
  while step>=t.MIN_STEP[name] and used<8:
   improved=False
   for sign in (1,-1):
    values=best.cand.as_dict();values[name]+=sign*step;cand=t.Candidate(**values)
    ok,why=t.validate_candidate(cand,best.cand)
    if not ok:continue
    res,raw=tuner.trial(cand,f'checked {name} {sign:+}');used+=1
    if res.score+t.MIN_IMPROVE_MS<best.score:
     tuner._record(res,raw,'accept',name);best=res;improved=True;break
    tuner._record(res,raw,'reject',name)
    if used>=8:break
   if not improved:step/=2
  print('PHASE_DONE',name,'best',best.cand.as_dict(),'score',best.score,flush=True)
 rec.finish(best,baseline,best.metrics)
 (ROOT/'best.json').write_text(json.dumps({'candidate':best.cand.as_dict(),'score':best.score,'metrics':best.metrics},indent=2))
 print('BEST',json.dumps(best.cand.as_dict()),flush=True)
except BaseException as e:
 (ROOT/'error.txt').write_text(repr(e));print('ABORT',repr(e),flush=True);raise
finally:
 try:
  plant.apply_candidate(best.cand if best else BASE)
 finally:
  plant.stop();plant._read(.5);status=plant.status();plant.close()
  (ROOT/'final_status.json').write_text(json.dumps(status,indent=2))
  print('STOPPED',status,flush=True)
