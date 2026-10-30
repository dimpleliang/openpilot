import sys, types, zstandard, capnp, numpy as np
sys.path.insert(0, "/home/claude/sp/opendbc_repo"); capnp.remove_import_hook()
SP="/home/claude/sp"
log=capnp.load(f"{SP}/openpilot/cereal/log.capnp",imports=[f"{SP}/openpilot/cereal",f"{SP}/opendbc_repo/opendbc/car",f"{SP}/openpilot",SP])
from opendbc.car import structs
import opendbc.car.toyota.ars510_radar_interface as A
src=open(f"{SP}/openpilot/selfdrive/controls/lib/longitudinal_planner.py").read()
ns={"np":np,"types":types,"STOP_DISTANCE":6.0}; exec(src[src.index("FAR_LEAD_K ="):src.index("def get_cruise_accel")],ns)
files=sys.argv[1].split(","); before=float(sys.argv[2]); after=float(sys.argv[3])
msgs=[]
for f in files:
  d=zstandard.ZstdDecompressor().stream_reader(open(f,"rb")).read(); msgs+=list(log.Event.read_multiple_bytes(d,traversal_limit_in_words=2**61))
plans=[(m.logMonoTime/1e9,m.longitudinalPlan.aTarget) for m in msgs if m.which()=="longitudinalPlan"]
t0=min(plans,key=lambda x:x[1])[0]
CP=structs.CarParams(); CP.carFingerprint="TOYOTA_RAV4_TSS2_2023"
ri=A.Ars510RadarInterface(CP, structs.CarParamsSP()); v=0; path=None; lead=None; cut=None; cnt=0; cap=None; a_prev=0.; last=-9; on=False
for m in msgs:
  w=m.which(); t=m.logMonoTime/1e9
  if w=="carState": v=m.carState.vEgo
  elif w=="modelV2": path=(list(m.modelV2.position.x), list(m.modelV2.position.y))
  elif w=="radarState": lead=m.radarState.leadOne
  elif w=="can":
    r=ri.update([(m.logMonoTime,[(c.address,bytes(c.dat),c.src) for c in m.can])])
    if r is not None:
      cut=[p for p in r.points if p.trackId==A.CUTIN_TRACK_ID]
  elif w=="longitudinalPlan" and lead is not None and path:
    a=m.longitudinalPlan.aTarget; ok=False; ci=None
    if cut and v>ns["CUTIN_MIN_V"]:
      p=cut[0]; py=float(np.interp(p.dRel,path[0],path[1])) if p.dRel<=path[0][-1] else 0.; off=abs(p.yRel-py)
      if ns["CUTIN_PATH_Y_M"][0]<off<ns["CUTIN_PATH_Y_M"][1] and (not lead.present or p.dRel<lead.dRel-6):
        ok=True; cnt+=1; ci=types.SimpleNamespace(dRel=p.dRel,vRel=p.vRel,vLead=v+p.vRel,off=p.yRel-py)
    if not ok: cnt=max(cnt-2,0)
    out=a; st=""
    if ci is not None and cnt>=ns["CUTIN_HOLD"]:
      res=ns["cutin_accel_ceil"](ci,1.45)
      if res:
        ceil=res[0]; cap=max(a_prev,ceil) if cap is None else float(np.clip(ceil,cap-0.5*0.05,cap+0.3*0.05))
        if a>cap: out=cap; st=f"CUT-IN coast d={ci.dRel:.0f} off={ci.off:+.1f} vr={ci.vRel:.1f}"
    else: cap=None
    a_prev=out
    if t0-before<=t<=t0+after and t-last>=0.5:
      last=t
      print(f"{t-t0:+6.1f} v={v*3.6:5.1f} lead={'%d %.0fm %.1f'%(lead.present,lead.dRel,lead.vRel)} cand={'%.0fm y%+.1f %.1f'%(cut[0].dRel,cut[0].yRel,cut[0].vRel) if cut else '-':18} cnt={cnt:2d} a={a:5.2f} -> {out:5.2f} {st}")
