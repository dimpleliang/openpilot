#!/usr/bin/env python3
"""Candidate cut-ins from the radar object list: |y - model path| < Y_MAX, 25-90 m, closing > 2 m/s, car/truck, age >= 8,
not the current lead, for HOLD s. Confirmed = openpilot lead at about that object's range within 10 s."""
import sys, glob, zstandard, capnp, numpy as np
sys.path.insert(0, "/home/claude/sp/opendbc_repo"); capnp.remove_import_hook()
SP="/home/claude/sp"
log=capnp.load(f"{SP}/openpilot/cereal/log.capnp",imports=[f"{SP}/openpilot/cereal",f"{SP}/opendbc_repo/opendbc/car",f"{SP}/openpilot",SP])
import opendbc.car.toyota.ars510.interface as I
REC=[]; _o=I.decode_native_slot; I.decode_native_slot=lambda i,s:(REC.append(_o(i,s)),REC[-1])[1]
Y_MAX=float(sys.argv[1]); HOLD=float(sys.argv[2]); files=sys.argv[3:]
tot=dict(n=0,conf=0); rows=[]
for f in files:
  d=zstandard.ZstdDecompressor().stream_reader(open(f,"rb")).read()
  ri=I.Ars510NativeRadarInterface(I.NativeInterfaceConfig(radar_bus=1)); v=0; path=None; lead=None; leads=[]; cand={}; flagged=[]; life={}
  for m in log.Event.read_multiple_bytes(d,traversal_limit_in_words=2**61):
    w=m.which(); t=m.logMonoTime/1e9
    if w=="carState": v=m.carState.vEgo
    elif w=="modelV2": path=(list(m.modelV2.position.x), list(m.modelV2.position.y))
    elif w=="radarState":
      l=m.radarState.leadOne; lead=(l.dRel,l.vRel) if l.present else None
      leads.append((t,lead))
    elif w=="can" and path and v>60/3.6:
      for c in m.can:
        if c.src not in (0,1) or c.address not in (0x80,0xB4,0x24): continue
        REC.clear(); ri.update_frame(t,c.src,c.address,bytes(c.dat))
        for o in REC:
          prev=life.get(o.slot)
          if prev is None or o.age<prev[0]: life[o.slot]=[o.age,(o.slot,t)]
          else: life[o.slot][0]=o.age
          key=life[o.slot][1]
          if not (o.geometry_valid and o.lateral_valid and 25<o.d_rel<90 and o.age>=8 and o.object_class in (2,3)): cand.pop(key,None); continue
          vr=o.v_long_ground-v
          py=float(np.interp(o.d_rel,path[0],path[1])) if o.d_rel<=path[0][-1] else 0.
          off=o.y_rel-py
          is_lead = lead is not None and abs(o.d_rel-lead[0])<6
          ok = abs(off)<Y_MAX and abs(off)>1.0 and vr<-2.0 and not is_lead
          if ok:
            e=cand.setdefault(key,[t,o.d_rel,off,vr,False])
            if not e[4] and t-e[0]>=HOLD:
              e[4]=True; flagged.append((t,o.d_rel,off,vr,v,key))
          else: cand.pop(key,None)
  for t,dd,off,vr,v0,key in flagged:
    conf=None
    for tl,l in leads:
      if t<=tl<=t+10 and l is not None:
        dp=dd+vr*(tl-t)
        if abs(l[0]-dp)<0.2*max(dp,10)+5: conf=tl-t; break
    tot["n"]+=1; tot["conf"]+=conf is not None
    rows.append(f"{f.split('/')[-2][-6:]}/{f.split('/')[-1]} v={v0*3.6:.0f} d={dd:.0f} off={off:+.1f} vRel={vr:.1f} -> {'lead after %.1fs'%conf if conf is not None else 'never'}")
print("\n".join(rows)); print("flagged",tot)
