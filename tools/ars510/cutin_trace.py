#!/usr/bin/env python3
"""Radar object-list view (lane crossing code, lateral speed) around the min-aTarget of one or two segments."""
import sys, zstandard, capnp
sys.path.insert(0, "/home/claude/sp/opendbc_repo"); capnp.remove_import_hook()
SP="/home/claude/sp"
log=capnp.load(f"{SP}/openpilot/cereal/log.capnp",imports=[f"{SP}/openpilot/cereal",f"{SP}/opendbc_repo/opendbc/car",f"{SP}/openpilot",SP])
import opendbc.car.toyota.ars510.interface as I
REC=[]; _o=I.decode_native_slot; I.decode_native_slot=lambda i,s:(REC.append(_o(i,s)),REC[-1])[1]
files=sys.argv[1].split(","); before=float(sys.argv[2]); after=float(sys.argv[3]); step=float(sys.argv[4]) if len(sys.argv)>4 else 0.5
msgs=[]
for f in files:
  d=zstandard.ZstdDecompressor().stream_reader(open(f,"rb")).read(); msgs+=list(log.Event.read_multiple_bytes(d,traversal_limit_in_words=2**61))
plans=[(m.logMonoTime/1e9,m.longitudinalPlan.aTarget) for m in msgs if m.which()=="longitudinalPlan"]
t0=min(plans,key=lambda x:x[1])[0]
ri=I.Ars510NativeRadarInterface(I.NativeInterfaceConfig(radar_bus=1)); v=0; last=-9; lead=(0,0,0)
for m in msgs:
  w=m.which(); t=m.logMonoTime/1e9
  if w=="carState": v=m.carState.vEgo
  elif w=="radarState":
    l=m.radarState.leadOne; lead=(int(l.present), l.dRel, l.vRel, int(l.radar))
  elif w=="can":
    for c in m.can:
      if c.src not in (0,1) or c.address not in (0x80,0xB4,0x24): continue
      REC.clear(); ri.update_frame(t,c.src,c.address,bytes(c.dat))
      if REC and t0-before<=t<=t0+after and t-last>=step-0.01:
        last=t
        objs=[o for o in REC if o.geometry_valid and o.lateral_valid and 5<o.d_rel<110 and abs(o.y_rel)<6 and (o.v_long_ground-v)<-0.5]
        s=" | ".join(f"s{o.slot} d{o.d_rel:5.1f} y{o.y_rel:+4.1f} vr{o.v_long_ground-v:5.1f} vlat{o.v_lat_ground:+4.1f} lc{o.lane_crossing_code} ln{o.raw_weight_state128} c{o.object_class}" for o in sorted(objs,key=lambda o:o.d_rel))
        print(f"{t-t0:+6.1f} v={v*3.6:5.1f} lead={'%d %.0fm %.1f %s'%(lead[0],lead[1],lead[2],'R' if lead[3] else 'C')} || {s}")
