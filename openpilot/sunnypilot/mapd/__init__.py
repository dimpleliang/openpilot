import os
from openpilot.common.basedir import BASEDIR

MAPD_BIN_DIR = os.path.join(BASEDIR, 'openpilot', 'third_party/mapd_pfeiferj')
MAPD_PATH = os.path.join(MAPD_BIN_DIR, 'mapd')

# mapd still runs (OSM downloads) but has no position source since ubloxd owns LastGPSPosition;
# map-based features stay off until mapd is rebuilt
# dimpleliang/ars510 11/02: on this fork selfdrive/hwyalert/hwyalertd.py writes the memory param LastGPSPosition from the
# 1 Hz GPS fix while onroad, so mapd resolves the road again (verified: RoadName / MapCurvatures fill within a second).
MAPD_AVAILABLE = True
