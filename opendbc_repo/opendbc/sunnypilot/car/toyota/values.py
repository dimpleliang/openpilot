"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import IntFlag


class ToyotaFlagsSP(IntFlag):
  SMART_DSU = 1
  RADAR_CAN_FILTER = 2
  ZSS = 4
  STOCK_LONGITUDINAL = 8
  STOP_AND_GO_HACK = 16
  # dimpleliang/ars510: renumbered off 32 (upstream #440 TSS2_SMOOTH) and 64 (#502 SP_NEED_DEBUG_BSM)
  SP_ENHANCED_BSM = 128
  AUTO_LOCK = 256    # dimpleliang/ars510: sunnypilot/opendbc#354 (renumbered: its 8/16 are taken upstream)
  AUTO_UNLOCK = 512


class ToyotaSafetyFlagsSP:
  DEFAULT = 0
  UNSUPPORTED_DSU = 1
  GAS_INTERCEPTOR = 2
