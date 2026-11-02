#!/usr/bin/env python3
"""Stream a file to stdout without leaving it in the page cache (and optionally rate-limited).

  python3 cat_nocache.py FILE [OFFSET] [RATE_BYTES_PER_S]

Used by the Mac's ts_pull.sh to pull rlogs from the comma while driving: a plain rsync / cat fills the page cache with
every file it reads and the kernel then reclaims in bursts (11/02: kswapd0 15-30 % CPU, modeld dropped frames, a
"locationd Temporary Error" alert). Each chunk is dropped from the cache right after it is written out
(posix_fadvise DONTNEED), so the cache never grows because of the transfer. Only the standard library, nice 19.
"""
import os
import sys
import time

CHUNK = 1 << 20  # 1 MiB


def main():
  path = sys.argv[1]
  offset = int(sys.argv[2]) if len(sys.argv) > 2 else 0
  rate = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0  # bytes/s, 0 = unlimited
  try:
    os.nice(19)
  except OSError:
    pass
  fd = os.open(path, os.O_RDONLY)
  out = sys.stdout.buffer
  try:
    size = os.fstat(fd).st_size
    pos = min(offset, size)
    t0 = time.monotonic()
    sent = 0
    while pos < size:
      n = min(CHUNK, size - pos)
      buf = os.pread(fd, n, pos)
      if not buf:
        break
      out.write(buf)
      out.flush()
      try:
        os.posix_fadvise(fd, pos, len(buf), os.POSIX_FADV_DONTNEED)
      except (OSError, AttributeError):
        pass
      pos += len(buf)
      sent += len(buf)
      if rate > 0:
        ahead = sent / rate - (time.monotonic() - t0)
        if ahead > 0:
          time.sleep(ahead)
    # the tail we did not read this time (resume) is not ours to drop; the whole file once we are done
    try:
      os.posix_fadvise(fd, 0, 0, os.POSIX_FADV_DONTNEED)
    except (OSError, AttributeError):
      pass
  finally:
    os.close(fd)


if __name__ == "__main__":
  main()
