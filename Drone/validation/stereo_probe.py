"""Check stereo depth against geometry on frames saved by a validation run.

    python Drone/validation/stereo_probe.py Drone/docs/evidence/stereo-probe

Reads left-*/right-*/frame-* PNGs plus track.csv, so it needs no Webots, ROS or
Pillow. Reports the measured disparity and the depth error against the known
obstacle face (arena obstacle centred at x=8, half-depth .5).
"""
import csv
import math
import struct
import sys
import zlib
from pathlib import Path

D = Path(sys.argv[1] if len(sys.argv) > 1 else 'Drone/docs/evidence/stereo-probe')
FOV, BASELINE, MOUNT, NEAR_FACE = 1.3962634, .075, .32, 7.5


def decode(path):
    """Minimal PNG reader for the 8-bit RGB(A) files Webots writes."""
    raw = path.read_bytes()
    pos, idat, w, h, channels = 8, b'', 0, 0, 4
    while pos < len(raw):
        length, kind = struct.unpack('>I4s', raw[pos:pos + 8])
        body = raw[pos + 8:pos + 8 + length]
        if kind == b'IHDR':
            w, h, depth, colour = struct.unpack('>IIBB', body[:10])
            if depth != 8 or colour not in (2, 6):
                raise SystemExit('%s: unsupported PNG (%d bit, colour %d)' % (path, depth, colour))
            channels = 3 if colour == 2 else 4
        elif kind == b'IDAT':
            idat += body
        pos += 12 + length
    data, stride, prev = zlib.decompress(idat), w * channels, bytearray(w * channels)
    out, i = bytearray(), 0
    for _ in range(h):
        filt, line = data[i], bytearray(data[i + 1:i + 1 + stride])
        i += 1 + stride
        for x in range(stride):
            a = line[x - channels] if x >= channels else 0
            b, c = prev[x], prev[x - channels] if x >= channels else 0
            if filt == 1:
                line[x] = (line[x] + a) & 255
            elif filt == 2:
                line[x] = (line[x] + b) & 255
            elif filt == 3:
                line[x] = (line[x] + (a + b) // 2) & 255
            elif filt == 4:
                p = a + b - c
                pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
                line[x] = (line[x] + (a if pa <= pb and pa <= pc else b if pb <= pc else c)) & 255
        out += line
        prev = line
    return w, h, channels, bytes(out)


def gray(buf, w, ch, x, y):
    i = (y * w + x) * ch
    return (buf[i] + 2 * buf[i + 1] + buf[i + 2]) // 4


def main():
    track = {round(float(r['t']), 3): float(r['x']) for r in csv.DictReader(open(D / 'track.csv'))}
    frames = sorted(int(p.stem.split('-')[1]) for p in D.glob('left-*.png'))
    if not frames:
        raise SystemExit('no left-*.png in %s' % D)
    print('frame   t     true_disp  measured  depth_err_m')
    for k in frames:
        rw, rh, rch, rgb = decode(D / ('frame-%05d.png' % k))
        w, h, ch, left = decode(D / ('left-%05d.png' % k))
        _, _, _, right = decode(D / ('right-%05d.png' % k))
        row, f = h // 2, w / (2 * math.tan(FOV / 2))
        t = k * 4 / 1000
        x = min(track.items(), key=lambda kv: abs(kv[0] - t))[1]
        true_depth = NEAR_FACE - (x + MOUNT)

        def red(u):  # RGB camera samples the same columns as the stereo pair
            i = 4 * ((rh // 2) * rw + u * rw // w)
            r, g, b = rgb[i:i + 3]
            return r > 65 and r > 2 * g and r > 2 * b
        group = [u for u in range(w) if red(u)]
        if len(group) < 6:
            print('%5d %5.2f   no obstacle silhouette' % (k, t))
            continue
        costs = {d: sum(abs(gray(left, w, ch, u, row + j) - gray(right, w, ch, u - d, row + j))
                        for u in group for j in (-3, 0, 3))
                 for d in range(1, 31)}
        best = min(costs, key=costs.get)
        before, after = costs.get(best - 1), costs.get(best + 1)
        sub = best
        if before is not None and after is not None and before - 2 * costs[best] + after > 0:
            sub = best + .5 * (before - after) / (before - 2 * costs[best] + after)
        measured = f * BASELINE / sub
        print('%5d %5.2f   %8.2f  %8.2f  %+11.2f'
              % (k, t, f * BASELINE / true_depth, sub, measured - true_depth))


if __name__ == '__main__':
    main()
