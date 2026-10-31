#!/usr/bin/env python3
"""Decodes g_buck.toc.ev (the event log of the last TOC frame, Src/buck_toc.c, README "Enabling" 8).

Usage: decode_events.py [--cret C_RET] EV0 EV1 ...   (the ev[k] values as Live Expressions shows them,
decimal or 0x-hex, the first g_buck.toc.ev_n of them; --cret: g_buck.toc.c_ret, the log's start in the
frame's counter)

Each entry: code << 23 | (CPU cycles since the return words' guard, 12 bits) << 11 | Timer A counter / 32.
One CPU cycle is 32 HRTIM ticks; a normal period is 8960 ticks (280 cycles)."""
import sys

STATE = [(0x01, 'ADC1 result'), (0x02, 'ADC2 result'), (0x20, 'ADC1 OVR'), (0x40, 'ADC2 OVR')]
SEEN = [(0x100, 'WAIT'), (0x08, 'CMP4 (1st A0 trigger)'), (0x04, 'CMP3 (2nd A0 trigger)'),
        (0x10, 'TA roll-over'), (0x80, 'master roll-over')]


def main(argv):
    cret = None
    if len(argv) >= 2 and argv[0] == '--cret':
        cret = int(argv[1], 0)
        argv = argv[2:]
    vals = [int(a, 0) for a in argv]
    if not vals:
        print(__doc__)
        return 1
    print(f'{"k":>2} {"cycles":>6} {"+cyc":>5} {"ticks":>7} {"TA cnt":>6}  seen since the last entry | state')
    base, prev = 0, None
    for k, v in enumerate(vals):
        code, cyc12, cnt = v >> 23, (v >> 11) & 0xFFF, (v & 0x7FF) * 32
        if prev is not None and base + cyc12 < prev:
            base += 0x1000                      # the 12-bit cycle field wrapped
        cyc = base + cyc12
        d = '' if prev is None else f'{cyc - prev:+5d}'
        seen = ', '.join(n for b, n in SEEN if code & b)
        state = ', '.join(n for b, n in STATE if code & b) or '-'
        print(f'{k:>2} {cyc:>6} {d:>5} {cyc * 32:>7} {cnt:>6}  {seen or "-"} | {state}')
        prev = cyc
    if cret is not None:
        print(f'(log start: the frame counter c_ret = {cret}; ticks above count from there)')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
