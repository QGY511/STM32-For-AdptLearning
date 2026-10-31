#!/usr/bin/env python3
"""Post-link checks of the 600 kHz control loop (make check).

Stage 2 (buck_obs_apt, 29 Oct 2026) added rule 5 for the TOC code; stage 4 (30 Oct 2026, the
feed-forward TOC) replaced it: see its text below.
Stage 3 (30 Oct 2026, the Padé estimator and its table removed): rule 4 requires the unused
g_buck.tune.alpha_i to be read nowhere; the other rules are unchanged (the CMP1 deadline guard,
no fused/chained multiply-add in CCM, the CCM placement of the loop and of the TOC code).

The loop's timing and its deadline check depend on how GCC compiles it, so a
harmless-looking source edit can break them without any test noticing. This
script checks the linked ELF:

  1. run_loop (and buck_run) lie entirely in CCM SRAM (0x10000000..0x10008000)
     and GCC has not split a cold part out of them.
  2. .ccmtext contains no fused or chained multiply-add (VFMA/VFMS/VFNMA/VFNMS,
     VMLA/VMLS/VNMLA/VNMLS): the algorithm is written for separate VMUL + VADD
     (bit-identical to the host tests, and 1 clock cheaper than VMLA).
  3. In every copy of the pass, the deadline check reads the HRTIM Timer A
     counter immediately before the CMP1 write, in one of two forms:
         ldr  rT, [rB, #144]      TIMA CNTR               ldr  rT, [rB, #144]
         sub  rD, rT, #2048       BUCK_CHECK_EARLIEST     sub  rD, rT, #2048
         cmp  rD, #6400           (window)                cmp  rD, #6400
         bcs  <trip>                                      it   cc
         str  rY, [rB, #156]      TIMA CMP1R              strcc rY, [rB, #156]
                                                          bcs  <trip>
     with the data flowing ldr -> sub -> cmp (rT and rD not rB), and <trip>
     reaching the fault-5 trip: within 40 instructions from the branch target
     (following unconditional branches inside run_loop), a mov of #5 into r0
     (BUCK_FAULT_DEADLINE, trip()'s argument) before any return. Between the
     strcc and its bcs only unconditional branches (at most two, inside
     run_loop) may lie: GCC can put a literal pool there (25 Oct, the first
     pass after arming). Every CMP1 write in run_loop must be guarded this
     way, except the arming write (since 25 Oct the arming is in run_loop):
     the one store to [rB, #156] that is followed within three instructions,
     with no store between, by the outputs-enable write [rB, #916] (HRTIM
     OENR) through the same base, and that is not on the path of an ordinary
     pass (see 4.). More than one such write is an error. The gates are off
     until that write, so the deadline check does not apply to it; its
     arming window (counter in 897..4095, ADC1 not converted again) is not
     checked here (review only). A store to #156
     is taken as a CMP1 write unless its base register holds g_buck's address
     throughout run_loop (a g_buck field can sit at the same offset): every
     instruction in run_loop that writes that register, in any operand
     position (register lists, second destinations), must be a literal load
     of &g_buck, apart from the callee-saved restores on the exit paths
     (an ldm/pop that returns, or that is followed by a branch out) and
     from writes whose value is never used as an address: every path from
     them (following branches inside run_loop) ends, at a return, a branch
     out or a call that does not return, before any instruction with that
     register inside its brackets or as its ldm/stm base (26 Oct: GCC used
     g_buck's register as a scratch on a block that only returns; its
     stores to busy_max, at offset 156, were taken for CMP1 writes, and 4.
     found no register holding &g_buck). Not analysed: a base writeback
     (other than an ldm's) or a call that changes that register is not
     taken as a write of it.
  4. The tunables reload (25 Oct): every field of g_buck.tune is read from
     g_buck at the loop entry and in the passes (the tunables pass), at least
     once in each (c.t = b->tune at both, and what is formed from it there:
     Ki*T, the score coefficients a, b, c, the target; and the load-current
     observer's gains obs_beta and obs_gamma, read and checked against the
     stable region at both, buck_obs_k). GCC keeps a field's load only where
     its value is used, so a tunables pass that no longer copies a, b, c into
     the score coefficients or no longer takes up (checks) the observer's
     gains, or an entry that does not take them from the tunables (e.g. the
     compiled default gains without the check), leaves that field without a
     read there. The entry is what run_loop executes from its first
     instruction up to its first ADC12_CSR read, following its branches;
     everything else in run_loop belongs to the passes. Since 25 Oct (the
     arming moved into run_loop) that read is the arming wait's, so a read
     at the entry means a read before the arming wait; GCC forms that read
     through a register holding ADC12_CSR's address (ldr rX, [ip]), so the
     entry's register values are tracked (mov/movw/movt/mvn immediates,
     literal loads, add/sub of an immediate; at a join only what both paths
     agree on) and a load of address 0x50000300 ends the entry, as does any
     ldr [rX, #768] (the form until 25 Oct). (Until 25 Oct the rule counted two reads anywhere, which the
     observer build passes without the entry's check: GCC repeats the gains'
     check in each copy's tunables pass.) The gains are checked where they
     are reloaded, not every pass (user, 25 Oct), so obs_beta and obs_gamma
     must not be read on the path of an ordinary pass: what run_loop executes
     from a pass head (the ADC12_CSR read) without taking the branch of the
     (loops & 31) == 0 test into the telemetry or tunables pass (tst #31,
     ands #31 or lsls #27, then beq/bne with no flag-setting instruction
     between; or and(s) #31, then cbz/cbnz on its result), up to the
     returns, the branches out of run_loop and the calls that do not return.
     The field offsets come from the ELF's debug info (buck_t.tune + the
     member); obs_beta and obs_gamma must be among them. Stage 3 (30 Oct 2026): alpha_i (the
     removed Padé estimator's alpha_I, kept in the structure for the layout) must instead not be read
     at all (at the entry or in the passes). Reads counted:
     ldr/vldr/ldrd [rG] and [rG, #imm] with rG holding &g_buck throughout
     run_loop (the rule of 3.). Not covered (review only): the rest of how
     run_pass() wires the results, e.g. the order inside the reload
     (coefficients formed from the previous tunables), whether the check's
     result decides which gains run, which values it feeds the observer
     (v_out and the raw measured average), where A2 is written, what
     out.i_load publishes, or which value drives D4.

Usage: check_loop.py OBJDUMP NM ELF
The guard rule is self-tested on every run against known-bad sequences (the
trip removed, the condition inverted, the counter or the difference not the
one compared, the store not conditional; 25 Oct: a literal pool between the
strcc and its bcs accepted, an instruction there, three branches or a
branch out rejected); the arming write's identification (25 Oct: another
base, a store between, OENR first, too far: rejected; 26 Oct: a branch
between rejected, one pair on the path of an ordinary pass not taken, two
pairs off it an error); the register holding &g_buck (26 Oct: a scratch
write whose paths return (also by bx or by an exit restore of the
register), branch out, or call buck_fatal or a call with a literal pool
after it, accepted, as is a write that loads through the register;
rejected: one used as an address (in brackets, also as an index, or as
an ldm or stm base), also on a conditional branch's target or
fall-through or after a conditional return (pop, ldm), one followed by a
branch back into the loop, into the middle of an instruction, by tbb, a
literal pool or the end, a load from memory, a literal of another
address, or no literal load at all); the read count, the entry (25 Oct:
ended by a register holding ADC12_CSR's address, not by another address,
and not at a join where only one path holds it; 26 Oct: also with that
path arriving last, and not past a call that does not return), the
entry's register values (26 Oct: a literal load, movw/movt, mvn, sub, a
copy of a known and of an unknown register; a load and a call make them
unknown), the path of an ordinary pass and the per-field verdict of 4.
against short known sequences.
"""
import re
import subprocess
import sys

CCM_LO, CCM_HI = 0x10000000, 0x10008000
FUSED = re.compile(r'^(vfma|vfms|vfnma|vfnms|vmla|vmls|vnmla|vnmls)\b')


REGN = r'(r\d+|ip|lr|fp|sl|sb)'
COND_BCS = re.compile(r'b(cs|hs)(\.n|\.w)?$')


def reaches_trip(body, a, lo, hi):
    """True when the code from address a reaches trip(b, 5): a mov of #5 into
    r0 within 40 instructions, following unconditional branches inside
    [lo, hi), before any return or branch out."""
    idx = {x[0]: j for j, x in enumerate(body)}
    seen = set()
    j, steps = idx.get(a), 0
    while j is not None and j < len(body) and steps < 40 and j not in seen:
        seen.add(j)
        _, op, args = body[j]
        if re.match(r'movs?(\.w)?$', op) and re.match(r'r0, #5$', args):
            return True
        if op.startswith(('pop', 'ldm')) and 'pc' in args or op.startswith('bx'):
            return False
        mb = re.match(r'b(\.n|\.w)?$', op)
        if mb:
            t = int(args.split()[0], 16)
            if not (lo <= t < hi):
                return False
            j = idx.get(t)
        else:
            j += 1
        steps += 1
    return False


def next_after(body, i, lo, hi):
    """The instruction executed after body[i], through up to two unconditional
    branches inside [lo, hi) (GCC can put a literal pool between the
    conditional store and its bcs: strcc; b <x>; x: bcs), or None."""
    idx = {x[0]: j for j, x in enumerate(body)}
    j = i + 1
    for _ in range(3):
        if j >= len(body):
            return None
        a, op, args = body[j]
        if not re.match(r'b(\.n|\.w)?$', op):
            return body[j]
        t = int(args.split()[0], 16)
        if not (lo <= t < hi) or t not in idx:
            return None
        j = idx[t]
    return None


def guard_fault(body, i, base, lo, hi):
    """None when the store body[i] (to [base, #156]) is guarded by the deadline
    check, else the reason. See 3. in the module docstring."""
    a, op, args = body[i]
    w = body[i - 4:i]
    if len(w) != 4:
        return 'no check before it'
    m0 = re.match(REGN + r', \[' + base + r', #144\]$', w[0][2])
    if not (w[0][1].startswith('ldr') and m0):
        return 'no counter read ([base, #144]) 4 instructions before'
    rt = m0.group(1)
    m1 = re.match(REGN + r', ' + rt + r', #2048$', w[1][2])
    if not (w[1][1].startswith('sub') and m1):
        return 'the counter is not reduced by 2048'
    rd = m1.group(1)
    if base in (rt, rd):
        return 'the check overwrites the HRTIM base'
    if not (w[2][1].startswith('cmp') and re.match(rd + r', #6400$', w[2][2])):
        return 'the reduced counter is not compared with 6400'
    if COND_BCS.match(w[3][1]):                         # branch form: bcs <trip>; str
        br = w[3]
    elif w[3][1] == 'it' and w[3][2] in ('cc', 'lo') and op.startswith(('strcc', 'strlo')):
        br = next_after(body, i, lo, hi)
        if br is None or not COND_BCS.match(br[1]):
            return 'no bcs to the trip right after the conditional store'
    else:
        return 'the store is neither behind bcs nor conditional on cc'
    t = int(br[2].split()[0], 16)
    if not reaches_trip(body, t, lo, hi):
        return f'the bcs at 0x{br[0]:08x} does not reach trip(b, 5)'
    return None


def arming_writes(body, cands):
    """The CMP1 writes among cands (indices of unguarded stores to [rB, #156])
    that are the arming write (run_loop, since 25 Oct): followed within three
    instructions by the outputs-enable write, a store to [rB, #916] (HRTIM
    OENR) through the same base, with no other store between. Gates off until
    then, so the deadline check does not apply; the arming window is checked
    in C (review only, not by this script)."""
    out = []
    for i in cands:
        m = re.match(REGN + r', \[' + REGN + r', #156\]$', body[i][2])
        if not m:
            continue
        for k in range(i + 1, min(i + 4, len(body))):
            op, args = body[k][1], body[k][2]
            if op.startswith('str') and re.match(REGN + r', \[' + m.group(2) + r', #916\]$', args):
                out.append(i)
                break
            if op.startswith(('str', 'vstr', 'stm', 'push')) or not re.match(r'(mov|add|sub|vmov|vadd|vsub|vmul|vcmp|vmrs|it|ldr|vldr)', op):
                break
    return out


def toc_tail_writes(body, cands):
    """Stage 4 (31 Oct): the unguarded stores to [rB, #156] among cands that are the OFF-ON frame's tail
    compare (Inc/buck_toc_start.h: a frame stretched to its minimum ends its ON span with Timer A CMP1):
    a register loaded with #8 (HRTIM_RST1R_CMP1) within the 6 instructions before, and followed within
    three instructions, with no other store between (but to the stack), by a store of that register to
    [rB, #192] (Timer A RST1R) through the same base. Inside a frame CMP1 is that reset only; the deadline check is the
    passes'. Rule 5d checks that it is on the way to a guarded software update."""
    out = []
    for i in cands:
        m = re.match(REGN + r', \[' + REGN + r', #156\]$', body[i][2])
        if not m:
            continue
        for k in range(i + 1, min(i + 4, len(body))):
            op, args = body[k][1], body[k][2]
            r = re.match(REGN + r', \[' + m.group(2) + r', #192\]$', args)
            if op.startswith('str') and r:
                reg = r.group(1)
                if any(body[j][1].startswith('mov') and body[j][2].replace(' ', '') == f'{reg},#8'
                       for j in range(max(0, i - 6), i)):
                    out.append(i)
                break
            if op.startswith(('str', 'vstr', 'stm', 'push')) and not re.search(r'\[sp(, #\d+)?\]$', args):
                break
    return out


def arming_write(body, cands, evp):
    """(index of the arming write or None, None) or (None, reason): the one
    write among cands that arming_writes() finds and that is not on the path
    of an ordinary pass (evp: its indices, None when not found); more than
    one is an error."""
    armw = [i for i in arming_writes(body, cands) if evp is None or i not in evp]
    if len(armw) > 1:
        return None, 'more than one CMP1 write followed by the outputs-enable write (OENR): ' + \
            ', '.join(f'0x{body[i][0]:08x}' for i in armw)
    return (armw[0] if armw else None), None


def tune_offsets(objdump, elf):
    """[(name, offset from &g_buck)] of every buck_tune_t field, from the ELF's
    DWARF (objdump --dwarf=info), and None; or None and the reason."""
    out = subprocess.run([objdump, '--dwarf=info', elf], capture_output=True, text=True, check=True).stdout
    dies, stack, cur = {}, [], None
    for line in out.splitlines():
        m = re.match(r'\s*<(\d+)><([0-9a-f]+)>: Abbrev Number: (\d+)(?: \((\w+)\))?', line)
        if m:
            cur = None
            if m.group(3) == '0':                   # end of a list of children
                continue
            depth = int(m.group(1))
            cur = {'tag': m.group(4), 'at': {}, 'kids': []}
            dies[int(m.group(2), 16)] = cur
            del stack[depth:]
            if depth > 0 and len(stack) == depth:
                stack[-1]['kids'].append(cur)
            stack.append(cur)
            continue
        a = re.match(r'\s*<[0-9a-f]+>\s+(DW_AT_\w+)\s*:\s*(.*)$', line)
        if a and cur is not None:
            v = a.group(2).strip()
            if a.group(1) == 'DW_AT_name' and v.startswith('(') and '): ' in v:
                v = v.split('): ', 1)[1]            # (indirect string, offset: 0x...): name
            cur['at'][a.group(1)] = v

    def struct_of(typedef):
        d = next((x for x in dies.values() if x['tag'] == 'DW_TAG_typedef' and x['at'].get('DW_AT_name') == typedef), None)
        for _ in range(8):                          # through const/volatile to the structure
            if d is None or d['tag'] == 'DW_TAG_structure_type':
                return d
            r = re.match(r'<0x([0-9a-f]+)>', d['at'].get('DW_AT_type', ''))
            d = dies.get(int(r.group(1), 16)) if r else None
        return None

    def members(s):
        res = []
        for k in s['kids']:
            if k['tag'] != 'DW_TAG_member':
                continue
            loc = k['at'].get('DW_AT_data_member_location', '')
            m = re.match(r'(\d+)$', loc) or re.search(r'DW_OP_plus_uconst: (\d+)', loc)
            if not m:
                return None
            res.append((k['at'].get('DW_AT_name', '?'), int(m.group(1))))
        return res

    st, sb = struct_of('buck_tune_t'), struct_of('buck_t')
    if st is None or sb is None:
        return None, 'buck_tune_t or buck_t not found in the debug info'
    mt, mb = members(st), members(sb)
    tune = next((o for n, o in (mb or []) if n == 'tune'), None)
    if not mt or tune is None:
        return None, 'the member offsets of buck_tune_t or buck_t.tune not found in the debug info'
    return [(n, tune + o) for n, o in mt], None


def tune_reads(body, gregs):
    """{offset: number of reads} of the words of g_buck that run_loop loads
    through a register in gregs (holding &g_buck): ldr/vldr [rG] or
    [rG, #imm] (a D-register vldr reads two words), ldrd (two words)."""
    reads = {}
    for _, op, args in body:
        o = op.split('.')[0]
        m = re.search(r'\[(r\d+|ip|lr|fp|sl|sb)(?:, #(-?\d+))?\]$', args)
        if not (m and m.group(1) in gregs and re.match(r'(v?ldr|ldrd)$', o)):
            continue
        off = int(m.group(2) or 0)
        words = 2 if o == 'ldrd' or (o == 'vldr' and args.startswith('d')) else 1
        for w in range(words):
            reads[off + 4 * w] = reads.get(off + 4 * w, 0) + 1
    return reads


COND_BR = re.compile(r'(b(eq|ne|cs|hs|cc|lo|mi|pl|vs|vc|hi|ls|ge|lt|gt|le)(\.n|\.w)?|cbn?z)$')
NORETURN = re.compile(r'<(__)?buck_fatal(_veneer)?>')   # buck_fatal() does not return: GCC may put any code after its call


ADC12_CSR = 0x50000300


def track(op, args, regs, words, lit):
    """regs ({register: known value}) after one instruction: values from mov,
    movw/movt and mvn immediates, literal loads (lit: the literal's address)
    and add/sub of an immediate to a known register; any other write of a
    register (or a call: r0-r3, ip, lr) makes it unknown."""
    o = op.split('.')[0]
    r = dict(regs)
    if o in ('bl', 'blx'):
        for k in ('r0', 'r1', 'r2', 'r3', 'ip', 'lr'):
            r.pop(k, None)
        return r
    m = re.match(REGN + r', #(-?\d+)$', args)
    if o in ('mov', 'movs', 'movw') and m:
        r[m.group(1)] = int(m.group(2)) & 0xFFFFFFFF
        return r
    if o in ('mvn', 'mvns') and m:
        r[m.group(1)] = ~int(m.group(2)) & 0xFFFFFFFF
        return r
    if o == 'movt' and m:
        if m.group(1) in r:
            r[m.group(1)] = (r[m.group(1)] & 0xFFFF) | ((int(m.group(2)) & 0xFFFF) << 16)
        return r
    m = re.match(REGN + r', ' + REGN + r'$', args)
    if o in ('mov', 'movs') and m:
        if m.group(2) in r:
            r[m.group(1)] = r[m.group(2)]
        else:
            r.pop(m.group(1), None)
        return r
    m = re.match(REGN + r', ' + REGN + r', #(-?\d+)$', args)
    if o in ('add', 'adds', 'addw', 'sub', 'subs', 'subw') and m:
        if m.group(2) in r:
            d = int(m.group(3))
            r[m.group(1)] = (r[m.group(2)] + (d if o.startswith('add') else -d)) & 0xFFFFFFFF
        else:
            r.pop(m.group(1), None)
        return r
    if o.startswith('ldr') and lit is not None and lit in words:
        d = re.match(REGN, args)
        if d:
            r[d.group(1)] = words[lit]
            return r
    if op == '.word' or COND_BR.match(op) or re.match(r'b(\.n|\.w)?$', op) or \
            re.match(r'(str|vstr|stm|push|vpush|cmp|cmn|tst|teq|vcmp|it|nop|dmb|dsb|isb|cps)', o):
        return r
    if o.startswith(('ldm', 'pop')):
        dl = re.findall(REGN, args[args.find('{'):]) if '{' in args else []
    else:
        dl = re.findall(REGN, args.split('[')[0])[:2 if o.startswith(('ldrd', 'umull', 'smull', 'umlal', 'smlal', 'vmov')) else 1]
    wb = re.match(REGN + r'!', args) or re.search(r'\[' + REGN + r'(, #-?\d+)?\]!', args) or re.search(r'\[' + REGN + r'\], #', args)
    for k in dl + ([wb.group(1)] if wb else []):
        r.pop(k, None)
    return r


def csr_read(op, args, regs):
    """True for a load of ADC12_CSR: ldr rX, [rY, #768] (any rY, the rule's
    form until 25 Oct) or a load through a register known to hold ADC12_CSR's
    address minus its offset (e.g. ldr rX, [ip] with ip = ADC1 base + 768)."""
    if not op.startswith('ldr'):
        return False
    if re.search(r', \[(r\d+|ip|lr|fp|sl|sb), #768\]$', args):
        return True
    m = re.search(r', \[' + REGN + r'(?:, #(-?\d+))?\]$', args)
    return bool(m) and m.group(1) in regs and regs[m.group(1)] + int(m.group(2) or 0) == ADC12_CSR


def entry_part(body, lo, hi, words=None, lit=None):
    """(indices of run_loop's entry, None) or (None, reason): the instructions
    reachable from body[0] before the first ADC12_CSR read (since 25 Oct the
    arming wait's; csr_read, with the registers' values tracked along the
    entry, track()), following branches inside [lo, hi) (calls return, except
    those that do not (buck_fatal, or a call with a literal pool right after
    it); returns and branches out end a path)."""
    words, lit = words or {}, lit or {}
    idx = {x[0]: j for j, x in enumerate(body)}
    state, todo = {0: {}}, [0]
    seen = set()
    while todo:
        j = todo.pop()
        if j >= len(body):
            continue
        a, op, args = body[j]
        regs = state[j]
        if op == '.word':
            if j > 0 and body[j - 1][1].split('.')[0] in ('bl', 'blx'):
                continue                                # after a call that does not return
            return None, f'the entry runs into a literal pool at 0x{a:08x}'
        if csr_read(op, args, regs):
            continue                                    # the arming wait (or a pass) begins: not the entry
        seen.add(j)
        o = op.split('.')[0]
        if o in ('tbb', 'tbh') or re.match(r'(mov|ldr)\S* pc', op + ' ' + args):
            return None, f'unanalysed control flow at 0x{a:08x}'
        if (o.startswith(('pop', 'ldm')) and 'pc' in args) or o.startswith('bx') or \
                (o in ('bl', 'blx') and NORETURN.search(args)):
            continue
        nregs = track(op, args, regs, words, lit.get(a))
        succ = []
        if re.match(r'b(\.n|\.w)?$', op) or COND_BR.match(op):
            t = int(args.split()[-2] if op.startswith('cb') else args.split()[0], 16)
            if lo <= t < hi:
                if t not in idx:
                    return None, f'branch into the middle of an instruction at 0x{a:08x}'
                succ.append(idx[t])
            if COND_BR.match(op):
                succ.append(j + 1)
        else:
            succ.append(j + 1)
        for k in succ:
            if k not in state:
                state[k] = nregs
                todo.append(k)
            else:                                       # a join: keep what both paths agree on
                m = {r: v for r, v in state[k].items() if nregs.get(r) == v}
                if m != state[k]:
                    state[k] = m
                    todo.append(k)
    return seen, None


FLAG_SET = re.compile(r'((add|adc|sub|sbc|rsb|and|orr|orn|eor|bic|lsl|lsr|asr|ror|rrx|mov|mvn|mul|neg)s|cmp|cmn|tst|teq)')


def sets_flags(op, args):
    """True when the instruction writes the APSR condition flags."""
    o = op.split('.')[0]
    return bool(FLAG_SET.match(o)) or (o == 'vmrs' and args.startswith('APSR'))


def rare_edges(body):
    """{index of a conditional branch: 'taken' or 'fall'}, the edge that a
    (loops & 31) == 0 test sends to the telemetry or tunables pass: beq/bne
    after tst rX, #31, ands rD, rX, #31 or lsls rD, rX, #27 with no
    flag-setting instruction between them, or cbz/cbnz rD after
    and(s) rD, rX, #31 with rD not written between them (within 12)."""
    rare = {}
    for j, (_, op, args) in enumerate(body):
        o = op.split('.')[0]
        flag = (o in ('tst', 'ands') and args.endswith(', #31')) or (o == 'lsls' and args.endswith(', #27'))
        m = re.match(REGN + r', ' + REGN + r', #31$', args) if o in ('and', 'ands') else None
        rd = m.group(1) if m else None
        if not (flag or rd):
            continue
        for k in range(j + 1, min(j + 13, len(body))):
            ko, ka = body[k][1], body[k][2]
            kb = ko.split('.')[0]
            if flag and kb in ('beq', 'bne'):
                rare[k] = 'taken' if kb == 'beq' else 'fall'
                break
            if rd and kb in ('cbz', 'cbnz') and ka.startswith(rd + ','):
                rare[k] = 'taken' if kb == 'cbz' else 'fall'
                break
            if re.match(r'b(\.n|\.w)?$', ko) or COND_BR.match(ko) or kb in ('bl', 'blx', 'bx') or ko == '.word':
                break
            if flag and sets_flags(ko, ka):
                flag = False
            if rd and ka.startswith(rd + ',') and not kb.startswith(('str', 'vstr', 'cmp', 'tst')):
                rd = None
            if not (flag or rd):
                break
    return rare


def every_pass_part(body, lo, hi):
    """(indices on the path of an ordinary pass, number of loops & 31 tests)
    or (None, reason): what run_loop executes from any pass head (ldr rX,
    [rY, #768]) without taking a rare edge (rare_edges), following branches
    inside [lo, hi); returns, branches out and the calls that do not return
    (buck_fatal, or any call with a literal pool right after it) end a path,
    other calls return."""
    idx = {x[0]: j for j, x in enumerate(body)}
    rare = rare_edges(body)
    heads = [j for j, (_, op, args) in enumerate(body)
             if op.startswith('ldr') and re.search(r', \[(r\d+|ip|lr|fp|sl|sb), #768\]$', args)]
    if not heads:
        return None, 'no pass head (ADC12_CSR read) found'
    if not rare:
        return None, 'no (loops & 31) test found'
    seen, todo = set(), list(heads)
    while todo:
        j = todo.pop()
        if j in seen or j >= len(body):
            continue
        a, op, args = body[j]
        if op == '.word':
            if j > 0 and body[j - 1][1].split('.')[0] in ('bl', 'blx'):
                continue                                # after a call that does not return
            return None, f'a pass runs into a literal pool at 0x{a:08x}'
        seen.add(j)
        o = op.split('.')[0]
        if o in ('tbb', 'tbh') or re.match(r'(mov|ldr)\S* pc', op + ' ' + args):
            return None, f'unanalysed control flow at 0x{a:08x}'
        if (o.startswith(('pop', 'ldm')) and 'pc' in args) or o.startswith('bx') or \
                (o in ('bl', 'blx') and NORETURN.search(args)):
            continue
        if re.match(r'b(\.n|\.w)?$', op) or COND_BR.match(op):
            t = int(args.split()[-2] if op.startswith('cb') else args.split()[0], 16)
            if rare.get(j) != 'taken' and lo <= t < hi:
                if t not in idx:
                    return None, f'branch into the middle of an instruction at 0x{a:08x}'
                todo.append(idx[t])
            if not COND_BR.match(op) or rare.get(j) == 'fall':
                continue
        todo.append(j + 1)
    return seen, len(rare)


REG = r'(?:r\d+|ip|lr|fp|sl|sb|sp|pc)'
BRANCH = re.compile(r'(b|bl|blx|bx|cbz|cbnz)(eq|ne|cs|hs|cc|lo|mi|pl|vs|vc|hi|ls|ge|lt|gt|le|al)?(\.n|\.w)?$')
NO_DEF = re.compile(r'(str|vstr|stm|push|vpush|cmp|cmn|tst|teq|vcmp|it|nop|dmb|dsb|isb|cps)')


def defs(op, args):
    """The core registers an instruction writes."""
    o = op.split('.')[0]
    if op == '.word' or BRANCH.match(op) or NO_DEF.match(o):
        return set()
    if o.startswith(('ldm', 'pop')):
        out = set(re.findall(REG, args[args.find('{'):])) if '{' in args else set()
        m = re.match(r'(' + REG + r')!', args)
        return out | ({m.group(1)} if m else set())
    if o == 'vmrs' and args.startswith('APSR'):
        return set()
    regs = re.findall(REG, args.split('[')[0])
    if o.startswith(('ldrd', 'umull', 'smull', 'umlal', 'smlal')) or (o == 'vmov' and len(regs) >= 2):
        return set(regs[:2])
    return set(regs[:1])


def toc_call_restore(body, i):
    """Stage 2: True for the ldm that ends a BUCK_TOC_CALL (Inc/buck_toc_fw.h): stmdb sp!, {L};
    vpush; bl; vpop; ldmia sp!, {L} with the same list, or a BUCK_TOC_CALL_CORE (31 Oct): stmdb sp!, {L};
    bl; ldmia sp!, {L}: it restores the values the stmdb saved, so it writes nothing new."""
    a, op, args = body[i]
    if not (op.split('.')[0].startswith('ldm') and args.startswith('sp!') and i >= 2):
        return False
    same = lambda s0: (s0[1].startswith('stmdb') and s0[2].startswith('sp!') and
                       s0[2][s0[2].find('{'):] == args[args.find('{'):])
    if same(body[i - 2]) and body[i - 1][1] == 'bl':
        return True                             # BUCK_TOC_CALL_CORE
    if i < 4:
        return False
    s0, v0, b0, v1 = body[i - 4], body[i - 3], body[i - 2], body[i - 1]
    return same(s0) and v0[1] == 'vpush' and b0[1] == 'bl' and v1[1] == 'vpop' and v0[2] == v1[2]


def exit_restore(body, i, lo, hi):
    """True for an ldm/pop at body[i] that returns, or that is followed by a
    branch out of [lo, hi), or that ends a BUCK_TOC_CALL (stage 2)."""
    a, op, args = body[i]
    if not op.split('.')[0].startswith(('ldm', 'pop')):
        return False
    if toc_call_restore(body, i):
        return True
    if 'pc' in re.findall(REG, args[args.find('{'):] if '{' in args else ''):
        return True
    if i + 1 < len(body) and re.match(r'b(\.n|\.w)?$', body[i + 1][1]):
        t = int(body[i + 1][2].split()[0], 16)
        return not (lo <= t < hi)
    return False


def address_unused(body, i, reg, lo, hi):
    """True when the value that body[i] writes into reg is never used as an
    address: every path from body[i + 1], following branches inside
    [lo, hi), ends (a return, a branch out, buck_fatal or a call with a
    literal pool right after it) before an instruction that has reg inside
    its brackets or as its ldm/stm base."""
    idx = {x[0]: j for j, x in enumerate(body)}
    seen, todo = set(), [i + 1]
    while todo:
        j = todo.pop()
        if j in seen:
            continue
        if j >= len(body):
            return False                                # runs off the end: not analysed
        seen.add(j)
        a, op, args = body[j]
        o = op.split('.')[0]
        if op == '.word':
            if j > 0 and body[j - 1][1].split('.')[0] in ('bl', 'blx'):
                continue                                # after a call that does not return
            return False
        if re.search(r'\[[^\]]*\b' + reg + r'\b', args) or \
                (o.startswith(('ldm', 'stm', 'vldm', 'vstm')) and re.match(reg + r'\b', args)):
            return False                                # used as an address
        if o in ('tbb', 'tbh') or re.match(r'(mov|ldr)\S* pc', op + ' ' + args):
            return False                                # unanalysed control flow
        if (o.startswith(('pop', 'ldm')) and 'pc' in args) or o.startswith('bx') or \
                (o in ('bl', 'blx') and NORETURN.search(args)):
            if re.match(r'(pop|ldm(ia|db)?|bx)(eq|ne|cs|hs|cc|lo|mi|pl|vs|vc|hi|ls|ge|lt|gt|le)$', o):
                todo.append(j + 1)                      # a conditional return (IT block) may fall through
            continue
        if re.match(r'b(\.n|\.w)?$', op) or COND_BR.match(op):
            t = int(args.split()[-2] if op.startswith('cb') else args.split()[0], 16)
            if lo <= t < hi:
                if t not in idx:
                    return False
                todo.append(idx[t])
            if not COND_BR.match(op):
                continue
        todo.append(j + 1)
    return True


def base_is_g_buck(body, reg, words, lit, g_buck, lo, hi):
    """True when reg holds g_buck's address throughout run_loop (body, in
    [lo, hi)): at least one instruction writes it, and every one that does
    is a literal load of &g_buck (lit: a PC-relative load's literal address,
    words: the literals), an exit restore, or a write whose value is never
    used as an address (address_unused)."""
    writers = [i for i, (a, op, args) in enumerate(body) if reg in defs(op, args) and not exit_restore(body, i, lo, hi)]
    loads = [i for i in writers if body[i][1].startswith('ldr') and body[i][0] in lit
             and words.get(lit[body[i][0]]) == g_buck]
    return bool(loads) and all(i in loads or address_unused(body, i, reg, lo, hi) for i in writers)


CHECKED_GAINS = ('obs_beta', 'obs_gamma')   # checked where reloaded, never every pass (user, 25 Oct)
UNUSED_TUNE = ('alpha_i',)   # stage 3 (30 Oct 2026): the Padé estimator's alpha_I is gone; the field stays for the
                             # layout (user decision 3) and must not be read anywhere in run_loop


def tune_verdict(offs, r_ent, r_pass, r_every):
    """(errors, number of fields that pass) of rule 4 from the reads
    ({offset: count}) at the entry, in the passes and on the path of an
    ordinary pass: every field at least once at the entry and once in the
    passes; obs_beta and obs_gamma among the fields and never on that path."""
    errs, n = [], 0
    names = {nm for nm, _ in offs}
    if not set(CHECKED_GAINS) <= names:
        errs.append('g_buck.tune has no obs_beta or obs_gamma (the observer gains this rule must cover)')
    for name, off in offs:
        ne, np_, nv = r_ent.get(off, 0), r_pass.get(off, 0), r_every.get(off, 0)
        bad = []
        if name in UNUSED_TUNE:
            if ne or np_:
                bad.append(f'read {ne} time(s) at the loop entry and {np_} time(s) in the passes, expected none '
                           '(unused since stage 3)')
        elif ne < 1 or np_ < 1:
            bad.append(f'read {ne} time(s) at the loop entry and {np_} time(s) in the passes, expected at least 1 in each')
        if name in CHECKED_GAINS and nv > 0:
            bad.append(f'read {nv} time(s) on the path of an ordinary pass (to be checked only where reloaded)')
        if bad:
            errs.append(f'g_buck.tune.{name} (offset {off}) is ' + '; '.join(bad))
        else:
            n += 1
    return errs, n


def selftest():
    """The guard rule against known-good and known-bad sequences; the arming
    write; the register holding &g_buck; of the tunables reload, the read
    count, the entry and its register values, the path of an ordinary pass
    and the per-field verdict."""
    r = tune_reads([(0, 'vldr', 's9, [r4, #48]'), (2, 'ldr.w', 'r0, [r4]'), (6, 'ldrd', 'r0, r1, [r4, #48]'),
                    (10, 'vldr', 'd8, [r4, #56]'), (14, 'vldr', 's1, [r5, #48]'), (18, 'vstr', 's9, [r4, #52]'),
                    (22, 'ldr', 'r1, [r4, #48]!')], {'r4'})
    if r != {48: 2, 0: 1, 52: 1, 56: 1, 60: 1}:
        return f'self-test of the tunables read count: {r}'
    seq = [(0x00, 'ldr', 'r4, [pc, #20]'), (0x02, 'vldr', 's0, [r4, #60]'), (0x06, 'cbz', 'r2, 12 <x>'),
           (0x08, 'ldr', 'r1, [r4, #64]'), (0x0a, 'b.n', '10 <x>'), (0x0c, '.word', '0x20000000'),
           (0x10, 'ldr.w', 'r3, [r5, #768]'), (0x14, 'vldr', 's1, [r4, #60]'), (0x18, 'bne.n', '10 <x>'),
           (0x1a, 'pop', '{r4, pc}')]
    seq[6] = (0x12, 'ldr.w', 'r3, [r5, #768]')          # the cbz target: the pass head
    seq[4] = (0x0a, 'b.n', '12 <x>')
    ent, why = entry_part(seq, 0, 0x20)
    if ent != {0, 1, 2, 3, 4}:
        return f'self-test of the loop entry: {ent} {why}'
    if tune_reads([seq[i] for i in ent], {'r4'}) != {60: 1, 64: 1} or \
            tune_reads([x for i, x in enumerate(seq) if i not in ent], {'r4'}) != {60: 1}:
        return 'self-test of the entry / pass read split'
    # the path of an ordinary pass: head, a read every pass, the loops & 31 test, back to the head;
    # the reload (the read of #64) only behind the test's rare edge
    pas = [(0x00, 'ldr.w', 'r3, [r1, #768]'), (0x04, 'vldr', 's0, [r4, #60]'), (0x08, 'tst.w', 'ip, #31'),
           (0x0c, 'vmul.f32', 's1, s1, s2'), (0x10, 'str', 'r2, [r0, #24]'), (0x12, 'beq.w', '20 <x>'),
           (0x16, 'b.n', '0 <x>'), (0x18, 'bl', '40 <f>'), (0x1c, '.word', '0x20000000'),
           (0x20, 'vldr', 's2, [r4, #64]'), (0x24, 'b.n', '0 <x>')]
    def rep(sq, k, new):
        return [x if i != k else (x[0],) + new for i, x in enumerate(sq)]
    bne = [(0x00, 'ldr.w', 'r3, [r1, #768]'), (0x04, 'vldr', 's0, [r4, #60]'), (0x08, 'lsls.w', 'r2, ip, #27'),
           (0x0c, 'bne.n', '14 <x>'), (0x0e, 'vldr', 's2, [r4, #64]'), (0x12, 'b.n', '0 <x>'), (0x14, 'b.n', '0 <x>')]
    cbz = rep(rep(pas, 2, ('ands.w', 'r5, ip, #31')), 5, ('cbz', 'r5, 20 <x>'))
    variants = [
        (pas, {60: 1}),                                         # beq: the taken edge is rare
        (bne, {60: 1}),                                         # bne: the fall-through is rare
        (cbz, {60: 1}),                                         # ands, then cbz on its result
        (rep(pas, 3, ('vldr', 's1, [r4, #64]')), {60: 1, 64: 1}),   # a reload read before the test: every pass
        (rep(pas, 1, ('bcs.n', '18 <x>')), {}),                 # a call that does not return (pool after it)
        (rep(rep(rep(pas, 1, ('bcs.n', '18 <x>')), 7, ('bl', '40 <__buck_fatal_veneer>')), 8, ('nop', '')), {}),
                                                                # buck_fatal's call, the reload right after it
        (rep(pas, 3, ('adds', 'r2, #1')), None),                # the flags rewritten before beq: no test
        ([(0x00, 'ldr.w', 'r3, [r1, #768]'), (0x04, 'tst.w', 'ip, #31'), (0x08, 'beq.w', '30 <x>'),
          (0x0c, 'b.n', '0 <x>'), (0x10, 'ldr.w', 'r3, [r1, #768]'), (0x14, 'vldr', 's0, [r4, #60]'),
          (0x18, 'tst.w', 'ip, #31'), (0x1c, 'beq.w', '30 <x>'), (0x20, 'b.n', '10 <x>'),
          (0x30, 'vldr', 's2, [r4, #64]'), (0x34, 'b.n', '0 <x>')], {60: 1}),   # two copies: a read in the second's
    ]
    for n, (sq, want) in enumerate(variants):
        evp, why = every_pass_part(sq, 0, 0x30)
        if want is None:
            if evp is not None:
                return f'self-test {n} of the ordinary-pass path: a test found where the flags were rewritten'
            continue
        got = tune_reads([sq[i] for i in evp], {'r4'}) if evp is not None else None
        if got != want:
            return f'self-test {n} of the ordinary-pass path: {evp} {why} {got}'
    # the per-field verdict
    offs = [('a', 0), ('obs_beta', 4), ('obs_gamma', 8)]
    all1 = {0: 1, 4: 1, 8: 1}
    verdicts = [
        ((offs, all1, all1, {}), 0),
        ((offs, all1, all1, {0: 1}), 0),                                  # another tunable every pass: not this rule's
        ((offs, {4: 1, 8: 1}, all1, {}), 1),                              # a field read only in the passes
        ((offs, all1, {0: 1, 4: 1}, {}), 1),                              # obs_gamma read only at the entry
        ((offs, all1, all1, {4: 2}), 1),                                  # obs_beta checked every pass
        (([('a', 0), ('obs_gamma', 8)], all1, all1, {}), 1),              # no obs_beta field
        ((offs + [('alpha_i', 12)], all1, all1, {}), 0),                  # stage 3: alpha_i unread: passes
        ((offs + [('alpha_i', 12)], all1, {0: 1, 4: 1, 8: 1, 12: 1}, {}), 1),   # alpha_i read in a pass: error
    ]
    for n, (args, nerr) in enumerate(verdicts):
        errs, _ = tune_verdict(*args)
        if len(errs) != nerr:
            return f'self-test {n} of the per-field verdict: {errs}'
    good = [(0x10, 'ldr.w', 'r6, [r5, #144]'), (0x14, 'sub.w', 'r4, r6, #2048'), (0x18, 'cmp.w', 'r4, #6400'),
            (0x1c, 'it', 'cc'), (0x1e, 'strcc.w', 'r2, [r5, #156]'), (0x22, 'bcs.w', '40 <x>'),
            (0x26, 'b.n', '10 <x>'), (0x40, 'ldr', 'r2, [sp, #20]'), (0x42, 'movs', 'r0, #5'), (0x44, 'b.n', '50 <x>'),
            (0x50, 'nop', '')]
    def mut(k, new):
        b = list(good); b[k] = (b[k][0],) + new; return b
    cases = [
        (good, True),
        (mut(8, ('movs', 'r0, #4')), False),                    # trip removed (no fault 5)
        (mut(5, ('bcc.w', '40 <x>')), False),                   # inverted condition
        (mut(1, ('sub.w', 'r4, r2, #2048')), False),            # difference of the duty, not the counter
        (mut(2, ('cmp.w', 'r6, #6400')), False),                # the raw counter compared
        (mut(3, ('it', 'hs')), False),                          # store on the late side
        (mut(4, ('str.w', 'r2, [r5, #156]')), False),           # unconditional store after it cc
        (mut(0, ('ldr.w', 'r6, [r5, #140]')), False),           # another register read
        (mut(1, ('sub.w', 'r5, r6, #2048')), False),            # base overwritten
        ([(0x10, 'ldr.w', 'r6, [r5, #144]'), (0x14, 'sub.w', 'r4, r6, #2048'), (0x18, 'cmp.w', 'r4, #6400'),
          (0x1c, 'bcs.w', '40 <x>'), (0x20, 'str.w', 'r2, [r5, #156]'), (0x40, 'movs', 'r0, #5')], True),
    ]
    # 25 Oct: a literal pool between the conditional store and its bcs (strcc; b <x>; x: bcs)
    lit_pool = good[:5] + [(0x22, 'b.n', '30 <x>'), (0x24, '.word', '0x40016800'), (0x30, 'bcs.w', '40 <x>'),
                           (0x34, 'b.n', '10 <x>')] + good[7:]
    cases += [
        (lit_pool, True),
        (lit_pool[:6] + [(0x24, '.word', '0x40016800'), (0x30, 'nop', ''), (0x32, 'bcs.w', '40 <x>')] + good[7:], False),
                                                                # an instruction between: not the store's bcs
        (lit_pool[:5] + [(0x22, 'b.n', '24 <x>'), (0x24, 'b.n', '26 <x>'), (0x26, 'b.n', '30 <x>'),
                         (0x30, 'bcs.w', '40 <x>')] + good[7:], False),    # more than two branches
        (lit_pool[:5] + [(0x22, 'b.n', '200 <x>')] + good[7:], False),     # a branch out of the loop
    ]
    for n, (b, want) in enumerate(cases):
        i = next(j for j, x in enumerate(b) if x[2].endswith('#156]'))
        got = guard_fault(b, i, 'r5', 0, 0x100) is None
        if got != want:
            return f'self-test case {n}: guard rule gives {got}, expected {want}'
    # the arming write: CMP1 then OENR through the same base, nothing stored between
    arm = [(0x00, 'str.w', 'r6, [r0, #156]'), (0x04, 'vmov.f32', 's15, s9'), (0x08, 'str.w', 'r4, [r0, #916]')]
    for n, (sq, want) in enumerate([
            (arm, [0]),
            ([arm[0], arm[1], (0x08, 'str.w', 'r4, [r1, #916]')], []),          # another base
            ([arm[0], (0x04, 'str', 'r2, [r4, #108]'), arm[2]], []),            # a store between
            ([arm[2], arm[1], (0x0c, 'str.w', 'r6, [r0, #156]')], []),          # OENR before CMP1
            ([arm[0], arm[1], arm[1], arm[1], arm[2]], []),                     # too far
            ([arm[0], (0x04, 'b.n', '8 <x>'), arm[2]], [])]):                   # a branch between (26 Oct)
        if arming_writes(sq, [j for j, x in enumerate(sq) if x[2].endswith('#156]')]) != want:
            return f'self-test {n} of the arming write'
    # 26 Oct: the arming write is the one such pair off the path of an ordinary pass; two off it are an error
    loop = [(0x00, 'str.w', 'r6, [r0, #156]'), (0x04, 'str.w', 'r4, [r0, #916]'),
            (0x08, 'ldr.w', 'r3, [r1, #768]'), (0x0c, 'tst.w', 'ip, #31'), (0x10, 'beq.w', '20 <x>')]
    for n, (sq, want, err) in enumerate([
            (loop + [(0x14, 'str.w', 'r6, [r0, #156]'), (0x18, 'str.w', 'r4, [r0, #916]'),
                     (0x1c, 'b.n', '8 <x>'), (0x20, 'b.n', '8 <x>')], 0, False),           # the second pair every pass
            (loop + [(0x14, 'b.n', '8 <x>'), (0x20, 'str.w', 'r6, [r0, #156]'),
                     (0x24, 'str.w', 'r4, [r0, #916]'), (0x28, 'b.n', '8 <x>')], None, True)]):   # behind the rare edge
        evp, why = every_pass_part(sq, 0, 0x30)
        got, gerr = arming_write(sq, [j for j, x in enumerate(sq) if x[2].endswith('#156]')], evp)
        if got != want or bool(gerr) != err:
            return f'self-test {n} of the arming write and the ordinary-pass path: {got} {gerr} {evp} {why}'
    # 26 Oct: the register holding &g_buck, also when GCC uses it as a scratch on a block that only returns
    gb = [(0x00, 'ldr', 'r2, [pc, #20]'), (0x02, 'str.w', 'r5, [r2, #156]'), (0x06, 'cbz', 'r0, c <x>'),
          (0x08, 'b.n', '2 <x>'), (0x0a, 'nop', ''), (0x0c, 'movs', 'r2, #20'), (0x0e, 'str.w', 'r2, [r9]'),
          (0x12, 'pop', '{r4, pc}'), (0x14, 'nop', ''), (0x18, '.word', '0x20000000')]
    for n, (sq, want) in enumerate([
            (gb, True),
            (rep(gb, 7, ('b.w', '200 <x>')), True),                             # a branch out
            (rep(gb, 7, ('bl', '40 <__buck_fatal_veneer>')), True),             # a call that does not return
            (rep(gb, 7, ('b.n', '2 <x>')), False),                              # back into the loop
            (rep(rep(gb, 7, ('popeq', '{r4, pc}')), 8, ('str.w', 'r1, [r2]')), False),   # a conditional return
            (rep(gb, 6, ('str.w', 'r1, [r2]')), False),                         # used as an address
            (rep(gb, 6, ('ldmia', 'r2!, {r0, r1}')), False),                    # ... as an ldm base
            (rep(gb, 0, ('nop', '')), False),                                   # no literal load of &g_buck
            (rep(rep(gb, 7, ('bl', '40 <f>')), 8, ('.word', '0x00000000')), True),   # a call, a literal pool after it
            (rep(gb, 7, ('bx', 'lr')), True),                                   # a return by bx
            (rep(gb, 7, ('pop', '{r2, r4, pc}')), True),                        # an exit restore of the register
            (rep(gb, 5, ('ldr', 'r2, [r2, #4]')), True),                        # the write loads through it
            (rep(gb, 6, ('ldr', 'r0, [r3, r2]')), False),                       # used as an index
            (rep(gb, 6, ('stmia', 'r2!, {r0, r1}')), False),                    # ... as an stm base
            (rep(rep(gb, 6, ('bne.n', '14 <x>')), 8, ('str.w', 'r1, [r2]')), False),     # on a branch's target
            (rep(rep(rep(gb, 6, ('bne.n', '14 <x>')), 7, ('str.w', 'r1, [r2]')), 8, ('pop', '{r4, pc}')), False),
                                                                                # ... on its fall-through
            (rep(rep(gb, 7, ('ldmiaeq.w', 'sp!, {r4, pc}')), 8, ('str.w', 'r1, [r2]')), False),  # a conditional ldm
            (rep(gb, 7, ('b.n', '10 <x>')), False),                             # into the middle of an instruction
            (rep(gb, 6, ('tbb', '[pc, r3]')), False),                           # unanalysed control flow
            (rep(gb, 7, ('nop', '')), False),                                   # runs into a literal pool
            (gb[:7], False),                                                    # runs off the end
            (rep(rep(gb, 5, ('ldr', 'r2, [r9, #4]')), 6, ('str.w', 'r1, [r2]')), False)]):   # a load from memory
        if base_is_g_buck(sq, 'r2', {0x18: 0x20000000}, {0x00: 0x18}, 0x20000000, 0, 0x20) != want:
            return f'self-test {n} of the register holding &g_buck'
    if base_is_g_buck(gb, 'r2', {0x18: 0x40016800}, {0x00: 0x18}, 0x20000000, 0, 0x20):
        return 'self-test of the register holding &g_buck (a literal of another address)'
    # the entry ends at the arming wait's ADC12_CSR read, also through a register holding its address
    ent_seq = [(0x00, 'ldr', 'r2, [pc, #40]'), (0x02, 'vldr', 's3, [r2, #64]'), (0x06, 'mov.w', 'r3, #1342177280'),
               (0x0a, 'add.w', 'ip, r3, #768'), (0x0e, 'ldr.w', 'r5, [ip]'), (0x12, 'vldr', 's4, [r2, #60]'),
               (0x16, 'bpl.w', '30 <x>'), (0x1a, 'ldr.w', 'r4, [r3, #768]'), (0x1e, 'b.n', '1a <x>'),
               (0x30, 'movs', 'r0, #4'), (0x32, 'bl', '80 <__buck_fatal_veneer>'), (0x36, '.word', '0x20000000')]
    ent, why = entry_part(ent_seq, 0, 0x40, {0x2c: 0x20000000}, {0x00: 0x2c})
    if ent != {0, 1, 2, 3}:
        return f'self-test of the entry boundary (a register holding ADC12_CSR\'s address): {ent} {why}'
    alt = [x if j != 3 else (0x0a, 'add.w', 'ip, r3, #772') for j, x in enumerate(ent_seq)]   # not the CSR
    ent, why = entry_part(alt, 0, 0x40, {}, {})
    if ent is None or 5 not in ent or 7 in ent:
        return f'self-test of the entry boundary (another address, then [rX, #768]): {ent} {why}'
    joined = [(0x00, 'cbz', 'r0, 8 <x>'), (0x02, 'mov.w', 'r3, #1342177536'), (0x06, 'b.n', 'c <x>'),
              (0x08, 'mov.w', 'r3, #1342177280'), (0x0c, 'ldr.w', 'r5, [r3, #512]'), (0x10, 'ldr.w', 'r5, [r3, #768]')]
    ent, why = entry_part(joined, 0, 0x20, {}, {})  # ADC12_CSR on one path only: not the boundary at the join
    if ent != {0, 1, 2, 3, 4}:
        return f'self-test of the entry boundary (a join): {ent} {why}'
    joined = [(0x00, 'mov.w', 'r3, #1342177280'), (0x04, 'cbz', 'r1, c <x>'), (0x06, 'mov.w', 'r3, #1342177536'),
              (0x0a, 'nop', ''), (0x0c, 'ldr.w', 'r5, [r3, #512]'), (0x10, 'ldr.w', 'r5, [r3, #768]')]
    ent, why = entry_part(joined, 0, 0x20, {}, {})  # 26 Oct: the same with the ADC12_CSR path arriving last
    if ent != {0, 1, 2, 3, 4}:
        return f'self-test of the entry boundary (a join, the other order): {ent} {why}'
    nr = [(0x00, 'cbnz', 'r0, c <x>'), (0x02, 'movs', 'r0, #4'), (0x04, 'bl', '80 <__buck_fatal_veneer>'),
          (0x08, 'vldr', 's4, [r2, #60]'), (0x0c, 'ldr.w', 'r5, [r3, #768]')]
    ent, why = entry_part(nr, 0, 0x20, {}, {})      # 26 Oct: nothing after a call that does not return
    if ent != {0, 1, 2}:
        return f'self-test of the loop entry (buck_fatal\'s call): {ent} {why}'
    # 26 Oct: the entry's register values: a literal load, movw/movt, mvn, sub, a copy (of an unknown register:
    # unknown); a load and a call make registers unknown (a call: r0-r3, ip, lr)
    regs = {}
    for op, args, la in [('ldr', 'r6, [pc, #8]', 0x10), ('movw', 'r7, #768', None), ('movt', 'r7, #20480', None),
                         ('mvn', 'r8, #896', None), ('sub.w', 'r9, r6, #256', None), ('mov', 'r0, r6', None),
                         ('mov', 'sl, r6', None), ('mov.w', 'fp, #1', None), ('mov', 'fp, r5', None),
                         ('movs', 'r4, #1', None), ('ldr', 'r4, [r6, #4]', None), ('mov.w', 'ip, #1', None),
                         ('bl', '40 <f>', None)]:
        regs = track(op, args, regs, {0x10: 0x50000000}, la)
    if regs != {'r6': 0x50000000, 'r7': 0x50000300, 'r8': 0xFFFFFC7F, 'r9': 0x4FFFFF00, 'sl': 0x50000000}:
        return f'self-test of the entry\'s register values: {regs}'
    return None


def main():
    objdump, nm, elf = sys.argv[1:4]
    errors = []
    st = selftest()
    if st:
        print('check_loop: FAIL:', st); return 1

    syms = {}
    for line in subprocess.run([nm, '-S', elf], capture_output=True, text=True, check=True).stdout.splitlines():
        f = line.split()
        if len(f) == 4:
            syms[f[3]] = (int(f[0], 16), int(f[1], 16))
    loop = next((n for n in syms if n == 'run_loop' or n.startswith('run_loop.')), None)
    if loop is None:
        print('check_loop: run_loop not found'); return 1
    for name in (loop, 'buck_run'):
        a, size = syms[name]
        if not (CCM_LO <= a and a + size <= CCM_HI):
            errors.append(f'{name} at 0x{a:08x}+{size} is not in CCM SRAM')
    for name in syms:
        if ('run_loop' in name or 'buck_run' in name) and ('cold' in name or 'unlikely' in name):
            errors.append(f'GCC split a cold part out of the loop: {name}')

    dis = subprocess.run([objdump, '-d', '--no-show-raw-insn', '-j', '.ccmtext', elf],
                         capture_output=True, text=True, check=True).stdout
    ins, words, lit = [], {}, {}
    for line in dis.splitlines():
        m = re.match(r'\s*([0-9a-f]+):\s+(\S+)\s*(.*)', line)
        if m:
            a = int(m.group(1), 16)
            ins.append((a, m.group(2), m.group(3).split('@')[0].strip()))
            if m.group(2) == '.word':
                words[a] = int(m.group(3).split()[0], 16)
            p = re.search(r'\[pc, #-?\d+\]\s*@ \(?([0-9a-f]+)', m.group(3))
            if p:
                lit[a] = int(p.group(1), 16)          # a PC-relative load: the literal's address
    for a, op, args in ins:
        if FUSED.match(op):
            errors.append(f'fused multiply-add at 0x{a:08x}: {op} {args}')

    lo, size = syms[loop]
    body = [x for x in ins if lo <= x[0] < lo + size]
    g_buck = syms.get('g_buck', (None, 0))[0]
    g_toc_mb = syms.get('g_toc_mb', (None, 0))[0]

    guarded, unguarded = 0, {}
    for i, (a, op, args) in enumerate(body):
        m = re.match(r'(r\d+|ip|lr|fp|sl), \[(r\d+|ip|lr|fp|sl), #156\]', args)
        if not (op.startswith('str') and m):
            continue
        base = m.group(2)
        if base_is_g_buck(body, base, words, lit, g_buck, lo, lo + size):
            continue
        if g_toc_mb is not None and base_is_g_buck(body, base, words, lit, g_toc_mb, lo, lo + size):
            continue                            # stage 2: a field of the TOC mailbox at offset 156
        why = guard_fault(body, i, base, lo, lo + size)
        if why is None:
            guarded += 1
        else:
            unguarded[i] = why
    # the arming write (since 25 Oct in run_loop): exactly one, not on the path of an ordinary pass
    evp_all, _ = every_pass_part(body, lo, lo + size)
    arm, why = arming_write(body, unguarded, evp_all)
    if why:
        errors.append(why)
    elif arm is not None:
        del unguarded[arm]
    # the OFF-ON frame's tail compare (stage 4, 31 Oct; rule 5d: exactly one, on the way to a guarded software
    # update). Not tested against every_pass_part: the TOC branch is on the path of an ordinary (HOLD) pass
    # there (it takes no loops & 31 edge); no pass writes RST1R
    toc_tails = toc_tail_writes(body, unguarded)
    for i in toc_tails:
        del unguarded[i]
    for i, why in sorted(unguarded.items()):
        errors.append(f'CMP1 write at 0x{body[i][0]:08x} is not guarded by the deadline check: {why}')
    if guarded == 0:
        errors.append('no guarded CMP1 write found in run_loop')

    # 4. the tunables reload: every g_buck.tune field read at the entry and in the passes, the observer's
    #    gains not on the path of an ordinary pass
    offs, why = tune_offsets(objdump, elf)
    nt, nloops = 0, 0
    if offs is None:
        errors.append(f'tunables reload: {why}')
    else:
        bases = set()
        for a, op, args in body:
            m = re.search(r'\[(r\d+|ip|lr|fp|sl|sb)(?:, #-?\d+)?\]$', args)
            if m and op.startswith(('ldr', 'vldr')):
                bases.add(m.group(1))
        gregs = {r for r in bases if base_is_g_buck(body, r, words, lit, g_buck, lo, lo + size)}
        if not gregs:
            errors.append('tunables reload: no register holds &g_buck throughout run_loop')
        else:
            ent, why = entry_part(body, lo, lo + size, words, lit)
            evp, nloops = every_pass_part(body, lo, lo + size)
            if ent is None:
                errors.append(f'tunables reload: the loop entry not found: {why}')
            elif evp is None:
                errors.append(f'tunables reload: the path of an ordinary pass not found: {nloops}')
            else:
                r_ent = tune_reads([x for i, x in enumerate(body) if i in ent], gregs)
                r_pass = tune_reads([x for i, x in enumerate(body) if i not in ent], gregs)
                r_every = tune_reads([x for i, x in enumerate(body) if i in evp], gregs)
                errs, nt = tune_verdict(offs, r_ent, r_pass, r_every)
                errors += ['tunables reload: ' + e for e in errs]

    # 5. stage 4 (the feed-forward TOC, 30 Oct 2026; Inc/buck_toc_start.h, Src/buck_toc.c):
    #  a. the TOC code lies in CCM: every function whose name starts with toc_ or buck_toc_;
    #  b. no fused multiply-add there either (rule 2 covers all of .ccmtext);
    #  c. every conditional store in the TOC functions is the TOC_GUARD form
    #         ldr  rC, [rH, #144]   Timer A CNTR       sub  rL, rC, rE   cmp  rL, rW
    #         it   cc/lo          strcc rV, [rA]      bcs  <miss>
    #     (at least 3: the held duty's CMP1 after the frame, the passes' CMP1 (toc_pass), the return words), and
    #     the CMP1 writes (in buck_toc_finish and toc_pass) reach toc_fail (fault 5) within 16
    #     instructions (a bl, or the b.w of a tail call); the only other CMP1 store there is toc_exit's BUCK_CMP_MIN after the gates are off;
    #  d. the inline start in run_loop (the HOLD copy, one per order): the guarded software updates of the
    #     same form, whose stored value is MSWU | TASWU (#3) through a register holding HRTIM1 + 0x384
    #     (CR2), at least 4 (two per order); in each order the high side is forced (a store to Timer A
    #     SET1R #188 or RST1R #192) BEFORE the solve's square root (the feed-forward: the span runs while
    #     it is solved), and the square root before that order's first update; run_loop calls
    #     buck_toc_finish (after PASS_TOC) through BUCK_TOC_CALL and buck_toc_ret (one per order) through
    #     BUCK_TOC_CALL_CORE (push of the caller-saved core registers, bl: no FP save), so buck_toc_ret
    #     must use no floating-point register (no s/d operand, no vpush/vpop/vmrs/vmsr), and each such
    #     call follows a guarded software update within 12 instructions (31 Oct: the return words right
    #     after the second update, on the path BUCK_TOC_END_LEAD_TICKS budgets);
    #  e. the TOC code calls nothing outside CCM: no bl to a flash function or a linker veneer (a struct
    #     initializer that GCC turns into memset/memcpy would run from flash; buck_fatal excepted).
    TOC_PREFIX = ('toc_', 'buck_toc_')
    tocfn = sorted((a, n, sz) for n, (a, sz) in syms.items()
                   if n.startswith(TOC_PREFIX) and sz > 0 and not n.startswith('__') and a < 0x20000000)   # code
    if not tocfn:
        errors.append('stage 4: no TOC functions found')
    for a, n, sz in tocfn:
        if not (CCM_LO <= a and a + sz <= CCM_HI):
            errors.append(f'stage 4: {n} at 0x{a:08x} is not in CCM SRAM')
    alldis = subprocess.run([objdump, '-d', '--no-show-raw-insn', elf], capture_output=True, text=True,
                            check=True).stdout
    allins = []
    for line in alldis.splitlines():
        m = re.match(r'\s*([0-9a-f]+):\s+(\S+)\s*(.*)', line)
        if m:
            allins.append((int(m.group(1), 16), m.group(2), m.group(3).split('@')[0].strip()))
    idx = {a: k for k, (a, _, _) in enumerate(allins)}
    fail_fn = [(a, sz) for a, n, sz in tocfn if n.startswith('toc_fail')]

    def guard_form(k):
        """The TOC_GUARD form around the conditional store allins[k]; returns the miss target or None."""
        if k < 4 or k + 1 >= len(allins):
            return None
        pat = [allins[k - j] for j in (4, 3, 2, 1)] + [allins[k + 1]]
        ok = (pat[0][1].startswith('ldr') and pat[0][2].endswith('#144]') and pat[1][1].startswith('sub') and
              pat[2][1].startswith('cmp') and pat[3][1] == 'it' and pat[3][2] in ('cc', 'lo') and
              pat[4][1].startswith(('bcs', 'bhs')))
        if not ok:
            return None
        t = re.search(r'([0-9a-f]+) <', pat[4][2])
        return int(t.group(1), 16) if t else None

    def reaches(tgt, fns, steps=16):
        k2, n_ = idx.get(tgt), 0
        while k2 is not None and n_ < steps:
            x = allins[k2]
            n_ += 1
            if x[1] in ('bl', 'b', 'b.n', 'b.w') and any(fa <= int(x[2].split()[0], 16) < fa + fs for fa, fs in fns):
                return True                     # a call, or a tail call (b.w after the epilogue)
            if x[1] in ('b', 'b.n', 'b.w'):
                k2 = idx.get(int(x[2].split()[0], 16))
            else:
                k2 += 1
        return False

    toc_guarded = toc_guarded_fail = 0
    for a0, n, sz in tocfn:
        for k, (a, op, args) in enumerate(allins):
            if not (a0 <= a < a0 + sz):
                continue
            if op.startswith(('strcc', 'strlo')):
                tgt = guard_form(k)
                if tgt is None:
                    errors.append(f'stage 4: conditional store at 0x{a:08x} ({n}) is not the TOC_GUARD form')
                    continue
                toc_guarded += 1
                if n.startswith(('buck_toc_finish', 'toc_pass')):
                    if reaches(tgt, fail_fn):
                        toc_guarded_fail += 1
                    else:
                        errors.append(f'stage 4: guarded store at 0x{a:08x} ({n}) does not reach toc_fail')
    if toc_guarded < 3:
        errors.append(f'stage 4: {toc_guarded} TOC guarded writes found in the TOC functions (3 expected at least)')
    if toc_guarded_fail < 2:
        errors.append(f'stage 4: {toc_guarded_fail} guarded CMP1 writes of buck_toc_finish and toc_pass reach toc_fail (2 expected)')
    # d. the inline start in run_loop
    swu = []
    for k, (a, op, args) in enumerate(body):
        if op.startswith(('strcc', 'strlo')) and '#156]' not in args and '[sp' not in args:
            kk = idx.get(a)
            if kk is not None and guard_form(kk) is not None:
                swu.append(k)
    n_swu_ok = 0
    for k in swu:
        # the stored register holds #3 (MSWU | TASWU): a mov of #3 into it within 12 instructions before
        reg = body[k][2].split(',')[0].strip()
        if any(body[j][1].startswith('mov') and body[j][2].replace(' ', '') == f'{reg},#3' for j in range(max(0, k - 12), k)):
            n_swu_ok += 1
        else:
            errors.append(f'stage 4: guarded store at 0x{body[k][0]:08x} in run_loop is not a software update (#3)')
    if n_swu_ok < 4:
        errors.append(f'stage 4: {n_swu_ok} guarded software updates in run_loop (4 expected at least: 2 per order)')
    sqrts = [k for k, (a, op, args) in enumerate(body) if op.startswith('vsqrt')]
    forced = [k for k, (a, op, args) in enumerate(body)
              if op.startswith('str') and not op.startswith('strd') and re.search(r'#(188|192)\]$', args)]
    if len(sqrts) != 2:
        errors.append(f'stage 4: {len(sqrts)} square roots in run_loop (2 expected: one solve per order)')
    # the OFF-ON tail compare (31 Oct): one, and a guarded software update follows it (unconditional
    # branches followed) within 48 instructions
    bidx = {a: k for k, (a, _, _) in enumerate(body)}
    swu_set = set(swu)

    def to_swu(k, steps=48):
        n_ = 0
        while k is not None and k < len(body) and n_ < steps:
            if k in swu_set:
                return True
            op, args = body[k][1], body[k][2]
            if op in ('b', 'b.n', 'b.w'):
                k = bidx.get(int(args.split()[0], 16))
            else:
                k += 1
            n_ += 1
        return False
    if len(toc_tails) != 1:
        errors.append(f'stage 4: {len(toc_tails)} OFF-ON tail compares (CMP1, then RST1R = CMP1) in run_loop (1 expected)')
    for i in toc_tails:
        if not to_swu(i):
            errors.append(f'stage 4: the tail compare at 0x{body[i][0]:08x} is not followed by a guarded software update')
    for q in sqrts:
        before = [k for k in forced if q - 40 <= k < q]
        after_swu = [k for k in swu if q < k <= q + 60]
        if not before:
            errors.append(f'stage 4: the square root at 0x{body[q][0]:08x} has no forced output store (SET1R/RST1R) '
                          f'in the 40 instructions before it: the span would wait for the solve')
        if not after_swu:
            errors.append(f'stage 4: no software update within 60 instructions after the square root at 0x{body[q][0]:08x}')
    fin = {syms[n][0] & ~1 for n in syms if n == 'buck_toc_finish'}
    ret = {syms[n][0] & ~1 for n in syms if n == 'buck_toc_ret'}
    calls = [(k, int(args.split()[0], 16)) for k, (a, op, args) in enumerate(body) if op == 'bl']
    n_fin = sum(1 for k, t in calls if t in fin)
    n_ret = sum(1 for k, t in calls if t in ret)
    for k, t in calls:
        if t in fin:
            if not (k >= 2 and body[k - 2][1].startswith('stmdb') and body[k - 1][1] == 'vpush'):
                errors.append(f'stage 4: bl at 0x{body[k][0]:08x} into the TOC code is not a BUCK_TOC_CALL')
        if t in ret:
            if not (k >= 1 and body[k - 1][1].startswith(('stmdb', 'push')) and
                    all(r in body[k - 1][2] for r in ('r0', 'r1', 'r2', 'r3', 'ip', 'lr'))):
                errors.append(f'stage 4: bl at 0x{body[k][0]:08x} into buck_toc_ret is not a BUCK_TOC_CALL_CORE')
            if not any(k - 12 <= j < k for j in swu):
                errors.append(f'stage 4: the call of buck_toc_ret at 0x{body[k][0]:08x} does not follow a software '
                              f'update within 12 instructions (the return words\' path)')
    for n_ in ('buck_toc_ret',):
        if n_ in syms:
            a0, sz = syms[n_][0] & ~1, syms[n_][1]
            for a, op, args in allins:
                if a0 <= a < a0 + sz and (op.startswith(('vpush', 'vpop', 'vmrs', 'vmsr')) or
                                         re.search(r'\b[sd]\d+\b', args)):
                    errors.append(f'stage 4: {n_} uses a floating-point register at 0x{a:08x} ({op} {args}): '
                                  f'BUCK_TOC_CALL_CORE does not save them')
    if n_fin < 1:
        errors.append('stage 4: run_loop does not call buck_toc_finish')
    if n_ret < 2:
        errors.append(f'stage 4: run_loop calls buck_toc_ret {n_ret} times (2 expected: one per order)')
    # e. no call from the TOC code out of CCM (buck_fatal, the no-return fault path, excepted)
    fatal_addrs = {syms[n][0] & ~1 for n in syms if n in ('buck_fatal', '__buck_fatal_veneer')}
    for a0, n, sz in tocfn:
        for a, op, args in allins:
            if a0 <= a < a0 + sz and op in ('bl', 'blx') and args:
                t = re.match(r'([0-9a-f]+)', args)
                if not t:
                    continue
                tgt = int(t.group(1), 16)
                if not (CCM_LO <= tgt < CCM_HI) and tgt not in fatal_addrs:
                    errors.append(f'stage 4: {n} at 0x{a:08x} calls 0x{tgt:08x} outside CCM ({args})')

    for e in errors:
        print('check_loop: FAIL:', e)
    if not errors:
        print(f'check_loop: ok ({loop} {size} B in CCM, {guarded} guarded CMP1 writes'
              f'{" and the arming write" if arm is not None else ""}, '
              f'no fused multiply-add, all {nt} tunables read at the loop entry (up to the first ADC12_CSR read: the '
              f'arming wait\'s) and in the passes, obs_beta and obs_gamma not on the path of an ordinary pass ({nloops} loops & 31 tests); '
              f'stage 4: {len(tocfn)} TOC functions in CCM, {toc_guarded} TOC guarded writes ({toc_guarded_fail} CMP1 to '
              f'toc_fail), {n_swu_ok} guarded software updates in the inline start, the high side forced before each '
              f'of its {len(sqrts)} solves, {len(toc_tails)} OFF-ON tail compare, the return words right after the second '
              f'update without FP saves, no call out of CCM)')
    return 1 if errors else 0


if __name__ == '__main__':
    sys.exit(main())
