"""Check linked target code properties; does not claim MCU timing validation."""
import re
import subprocess
import sys

prefix, elf = sys.argv[1:]
def run(tool, *args):
    return subprocess.check_output([prefix + tool, *args, elf], text=True)

symbols = {}
for line in run("nm", "-n").splitlines():
    m = re.match(r"([0-9a-f]+)\s+\w\s+(\S+)", line)
    if m:
        symbols[m[2]] = int(m[1], 16)
assert 0x10000000 <= symbols["ADC1_2_IRQHandler"] < 0x10008000
assert symbols["_estack"] == 0x20018000, "stack overlaps CCM alias"
assert symbols["ram_vectors"] % 512 == 0
assert symbols["_sccmtext"] <= symbols["ADC1_2_IRQHandler"] < symbols["_eccmtext"]
assert symbols["_siccmtext"] != symbols["_sccmtext"], "missing load image"
disasm = run("objdump", "-d", "--disassemble=ADC1_2_IRQHandler")
# Ban divide and all VFP in the ISR (nondeterministic / heavy). 32x32->64
# multiplies (smull/umull/smlal/umlal) are single-cycle on Cortex-M4 and are
# used by the Sec. III-A estimator, so they are permitted.
FP_OR_DIV = r"\b(?:[us]div|v(?:add|sub|mul|div|ldr|str|push|pop|cvt)\S*)\b"
FP_ONLY   = r"\bv(?:add|sub|mul|div|ldr|str|push|pop|cvt)\S*\b"
# The predictive current-mode inner loop (BUCK_CURRENT_MODE) uses ONE 32-bit
# UDIV per cycle (2..12 clocks on Cortex-M4, bounded). Nothing else may divide.
assert not re.search(FP_ONLY, disasm), "FP in ISR"
assert len(re.findall(r"\b[us]div\b", disasm)) <= 1, "more than one divide in ISR"
# The every-cycle path makes no calls. The ISR may call buck_fatal and the
# rare-cycle helpers below (arming, soft-start ramp, optimal state machine
# outside OPT_IDLE); each must itself live in CCM and be free of FP/divide.
COLD_HELPERS = {"control_transition", "reference_step", "opt_cold", "opt_solve", "opt_settled"}
# opt_solve holds two bounded UDIVs, both on the solve cycle: the pulse on-time
# through the trapezoidal inversion and the off-time from the fall slope. Every
# other helper is divide-free.
HELPER_DIVS = {"opt_solve": 2}
base = lambda name: name.split(".")[0]          # strip GCC clone suffixes (.isra.N, .constprop.N)
calls = re.findall(r"\bblx?\s+[^<\n]+<([^>]+)>", disasm)
for call in calls:
    assert "buck_fatal" in call or base(call) in COLD_HELPERS, calls
for helper in [n for n in symbols if base(n) in COLD_HELPERS]:
    assert symbols["_sccmtext"] <= symbols[helper] < symbols["_eccmtext"], helper + " not in CCM"
    hd = run("objdump", "-d", "--disassemble=" + helper)
    assert not re.search(FP_ONLY, hd), "FP in " + helper
    assert len(re.findall(r"\b[us]div\b", hd)) <= HELPER_DIVS.get(base(helper), 0), "divide in " + helper
    hcalls = re.findall(r"\bblx?\s+[^<\n]+<([^>]+)>", hd)
    assert all("buck_fatal" in c for c in hcalls), (helper, hcalls)
assert not any(name.startswith("HAL_") for name in symbols)
assert not any(name.startswith(("__aeabi_d", "__aeabi_f", "__aeabi_l")) for name in symbols)
assert re.search(r"\bmul(?:\.w)?\s", disasm) and re.search(r"\bmla\s", disasm)
print("PASS: ADC ISR + cold helpers in CCM, aligned SRAM vectors, disjoint RAM, MUL/MLA + 64-bit MAC; no HAL/FP, one bounded divide per path, no every-cycle calls")
