"""Execute the linked Thumb ISR against mocked registers (NOT cycle accurate).
Requires: unicorn, pyelftools. Usage: python Tests/test_target.py build/*.elf
Checks actual compiled control branches, writes and fault shutdown, rather
than treating arithmetic unit tests as validation of the whole handler.
"""
import random
import struct
import sys
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn import UC_HOOK_CODE, UC_HOOK_MEM_WRITE, UC_HOOK_MEM_READ
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR

with open(sys.argv[1], "rb") as f:
    elf = ELFFile(f)
    syms = {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
    sections = [(s['sh_addr'], s.data()) for s in elf.iter_sections()
                if s['sh_flags'] & 2 and s['sh_type'] != 'SHT_NOBITS']
    ccm_image = elf.get_section_by_name('.ccmtext').data()

ADC = 0x50000000
CNT = 0x40016890
CMP = 0x4001689c
CR2 = 0x40016b84
OEN = 0x40016b94
ODIS = 0x40016b98
RETURN = 0x0807fff0
u = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
for base, size in [(0x08000000, 0x80000), (0x10000000, 0x8000),
                   (0x20000000, 0x18000), (0x40000000, 0x30000),
                   (0x48000000, 0x2000), (0x50000000, 0x1000),
                   (0xe0000000, 0x100000)]:
    u.mem_map(base, size)
for address, data in sections:
    u.mem_write(address, data)
# Exercise the actual initialization copy, including far-call veneers.
u.mem_write(syms['_siccmtext'], ccm_image)
u.mem_write(syms['_sccmtext'], bytes(len(ccm_image)))

def wr(addr, val): u.mem_write(addr, struct.pack('<I', val & 0xffffffff))
def rd(addr): return struct.unpack('<I', u.mem_read(addr, 4))[0]
def status(i): return rd(syms['g_buck_status'] + 4*i)

writes = []
count = 0
write_instruction = None
phases = []
booting = True
fake_clock = 0
def on_code(uc, addr, size, data):
    global count
    if addr == RETURN:
        uc.emu_stop()
        return
    count += 1
    if status(1) and status(0) == 2 and uc.mem_read(addr, 2) == b'\x00\xbf':
        uc.emu_stop()

def on_write(uc, access, address, size, value, data):
    global write_instruction
    writes.append((address, value))
    if address == CMP: write_instruction = count

def on_read(uc, access, address, size, value, data):
    global fake_clock
    if booting:
        if address == 0x40021000: wr(address, rd(address) | (1 << 10) | (1 << 17) | (1 << 25))
        if address == 0x40021008: wr(address, (rd(address) & ~12) | ((rd(address) & 3) << 2))
        if address == 0x40016b88: wr(address, 1 << 16) # DLL ready
        if address == ADC + 8: wr(address, rd(address) & ~(1 << 31)) # ADC calibration completes
        if address == ADC and rd(ADC+8) & 1: wr(address, rd(address) | 1)
    if address == 0xe0001004:
        fake_clock += 64
        wr(address, fake_clock)
    if address == ADC + 0x40: wr(ADC, rd(ADC) & ~4) # EOC clear on DR read
    if address == CNT: wr(CNT, phases.pop(0) if phases else 7000)

u.hook_add(UC_HOOK_CODE, on_code)
u.hook_add(UC_HOOK_MEM_WRITE, on_write)
u.hook_add(UC_HOOK_MEM_READ, on_read)
def call(name):
    u.reg_write(UC_ARM_REG_SP, 0x20017000)
    u.reg_write(UC_ARM_REG_LR, RETURN | 1)
    u.emu_start(syms[name] | 1, RETURN, count=100000)
    assert status(1) == 0, f'{name}: initialization fault {status(1)}'

wr(0xe0042004, 1 << 5) # preserve other debug control settings
wr(0xe000ed10, (1 << 2) | (1 << 1)) # clear inherited deep/sleep-on-exit
call('buck_init')
assert rd(0xe0042004) == (1 << 5) | 1, 'debug AHB clock must survive Sleep'
assert rd(0xe000ed10) & 6 == 0, 'initialization must leave deep sleep disabled'
assert u.mem_read(syms['_sccmtext'], len(ccm_image)) == ccm_image
assert rd(0xe000ed08) == syms['ram_vectors']
assert rd(syms['ram_vectors'] + (16+18)*4) == syms['ADC1_2_IRQHandler']
assert rd(0x40016894) == 8960 and rd(0x40016898) == 0
assert rd(CMP) == 896, 'startup compare must be 10%'
assert rd(0x400168bc) == 4 and rd(0x400168c0) == 8 # set PER, reset CMP1
assert rd(0x400168b8) & 511 == 68 and (rd(0x400168b8) >> 16) & 511 == 68
assert rd(0x400168e4) & (1 << 8) # hardware DTEN
assert rd(0x400168ec) == 0 # no greater-than / up-down erratum modes
assert rd(0x40016bbc) == (1 << 11) # ADCTRG1 on timer A CMP3
assert (rd(ADC+12) >> 5) & 31 == 21 and (rd(ADC+12) >> 10) & 3 == 1
assert rd(ADC+16) == 1 << 26 # bulb, no oversampling
assert (rd(ADC+0x30) >> 6) & 31 == 1 # regular rank 1 = channel 1
assert rd(ADC+4) == (1 << 2) | (1 << 4) # EOC and OVR interrupts only
assert rd(0x48000024) & 255 == 0xdd # PA8/PA9 AF13
assert (OEN, 3) not in writes
call('buck_start_sampling')
assert rd(ADC+8) & 4 and rd(0x40016800) & (1 << 17)
booting = False

def irq(adc, flags=4, counter=(6000, 7000)):
    global writes, count, write_instruction, phases
    writes, count, write_instruction, phases = [], 0, None, list(counter)
    wr(ADC, flags)
    wr(ADC+0x40, adc)
    u.reg_write(UC_ARM_REG_SP, 0x20017000)
    u.reg_write(UC_ARM_REG_LR, RETURN | 1)
    u.emu_start(syms['ADC1_2_IRQHandler'] | 1, RETURN, count=2000)
    assert count < 2000, 'handler failed to return or latch a fault'
    return [v for a, v in writes if a == CMP]

wr(syms['g_buck_command'], 0)
for _ in range(8): assert not irq(4095) # discard warm-up even if result invalid
irq(4095)
assert status(0) == 0 and status(1) == 0 and status(3) == 0
wr(syms['g_buck_command'], 1)
assert irq(4095) == [896] and status(0) == 1 and status(1) == 0
assert status(3) == 896, 'arm at 10% even with full-scale A0'
assert (OEN, 3) in writes
saved = bytes(u.mem_read(0x20000000, 0x18000))

# Exact one compare store per valid armed sample, all arithmetic regimes.
rng = random.Random(474600)
maximum = 0
samples = [0, 26, 27, 512, 614, 615, 616, 4095] + list(range(4096)) + [rng.randrange(4096) for _ in range(1000)]
for adc in samples:
    result = irq(adc)
    assert len(result) == 1 and 896 <= result[0] <= 3584
    assert status(1) == 0
    assert status(2) == adc and status(3) == result[0]
    assert all(a != CR2 for a, _ in writes), 'unsafe software update in running ISR'
    maximum = max(maximum, write_instruction)

wr(syms['g_buck_command'], 0)
irq(100)
assert status(0) == 0 and status(3) == 0 and (ODIS, 3) in writes

def restore(): u.mem_write(0x20000000, saved)
def check_fault(adc, expected, flags=4, counter=(6000, 7000), before_write=True):
    restore()
    result = irq(adc, flags, counter)
    assert status(1) == expected and status(0) == 2
    if before_write: assert not result
    assert (ODIS, 3) in writes
    assert (rd(0x48000000) >> 16) & 15 == 5, 'both gates must revert to GPIO output-low'
    assert (0x48000018, 0x03000000) in writes

check_fault(0, 4, flags=4 | 16)
check_fault(0, 5, counter=(0,))
check_fault(0, 5, counter=(8448,))
check_fault(0, 5, counter=(8430, 0), before_write=False)
# Former prebias/overvoltage thresholds must not inhibit arming/restarting.
for adc in [0, 27, 512, 615, 4095]:
    restore()
    wr(syms['g_buck_command'], 0)
    irq(adc)
    wr(syms['g_buck_command'], 1)
    assert irq(adc) == [896] and status(0) == 1 and status(1) == 0
    assert (OEN, 3) in writes

# Exercise the linked PI at both saturation rails, including recovery.
restore()
wr(syms['control'] + 8, 512 << 16) # completed reference ramp
wr(syms['control'] + 12, 3584 << 16)
assert irq(0) == [3584], 'positive error must clip to exactly 40%'
for _ in range(1000):
    assert 896 <= irq(4095)[0] <= 3584 and status(1) == 0
assert irq(4095) == [896], 'negative error must clip to exactly 10%'
assert irq(0)[0] > 896, 'integrator must recover from low saturation'

print(f'PASS: linked ARM ISR, {len(samples)} samples/all 12-bit A0 codes/one write each, 10..40% saturation and recovery, A0-independent arm/stop, OVR/deadline shutdown; max {maximum} executed instructions to compare store (not cycles)')
