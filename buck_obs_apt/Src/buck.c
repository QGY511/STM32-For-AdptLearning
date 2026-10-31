/* NUCLEO-G474RE buck converter: peripherals and the 600 kHz control loop.
 * Register-level (CMSIS only, no HAL). See Inc/buck_config.h for pins,
 * timing and parameters and Inc/buck_ctrl.h for the per-cycle algorithm. */

#include "stm32g4xx.h"
#include <stddef.h>
#include "buck.h"
#include "buck_toc_fw.h"
#include "buck_toc_start.h"

#define TIMER_A       (HRTIM1->sTimerxRegs[0])
#define HRTIM_MASTER  (HRTIM1->sMasterRegs)
#define HRTIM_COMMON  (HRTIM1->sCommonRegs)
#define GATE_PINS     ((1U << 8) | (1U << 9))   /* PA8 = D7 (HS), PA9 = D8 (LS) */
#define D4_PIN        (1U << 5)                 /* PB5 = D4 */
_Static_assert(D4_PIN == 32U, "run_pass takes g from bit 5 of the BSRR word");
/* The control loop and its literal pool run from zero-wait CCM SRAM. */
#define FAST_CODE     __attribute__((section(".ccmtext"), noinline, optimize("O3")))
#define COLD_CODE     __attribute__((section(".ccmtext"), noinline, cold))
/* run_loop() was reached only through the cold arm() (until 25 Oct; now buck_run()
 * calls it directly), so GCC's static profile marked it "unlikely executed" and
 * optimised it for SIZE despite optimize("O3") (block order with the exits
 * inline, no hoisting of constants, extra spills). hot restores the speed
 * heuristics (kept: the passes' register allocation was chosen with it).
 * -fno-if-conversion -fno-crossjumping -fno-schedule-insns, with the copy
 * order in run_loop(): the register allocation that measured best for this
 * loop body (28 Oct: 35 flag sets x all 24 copy orders, 840 builds, counted
 * with both clock counters, static and nominal, on the CMP1 write after EOC,
 * the normal pass and its busy_max window, the longest pass, the first pass
 * and the arming path: no worse than the 25 Oct build on any of them, and the
 * best on the most). (25 Oct to 28 Oct, the build that ran on the board on
 * 27 Oct: -fno-tree-coalesce-vars -fno-tree-ter, from 208 flag sets and all 24
 * copy orders on that body.) */
#define RUN_CODE      __attribute__((section(".ccmtext"), noinline, hot, optimize("O3", "no-if-conversion", "no-crossjumping", "no-schedule-insns")))
#define LIKELY(x)     __builtin_expect(!!(x), 1)
#define UNLIKELY(x)   __builtin_expect(!!(x), 0)

/* 1: bulb sampling on both ADCs: the input is tracked from the end of the
 * previous conversion. 0: standard sampling, tracked from the trigger. In
 * both, the sample is held 2.5 ADC clocks (the SMP setting) after the trigger
 * (RM0440 Figure 93). */
#ifndef BUCK_ADC_BULB
#define BUCK_ADC_BULB 1
#endif

/* 1: the loop polls the ADC1/ADC2 common status register (ADC12_CSR), whose
 * single read holds ADC1's end of conversion (the pacing event) and both
 * ADCs' overrun flags and ADC2's end of conversion: one bus read per pass
 * instead of two. 0: poll ADC1_ISR, then read ADC2_ISR (untested fallback if
 * the common register does not mirror the flags in independent mode). */
#ifndef BUCK_ADC_CSR_POLL
#define BUCK_ADC_CSR_POLL 1
#endif

buck_t g_buck = {
    .command = BUCK_STOP,
    .state = BUCK_STOPPED,
    .tune = BUCK_TUNE_DEFAULTS,
    .warmup = BUCK_WARMUP_CYCLES,
    .toc = { .toc_enable = BUCK_TOC_ENABLE_DEFAULT, .learn_enable = BUCK_LEARN_ENABLE_DEFAULT,
             .g = { { 1.0f, 1.0f }, { 1.0f, 1.0f } } },
};

/* HRTIM update configuration (stage 2, user decision 6 and RM0440 item 2, stage1/rm0440.txt):
 * the master's registers (MPER, MCMPx) and the ADC trigger registers (update source:
 * master) are preloaded and transfer at every master roll-over (MREPU, MREP = 0), and
 * Timer A transfers its preloads at the master update (MSTU) instead of its own roll-over,
 * so the TOC frame's period, compares and output sources change in both timers in one
 * update event (stage 4: the frame's end; its start is a software update of both, CR2
 * MSWU | TASWU, Src/buck_toc.c), and the two counters stay locked. With equal periods (always: normal
 * periods 8960, a frame PER' = MPER') both are the same roll-over as in
 * Buck_disturb_obs, where Timer A updated on its own roll-over (TREPU) and the master
 * had no preload; the CMP1 duty preload is therefore taken at the same instant
 * (README bench test T2). Constants (also read by the emulator from the ELF). */
__attribute__((used)) const uint32_t buck_hw_mcr_cfg = HRTIM_MCR_CONT | HRTIM_MCR_PREEN | HRTIM_MCR_MREPU;
__attribute__((used)) const uint32_t buck_hw_timacr_cfg = HRTIM_TIMCR_CONT | HRTIM_TIMCR_PREEN | HRTIM_TIMCR_MSTU;

/* Vector table in SRAM1: the vector fetch does not wait on flash. */
static uint32_t ram_vectors[128] __attribute__((aligned(512)));
extern uint32_t g_pfnVectors[];
extern uint32_t _siccmtext, _sccmtext, _eccmtext;

static void wait_mask(volatile uint32_t *reg, uint32_t mask, uint32_t value, uint32_t fault)
{
    uint32_t timeout = 2000000U;
    while ((*reg & mask) != value) {
        if (--timeout == 0U) buck_fatal(fault);
    }
}

static void delay_cycles(uint32_t cycles)
{
    const uint32_t start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) { __NOP(); }
}

/* HSI16 /4 x84 = 336 MHz VCO; R /2 = 168 MHz SYSCLK; P /6 = 56 MHz ADC. */
static void clock_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    (void)RCC->APB1ENR1;
    PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS) | PWR_CR1_VOS_0;
    wait_mask(&PWR->SR2, PWR_SR2_VOSF, 0U, BUCK_FAULT_CLOCK);
    PWR->CR5 &= ~PWR_CR5_R1MODE;            /* Range 1 boost, required above 150 MHz */
    delay_cycles(160U);                     /* > 5 us at the 16 MHz reset clock */
    /* Read-modify-write: FLASH_ACR.DBG_SWEN (bit 18) gates the SWD port. */
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_4WS |
                 FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN;
    wait_mask(&FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_4WS, BUCK_FAULT_CLOCK);
    RCC->CR |= RCC_CR_HSION;
    wait_mask(&RCC->CR, RCC_CR_HSIRDY, RCC_CR_HSIRDY, BUCK_FAULT_CLOCK);
    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSI | ((4U - 1U) << RCC_PLLCFGR_PLLM_Pos) |
                   (84U << RCC_PLLCFGR_PLLN_Pos) | RCC_PLLCFGR_PLLREN |
                   RCC_PLLCFGR_PLLPEN | (6U << RCC_PLLCFGR_PLLPDIV_Pos);   /* BUCK_ADC_HZ: the A0
                                                  trigger spacing (BUCK_VSAMPLE_GAP_TICKS) is in these clocks */
    RCC->CR |= RCC_CR_PLLON;
    wait_mask(&RCC->CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY, BUCK_FAULT_CLOCK);
    /* Switch through AHB /2 first (limits the regulator step), then /1. */
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_SW | RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) |
                RCC_CFGR_HPRE_DIV2 | RCC_CFGR_SW_PLL;
    wait_mask(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL, BUCK_FAULT_CLOCK);
    delay_cycles(168U);                     /* 2 us at 84 MHz */
    RCC->CFGR &= ~RCC_CFGR_HPRE;
    SystemCoreClock = BUCK_CPU_HZ;
    RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_ADC12SEL) | RCC_CCIPR_ADC12SEL_0;   /* ADC12 kernel = PLLP */
    SysTick->CTRL = 0U;
}

static void gpio_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
    (void)RCC->AHB2ENR;
    /* PA8/PA9: GPIO outputs driven low until HRTIM owns them; pull-downs. */
    GPIOA->BSRR = GATE_PINS << 16;
    GPIOA->OTYPER &= ~GATE_PINS;
    GPIOA->PUPDR = (GPIOA->PUPDR & ~((3U << 16) | (3U << 18) | (3U << 0) | (3U << 2) | (3U << 8))) |
                   (2U << 16) | (2U << 18);
    GPIOA->OSPEEDR |= (3U << 16) | (3U << 18);
    /* PA0 (A0), PA1 (A1), PA4 (A2): analog. */
    GPIOA->MODER = (GPIOA->MODER & ~((3U << 16) | (3U << 18))) |
                   (1U << 16) | (1U << 18) | (3U << 0) | (3U << 2) | (3U << 8);
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~0xFFU) | 0xDDU;   /* PA8, PA9: AF13 (selected later) */
    /* PB5 (D4): push-pull output, low. */
    GPIOB->BSRR = D4_PIN << 16;
    GPIOB->OTYPER &= ~D4_PIN;
    GPIOB->OSPEEDR |= 3U << 10;
    GPIOB->MODER = (GPIOB->MODER & ~(3U << 10)) | (1U << 10);
}

/* Timer A: PWM on CHA1 (high side) with CHA2 derived by the dead-time unit.
 * Master timer: same period, compares at T/8, 3T/8, 5T/8, 7T/8 for the
 * current samples. Both counters start in the same MCR write. */
static void hrtim_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_HRTIM1EN;
    (void)RCC->APB2ENR;
    RCC->APB2RSTR |= RCC_APB2RSTR_HRTIM1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_HRTIM1RST;
    HRTIM_COMMON.ODISR = HRTIM_ODISR_TA1ODIS | HRTIM_ODISR_TA2ODIS;
    HRTIM_COMMON.DLLCR = HRTIM_DLLCR_CALEN | HRTIM_DLLCR_CALRTE;
    wait_mask(&HRTIM_COMMON.ISR, HRTIM_ISR_DLLRDY, HRTIM_ISR_DLLRDY, BUCK_FAULT_DLL);

    /* Master: continuous, x32 (CKPSC = 0); preload enabled below, after the ADC trigger
     * registers are written (with PREEN = 0 they take the value at once). */
    HRTIM_MASTER.MCR = HRTIM_MCR_CONT;
    HRTIM_MASTER.MPER = BUCK_HRTIM_PERIOD;
    HRTIM_MASTER.MREP = 0U;
    HRTIM_MASTER.MCMP1R = BUCK_ISAMPLE_CMP(0U);
    HRTIM_MASTER.MCMP2R = BUCK_ISAMPLE_CMP(1U);
    HRTIM_MASTER.MCMP3R = BUCK_ISAMPLE_CMP(2U);
    HRTIM_MASTER.MCMP4R = BUCK_ISAMPLE_CMP(3U);

    /* Timer A: continuous, x32, preloaded, update at every master update (MSTU; REP = 0). */
    TIMER_A.TIMxCR = *(volatile const uint32_t *)&buck_hw_timacr_cfg;   /* read from flash: the symbol stays */
    TIMER_A.PERxR = BUCK_HRTIM_PERIOD;
    TIMER_A.REPxR = 0U;
    TIMER_A.CMP1xR = BUCK_CMP_MIN;
    TIMER_A.CMP2xR = BUCK_TOC_CMP2_BACKSTOP;    /* a no-op reset (below); a TOC frame's first span end */
    TIMER_A.CMP3xR = BUCK_VSAMPLE_CMP;          /* 2nd v_out sample */
    TIMER_A.CMP4xR = BUCK_VSAMPLE0_CMP;         /* 1st v_out sample */
    TIMER_A.SETx1R = HRTIM_SET1R_PER;           /* reference high at the period start */
    TIMER_A.RSTx1R = HRTIM_RST1R_CMP1 | HRTIM_RST1R_CMP2;   /* low at CMP1; CMP2 the TOC's backstop
                                                                (stage 4: the high side is off by then) */
    TIMER_A.SETx2R = 0U;                        /* CHA2 comes from the dead-time unit */
    TIMER_A.RSTx2R = 0U;
    TIMER_A.DTxR = BUCK_DT_COUNTS | (BUCK_DT_COUNTS << HRTIM_DTR_DTF_Pos) |
                   HRTIM_DTR_DTRSLK | HRTIM_DTR_DTRLK | HRTIM_DTR_DTFSLK | HRTIM_DTR_DTFLK;
    TIMER_A.OUTxR = HRTIM_OUTR_DTEN | HRTIM_OUTR_IDLM1 | HRTIM_OUTR_IDLM2 |
                    HRTIM_OUTR_FAULT1_1 | HRTIM_OUTR_FAULT2_1;
    TIMER_A.TIMxDIER = 0U;

    /* ADC trigger 1 (ADC1, v_out): Timer A CMP4 and CMP3. ADC trigger 3
     * (ADC2, i_L): Master CMP1..4. Update source: master (not preloaded). */
    HRTIM_COMMON.CR1 = 0U;
    HRTIM_COMMON.ADC1R = HRTIM_ADC1R_AD1TAC4 | HRTIM_ADC1R_AD1TAC3;
    HRTIM_COMMON.ADC3R = HRTIM_ADC3R_AD3MC1 | HRTIM_ADC3R_AD3MC2 |
                         HRTIM_ADC3R_AD3MC3 | HRTIM_ADC3R_AD3MC4;
    HRTIM_COMMON.ADCPS1 = 0U;
    HRTIM_MASTER.MCR = *(volatile const uint32_t *)&buck_hw_mcr_cfg;   /* master preload + update at the roll-over */
    /* Transfer every preload to the active registers and zero both counters. */
    HRTIM_COMMON.CR2 = HRTIM_CR2_MSWU | HRTIM_CR2_TASWU | HRTIM_CR2_MRST | HRTIM_CR2_TARST;
    __DSB();
    /* Hand PA8/PA9 to HRTIM (outputs are still disabled: inactive = low). */
    GPIOA->MODER = (GPIOA->MODER & ~((3U << 16) | (3U << 18))) | (2U << 16) | (2U << 18);
}

/* 12-bit, one regular rank, rising-edge HRTIM trigger, OVRMOD (the data
 * register always holds the newest result; an overrun is flagged). */
static void adc_setup(ADC_TypeDef *a, uint32_t channel, uint32_t extsel, uint32_t cfgr2)
{
    a->CR = ADC_CR_ADVREGEN;
    delay_cycles(BUCK_CPU_HZ / 1000000U * 25U);     /* regulator start-up > 20 us */
    a->DIFSEL = 0U;
    a->CR |= ADC_CR_ADCAL;
    wait_mask(&a->CR, ADC_CR_ADCAL, 0U, BUCK_FAULT_ADC_START);
    delay_cycles(32U);
    a->SMPR1 = 0U;                                  /* 2.5 ADC clocks */
    a->SMPR2 = 0U;
    a->SQR1 = channel << ADC_SQR1_SQ1_Pos;          /* L = 0: one conversion */
    a->CFGR = ADC_CFGR_JQDIS | ADC_CFGR_OVRMOD | (extsel << ADC_CFGR_EXTSEL_Pos) | ADC_CFGR_EXTEN_0;
    a->CFGR2 = cfgr2;
    a->ISR = ADC_ISR_ADRDY | ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;
    a->CR |= ADC_CR_ADEN;
    wait_mask(&a->ISR, ADC_ISR_ADRDY, ADC_ISR_ADRDY, BUCK_FAULT_ADC_START);
}

/* ADC1: A0 = PA0 = ADC1_IN1 on HRTIM ADC trigger 1 (EXTSEL 21), triggered
 * 2x oversampling without shift: one result per switching cycle = sum of the
 * two samples near the end of the cycle.
 * ADC2: A1 = PA1 = ADC2_IN2 on HRTIM ADC trigger 3 (EXTSEL 22), triggered
 * 4x oversampling without shift: one result per switching cycle = sum of the
 * four equally spaced samples. ADC1's EOC paces the control loop.
 * Triggered oversampling (TROVS): each conversion waits for its own trigger.
 * RM0440 excludes bulb mode from continuous conversion mode and injected
 * conversions, and does not describe bulb timing for back-to-back
 * oversampled conversions; bulb + TROVS is what ADC2 has run on the board. */
static void adc_init(void)
{
    const uint32_t bulb = BUCK_ADC_BULB ? ADC_CFGR2_BULB : 0U;
    RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
    (void)RCC->AHB2ENR;
    RCC->AHB2RSTR |= RCC_AHB2RSTR_ADC12RST;
    RCC->AHB2RSTR &= ~RCC_AHB2RSTR_ADC12RST;
    ADC12_COMMON->CCR = 0U;                         /* independent, asynchronous kernel clock /1 */
    adc_setup(ADC1, 1U, 21U, bulb | ADC_CFGR2_ROVSE | ADC_CFGR2_TROVS |
                             (0U << ADC_CFGR2_OVSR_Pos) | (0U << ADC_CFGR2_OVSS_Pos));
    adc_setup(ADC2, 2U, 22U, bulb | ADC_CFGR2_ROVSE | ADC_CFGR2_TROVS |
                             (1U << ADC_CFGR2_OVSR_Pos) | (0U << ADC_CFGR2_OVSS_Pos));
    ADC1->IER = 0U;                                 /* polled by buck_run(), no interrupt */
    ADC2->IER = 0U;
}

/* DAC1 channel 1 on PA4 (A2): normal mode, output buffer on, software
 * updates (no trigger: DOR follows DHR one clock after the write). */
static void dac_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_DAC1EN;
    (void)RCC->AHB2ENR;
    RCC->AHB2RSTR |= RCC_AHB2RSTR_DAC1RST;
    RCC->AHB2RSTR &= ~RCC_AHB2RSTR_DAC1RST;
    DAC1->MCR = DAC_MCR_HFSEL_1;                    /* AHB > 160 MHz interface mode */
    DAC1->DHR12R1 = 0U;
    DAC1->CR = DAC_CR_EN1;
    wait_mask(&DAC1->SR, DAC_SR_DAC1RDY, DAC_SR_DAC1RDY, BUCK_FAULT_DAC);
}

/* ---- Boot-time bus benchmark ----------------------------------------------
 * Times 16 unrolled accesses with DWT->CYCCNT, for each register the control
 * loop touches, against the same 16 accesses on SRAM1, and stores the extra
 * clocks per access in g_buck.bench[] (Inc/buck.h lists the entries). The
 * core-only cycle model has no wait states; these are the numbers to add.
 * Reads are isolated (each load feeds an add; the loop's VLDRs of DWT_CYCCNT
 * and the ADC data registers are the same single-word bus reads). Writes are
 * back to back, so the result is how long one write keeps the bus busy; the
 * write-then-load rows give the stall of a load issued right after a write.
 * Runs once from buck_init(), from CCM like the loop, before buck_start():
 * interrupts are off, the counters and ADCs are not started, the gates are
 * disabled, and every write stores the value the register already holds
 * (CMP1 = BUCK_CMP_MIN, DAC = 0, PB5 reset), so it changes no state.
 * Build with -DBUCK_BENCH=0 to leave it out. */
#ifndef BUCK_BENCH
#define BUCK_BENCH 1
#endif
#if BUCK_BENCH
/* The benchmark's code sits in CCM in front of the control code, in a section
 * of its own: the linker script places .ccmtext and .ccmtext.* input sections
 * in the order this object declares them, and top-level asm is emitted before
 * any function. .ccmtext is 8-byte aligned so that everything in it (and the
 * buck_fatal veneer after it) moves by the same multiple of 8: the control
 * code and its literal pools stay byte for byte those of a build without the
 * benchmark (every branch between them is PC-relative). */
__asm__(".pushsection .ccmtext.bench,\"ax\",%progbits\n\t.popsection\n\t"
        ".pushsection .ccmtext,\"ax\",%progbits\n\t.balign 8\n\t.popsection");
#define BENCH_CODE    __attribute__((section(".ccmtext.bench"), noinline, cold))
enum { BENCH_RD, BENCH_WR, BENCH_WRLD };
static volatile uint32_t bench_sram[2];         /* the SRAM1 reference */

/* Clocks for 16 accesses of one kind to a (and b for the load after a write),
 * each run between two DSBs so posted writes are included; minimum of 8 runs.
 * The 16 copies are written out (not .rept): GCC sizes an asm statement by
 * its line count, and must see the real length to pick branches that reach
 * over it. The load's result is used at once (add), so loads do not pipeline.
 * At most 6 registers per asm: -O0 builds can allocate them. */
#define BENCH_X4(s)       s s s s
#define BENCH_X16(s)      BENCH_X4(BENCH_X4(s))
#define BENCH_TIMED(body) "dsb\n\tldr %[t], [%[dwt], #4]\n\t" BENCH_X16(body) \
                          "dsb\n\tldr %[x], [%[dwt], #4]\n\tsub %[t], %[x], %[t]"
static BENCH_CODE uint32_t bench_time(uint32_t kind, uintptr_t a, uintptr_t b, uint32_t v)
{
    uint32_t best = ~0U;
    for (uint32_t run = 0U; run < 8U; ++run) {
        uint32_t t, x;
        if (kind == BENCH_RD) {
            __asm__ volatile(BENCH_TIMED("ldr %[x], [%[a]]\n\tadd %[x], %[x], %[x]\n\t")
                             : [t] "=&r"(t), [x] "=&r"(x)
                             : [a] "r"(a), [dwt] "r"(DWT) : "memory");
        } else if (kind == BENCH_WR) {
            __asm__ volatile(BENCH_TIMED("str %[v], [%[a]]\n\t")
                             : [t] "=&r"(t), [x] "=&r"(x)
                             : [a] "r"(a), [v] "r"(v), [dwt] "r"(DWT) : "memory");
        } else {
            __asm__ volatile(BENCH_TIMED("str %[v], [%[a]]\n\tldr %[x], [%[b]]\n\tadd %[x], %[x], %[x]\n\t")
                             : [t] "=&r"(t), [x] "=&r"(x)
                             : [a] "r"(a), [b] "r"(b), [v] "r"(v), [dwt] "r"(DWT) : "memory");
        }
        if (t < best) best = t;
    }
    return best;
}

static BENCH_CODE void buck_bench(buck_t *bk)
{
    const uintptr_t sram = (uintptr_t)&bench_sram[0], sram2 = (uintptr_t)&bench_sram[1];
    const uintptr_t ccm = (uintptr_t)&buck_run & ~1U;          /* code words in CCM, read as data */
    const uintptr_t cmp1 = (uintptr_t)&TIMER_A.CMP1xR, dhr = (uintptr_t)&DAC1->DHR12R1;
    const uintptr_t bsrr = (uintptr_t)&GPIOB->BSRR;
    const uint32_t cmp1_v = TIMER_A.CMP1xR, dhr_v = DAC1->DHR12R1, bsrr_v = D4_PIN << 16;
    const uint32_t rd = bench_time(BENCH_RD, sram, 0U, 0U);
    const uint32_t wr = bench_time(BENCH_WR, sram, 0U, 0U);
    const uint32_t wrld = bench_time(BENCH_WRLD, sram2, sram, 0U);
    const uint32_t wrlit = bench_time(BENCH_WRLD, sram2, ccm, 0U);
    const struct { uint8_t idx, kind; uintptr_t a, b; uint32_t v, ref; } row[] = {
        { BUCK_BENCH_RD_ADC_CSR,       BENCH_RD,   (uintptr_t)&ADC12_COMMON->CSR, 0U, 0U, rd },
        { BUCK_BENCH_RD_ADC1_DR,       BENCH_RD,   (uintptr_t)&ADC1->DR, 0U, 0U, rd },
        { BUCK_BENCH_RD_ADC2_DR,       BENCH_RD,   (uintptr_t)&ADC2->DR, 0U, 0U, rd },
        { BUCK_BENCH_RD_DWT_CYCCNT,    BENCH_RD,   (uintptr_t)&DWT->CYCCNT, 0U, 0U, rd },
        { BUCK_BENCH_RD_HRTIM_CNT,     BENCH_RD,   (uintptr_t)&TIMER_A.CNTxR, 0U, 0U, rd },
        { BUCK_BENCH_WR_HRTIM_CMP1,    BENCH_WR,   cmp1, 0U, cmp1_v, wr },
        { BUCK_BENCH_WR_DAC_DHR,       BENCH_WR,   dhr, 0U, dhr_v, wr },
        { BUCK_BENCH_WR_GPIOB_BSRR,    BENCH_WR,   bsrr, 0U, bsrr_v, wr },
        { BUCK_BENCH_WRLD_HRTIM_CMP1,  BENCH_WRLD, cmp1, sram, cmp1_v, wrld },
        { BUCK_BENCH_WRLD_DAC_DHR,     BENCH_WRLD, dhr, sram, dhr_v, wrld },
        { BUCK_BENCH_WRLD_GPIOB_BSRR,  BENCH_WRLD, bsrr, sram, bsrr_v, wrld },
        { BUCK_BENCH_WRLIT_HRTIM_CMP1, BENCH_WRLD, cmp1, ccm, cmp1_v, wrlit },
        { BUCK_BENCH_RD_CCM,           BENCH_RD,   ccm, 0U, 0U, rd },
    };
    _Static_assert(sizeof row / sizeof row[0] == BUCK_BENCH_N, "one row per bench entry");
    for (uint32_t i = 0U; i < BUCK_BENCH_N; ++i) {
        const uint32_t c = bench_time(row[i].kind, row[i].a, row[i].b, row[i].v);
        bk->bench[row[i].idx] = (float)(int32_t)(c - row[i].ref) * (1.0f / 16.0f);
    }
}
#endif

void buck_init(void)
{
    __disable_irq();
    g_buck.rst_cause = RCC->CSR;
    RCC->CSR |= RCC_CSR_RMVF;
    gpio_init();
    clock_init();
    /* Control-loop code into CCM, vector table into SRAM1. */
    const uint32_t *src = &_siccmtext;
    for (uint32_t *dst = &_sccmtext; dst < &_eccmtext; ++dst) *dst = *src++;
    for (uint32_t i = 0U; i < 118U; ++i) ram_vectors[i] = g_pfnVectors[i];
    SCB->VTOR = (uint32_t)ram_vectors;
    __DSB();
    __ISB();
    hrtim_init();
    adc_init();
    dac_init();
#if BUCK_BENCH
    buck_bench(&g_buck);    /* once, before buck_start(): see buck_bench() */
#endif
}

/* Arm both ADCs BEFORE the counters start, so the first two voltage and the
 * first four current triggers form the first oversampling groups (the groups
 * stay aligned with the switching cycles from then on). */
void buck_start(void)
{
    ADC1->CR |= ADC_CR_ADSTART;
    ADC2->CR |= ADC_CR_ADSTART;
    HRTIM_MASTER.MCR |= HRTIM_MCR_MCEN | HRTIM_MCR_TACEN;
}

void buck_fatal(uint32_t reason)
{
    __disable_irq();
    if (RCC->APB2ENR & RCC_APB2ENR_HRTIM1EN)
        HRTIM_COMMON.ODISR = HRTIM_ODISR_TA1ODIS | HRTIM_ODISR_TA2ODIS;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
    (void)RCC->AHB2ENR;
    GPIOA->BSRR = GATE_PINS << 16;
    GPIOA->MODER = (GPIOA->MODER & ~((3U << 16) | (3U << 18))) | (1U << 16) | (1U << 18);
    GPIOB->BSRR = D4_PIN << 16;
    g_buck.fault = reason;
    g_buck.state = BUCK_FAULTED;
    __DSB();
    for (;;) __NOP();   /* stay awake with both gates low; reset to recover */
}

static COLD_CODE void outputs_off(buck_t *b)
{
    HRTIM_COMMON.ODISR = HRTIM_ODISR_TA1ODIS | HRTIM_ODISR_TA2ODIS;
    GPIOB->BSRR = D4_PIN << 16;
    DAC1->DHR12R1 = 0U;
    b->out.g = 0U;
}

/* Latched fault: gates off, never re-armed until reset. The loop keeps
 * converting so the ADC telemetry stays live. */
static COLD_CODE void trip(buck_t *b, uint32_t code)
{
    outputs_off(b);
    b->fault = code;
    b->state = BUCK_FAULTED;
}

/* Leaving the running state on a bad sample or a STOP command. */
static COLD_CODE void run_exit(buck_t *b, uint32_t bad)
{
    if (bad) {
        b->toc.f4_site = 1U; b->toc.f4_bad = bad;                /* diagnostics (31 Oct) */
        b->toc.f4_csr = ADC12_COMMON->CSR; b->toc.f4_cnt = TIMER_A.CNTxR;
        ADC1->ISR = ADC_ISR_OVR;
        ADC2->ISR = ADC_ISR_OVR | ADC_ISR_EOC;
        ++b->adc_miss;
        trip(b, BUCK_FAULT_ADC);     /* the sample pipeline is broken */
    } else {
        outputs_off(b);              /* command != RUN */
        TIMER_A.CMP1xR = BUCK_CMP_MIN;
        b->state = BUCK_STOPPED;
    }
}

/* A 32-bit peripheral read or write at a fixed offset from a base register, as one
 * ldr/str with an immediate offset (structure scout, 27 Oct): GCC otherwise forms
 * the ADC2, DAC1 and ADC12_CSR addresses from ADC1's base with three adds in the
 * pass head (one clock each on a late pass). Volatile: kept in order with the
 * other volatile accesses, as the C volatile accesses they replace. */
#define IO_RD(base, off) __extension__ ({ uint32_t v_; \
    __asm__ volatile("ldr %0, [%1, %2]" : "=r"(v_) : "r"(base), "i"(off)); v_; })
#define IO_WR(base, off, v) __asm__ volatile("str %0, [%1, %2]" : : "r"(v), "r"(base), "i"(off))
#define CSR_BAD_MASK (ADC_CSR_OVR_MST | ADC_CSR_OVR_SLV | ADC_CSR_EOC_SLV)

/* Waits for ADC1's end of conversion and returns 0 when the pair of samples
 * is good: no overrun, and ADC2 (the current, 4 samples summed) has its result
 * for the cycle. A missing sample stream is fatal (timeout after 840 polls,
 * ~15 periods). ahb2 = ADC1's base; ADC2 and the common registers follow it. */
static inline __attribute__((always_inline)) uint32_t wait_sample(uintptr_t ahb2)
{
    uint32_t n = 3U * BUCK_CYCLES_PER_PERIOD;
#if BUCK_ADC_CSR_POLL
    volatile uint32_t *const csr = &((ADC_Common_TypeDef *)(ahb2 + (ADC12_COMMON_BASE - ADC1_BASE)))->CSR;
    /* The first read is peeled off the poll loop and the wait loop is out of
     * line: a late pass (the one that counts) finds EOC already set and goes
     * on without a taken branch. */
    uint32_t st = *csr;
    if (UNLIKELY((st & ADC_CSR_EOC_MST) == 0U)) {
        --n;                                        /* the same 840 not-ready reads before the fault */
        while (((st = *csr) & ADC_CSR_EOC_MST) == 0U) {
            if (UNLIKELY(--n == 0U)) buck_fatal(BUCK_FAULT_ADC);
        }
    }
    return (st & (ADC_CSR_OVR_MST | ADC_CSR_OVR_SLV | ADC_CSR_EOC_SLV)) ^ ADC_CSR_EOC_SLV;
#else
    while ((((ADC_TypeDef *)ahb2)->ISR & ADC_ISR_EOC) == 0U) {
        if (UNLIKELY(--n == 0U)) buck_fatal(BUCK_FAULT_ADC);
    }
    const uint32_t s2 = ((ADC_TypeDef *)(ahb2 + (ADC2_BASE - ADC1_BASE)))->ISR;
    return (s2 & (ADC_ISR_OVR | ADC_ISR_EOC)) ^ ADC_ISR_EOC;
#endif
}

/* wait_sample() for run_pass(): the same reads, through IO_RD (the arming wait and
 * buck_run() keep wait_sample(), whose ADC12_CSR read through a register holding its
 * address tells make check and the counters where the arming wait starts). mask is
 * CSR_BAD_MASK (the first pass passes it from a register, bus scout). Only the
 * ADC12_CSR form: with BUCK_ADC_CSR_POLL = 0, run_pass() calls wait_sample(). */
#if BUCK_ADC_CSR_POLL
static inline __attribute__((always_inline)) uint32_t wait_sample_io(uintptr_t ahb2, uint32_t mask)
{
    enum { O_CSR = ADC12_COMMON_BASE - ADC1_BASE + offsetof(ADC_Common_TypeDef, CSR) };
    uint32_t n = 3U * BUCK_CYCLES_PER_PERIOD;
    uint32_t st = IO_RD(ahb2, O_CSR);
    if (UNLIKELY((st & ADC_CSR_EOC_MST) == 0U)) {
        --n;                                        /* the same 840 not-ready reads before the fault */
        while (((st = IO_RD(ahb2, O_CSR)) & ADC_CSR_EOC_MST) == 0U) {
            if (UNLIKELY(--n == 0U)) buck_fatal(BUCK_FAULT_ADC);
        }
    }
    return (st & mask) ^ ADC_CSR_EOC_SLV;
}
#endif

/* Hides a value from the optimiser. Used to keep the ADC1/ADC2/DAC1 base
 * address in a register: GCC otherwise reloads it from the literal pool every
 * pass. It is hidden again at the top of every pass, so that the ADC2/DAC1
 * addresses are formed from it there instead of being hoisted out of the loop
 * and spilled to the stack. The arming and the first pass take HRTIM's and
 * DWT's bases from registers loaded at the entry, the copies of the pass from
 * registers formed once after the first pass (run_ctx_t.q, run_loop); GPIOB's
 * is formed with a MOV in the pass (run_pass). */
#define OPAQUE(x) __asm__("" : "+r"(x))

/* Everything the running loop keeps in registers. */
typedef struct {
    buck_state_t s;             /* controller state */
    buck_tune_t t;              /* tunables, re-read every BUCK_TELEMETRY_EVERY passes */
    uint32_t loops;
    buck_obs_t obs;             /* load-current observer: v^ and the estimate i^ (A2, out.i_load) */
    buck_obs_k_t ok;            /* its gains beta, gamma: checked at each (re)load of the tunables */
    float ki_t_v, ki_t_i;       /* Ki*T of both PIs: formed once per (re)load of the tunables */
    buck_score_k_t sk;          /* trigger-score coefficients a, b, c: copied likewise */
    float target;               /* reference target (t.vref_counts), V */
    float step;                 /* BUCK_SOFTSTART_STEP_V, held in a register */
    uintptr_t ahb2;             /* ADC1's base; ADC2 and DAC1 are addressed from it */
    /* The arming's and the first pass's constants, loaded at the entry (bus scout) */
    buck_kc_t kc;
    uintptr_t hr, dwt;          /* HRTIM1, DWT */
#if BUCK_ADC_CSR_POLL
    uint32_t csr_mask;          /* CSR_BAD_MASK */
#endif
    uint32_t mode;              /* the reference step chosen by the pass after a tunables pass */
    /* The ordinary copies' constants (tscout, 28 Oct): formed once after the first pass, from the
     * entry's registers where they exist (kc, hr, dwt), and hidden from the optimiser there. Every
     * copy and the pass after each tunables pass take them from registers; no copy entry re-forms
     * them from the literal pool (before: up to 9 CCM literal loads after each tunables pass). */
    struct { float vsum, d_min, d_max, per, obs_a, dac_k; uintptr_t hr, dwt; } q;
} run_ctx_t;

enum { PASS_NORMAL = 0U, PASS_RELOADED = 1U, PASS_EXIT = 2U, PASS_TOC = 3U };

/* The arming check's counter and CYCCNT (run_loop), held until the first pass
 * after arming stores them with its own (g_buck.arm_cnt ... first_cyc). */
typedef struct { uint32_t cnt, cyc; } run_stamp_t;
#define STAMP_NONE  0xFFFFFFFFU     /* first_cnt, first_cyc: the first pass stopped before its check */

/* The deadline check and the CMP1 write (run_pass), one asm block: the counter
 * is read once the compare value exists, and CMP1 is written (as a preload:
 * active from the next period) right after the check, so the check covers the
 * write itself whatever GCC's register allocation (in C it could reload the
 * HRTIM address between them). The write happens only inside the window;
 * counts below BUCK_CHECK_EARLIEST mean the counter has wrapped: the pass is
 * more than a period late. */
#define DEADLINE_ASM                                                           \
    "ldr    %[cnt], [%[hr], #%c[o_cnt]]\n\t"   /* Timer A CNTR */            \
    "sub    %[late], %[cnt], #%c[early]\n\t"                                 \
    "cmp    %[late], #%c[win]\n\t"                                           \
    "it     lo\n\t"                                                          \
    "strlo  %[cmp], [%[hr], #%c[o_cmp1]]\n\t"   /* Timer A CMP1R */           \
    "bhs    %l[missed]"                         /* outside the window: the trip */
/* asm goto (27 Oct, path scout): the branch to the trip is the asm's own last
 * instruction, right after the conditional store (before: a bcs that GCC
 * emitted on the asm's carry output). The same six instructions. */
#define DEADLINE_OUT  [cnt] "=&r"(cnt), [late] "=&r"(late)
#define DEADLINE_IN                                                            \
    [hr] "r"(hrtim), [cmp] "r"(cmp),                                           \
    [o_cnt] "i"(offsetof(HRTIM_TypeDef, sTimerxRegs[0].CNTxR)),               \
    [o_cmp1] "i"(offsetof(HRTIM_TypeDef, sTimerxRegs[0].CMP1xR)),             \
    [early] "i"(BUCK_CHECK_EARLIEST), [win] "i"(BUCK_WRITE_DEADLINE - BUCK_CHECK_EARLIEST)

/* One pass, paced by ADC1's end of conversion early in cycle k+1: ADC1->DR =
 * 2 x v_out[k+1] (end of cycle k, two samples summed), ADC2->DR = 4 x i_Lm[k].
 * - Every pass: the command is re-read; CMP1, A2 and D4 are written. A2 and
 *   g_buck.out.i_load carry the load current from the fixed-gain observer
 *   (buck_iload_obs), fed this pass's v_out and measured current.
 *   The reference takes its step (ramp_mode, a constant: see buck_ramp_mode).
 * - Every BUCK_TELEMETRY_EVERY passes, half an interval apart so that no
 *   pass carries both (one pass that did both was ~100 clocks longer than
 *   the others and delayed the next pass's CMP1 write):
 *   - telemetry (loops = 32 mod 64): busy_max is updated with that pass's
 *     control work (from reading the samples to here), and g_buck.out, the
 *     raw ADC codes and loops are published;
 *   - tunables (loops = 0 mod 64): they are re-read, the observer's gains are
 *     checked (buck_obs_k: a pair outside the stable region is not taken up),
 *     Ki*T and the score coefficients are formed, and the pass returns
 *     PASS_RELOADED. The next pass (epoch = 1) runs in the same copy: after
 *     its CMP1 write it forms the target of the new tunables and picks the
 *     ramp step for the next BUCK_TELEMETRY_EVERY passes (buck_ramp_mode;
 *     run_loop() then stays in the copy or changes to the copy of that step),
 *     and takes that step itself (tscout, 28 Oct: in the tunables pass until
 *     then; the same values, used from the same passes).
 *   Between those, the tunables, the controller state and this pass's
 *   results stay in registers.
 * - On a stop or a trip: g_buck.st, the raw codes and loops are written back
 *   (g_buck.out keeps its last published values) and it returns PASS_EXIT.
 *   A trip also stores the counter (trip_cnt) and CYCCNT (trip_cyc).
 * - first != NULL only in the first pass after arming (its own copy, run_loop):
 *   it also reads CYCCNT just before its counter read (1 clock on the path to
 *   its CMP1 write) and stores the start's timing stamps (g_buck.arm_cnt,
 *   arm_cyc, first_cnt, first_cyc) after its CMP1 write, or on its trip or
 *   stop (not on a sample timeout: buck_fatal). The other copies (first =
 *   NULL) carry none of this.
 * A pass must fit one period ON AVERAGE (280 clocks): a longer pass delays
 * the next one, and the delay accumulates until the CMP1 write misses its
 * deadline (fault 5).
 * Only ADC2's status is checked: its group ends ~100 ns before ADC1's
 * conversion, so a cycle late enough to overrun ADC1 has overrun ADC2 too. */
static inline __attribute__((always_inline)) uint32_t run_pass(buck_t *b, run_ctx_t *c, const uint32_t ramp_mode,
                                                               const run_stamp_t *const first, const uint32_t epoch)
{
    __asm__ volatile("" : "+r"(c->ahb2));   /* OPAQUE, and a scheduling barrier at the top of the pass (scouted 27 Oct, applied 28 Oct) */
    const uintptr_t ahb2 = c->ahb2;
    /* The first pass takes the constants of its path to the CMP1 write (the CSR mask, the
     * HRTIM and DWT bases, buck_kc_t) from the registers loaded at the entry (bus scout);
     * the copies take theirs from c->q (registers, formed once after the first pass). */
    buck_kc_t kq = buck_kc_init();
    kq.vsum = c->q.vsum; kq.d_min = c->q.d_min; kq.d_max = c->q.d_max; kq.per = c->q.per;
    /* The pass after a tunables pass (epoch): the constants its copy's loop holds in registers
     * but GCC re-forms in that pass after its wait (the voltage PI's limits, the duty's rounding,
     * the CSR mask) are formed before its wait, off its CMP1 path (tscout, 28 Oct: 5 clocks on
     * its CMP1 write, 3 on the tunables pass before it). */
#if BUCK_ADC_CSR_POLL
    uint32_t maskq = CSR_BAD_MASK;
    if (epoch)
        __asm__ volatile("" : "+t"(kq.iref_lo), "+t"(kq.iref_hi), "+t"(kq.half), "+r"(maskq));
    const uint32_t bad = wait_sample_io(ahb2, (first != NULL) ? c->csr_mask : maskq);
#else
    if (epoch)
        __asm__ volatile("" : "+t"(kq.iref_lo), "+t"(kq.iref_hi), "+t"(kq.half));
    const uint32_t bad = wait_sample(ahb2);     /* the ADC1_ISR poll and the ADC2_ISR read */
#endif
    DWT_Type *const dwt = (first != NULL) ? (DWT_Type *)c->dwt : (DWT_Type *)c->q.dwt;
    HRTIM_TypeDef *const hrtim = (first != NULL) ? (HRTIM_TypeDef *)c->hr : (HRTIM_TypeDef *)c->q.hr;
    /* t0 is loaded straight into an FPU register (a VLDR of the counter's
     * bits): only the telemetry pass uses it, and core registers are scarcer.
     * (Depending on the copy, GCC keeps it in an S register or a stack slot.) */
    float t0_bits;
    __asm__ volatile("vldr %0, %1" : "=t"(t0_bits) : "Uv"(dwt->CYCCNT) : "memory");
    __asm__ volatile("" ::: "memory");      /* the command: re-read every pass */
    const uint32_t adc_v  = IO_RD(ahb2, offsetof(ADC_TypeDef, DR));     /* ADC1->DR; reading DR clears EOC */
    const uint32_t adc_i4 = IO_RD(ahb2, ADC2_BASE - ADC1_BASE + offsetof(ADC_TypeDef, DR));   /* ADC2->DR */
    const uint32_t command = *(volatile const uint32_t *)&b->command;   /* written by the debugger */
    if (UNLIKELY((bad | (command ^ BUCK_RUN)) != 0U)) {
        b->adc_v_raw = adc_v; b->adc_i4_raw = adc_i4; b->loops = c->loops;
        b->st = c->s;
        if (first != NULL) {
            b->arm_cnt = first->cnt; b->arm_cyc = first->cyc;
            b->first_cnt = STAMP_NONE; b->first_cyc = STAMP_NONE;
        }
        run_exit(b, bad);
        return PASS_EXIT;
    }
    /* The feed-forward TOC (stage 4): a rising score edge (trig = the last pass's g ^ 1 while the TOC
     * is enabled, and this sample's S >= S_th) in a HOLD pass (not the first pass after arming nor the
     * pass after a tunables pass, both constant here) starts the frame at once, inline, before the PIs
     * (Inc/buck_toc_start.h): the score needs v_out alone, and the frame replaces the period this pass's
     * CMP1 would be for (the PIs restart after the frame from the measured state). Then, still in this
     * pass, its observer step, A2 and D4 (g = 1), as an ordinary pass makes them (31 Oct, user: the step
     * of the edge's sample in the edge pass itself), after the frame's words (the return words are in:
     * nothing of the frame waits for it) and on every path of the start (a reject too). PASS_TOC: loops
     * is counted on by the TOC code. With the TOC disabled (trig = 0) the pass pays the load of trig and
     * its branch. */
    if (ramp_mode == BUCK_RAMP_HOLD && first == NULL && epoch == 0U) {
        if (g_toc_mb.trig != 0U) {
            const float v = (float)adc_v * kq.vsum;                         /* buck_adc_to_v */
            if (UNLIKELY(buck_trigger_score_ko(v - c->s.vref, c->sk, 0) >= c->t.s_th)) {
                buck_toc_start_inline(b, hrtim, v, buck_adc_to_i_o(&c->t, adc_i4, 0), c->s.vref);
                /* v_out and the measured average back from the mailbox (the start stored them: held in
                 * registers across it they cost spills inside the first span's solve), opaque: not the
                 * ordinary pass's expressions (GCC would hoist the step in front of the branch, onto the
                 * path of the ordinary pass's CMP1 write) */
                float vo = g_toc_mb.v, im = g_toc_mb.im;
                __asm__("" : "+t"(vo), "+t"(im));
                IO_WR(ahb2, DAC1_BASE - ADC1_BASE + offsetof(DAC_TypeDef, DHR12R1),
                      buck_iload_dac_k(buck_obs_step_ao(&c->obs, vo, im, c->ok, c->q.obs_a, 0), c->q.dac_k));
                uint32_t d4 = D4_PIN;
                __asm__("" : "+r"(d4));
                uintptr_t ahb2_gpio = AHB2PERIPH_BASE;
                __asm__ volatile("" : "+r"(ahb2_gpio));
                ((GPIO_TypeDef *)(ahb2_gpio + (GPIOB_BASE - AHB2PERIPH_BASE)))->BSRR = d4;
                return PASS_TOC;
            }
        }
    }
    /* voltage PI -> i_ref, current PI on i_ref - i_meas -> duty (stage 3: the measured average) */
    buck_step_t st;
    buck_fast_pend_t pd;                    /* for the state updates after the write (buck_ctrl_fast_commit) */
    const uint32_t cmp = (first != NULL)
        ? buck_ctrl_fast_pre_kc(&c->t, c->ki_t_v, c->ki_t_i, &c->s, adc_v, adc_i4, &st, &pd, &c->kc, 1)
        : buck_ctrl_fast_pre_kc(&c->t, c->ki_t_v, c->ki_t_i, &c->s, adc_v, adc_i4, &st, &pd, &kq, 0);
    /* The deadline check and the CMP1 write (DEADLINE_ASM). The first pass
     * after arming reads CYCCNT with the instruction just before the counter
     * read (first_cyc), as the arming check reads its pair (run_loop). */
    uint32_t cnt, late, cyc = 0U;
    if (first == NULL)
        __asm__ goto(DEADLINE_ASM : DEADLINE_OUT : DEADLINE_IN : "memory", "cc" : missed);
    else
        __asm__ goto("ldr    %[cyc], [%[dwt], #%c[o_cyc]]\n\t" DEADLINE_ASM
                     : [cyc] "=&r"(cyc), DEADLINE_OUT
                     : [dwt] "r"(dwt), [o_cyc] "i"(offsetof(DWT_Type, CYCCNT)), DEADLINE_IN : "memory", "cc" : missed);
    (void)late;
    if (0) {
missed: ;   /* the deadline branch of the asm above (bhs): late >= window */
        uint32_t trip_cyc;                          /* CYCCNT at the start of the trip branch (the
                                                       memory clobber keeps the stores below after it;
                                                       every trip branch, the first pass's included,
                                                       starts with a literal load of DWT's base from
                                                       CCM and then this read: 11 clocks static after
                                                       the counter read in the copies, 10 in the first
                                                       pass, whose counter read pipelines behind its
                                                       CYCCNT read (28 Oct; before, 11-16; README).
                                                       In a BUCK_ADC_CSR_POLL = 0 build one branch
                                                       has a register move between the two loads */
        __asm__ volatile("ldr    %0, [%1, #%c2]" : "=r"(trip_cyc) : "r"(DWT), "i"(offsetof(DWT_Type, CYCCNT)) : "memory");
        /* the state updates (after the write in the passes; here after trip_cyc's
         * read): g_buck.st holds the updated state, as before. The empty asm makes
         * them not the passes' own expressions, which GCC would otherwise hoist in
         * front of the asm goto. */
        __asm__("" : "+t"(pd.p_v), "+t"(pd.p_i));
        buck_ctrl_fast_commit(&c->s, &st, &pd);
        b->adc_v_raw = adc_v; b->adc_i4_raw = adc_i4; b->loops = c->loops;
        b->st = c->s;
        b->trip_cnt = cnt;
        b->trip_cyc = trip_cyc;
        if (first != NULL) {
            b->arm_cnt = first->cnt; b->arm_cyc = first->cyc;
            b->first_cnt = cnt; b->first_cyc = cyc;
        }
        trip(b, BUCK_FAULT_DEADLINE);
        return PASS_EXIT;
    }
    /* The state updates the CMP1 value does not need (both integrators): after the write, before
     * their next use (scouted 27 Oct, applied 28 Oct). */
    buck_ctrl_fast_commit(&c->s, &st, &pd);
    if (first != NULL) {                    /* after the CMP1 write: off the path to it */
        b->arm_cnt = first->cnt; b->arm_cyc = first->cyc;
        b->first_cnt = cnt; b->first_cyc = cyc;
    }
    /* 3) trigger score (stage 3: no estimator prediction any more) */
    buck_out_t o;                           /* this pass's results (registers) */
    const uint32_t g = buck_ctrl_slow_ko(&c->t, c->sk, &c->s, &st, &o, first != NULL);
    /* 4) A2: 0..10 A -> 0..4095 (USAT; negative estimates read 0): the load
     * current from the observer (buck_iload_obs), fed v_out[k+1] and the raw
     * measured average of cycle k (user, 25 Oct) */
    /* buck_iload_obs() with the DAC1->DHR12R1 write as IO_WR (the same step and code; the
     * ordinary passes take a and the A2 scale from c->q). The pass after a tunables pass (epoch)
     * pins the prediction's addition with the product first, the 25 Oct build's order, which GCC
     * swaps in that pass only (buck_obs_step_ao; user, 28 Oct, second set; 0 clocks); the other
     * passes keep the compiler's order (the product first in the copies, v^ first in the first
     * pass, both as in the 25 Oct build). */
    const float oa = (first != NULL) ? BUCK_OBS_A : c->q.obs_a, dk = (first != NULL) ? BUCK_DAC_PER_A : c->q.dac_k;
    IO_WR(ahb2, DAC1_BASE - ADC1_BASE + offsetof(DAC_TypeDef, DHR12R1),
          buck_iload_dac_k(buck_obs_step_ao(&c->obs, st.v, st.im, c->ok, oa, epoch != 0U), dk));
    /* g = 1: set PB5, g = 0: reset PB5. The word is hidden from the optimiser so that the
     * telemetry pass takes g back from it (one UBFX: bit 5 is g) instead of comparing
     * S with S_th a second time (math scout). GPIOB's BSRR from 0x48000000 (one MOV
     * immediate, formed here every pass: the volatile asm keeps it out of the loop
     * preheader) and the offset in the store: no literal-pool load from CCM (bus scout). */
    uint32_t d4 = g ? D4_PIN : D4_PIN << 16;
    __asm__("" : "+r"(d4));
    {
        uintptr_t ahb2_gpio = AHB2PERIPH_BASE;
        __asm__ volatile("" : "+r"(ahb2_gpio));
        ((GPIO_TypeDef *)(ahb2_gpio + (GPIOB_BASE - AHB2PERIPH_BASE)))->BSRR = d4;
    }
    /* the TOC's trigger word and the next pass's v_out,prev (stage 4), in every pass: a HOLD pass
     * after any other finds them current */
    g_toc_mb.trig = (g ^ 1U) & g_toc_mb.en;
    g_toc_mb.v_prev = st.v;
    if (epoch) {
        /* the pass after a tunables pass (tscout, 28 Oct): the target of the tunables that pass
         * read, and the reference step for the next BUCK_TELEMETRY_EVERY passes (buck_ramp_mode),
         * here after the CMP1 write instead of in the tunables pass: vref is still the value after
         * the tunables pass's step, so the mode is the same. Its own step is that mode's step. */
        c->target = buck_ramp_target(&c->t);
        c->mode = buck_ramp_mode(c->s.vref, c->target);
        c->s.vref = buck_ramp_step(c->mode, c->s.vref, c->target, c->step);
        ++c->loops;                         /* loops = 1 mod 64 after it: never telemetry or tunables */
        return PASS_NORMAL;
    }
    c->s.vref = buck_ramp_step(ramp_mode, c->s.vref, c->target, c->step);
    if (UNLIKELY((++c->loops & (BUCK_TELEMETRY_EVERY / 2U - 1U)) == 0U)) {
        /* the telemetry pass out of line: the tunables pass, the longer, falls through (tscout) */
        if (UNLIKELY((c->loops & (BUCK_TELEMETRY_EVERY / 2U)) != 0U)) {
            /* telemetry. busy_max: this pass's control work, sampled every 64th pass */
            uint32_t t0;
            __asm__("vmov %0, %1" : "=r"(t0) : "t"(t0_bits));
            /* the second CYCCNT read through the DWT base the pass holds in a register (tscout): no
             * literal-pool load inside the window it measures */
            const uint32_t busy = dwt->CYCCNT - t0;
            __asm__ volatile("" ::: "memory");
            if (busy > b->busy_max) b->busy_max = busy;
            o.i_load = c->obs.ih;
            o.g = (d4 >> 5) & 1U;                   /* = g: D4_PIN is bit 5 */
            b->out = o;
            /* the TOC's enable from g_buck.toc.toc_enable (stage 4: it takes effect here, every 64 passes, as
             * the tunables do in the tunables pass), held off for BUCK_TOC_HOLDOFF_PASSES after a frame */
            g_toc_mb.en = (g_buck.toc.toc_enable != 0U && (int32_t)(c->loops - g_toc_mb.hold_until) >= 0) ? 1U : 0U;
            b->adc_v_raw = adc_v; b->adc_i4_raw = adc_i4; b->loops = c->loops;
            return PASS_NORMAL;
        }
        /* the tunables (all take effect from the next pass); the observer's
         * gains only if the pair is inside the stable region, otherwise the
         * gains in use stay */
        c->ok = buck_obs_k(&b->tune, c->ok);
        c->t = b->tune;
        /* From here on the passes use this copy, not g_buck.tune: the barrier keeps GCC from
         * re-loading the tunables from g_buck in the pass after this one instead of holding the
         * copy (legal C, but a debugger write at that pass's sample would then be taken one pass
         * early; a BUCK_ADC_CSR_POLL = 0 build of 28 Oct did that). */
        __asm__ volatile("" ::: "memory");
        c->ki_t_v = buck_pi_ki_t(c->t.ki_v);
        c->ki_t_i = buck_pi_ki_t(c->t.ki_i);
        c->sk = buck_score_k(&c->t);
        return PASS_RELOADED;               /* the target and the ramp step: the next pass (epoch) */
    }
    return PASS_NORMAL;
}

/* Arming and running (user, 25 Oct, after the board's fault 5 on the first pass
 * after arming). buck_run() calls this in the stopped cycle that sees
 * command = RUN (where it called arm() until then). The entry, everything
 * that does not depend on the arming sample (the prologue, the observer
 * gains' check, the copy of the tunables, Ki*T, the score coefficients, the
 * target, the constants), runs in the rest of that period. The loop then
 * waits for the next sample and arms on it with arm()'s conditions, and goes
 * straight to the first pass: only the arming itself lies between the arming
 * sample and the first pass's CMP1 write (before: arm(), its return and this
 * entry, ~400 clocks with wait states against a 416-clock limit, more on the
 * board). The passes exist in one copy per reference step (Inc/buck_ctrl.h,
 * buck_ramp_mode): the step is chosen once per BUCK_TELEMETRY_EVERY passes,
 * by the pass after the tunables are re-read (and after the first pass), and
 * no other pass pays a branch for it; at the target (steady state) the pass
 * has no ramp code at all. */
static RUN_CODE void run_loop(buck_t *b)
{
    run_ctx_t c;
    c.ok = buck_obs_k(&b->tune, BUCK_OBS_K_DEFAULTS);   /* the observer's gains: no gains in use yet, so the
                                                       compiled defaults if the tunables' pair is outside */
    c.t = b->tune;
    /* The first telemetry or tunables pass comes BUCK_RUN_QUIET_PASSES passes
     * after arming: the first pass after arming may start late (it always did
     * until 25 Oct, when arm() and this entry ran in the period before it; now
     * only the arming does, and the first pass normally waits for its sample),
     * and a long pass there would delay the next pass's CMP1 write too (a
     * possible fault 5 on a restart). loops therefore skips ahead by up to 31
     * at each start (to the next value that is BUCK_RUN_QUIET_PASSES short of
     * a multiple of 32). */
    c.loops = ((b->loops + BUCK_RUN_QUIET_PASSES + (BUCK_TELEMETRY_EVERY / 2U - 1U)) & ~(BUCK_TELEMETRY_EVERY / 2U - 1U))
              - BUCK_RUN_QUIET_PASSES;
    c.ki_t_v = buck_pi_ki_t(c.t.ki_v);
    c.ki_t_i = buck_pi_ki_t(c.t.ki_i);
    c.sk = buck_score_k(&c.t);
    c.target = buck_ramp_target(&c.t);
    c.step = BUCK_SOFTSTART_STEP_V;
    __asm__("" : "+t"(c.step));                 /* not a literal: GCC keeps it in a register or a stack slot.
                                                   Always the constant: the FREE_UP bound assumes it. */
    /* ADC2 and DAC1 are addressed from ADC1's base (one register). */
    c.ahb2 = ADC1_BASE;
    OPAQUE(c.ahb2);
    /* The arming's and the first pass's constants into registers here, in the period of
     * the entry (bus scout): no literal-pool load between the arming sample and the arming
     * CMP1 write, nor between the first pass's sample and its CMP1 write.
     * Hidden from the optimiser so that they are not re-formed there. */
    c.kc = buck_kc_init();
    __asm__("" : "+t"(c.kc.vsum), "+t"(c.kc.d_min), "+t"(c.kc.d_max), "+t"(c.kc.per), "+t"(c.kc.inv_vin));
    c.hr = (uintptr_t)HRTIM1; OPAQUE(c.hr);
    g_toc_mb.trig = 0U;                         /* no request before g has been 0 */
    g_toc_mb.en = 0U;                           /* nor before the first telemetry pass (BUCK_RUN_QUIET_PASSES) */
    g_toc_mb.hold_until = c.loops;
    g_toc_mb.vref = c.target;                   /* the solve's constants for the reference the ramp ends at */
    BUCK_TOC_CALL(buck_toc_prime);
    c.dwt = (uintptr_t)DWT; OPAQUE(c.dwt);
#if BUCK_ADC_CSR_POLL
    c.csr_mask = CSR_BAD_MASK; OPAQUE(c.csr_mask);
#endif
    /* The entry ends here, before the arming wait: the values formed above
     * are inputs (they must exist here) and the memory clobber keeps the
     * reads of the tunables before it. Without it GCC moves Ki*T, the target
     * and loops past the wait into the arming path (25 Oct prototype). As
     * inputs, not outputs ("+t"), they leave GCC's register allocation of the
     * passes closest to the build before (outputs cost 1-3 clocks per pass
     * there). */
    __asm__ volatile("" : : "t"(c.ki_t_v), "t"(c.ki_t_i), "t"(c.target), "t"(c.step),
                            "t"(c.ok.beta), "t"(c.ok.gamma), "r"(c.loops) : "memory");

    /* Waiting for the arming sample (gates off, g_buck.state still STOPPED):
     * per sample, what the stopped loop in buck_run() does (raw codes, a bad
     * sample counted, the command), then arm()'s conditions. */
    run_stamp_t arm;
    for (;;) {
        const uintptr_t ahb2 = c.ahb2;
        ADC_TypeDef *const adc1 = (ADC_TypeDef *)ahb2;
        ADC_TypeDef *const adc2 = (ADC_TypeDef *)(ahb2 + (ADC2_BASE - ADC1_BASE));
        const uint32_t bad = wait_sample(ahb2);
        __asm__ volatile("" ::: "memory");
        const uint32_t adc_v  = adc1->DR;       /* reading DR clears EOC */
        const uint32_t adc_i4 = adc2->DR;
        b->adc_v_raw = adc_v;
        b->adc_i4_raw = adc_i4;
        if (UNLIKELY(bad != 0U)) {              /* as stopped: flags cleared, counted; the converter stays stopped */
            adc1->ISR = ADC_ISR_OVR;
            adc2->ISR = ADC_ISR_OVR | ADC_ISR_EOC;
            ++b->adc_miss;
            return;
        }
        if (UNLIKELY(*(volatile const uint32_t *)&b->command != BUCK_RUN)) return;   /* stays stopped */
        /* Only after this period's CMP1 (the high-side reference is already
         * low) and before the write deadline: the low side then conducts for
         * the rest of the period and the first high-side pulse starts at the
         * boundary, as buck_ctrl_arm() models it (the write below comes
         * 29 / 33 clocks after the counter read, static / nominal, since
         * 28 Oct; BUCK_ARM_LATEST was set for 77-112). Otherwise
         * retry on the next sample. CYCCNT is read by the instruction just
         * before the counter (arm_cyc; the first pass reads its pair the same
         * way): it moves the counter read a few clocks later, not the write
         * relative to it. */
        uint32_t cnt, cyc;
        __asm__ volatile("ldr    %[cyc], [%[dwt], #%c[o_cyc]]\n\t"
                         "ldr    %[cnt], [%[hr], #%c[o_cnt]]"     /* Timer A CNTR */
                         : [cyc] "=&r"(cyc), [cnt] "=&r"(cnt)
                         : [dwt] "r"(c.dwt), [hr] "r"(c.hr), [o_cyc] "i"(offsetof(DWT_Type, CYCCNT)),
                           [o_cnt] "i"(offsetof(HRTIM_TypeDef, sTimerxRegs[0].CNTxR))
                         : "memory");
        if (cnt <= BUCK_CMP_MIN || cnt >= BUCK_ARM_LATEST) continue;
        /* ... and ADC1 has not converted again since this cycle's samples
         * were read (the DR read cleared EOC). Otherwise the counter has
         * wrapped into the next period (a RUN right after a STOP, a debugger
         * halt), whose sample is already waiting: the first pass would find
         * an overrun (fault 4). Read after the counter, so a conversion that
         * ends before the counter read is always seen (user, 28 Sep). */
        if ((adc1->ISR & ADC_ISR_EOC) != 0U) continue;
        /* Armed. The controller state goes straight into the loop's registers
         * (g_buck.st is written back at a stop or trip, as before, but no
         * longer here: during a run it holds the previous stop's state); out.*
         * is published with the arming sample's results after the CMP1 and
         * OENR writes (scouted 27 Oct, applied 28 Oct), and the observer
         * starts from them. CMP1 before the outputs (make check finds this
         * write by the OENR write after it). */
        buck_out_t o;
        buck_arm_pend_t pa;
        HRTIM_TypeDef *const hr = (HRTIM_TypeDef *)c.hr;
        const uint32_t cmp0 = buck_ctrl_arm_cmp_kc(adc_v, &pa, &c.kc);
        hr->sTimerxRegs[0].CMP1xR = cmp0;
        hr->sCommonRegs.OENR = HRTIM_OENR_TA1OEN | HRTIM_OENR_TA2OEN;
        /* Everything the arming CMP1 value does not need (the controller
         * state, out.*) after these two writes: the
         * empty volatile asm keeps GCC from moving it (or the stores of
         * out.*) in front of them (scouted 27 Oct, applied 28 Oct). */
        __asm__ volatile("" ::: "memory");
        buck_ctrl_arm_rest_kc(&c.t, &c.s, &pa, cmp0, adc_i4, &o);
        b->state = BUCK_RUNNING;
        b->out = o;
        c.obs = buck_obs_seed(&o);              /* i^ = the arming sample's i_meas, v^ = its v_out */
        arm.cnt = cnt;
        arm.cyc = cyc;
        /* the stores of state and out.* before the first pass's wait (math scout: GCC
         * otherwise sank them past its EOC poll) */
        __asm__ volatile("" ::: "memory");
        break;
    }

    /* The first pass: one more copy of the pass, with the timing stamps
     * (first = &arm), so that the copies below carry none of it, and with the
     * constants of its CMP1 path from the registers loaded at the entry. The
     * copy for the next passes is chosen below from the reference after its
     * step. It is never a telemetry or tunables pass (BUCK_RUN_QUIET_PASSES).
     * It takes the UP step whatever buck_ramp_mode() would choose (math scout):
     * the arming leaves vref = min(v_out, target) <= target (both finite), so the mode is
     * HOLD, UP or FREE_UP, never DOWN, and the UP step gives the same value bit for bit in
     * all three: in HOLD vref == target and r = vref + step >= target (step > 0), so it
     * returns target, the same bits as vref; in FREE_UP r < target (BUCK_RAMP_FAR_V), so it
     * returns r. */
    uint32_t r = run_pass(b, &c, BUCK_RAMP_UP, &arm, 0U);
    if (r == PASS_EXIT) return;
    const uint32_t mode0 = buck_ramp_mode(c.s.vref, c.target);
    uint32_t mode = mode0;
    /* the copies' constants, once (run_ctx_t.q) */
    c.q.vsum = c.kc.vsum; c.q.d_min = c.kc.d_min; c.q.d_max = c.kc.d_max; c.q.per = c.kc.per;
    c.q.obs_a = BUCK_OBS_A; c.q.dac_k = BUCK_DAC_PER_A; c.q.hr = c.hr; c.q.dwt = c.dwt;
    __asm__ volatile("" : "+t"(c.q.vsum), "+t"(c.q.d_min), "+t"(c.q.d_max), "+t"(c.q.per),
                          "+t"(c.q.obs_a), "+t"(c.q.dac_k), "+r"(c.q.hr), "+r"(c.q.dwt));
    /* the first pass's successor: straight into the copy of its mode, not through the loop's
     * test of mode (tscout: 15-17 clocks less on the first pass) */
    if (mode0 == BUCK_RAMP_HOLD) goto copy_HOLD;
    if (mode0 == BUCK_RAMP_FREE_UP) goto copy_FREE_UP;
    if (mode0 == BUCK_RAMP_UP) goto copy_UP;
    goto copy_DOWN;
    for (;;) {
        /* One copy of the pass per reference step, chosen after the first pass
         * and by the pass after each tunables pass. The order of the copies
         * (with RUN_CODE's flags) steers GCC's register allocation; this order
         * and the flags were chosen together for this loop body (28 Oct, see
         * RUN_CODE). After a tunables pass, the next pass runs in the same copy
         * (its registers: no literal-pool reloads) with the step chosen at run
         * time (epoch = 1), which also chooses the mode; the copy loops on if
         * the mode stays, else the copy of the new mode follows.
         * Normal pass, README table convention, static / nominal: HOLD 137 /
         * 145, FREE_UP 138 / 146, UP 143 / 151, DOWN 143 / 151 clocks (25 Oct:
         * 140 / 143 / 147 / 147 static; BUCK_Adpt_Learning: 150 / 154 / 160 /
         * 160). */
#define RUN_COPY(M)                                                             \
        copy_##M:                                                               \
        do {                                                                    \
            do r = run_pass(b, &c, BUCK_RAMP_##M, NULL, 0U); while (r == PASS_NORMAL); \
            if (r == PASS_EXIT) return;                                         \
            if (r == PASS_TOC) { RUN_TOC(); continue; }                         \
            r = run_pass(b, &c, BUCK_RAMP_##M, NULL, 1U);                       \
            if (r == PASS_EXIT) return;                                         \
        } while (c.mode == BUCK_RAMP_##M)
        /* PASS_TOC (HOLD copy only, stage 4; the frame has started in the pass, its return words and the
         * pass's observer step are in): the state into the mailbox and the rest (buck_toc_finish: with a
         * frame, its samples after it); then on in the same copy with the state taken back. */
#define RUN_TOC()                                                               \
        do {                                                                    \
            g_toc_mb.s = c.s; g_toc_mb.obs = c.obs; g_toc_mb.ok = c.ok;         \
            g_toc_mb.kp_v = c.t.kp_v; g_toc_mb.kp_i = c.t.kp_i;                 \
            g_toc_mb.ki_t_v = c.ki_t_v; g_toc_mb.ki_t_i = c.ki_t_i;             \
            g_toc_mb.isense_zero = c.t.isense_zero;                             \
            g_toc_mb.isense_gain = c.t.isense_gain;                             \
            g_toc_mb.s_th = c.t.s_th; g_toc_mb.sk = c.sk; g_toc_mb.loops = c.loops; \
            BUCK_TOC_CALL(buck_toc_finish);                                     \
            if (g_toc_mb.ret == BUCK_TOC_RET_EXIT) return;                      \
            c.s = g_toc_mb.s; c.obs = g_toc_mb.obs; c.loops = g_toc_mb.loops;   \
        } while (0)
        if (mode == BUCK_RAMP_HOLD) {
            RUN_COPY(HOLD);
        } else if (mode == BUCK_RAMP_UP) {
            RUN_COPY(UP);
        } else if (mode == BUCK_RAMP_DOWN) {
            RUN_COPY(DOWN);
        } else {
            RUN_COPY(FREE_UP);
        }
#undef RUN_COPY
#undef RUN_TOC
        mode = c.mode;
    }
}

/* The control loop: stopped/faulted cycles here (warm-up; ADC overruns while
 * stopped are harmless, e.g. after a debugger halt), arming and running
 * cycles in run_loop(), which this calls in the stopped cycle that sees
 * command = RUN (25 Oct; until then it called arm() there, and run_loop()
 * once armed). Thread mode, no interrupt: polling the EOC flag is the same
 * hardware event without ~50 clocks of exception entry/exit and register
 * save/restore per 280-clock period. */
FAST_CODE __attribute__((noreturn)) void buck_run(void)
{
    buck_t *const b = &g_buck;
    for (;;) {
        const uint32_t bad = wait_sample(ADC1_BASE);
        __asm__ volatile("" ::: "memory");
        const uint32_t adc_v  = ADC1->DR;
        const uint32_t adc_i4 = ADC2->DR;
        b->adc_v_raw = adc_v;
        b->adc_i4_raw = adc_i4;
        if (bad != 0U) {
            ADC1->ISR = ADC_ISR_OVR;
            ADC2->ISR = ADC_ISR_OVR | ADC_ISR_EOC;
            ++b->adc_miss;
        } else if (b->warmup != 0U) {
            --b->warmup;
        } else if (b->state == BUCK_STOPPED && b->command == BUCK_RUN) {
            run_loop(b);    /* its entry in this period; it arms on a later sample and returns
                               on a stop, a fault or a bad sample while waiting to arm */
        }
    }
}

void NMI_Handler(void)        { buck_fatal(BUCK_FAULT_CPU); }
void HardFault_Handler(void)  { buck_fatal(BUCK_FAULT_CPU); }
void MemManage_Handler(void)  { buck_fatal(BUCK_FAULT_CPU); }
void BusFault_Handler(void)   { buck_fatal(BUCK_FAULT_CPU); }
void UsageFault_Handler(void) { buck_fatal(BUCK_FAULT_CPU); }
