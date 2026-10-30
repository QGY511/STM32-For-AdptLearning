#include "stm32g4xx.h"
#include "buck_control.h"
#include "buck_pi.h"
#include "buck_estimator.h"
#if BUCK_OPTIMAL
#include "buck_optimal.h"
#endif

#define TIMER_A       (HRTIM1->sTimerxRegs[0])
#define HRTIM_COMMON  (HRTIM1->sCommonRegs)
#define GATE_MASK     ((1U << 8) | (1U << 9))
#define FAST_CODE     __attribute__((section(".ccmtext"), noinline, optimize("O3")))
/* Rare-cycle code (optimal state machine outside OPT_IDLE, arming, soft-start
 * ramp): same zero-wait-state CCM, but kept out of line so it does not lengthen
 * the every-cycle path or raise its register pressure. */
#define COLD_CODE     __attribute__((section(".ccmtext"), noinline, cold))
#define LIKELY(x)     __builtin_expect(!!(x), 1)
#define UNLIKELY(x)   __builtin_expect(!!(x), 0)

/* ONE object for everything the ISR touches (see buck_control.h): the hot path
 * loads its address once and reaches every field as base + offset. */
buck_t g_buck = {
    .tune = { .kp_q16 = BUCK_KP_DEFAULT_Q16, .ki_q16 = BUCK_KI_DEFAULT_Q16,
              .setpoint_adc = BUCK_SETPOINT_DEFAULT_ADC,
              .command = BUCK_AUTO_START, .optimal_en = BUCK_OPTIMAL_ARM_AT_BOOT,
              .kpi_q16 = BUCK_ACM_KP_Q16, .kii_q16 = BUCK_ACM_KI_Q16,
              .isense_zero = BUCK_ISENSE_ZERO_COUNTS, .isense_gain_q16 = BUCK_ISENSE_GAIN_Q16 },
    .ctl  = { .warmup = BUCK_WARMUP_CYCLES, .active_duty = BUCK_DUTY_MIN },
};
#define control (g_buck.ctl)   /* ISR-private state, addressed off the same base */

#if BUCK_DIAG_WRITE_PHASE
/* Worst-case HRTIM count just after the compare write. Measures the deadline
 * margin: WRITE_DEADLINE - g_buck_write_phase_max ticks of slack. Debug only. */
volatile uint32_t g_buck_write_phase_max;
#endif
#if BUCK_DIAGNOSTICS
/* Worst-case ISR C-body duration, CPU clocks (entry->done, excludes hardware
 * exception entry/exit, ~22 clocks). Compare against the 280-clock period. */
volatile uint32_t g_buck_isr_cycles_max;
#endif
#if BUCK_OPTIMAL
/* Learned timing gains (TPEL Sec. V): the applied pulse is diag(gamma_on,
 * gamma_off) times the closed-form solution. Thread-owned (buck_learn_gamma);
 * the thread publishes them into g_buck.so. Q16, 1.0 at boot. */
volatile int32_t g_buck_opt_gamma_on_q16  = 65536;
volatile int32_t g_buck_opt_gamma_off_q16 = 65536;
/* Learned equilibrium correction b (Q20 A): the solve targets the measured
 * load plus b (buck_learn_ieq, the equilibrium half of the dual correction).
 * Thread-owned; published into g_buck.so.ieq_q20. */
volatile int32_t g_buck_opt_ieq_q20 = 0;
/* Series resistance the solve assumes for the fall (V_ref + r i)/L, mOhm. Live
 * tunable: the thread republishes g_buck.so when it changes. */
volatile float g_buck_opt_r_off_mohm = (float)BUCK_OPT_R_OFF_MOHM;
/* Output-capacitor ESR, mOhm (0 = off): the solve's droop and the terminal
 * residual are corrected for the ESR term of the sample. Live tunable. */
volatile float g_buck_opt_esr_mohm = (float)BUCK_OPT_ESR_MOHM;
/* State-anchored trigger scales/tilt: runtime-tunable (defaults from config).
 * The THREAD converts them to the integer coefficients in g_buck.sa. */
volatile float g_buck_sa_va  = BUCK_SA_VA_COUNTS;
volatile float g_buck_sa_dv  = BUCK_SA_DVA_COUNTS;
volatile float g_buck_sa_rho = BUCK_SA_RHO;
/* IDLE -> (trigger, period n) WAIT: the solve cycle (n+1) -> WRITE: PER/CMP1
 * written first thing (n+2; a rejected solve returns to IDLE from here so the
 * late-entered cycle after the solve never runs the hot path) -> PULSE: the
 * stretched period (n+3), reseed -> SETTLE: first normal period, terminal
 * state sampled (n+4) -> IDLE with the trigger masked for the linear-
 * controller transition; when the mask ends the settled load closes the
 * record and the thread runs the dual correction. */
enum { OPT_IDLE = 0U, OPT_WAIT = 1U, OPT_PULSE = 2U, OPT_SETTLE = 3U, OPT_WRITE = 4U };
#define REC (g_buck.opt.rec[control.rec_idx])   /* the record of the transient in flight */
#if BUCK_DIAGNOSTICS
volatile uint32_t g_buck_opt_entry_phase;   /* HRTIM count at entry of the solve cycle's cold path (max) */
volatile uint32_t g_buck_opt_solve_cycles;  /* CPU clocks of the solve cycle's cold path, entry -> result stored (max) */
volatile uint32_t g_buck_isr_entry_phase;   /* HRTIM count at ISR entry (max): sample-to-ISR latency */
#endif
#else
#define OPT_IDLE 0U
#endif
/* SRAM1 vector fetch avoids flash misses. Linkers reserve the CCM alias. */
static uint32_t ram_vectors[128] __attribute__((aligned(512)));
extern uint32_t g_pfnVectors[];
extern uint32_t _siccmtext, _sccmtext, _eccmtext;

static void wait_mask(volatile uint32_t *reg, uint32_t mask, uint32_t value,
                      uint32_t fault)
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

static void clock_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    (void)RCC->APB1ENR1;
    PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS) | PWR_CR1_VOS_0;
    wait_mask(&PWR->SR2, PWR_SR2_VOSF, 0U, BUCK_FAULT_CLOCK);
    PWR->CR5 &= ~PWR_CR5_R1MODE; /* Range 1 boost, required above 150 MHz. */
    delay_cycles(160U); /* >5 us at reset HSI clock. */
    /* Read-modify-write: FLASH_ACR.DBG_SWEN (bit 18, reset 1) gates the SWD
     * debug port. A plain assignment clears it and ST-LINK reports
     * "Target is not responding" until the next reset. */
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_4WS |
                 FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN;
    wait_mask(&FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_4WS, BUCK_FAULT_CLOCK);
#if BUCK_USE_HSE
    _Static_assert(BUCK_HSE_HZ % 4000000U == 0U, "HSE must divide to 4 MHz");
    _Static_assert(BUCK_HSE_HZ >= 8000000U && BUCK_HSE_HZ <= 48000000U, "HSE range");
    if (BUCK_HSE_BYPASS) RCC->CR |= RCC_CR_HSEBYP;
    RCC->CR |= RCC_CR_HSEON;
    wait_mask(&RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY, BUCK_FAULT_CLOCK);
    const uint32_t pll_source = RCC_PLLCFGR_PLLSRC_HSE;
    const uint32_t pll_m = BUCK_HSE_HZ / 4000000U;
#else
    RCC->CR |= RCC_CR_HSION;
    wait_mask(&RCC->CR, RCC_CR_HSIRDY, RCC_CR_HSIRDY, BUCK_FAULT_CLOCK);
    const uint32_t pll_source = RCC_PLLCFGR_PLLSRC_HSI;
    const uint32_t pll_m = 4U;
#endif
    /* VCO=4*84=336 MHz; R/2=168 MHz CPU; P/6=56 MHz ADC. */
    RCC->PLLCFGR = pll_source | ((pll_m - 1U) << RCC_PLLCFGR_PLLM_Pos) |
                   (84U << RCC_PLLCFGR_PLLN_Pos) | RCC_PLLCFGR_PLLREN |
                   RCC_PLLCFGR_PLLPEN | (6U << RCC_PLLCFGR_PLLPDIV_Pos);
    RCC->CR |= RCC_CR_PLLON;
    wait_mask(&RCC->CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY, BUCK_FAULT_CLOCK);
    /* Intermediate /2 transition limits regulator undershoot. */
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_SW | RCC_CFGR_HPRE |
                              RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) |
                 RCC_CFGR_HPRE_DIV2 | RCC_CFGR_SW_PLL;
    wait_mask(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL, BUCK_FAULT_CLOCK);
    delay_cycles(168U); /* 2 us at 84 MHz. */
    RCC->CFGR &= ~RCC_CFGR_HPRE;
    SystemCoreClock = BUCK_CPU_HZ;
    RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_ADC12SEL) | RCC_CCIPR_ADC12SEL_0;
    SysTick->CTRL = 0U;
}

static void gpio_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
    (void)RCC->AHB2ENR;
    GPIOA->BSRR = GATE_MASK << 16;
    GPIOA->OTYPER &= ~GATE_MASK;
    GPIOA->PUPDR = (GPIOA->PUPDR & ~((3U << 16) | (3U << 18) | 3U | (3U << 2))) |
                   (2U << 16) | (2U << 18);
    GPIOA->OSPEEDR |= (3U << 16) | (3U << 18);
    GPIOA->MODER = (GPIOA->MODER & ~((3U << 16) | (3U << 18) | 3U | (3U << 2))) |
                   (1U << 16) | (1U << 18) | 3U | (3U << 2); /* Gates low; PA0 (v_out) and PA1 (i_L) analog. */
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~0xFFU) | 0xDDU; /* AF13. */
#if BUCK_SCOPE_PIN || BUCK_OPTIMAL
    GPIOB->BSRR = 1U << (5 + 16);
    GPIOB->MODER = (GPIOB->MODER & ~(3U << 10)) | (1U << 10);
    GPIOB->OSPEEDR |= 3U << 10;
#endif
}

static void hrtim_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_HRTIM1EN;
    (void)RCC->APB2ENR;
    RCC->APB2RSTR |= RCC_APB2RSTR_HRTIM1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_HRTIM1RST;
    HRTIM_COMMON.ODISR = HRTIM_ODISR_TA1ODIS | HRTIM_ODISR_TA2ODIS;
    HRTIM_COMMON.DLLCR = HRTIM_DLLCR_CALEN | HRTIM_DLLCR_CALRTE;
    wait_mask(&HRTIM_COMMON.ISR, HRTIM_ISR_DLLRDY, HRTIM_ISR_DLLRDY, BUCK_FAULT_DLL);
    TIMER_A.TIMxCR = HRTIM_TIMCR_CONT | HRTIM_TIMCR_PREEN | HRTIM_TIMCR_TREPU;
    TIMER_A.PERxR = BUCK_HRTIM_PERIOD;
    TIMER_A.REPxR = 0U; /* Update EVERY period, no control-loop decimation. */
    TIMER_A.CMP1xR = BUCK_DUTY_MIN;
    TIMER_A.CMP3xR = BUCK_SAMPLE_COMPARE;
    TIMER_A.SETx1R = HRTIM_SET1R_PER;
    TIMER_A.RSTx1R = HRTIM_RST1R_CMP1;
    /* Positive deadtime derives BOTH outputs from crossbar output 1.
     * Do not invert CHA2 or enable push-pull (which alternates periods). */
    TIMER_A.SETx2R = 0U;
    TIMER_A.RSTx2R = 0U;
    TIMER_A.DTxR = BUCK_DT_COUNTS | (BUCK_DT_COUNTS << HRTIM_DTR_DTF_Pos) |
                    HRTIM_DTR_DTRSLK | HRTIM_DTR_DTRLK |
                    HRTIM_DTR_DTFSLK | HRTIM_DTR_DTFLK;
    TIMER_A.OUTxR = HRTIM_OUTR_DTEN | HRTIM_OUTR_IDLM1 | HRTIM_OUTR_IDLM2 |
                    HRTIM_OUTR_FAULT1_1 | HRTIM_OUTR_FAULT2_1;
    TIMER_A.TIMxDIER = 0U; /* ADC is the only cycle interrupt source. */
    HRTIM_COMMON.CR1 = HRTIM_CR1_ADC1USRC_0;
    HRTIM_COMMON.ADC1R = HRTIM_ADC1R_AD1TAC3;
    HRTIM_COMMON.ADCPS1 = 0U;
    HRTIM_COMMON.CR2 = HRTIM_CR2_TASWU | HRTIM_CR2_TARST;
    __DSB();
    GPIOA->MODER = (GPIOA->MODER & ~((3U << 16) | (3U << 18))) |
                   (2U << 16) | (2U << 18);
}

/* One converter: 12-bit, one regular rank on `channel`, 2.5 clocks first
 * sample then bulb mode (ES0430 2.7.8), hardware-triggered by HRTIM ADC trigger
 * 1 (EXTSEL 21) on the rising edge, OVRMOD (no data FIFO). */
static void adc_setup(ADC_TypeDef *a, uint32_t channel)
{
    a->CR = ADC_CR_ADVREGEN;
    delay_cycles(BUCK_CPU_HZ / 1000000U * 25U);
    a->DIFSEL = 0U;
    a->CR |= ADC_CR_ADCAL;
    wait_mask(&a->CR, ADC_CR_ADCAL, 0U, BUCK_FAULT_ADC_START);
    delay_cycles(32U); /* >4 ADC clocks after calibration. */
    a->SMPR1 = 0U;
    a->SMPR2 = 0U;
    a->SQR1 = channel << ADC_SQR1_SQ1_Pos;
    a->CFGR = ADC_CFGR_JQDIS | ADC_CFGR_OVRMOD |
              (21U << ADC_CFGR_EXTSEL_Pos) | ADC_CFGR_EXTEN_0;
    a->CFGR2 = ADC_CFGR2_BULB;
    a->ISR = ADC_ISR_ADRDY | ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;
    a->CR |= ADC_CR_ADEN;
    wait_mask(&a->ISR, ADC_ISR_ADRDY, ADC_ISR_ADRDY, BUCK_FAULT_ADC_START);
}

/* A0 = PA0 = ADC1_IN1 (v_out) and A1 = PA1 = ADC2_IN2 (inductor current, shunt
 * x amplifier). Independent mode, both asynchronous PLLP /1, both on the same
 * HRTIM trigger: they convert at the same instant, and ADC1's EOC is the one
 * interrupt (ADC2's result is ready at the same time; the ISR reads both DRs). */
static void adc_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
    (void)RCC->AHB2ENR;
    RCC->AHB2RSTR |= RCC_AHB2RSTR_ADC12RST;
    RCC->AHB2RSTR &= ~RCC_AHB2RSTR_ADC12RST;
    ADC12_COMMON->CCR = 0U; /* Independent, async PLLP /1. */
    adc_setup(ADC1, 1U);
    adc_setup(ADC2, 2U);
    ADC1->IER = ADC_IER_EOCIE | ADC_IER_OVRIE;
    ADC2->IER = 0U;
}

void buck_init(void)
{
    __disable_irq();
    /* Capture and clear the reset-cause flags FIRST, before anything can fault,
     * so a reboot's reason survives in telemetry. Bit27 BORRSTF (brownout),
     * bit26 PINRSTF (NRST), bit28 SFTRSTF, bit29 IWDGRSTF, bit30 WWDGRSTF.
     * A brownout here means the supply collapsed (e.g. optimal-pulse current). */
    g_buck.status.rst_cause = RCC->CSR;
    RCC->CSR |= RCC_CSR_RMVF;   /* clear so the NEXT reset's cause is unambiguous */
    /* Keep the debug AHB clock available during WFI/SLEEPONEXIT. Do not
     * depend on ST-LINK's low-power setting surviving a reset/download.
     * This is a startup-only write, outside the cycle-by-cycle path. */
    DBGMCU->CR |= DBGMCU_CR_DBG_SLEEP;
    SCB->SCR &= ~(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk);
    /* No exception handler uses the FPU, so disable automatic FP state
     * preservation: exception entry never reserves/stacks an FP frame, even
     * when thread-mode code (the adaptation) uses the FPU. Keeps ISR entry
     * latency free of any FP cost. */
    FPU->FPCCR &= ~(FPU_FPCCR_ASPEN_Msk | FPU_FPCCR_LSPEN_Msk);
    __DSB();
    __ISB();
    gpio_init();
    clock_init();
    uint32_t *src = &_siccmtext;
    for (uint32_t *dst = &_sccmtext; dst < &_eccmtext; ++dst) *dst = *src++;
    for (uint32_t i = 0U; i < 118U; ++i) ram_vectors[i] = g_pfnVectors[i];
    SCB->VTOR = (uint32_t)ram_vectors;
    __DSB();
    __ISB();
#if BUCK_OPTIMAL
    {
        buck_sa_coef_t c;
        buck_sa_coef_from_float(BUCK_SA_VA_COUNTS, BUCK_SA_DVA_COUNTS, BUCK_SA_RHO, &c);
        g_buck.sa.inv_va_q30 = c.inv_va_q30; g_buck.sa.inv_dva_q30 = c.inv_dva_q30;
        g_buck.sa.rho_q30 = c.rho_q30;       g_buck.sa.sq_q30 = c.sq_q30;
        /* ISR-side pulse solve: sqrt table + V_ref coefficients and the gains
         * (the thread republishes them when the setpoint or a learned gain changes). */
        buck_sqrt_lut_init();
        buck_so_coef_t so;
        buck_so_coef_from_float((float)BUCK_SETPOINT_DEFAULT_ADC, (float)BUCK_OPT_R_OFF_MOHM, (float)BUCK_OPT_ESR_MOHM, &so);
        g_buck.so = so;
    }
#endif
    hrtim_init();
    adc_init();
    NVIC_SetPriorityGrouping(0U);
    NVIC_SetPriority(ADC1_2_IRQn, 0U);
    NVIC_ClearPendingIRQ(ADC1_2_IRQn);
    NVIC_EnableIRQ(ADC1_2_IRQn);
}

void buck_start_sampling(void)
{
    ADC1->CR |= ADC_CR_ADSTART; /* Arm once, not every ISR. */
    ADC2->CR |= ADC_CR_ADSTART;
    HRTIM1->sMasterRegs.MCR |= HRTIM_MCR_TACEN;
    __enable_irq();
}

void buck_fatal(uint32_t reason)
{
    __disable_irq();
    if (RCC->APB2ENR & RCC_APB2ENR_HRTIM1EN)
        HRTIM_COMMON.ODISR = HRTIM_ODISR_TA1ODIS | HRTIM_ODISR_TA2ODIS;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    (void)RCC->AHB2ENR;
    GPIOA->BSRR = GATE_MASK << 16;
    GPIOA->MODER = (GPIOA->MODER & ~((3U << 16) | (3U << 18))) |
                   (1U << 16) | (1U << 18);
    g_buck.status.fault = reason;
    g_buck.status.state = BUCK_FAULTED;
    __DSB();
    /* Remain awake with both gates forced low so a latched startup/ADC/
     * deadline fault stays readable through ST-LINK. Reset required. */
    for (;;) __NOP();
}

/* Thread-mode STOP: the running ISR does not poll the command word (saves the
 * hot path a load and a compare every cycle). Clearing `running` sends the
 * next ISR down the cold arming path, which sees command != RUN and keeps the
 * outputs disabled. Outputs are disabled here too, so the stop takes effect
 * within one main-loop iteration, not one ISR. */
void buck_command_service(void)
{
    if (g_buck.tune.command != BUCK_RUN && g_buck.status.state == BUCK_RUNNING) {
        HRTIM_COMMON.ODISR = HRTIM_ODISR_TA1ODIS | HRTIM_ODISR_TA2ODIS;
        control.running = 0U;   /* next ISR: cold path -> command != RUN -> disarmed */
    }
}

#if BUCK_OPTIMAL
static COLD_CODE void opt_solve(uint32_t adc, uint32_t iadc);
static COLD_CODE void opt_settled(int32_t error);
#endif
static COLD_CODE void control_transition(uint32_t adc)
{
    if (control.warmup != 0U) { --control.warmup; return; } /* discard the first conversions */
    if (g_buck.tune.command != BUCK_RUN) {
        HRTIM_COMMON.ODISR = HRTIM_ODISR_TA1ODIS | HRTIM_ODISR_TA2ODIS;
        control.running = 0U;
        g_buck.status.state = BUCK_DISARMED;
        control.reference_q16 = 0U;
#if BUCK_CURRENT_MODE
        control.integrator_q16 = 0;
        control.integrator_i_q16 = BUCK_MIN_Q16;
#else
        control.integrator_q16 = BUCK_MIN_Q16;
#endif
        TIMER_A.CMP1xR = BUCK_DUTY_MIN;
        g_buck.status.duty_ticks = 0U; /* Both outputs disabled. */
        return;
    }
    const uint32_t phase = TIMER_A.CNTxR;
    if (phase < BUCK_SAMPLE_COMPARE || phase >= BUCK_WRITE_DEADLINE)
        buck_fatal(BUCK_FAULT_DEADLINE);
    control.reference_q16 = 0U;
#if BUCK_CURRENT_MODE
    control.integrator_q16 = 0;                 /* current reference */
    control.integrator_i_q16 = BUCK_MIN_Q16;    /* duty */
#else
    control.integrator_q16 = BUCK_MIN_Q16;
#endif
    TIMER_A.CMP1xR = BUCK_DUTY_MIN;
    /* Low-side phase: next HS rising edge comes from hardware PER. */
    HRTIM_COMMON.OENR = HRTIM_OENR_TA1OEN | HRTIM_OENR_TA2OEN;
    g_buck.status.duty_ticks = BUCK_DUTY_MIN;
    control.active_duty = BUCK_DUTY_MIN;
    control.adc_prev = adc; control.adc_raw_prev = adc;   /* sample validity reference */
    control.running = 1U;
    g_buck.status.state = BUCK_RUNNING;
#if BUCK_ESTIMATOR
    buck_est_reset(&control.est, adc); /* seed dv/dt reference */
#if BUCK_OPTIMAL
    control.opt_state = OPT_IDLE;
    control.opt_cooldown = BUCK_OPT_COOLDOWN_START; /* held there by the ramp until regulated */
    control.rec_idx = 0U; control.settle_pending = 0U;
    g_buck.opt.learn_req = 0U; g_buck.opt.trig = 0U;
#endif
#endif
}

/* Soft-start ramp / live setpoint change. Out of line: after the ramp the
 * every-cycle path only compares reference against target. While ramping the
 * trigger hold-off is re-armed every cycle, so the optimal response stays off
 * during soft-start and for BUCK_OPT_COOLDOWN_START cycles after it (no
 * separate 'regulated' flag on the hot path). */
static COLD_CODE void reference_step(uint32_t target_q16)
{
    if (control.reference_q16 < target_q16) {
        uint32_t next = control.reference_q16 + BUCK_REFERENCE_STEP_Q16;
        if (next >= target_q16) next = target_q16;
        control.reference_q16 = next;
#if BUCK_OPTIMAL
        control.opt_cooldown = BUCK_OPT_COOLDOWN_START;
#endif
    } else {
        control.reference_q16 = target_q16; /* live setpoint lowered: snap down */
    }
}

#if BUCK_OPTIMAL
/* The two cold cycles after a trigger (OPT_WAIT: solve; OPT_WRITE: write). The
 * trigger cleared `running`, so the ISR arrives here through the hot path's
 * existing cold branch at no cost to the every-cycle path, and does nothing
 * else in these cycles: the PI work is skipped (the pulse overrides the period
 * the PI would have programmed). The preloaded CMP1 keeps last cycle's duty
 * meanwhile. */
static COLD_CODE void opt_solve(uint32_t adc, uint32_t iadc)
{
    buck_opt_rec_t *const r = &REC;
    if (control.opt_state == OPT_WAIT) {
        /* ---- Solve cycle (period n+1; trigger sampled in n). No HRTIM write,
         * no deadline: the ISR is entered ~140 clocks into the period, which
         * leaves no room to solve AND write before the preload window closes.
         * The pulse starts with period n+3, so solve for the state at THAT
         * rollover. Everything comes from the MEASURED current. */
#if BUCK_DIAGNOSTICS
        const uint32_t t0 = DWT->CYCCNT;
        { const uint32_t e = TIMER_A.CNTxR; if (e > g_buck_opt_entry_phase) g_buck_opt_entry_phase = e; }
#endif
        const int32_t vout = buck_v_q28(adc);
        const int32_t dv_period = vout - buck_v_q28(r->adc0);   /* voltage change since the trigger sample, Q28 V */
        const int32_t i_meas_q16 = ((int32_t)iadc - g_buck.tune.isense_zero) * g_buck.tune.isense_gain_q16;
        const int32_t is_q20 = BUCK_SHL(i_meas_q16, 4);
        const int32_t iavg_q20 = is_q20 + BUCK_SHL(((int32_t)control.active_duty - BUCK_ISENSE_RIPPLE_OFF) * BUCK_ISENSE_RIPPLE_Q16, 4);
        /* New load over the period after the edge: the mean of the two measured
         * period averages against the voltage change between their samples
         * (eqs 7-8); the equilibrium adds the learned correction b. */
        const int32_t iload = buck_est_iload_raw((r->iavg0_q20 + iavg_q20) >> 1, dv_period);
        const int32_t ieq = iload + g_buck.so.ieq_q20;
        r->iload_q20 = iload; r->ieq_q20 = ieq;
        /* Current at the pulse start (the valley of period n+3): this sample's
         * valley plus two more periods of the drift the frozen duty produced over
         * the last one (the droop pulls the current up while the loop waits). */
        const int32_t drift = iavg_q20 - r->iavg0_q20;
        const int32_t iv = is_q20 - BUCK_ISENSE_RISE_Q20 + 2 * drift;
        r->i0_q20 = iv;
        /* Droop to refill: this sample's error, plus what the load-inductor gap
         * adds over the two periods until the pulse, minus the ESR term of the
         * sample (the ADC reads v_C + ESR (i_s - I_load); the inductor lags the
         * load during a droop, so the sample sits BELOW the capacitor). */
        const int32_t dv = (int32_t)(control.reference_q16 >> 16) - (int32_t)adc
                         + ((((ieq - r->iavg0_q20) >> 16) * g_buck.so.dq_q16) >> 20)
                         + BUCK_SMULLH(is_q20 - iload, g_buck.so.esr_cpa_q12);
        buck_so_out_t o;
        buck_opt_solve_i(&g_buck.so, iv, ieq, vout, dv, &o);
        r->gon_q16 = g_buck.so.gon_q16; r->goff_q16 = g_buck.so.goff_q16;
        r->ton = o.ton; r->toff = o.toff; r->sat = o.sat; r->ipk_q16 = o.ipk_q16; r->term_q20 = o.term_q20;
        /* (optimal_en cleared inside this 2-cycle window still fires this pulse:
         * disabling takes effect at the next trigger; nothing is added here.) */
#if BUCK_DIAGNOSTICS
        { const uint32_t el = DWT->CYCCNT - t0; if (el > g_buck_opt_solve_cycles) g_buck_opt_solve_cycles = el; }
#endif
        control.opt_state = OPT_WRITE;       /* `running` stays 0: next cycle writes (or rejects) */
        return;
    }
    /* ---- Write cycle (period n+2). The solve ran the previous ISR into this
     * period, so this cycle is entered late (bench: the write lands ~240 clocks
     * into the period against the 264-clock deadline); nothing may sit in front
     * of the phase read and the writes but the one test below. */
    control.running = 1U;
    const uint32_t phase = TIMER_A.CNTxR;
    const uint32_t ton = r->ton;
    if (ton == 0U) {
        /* No admissible pulse (load already at the peak limit): back to the
         * PI; CMP1 still holds last cycle's duty. The
         * short hold-off keeps a trigger that stays true from taking every
         * other cycle away from the PI. */
        control.opt_cooldown = BUCK_OPT_REJECT_HOLD_CYCLES;
        control.opt_state = OPT_IDLE;
        g_buck.opt.trig = 0U;
        ++g_buck.status.opt_reject_count;
        return;
    }
    if (phase < BUCK_SAMPLE_COMPARE || phase >= BUCK_WRITE_DEADLINE)
        buck_fatal(BUCK_FAULT_DEADLINE);
    /* Stretched pulse next period. PER first: if a rollover splits the pair, a
     * normal on-time with a long off follows (an undershoot) rather than a
     * near-full-period on-time. */
    TIMER_A.PERxR = ton + r->toff;
    TIMER_A.CMP1xR = ton;
    GPIOB->BSRR = 1U << 5;   /* D4 high: optimal pulse applied */
    control.opt_state = OPT_PULSE;
    ++g_buck.status.opt_count;
    g_buck.status.opt_write_phase = phase;
    /* The rest of this cycle has slack: precompute the post-pulse reseed the
     * PULSE cycle will apply (the current reference I_eq and the duty that
     * holds it), so that cycle only stores it before its own deadline. */
    int32_t new_iref; uint32_t nd;
    buck_opt_reseed_i(&g_buck.so, r->duty0, r->iavg0_q20, r->ieq_q20, &new_iref, &nd);
    r->new_iref_q16 = new_iref;
    r->new_duty = nd;
}

/* The cycles after the pulse write, entered from the hot path AFTER its deadline
 * check with the PI results; it owns the HRTIM preload writes for these cycles
 * so they land within the same guarded window as the normal compare write.
 * Stores only: nothing here may run the ISR into the next period. */
static COLD_CODE void opt_cold(uint32_t st, uint32_t duty, int32_t integ, int32_t integ_i,
                               uint32_t adc, uint32_t adc_raw, int32_t error, int32_t i_meas_q16, int32_t i_avg_q16)
{
    buck_opt_rec_t *const r = &REC;
    control.adc_prev = adc; control.adc_raw_prev = adc_raw;
    if (st == OPT_PULSE) {
        /* Fires ~600 ticks into the stretched period: restore the PWM for the
         * following period and reseed both loops at the operating point the
         * pulse ends at (precomputed by the write cycle). D4 stays high so the
         * marker spans the whole large signal. */
        const uint32_t nd = r->new_duty;
        TIMER_A.PERxR = BUCK_HRTIM_PERIOD;
        TIMER_A.CMP1xR = nd;
#if BUCK_CURRENT_MODE
        control.integrator_q16 = r->new_iref_q16;                     /* voltage loop: I_eq */
        control.integrator_i_q16 = (int32_t)(nd << BUCK_PI_SHIFT);    /* current loop: the duty that holds it */
#else
        control.integrator_q16 = (int32_t)(nd << BUCK_PI_SHIFT);
#endif
        /* The load estimate skipped the solve/write/pulse cycles: reference its
         * dv to the voltage the pulse aims for and its current to the
         * equilibrium, not to the mid-pulse sample. */
        control.est.vout_prev_q28 = buck_v_q28(control.reference_q16 >> 16);
        control.est.iavg_q20 = r->ieq_q20;
        control.active_duty = nd;        /* on-time of the SETTLE period */
        control.opt_state = OPT_SETTLE;
        (void)duty; (void)integ; (void)integ_i; (void)error; (void)i_meas_q16; (void)i_avg_q16;
        return;
    }
    /* SETTLE: the first normal period after the large signal. Both loops run
     * normally (this cycle does the normal compare write and keeps the load
     * estimate going) and this cycle's sample, 112 ns after the stretched period
     * ended, IS the terminal state the ON-OFF solution was meant to reach: store
     * it for the thread. End the D4 marker; mask the trigger for the linear-
     * controller transition. */
    TIMER_A.CMP1xR = duty;
    control.integrator_q16 = integ;
#if BUCK_CURRENT_MODE
    control.integrator_i_q16 = integ_i;
#else
    (void)integ_i;
#endif
    control.active_duty = duty;
    const int32_t is_q20 = BUCK_SHL(i_meas_q16, 4), iavg_q20 = BUCK_SHL(i_avg_q16, 4);
    buck_load_update(&control.est, iavg_q20, buck_v_q28(adc));
    r->zv_f = -error;
    r->is_f_q20 = is_q20;
    r->iavg_f_q20 = iavg_q20;
    GPIOB->BSRR = 1U << (5 + 16);   /* D4 low */
    g_buck.opt.trig = 0U;
    control.settle_pending = 1U;
    control.opt_cooldown = BUCK_OPT_SETTLE_CYCLES;
    control.opt_state = OPT_IDLE;
}

/* End of the linear-controller transition (the trigger mask after a pulse ran
 * out): the loop has settled at the new load. Close the record with the
 * settled load and hand it to the thread; the next transient writes the other
 * record. Called from the hot path's masked branch, out of line. */
static COLD_CODE void opt_settled(int32_t error)
{
    buck_opt_rec_t *const r = &REC;
    r->iload_s_q20 = control.est.iload_q20;
    r->zv_s = -error;
    control.settle_pending = 0U;
    g_buck.opt.learn_req = control.rec_idx + 1U;
    control.rec_idx ^= 1U;
}
#endif

FAST_CODE void ADC1_2_IRQHandler(void)
{
#if BUCK_DIAGNOSTICS
    const uint32_t t_entry = DWT->CYCCNT;
    { const uint32_t e = TIMER_A.CNTxR; if (e > g_buck_isr_entry_phase) g_buck_isr_entry_phase = e; }
#endif
#if BUCK_SCOPE_PIN
    GPIOB->BSRR = 1U << 5;
#endif
    const uint32_t flags = ADC1->ISR;
    const uint32_t adc_raw = ADC1->DR; /* v_out; clears EOC: no W1C write on hot path. */
    const uint32_t iadc = ADC2->DR;    /* i_L: converted on the same trigger at the same instant */
    /* IER enables only EOC and OVR, so a non-OVR entry is an EOC entry. */
    if (UNLIKELY(flags & ADC_ISR_OVR)) {
        /* Tolerate a transient overrun (e.g. the ADC free-runs while the core is
         * halted at a debugger breakpoint, then flags OVR on resume): clear it and
         * skip the glitched cycle. Only a SUSTAINED overrun -- the ISR genuinely
         * too slow every cycle -- latches the fault. OVRMOD keeps the ADC running. */
        ADC1->ISR = ADC_ISR_OVR; /* W1C */
        if (UNLIKELY(++control.ovr_run > BUCK_ADC_OVR_MAX))
            buck_fatal(BUCK_FAULT_ADC_OVERRUN);
        goto done;
    }
    control.ovr_run = 0U;
    /* Not running (warm-up, disarmed, stopped by the thread) or the solve /
     * pulse-write cycle after an optimal trigger: cold path, out of line. */
    if (UNLIKELY(!control.running)) {
#if BUCK_OPTIMAL
        const uint32_t os = control.opt_state;
        if (os == OPT_WAIT || os == OPT_WRITE) opt_solve(adc_raw, iadc);
        else
#endif
            control_transition(adc_raw);
        goto done_adc;
    }
    /* --- Sample validity (BUCK_ADC_JUMP_COUNTS): a one-period jump this large is
     * not physics. Hold the last accepted sample for this cycle unless the
     * previous raw sample already agreed (a real collapse is accepted on its
     * second sample). The cycles right after the pulse are exempt. */
    uint32_t adc = adc_raw;
    const uint32_t prev = control.adc_prev;
#if BUCK_OPTIMAL
    const uint32_t st = control.opt_state;   /* never OPT_WAIT/WRITE here */
#else
    const uint32_t st = OPT_IDLE;
#endif
    if (UNLIKELY((uint32_t)(adc_raw - prev + BUCK_ADC_JUMP_COUNTS) > 2U * BUCK_ADC_JUMP_COUNTS)) {
        if ((uint32_t)(adc_raw - control.adc_raw_prev + BUCK_ADC_JUMP_COUNTS) > 2U * BUCK_ADC_JUMP_COUNTS
            && st == OPT_IDLE) {
            adc = prev;
            ++g_buck.status.adc_glitch;
        }
    }
    /* --- Critical path: sample -> voltage PI -> current PI -> compare write. --- */
    const uint32_t setpoint = control.reference_q16 >> 16;
    const int32_t error = (int32_t)setpoint - (int32_t)adc;
    /* Measured inductor current: the sample, and this period's average (the
     * ripple correction for the on-time that is driving this period). */
    const int32_t i_meas_q16 = ((int32_t)iadc - g_buck.tune.isense_zero) * g_buck.tune.isense_gain_q16;
    const int32_t i_avg_q16  = i_meas_q16 + ((int32_t)control.active_duty - BUCK_ISENSE_RIPPLE_OFF) * BUCK_ISENSE_RIPPLE_Q16;
    int32_t p;
#if BUCK_CURRENT_MODE
    /* Voltage loop -> average-current reference u (Q16 A); current loop -> duty. */
    const int32_t u = buck_pi_u(error, control.integrator_q16,
                                g_buck.tune.kp_q16, g_buck.tune.ki_q16,
                                BUCK_ACM_IMIN_Q16, BUCK_ACM_IMAX_Q16, &p);
    int32_t pi;
    const int32_t ui = buck_pi_u((u - i_avg_q16) >> 8, control.integrator_i_q16,
                                 g_buck.tune.kpi_q16, g_buck.tune.kii_q16,
                                 BUCK_MIN_Q16, BUCK_MAX_Q16, &pi);
    const uint32_t duty = (uint32_t)ui >> BUCK_PI_SHIFT;
    const int32_t integ_i_new = ui - pi;
#else
    const int32_t u = buck_pi_u(error, control.integrator_q16,
                                g_buck.tune.kp_q16, g_buck.tune.ki_q16,
                                BUCK_MIN_Q16, BUCK_MAX_Q16, &p);
    const uint32_t duty = (uint32_t)u >> BUCK_PI_SHIFT;
    const int32_t integ_i_new = 0;
#endif
    const uint32_t phase = TIMER_A.CNTxR;
    if (UNLIKELY(phase < BUCK_SAMPLE_COMPARE || phase >= BUCK_WRITE_DEADLINE))
        buck_fatal(BUCK_FAULT_DEADLINE);
#if BUCK_OPTIMAL
    if (UNLIKELY(st != OPT_IDLE)) {
        /* PULSE / SETTLE: the cold path does this cycle's HRTIM writes,
         * integrator updates and state transition. No trigger. */
        opt_cold(st, duty, u - p, integ_i_new, adc, adc_raw, error, i_meas_q16, i_avg_q16);
        g_buck.status.duty_ticks = duty;
        goto done_adc;
    }
#endif
    TIMER_A.CMP1xR = duty; /* ONE write, applied at next PER/REP. */
#if BUCK_DIAG_WRITE_PHASE
    __DSB();
    const uint32_t wphase = TIMER_A.CNTxR; /* phase just after the write */
    if (wphase > g_buck_write_phase_max) g_buck_write_phase_max = wphase;
#endif
#if BUCK_SCOPE_PIN
    GPIOB->BSRR = 1U << (5 + 16);
#endif
    /* --- Everything below is off the critical path (post-write). --- */
    control.integrator_q16 = u - p; /* saturation-tracking anti-windup */
#if BUCK_CURRENT_MODE
    control.integrator_i_q16 = integ_i_new;
    g_buck.status.d_term = integ_i_new;
#endif
    const uint32_t target_q16 = g_buck.tune.setpoint_adc << 16;
    if (UNLIKELY(control.reference_q16 != target_q16)) reference_step(target_q16);
    g_buck.status.setpoint = setpoint;
    g_buck.status.error = error;
    g_buck.status.i_term = control.integrator_q16;
    g_buck.status.duty_ticks = duty;
    g_buck.status.i_avg_q16 = i_avg_q16;
    const int32_t dv = (int32_t)adc - (int32_t)prev;     /* one-period change of v_out, counts */
    control.adc_prev = adc;
    control.adc_raw_prev = adc_raw;
#if BUCK_ESTIMATOR
    /* Load estimate from the measured current (eqs 7-8, 20-21): no model
     * current, no table. */
    const int32_t vout = buck_v_q28(adc);
    const int32_t iavg_q20 = BUCK_SHL(i_avg_q16, 4);
    buck_load_update(&control.est, iavg_q20, vout);
    const uint32_t duty_active = control.active_duty;   /* the on-time that produced this period's current */
    control.active_duty = duty; /* becomes active next period */
#if BUCK_EST_DISPLAY
    g_buck.status.est_iload_ma = buck_q20_to_ma(control.est.slow_iload_q20);
#endif
    g_buck.status.est_iload_a = control.est.iload_q20 >> 20;
#if BUCK_OPTIMAL
    /* Time-optimal load-step-up response, runtime-gated by tune.optimal_en.
     * opt_cooldown masks the trigger during soft-start (re-armed by the ramp
     * every cycle), for the linear-controller transition after a pulse and
     * briefly after a rejected solve. State-anchored trigger on the output-
     * capacitor state, EVERY cycle, integer:
     *   z_v = v_out - V_ref (counts),  d_v = v_out(n) - v_out(n-1) (counts):
     *   fire when S = (z_v/V_a)^2 + (d_v/dV_a)^2 + 2 rho (..)(..) > 1 on the
     *   droop side (z_v < 0, d_v <= 0: below the reference and not recovering;
     *   the solver is the step-up ON-OFF order). The two sign tests come first
     *   so the score is only computed while the output is below and falling. */
    if (LIKELY(g_buck.tune.optimal_en)) {
        const int32_t zv = -error;                            /* v_out - V_ref */
        if (UNLIKELY(control.opt_cooldown != 0U)) {
            /* When the transition after a real pulse ends, the loop has settled
             * at the new load: close the record for the thread. */
            if (--control.opt_cooldown == 0U && control.settle_pending != 0U) opt_settled(error);
        } else if (zv < 0 && dv <= 0) {
            g_buck.opt.sa_zv = zv;                            /* debug-panel score: the last candidate sample */
            g_buck.opt.sa_dv = dv;
            if (UNLIKELY(buck_sa_trigger_i(zv, dv, &g_buck.sa))) {
                /* Load step-up detected: capture the state for the solve next cycle.
                 * The new load itself is measured next cycle by the solve (i_avg -
                 * C dv/dt over the period AFTER the edge: this cycle's dv is the one
                 * with the capacitor's ESR step on it). */
                buck_opt_rec_t *const r = &REC;
                r->zv0 = zv; r->dv0 = dv; r->adc0 = adc;
                r->iavg0_q20 = iavg_q20; r->duty0 = duty_active;
                g_buck.opt.trig = 1U;
                ++g_buck.status.opt_trig_count; /* trigger events (fires even if the solve rejects) */
                control.opt_state = OPT_WAIT;
                control.running = 0U;   /* next cycle: cold entry -> opt_solve (no hot-path compare) */
            }
        }
    }
#else
    (void)dv; (void)duty_active;
#endif
#else
    (void)dv;
    control.active_duty = duty;
#endif
done_adc:
    g_buck.status.adc_raw = adc_raw;
done:
#if BUCK_SCOPE_PIN
    GPIOB->BSRR = 1U << (5 + 16);
#endif
#if BUCK_DIAGNOSTICS
    { const uint32_t el = DWT->CYCCNT - t_entry;
      if (el > g_buck_isr_cycles_max) g_buck_isr_cycles_max = el; }
#endif
    ;
}

void NMI_Handler(void)       { buck_fatal(BUCK_FAULT_CPU); }
void HardFault_Handler(void) { buck_fatal(BUCK_FAULT_CPU); }
void MemManage_Handler(void) { buck_fatal(BUCK_FAULT_CPU); }
void BusFault_Handler(void)  { buck_fatal(BUCK_FAULT_CPU); }
void UsageFault_Handler(void){ buck_fatal(BUCK_FAULT_CPU); }
