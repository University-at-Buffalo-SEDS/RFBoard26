/* Host tests for Core/Src/board_crc32.c against a model of the STM32 CRC
 * unit (CR/INIT/POL/DR, REV_IN/REV_OUT, MSB-first shift register, 8/16/32-bit
 * DR writes) and a bitwise software reference. */
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "board_crc32.h"
#include "board_crc32_host_model.h"
#include "sedsnet.h"

static int failures;
#define CHECK(cond)                                                          \
  do {                                                                       \
    if (!(cond)) {                                                           \
      fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
      ++failures;                                                            \
    }                                                                        \
  } while (0)

/* ---- reference ---------------------------------------------------------- */
static uint32_t ref_crc(uint32_t crc, const uint8_t *p, size_t n)
{
  crc = ~crc;
  while (n--) {
    crc ^= *p++;
    for (int i = 0; i < 8; ++i) crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320U : 0U);
  }
  return ~crc;
}

/* ---- SEDSnet stub ------------------------------------------------------- */
static SedsCrc32Fn installed_fn;
static int install_calls;
void seds_set_crc32_fn(SedsCrc32Fn fn)
{
  installed_fn = fn;
  ++install_calls;
}

/* ---- CRC unit model ----------------------------------------------------- */
enum fault {
  FAULT_NONE,
  FAULT_ABSENT,         /* unmapped: reads 0, writes ignored */
  FAULT_FIXED_POLY,     /* POL not programmable (CRC-32C) */
  FAULT_NO_REV_OUT,     /* REV_OUT ignored */
  FAULT_NO_REV_IN,      /* REV_IN ignored */
  FAULT_IGNORE_INIT,    /* RESET does not load INIT */
  FAULT_WIDE_BYTES,     /* 8-bit DR writes processed as 32 bits */
  FAULT_COUNT
};
static enum fault fault;
static bool clock_on;
static uint32_t m_pol = 0x04C11DB7U, m_init = 0xFFFFFFFFU, m_cr, m_state = 0xFFFFFFFFU;
static atomic_ulong dr8_writes, dr32_writes, model_ops, hw_calls;
static _Atomic(pthread_t *) owner;
static __thread pthread_t self_id;
static void (*on_dr_write)(void);

static void model_reset(enum fault f)
{
  fault = f;
  clock_on = false;
  m_pol = 0x04C11DB7U;
  m_init = 0xFFFFFFFFU;
  m_cr = 0U;
  m_state = 0xFFFFFFFFU;
  dr8_writes = dr32_writes = model_ops = 0U;
  owner = NULL;
  on_dr_write = NULL;
}

/* A hardware call runs INIT .. DR read; only one context may be inside. */
static void model_enter(void)
{
  self_id = pthread_self();
  pthread_t *expected = NULL;
  if (!atomic_compare_exchange_strong(&owner, &expected, &self_id)) {
    fprintf(stderr, "CRC unit entered by two contexts\n");
    abort();
  }
}
static void model_owned(void)
{
  if (atomic_load(&owner) != &self_id) {
    fprintf(stderr, "CRC data access outside an owned call\n");
    abort();
  }
}

void crc_model_clock_enable(void) { clock_on = true; }
void crc_model_write_pol(uint32_t v)
{
  ++model_ops;
  if (!clock_on) abort();
  if (fault != FAULT_ABSENT && fault != FAULT_FIXED_POLY) m_pol = v;
  if (fault == FAULT_FIXED_POLY) m_pol = 0x1EDC6F41U;
}
void crc_model_write_init(uint32_t v)
{
  ++model_ops;
  if (!clock_on) abort();
  model_enter();
  if (fault != FAULT_ABSENT) m_init = v;
}
void crc_model_write_cr(uint32_t v)
{
  ++model_ops;
  if (!clock_on) abort();
  if (atomic_load(&owner) != NULL) model_owned();
  if (fault == FAULT_ABSENT) return;
  if ((v & 0x18U) != 0U) abort(); /* only 32-bit POLYSIZE is used */
  m_cr = v & 0xF8U;
  if ((v & 1U) && fault != FAULT_IGNORE_INIT) m_state = m_init;
}
static uint32_t reflect(uint32_t v, unsigned w)
{
  uint32_t r = 0;
  for (unsigned i = 0; i < w; ++i) {
    r = (r << 1) | (v & 1U);
    v >>= 1;
  }
  return r;
}
void crc_model_write_dr(uint32_t v, unsigned width)
{
  ++model_ops;
  if (!clock_on) abort();
  model_owned();
  if (width == 8U) ++dr8_writes; else ++dr32_writes;
  if (on_dr_write) {
    void (*cb)(void) = on_dr_write;
    on_dr_write = NULL;
    cb();
  }
  if (fault == FAULT_ABSENT) return;
  if (fault == FAULT_WIDE_BYTES) width = 32U;
  v &= width == 32U ? 0xFFFFFFFFU : ((1U << width) - 1U);
  unsigned mode = (m_cr >> 5) & 3U;
  if (fault == FAULT_NO_REV_IN) mode = 0U;
  if (mode != 0U) {
    unsigned group = mode == 1U ? 8U : mode == 2U ? 16U : 32U;
    if (group > width) abort();
    uint32_t out = 0;
    for (unsigned s = 0; s < width; s += group) {
      uint32_t mask = group == 32U ? 0xFFFFFFFFU : ((1U << group) - 1U);
      out |= reflect((v >> s) & mask, group) << s;
    }
    v = out;
  }
  for (int bit = (int)width - 1; bit >= 0; --bit) {
    uint32_t fb = ((m_state >> 31) ^ (v >> bit)) & 1U;
    m_state <<= 1;
    if (fb) m_state ^= m_pol;
  }
}
uint32_t crc_model_read_dr(void)
{
  ++model_ops;
  if (!clock_on) abort();
  model_owned();
  uint32_t r = 0U;
  if (fault != FAULT_ABSENT) {
    r = (m_cr & 0x80U) && fault != FAULT_NO_REV_OUT ? reflect(m_state, 32) : m_state;
  }
  ++hw_calls;
  atomic_store(&owner, NULL);
  return r;
}

/* Raw model use (no glue) to pin the model to published STM32 behaviour. */
static uint32_t model_raw(uint32_t cr, const uint8_t *p, size_t n, unsigned width)
{
  crc_model_write_init(0xFFFFFFFFU);
  crc_model_write_cr(cr | 1U);
  for (size_t i = 0; i < n; ++i) crc_model_write_dr(p[i], width);
  return crc_model_read_dr();
}

static uint64_t rng = 0x243F6A8885A308D3ULL;
static uint32_t rnd(void)
{
  rng ^= rng << 13;
  rng ^= rng >> 7;
  rng ^= rng << 17;
  return (uint32_t)rng;
}

static const uint8_t check9[] = "123456789";

static void test_model_matches_stm32_reference(void)
{
  model_reset(FAULT_NONE);
  crc_model_clock_enable();
  /* Default unit (no reversal): byte-wise "123456789" is CRC-32/MPEG-2. */
  CHECK(model_raw(0U, check9, 9, 8U) == 0x0376E6E7U);
  /* Default unit, one 32-bit write of 0x12345678. */
  crc_model_write_init(0xFFFFFFFFU);
  crc_model_write_cr(1U);
  crc_model_write_dr(0x12345678U, 32U);
  CHECK(crc_model_read_dr() == 0xDF8A8A2BU);
  /* REV_IN byte + REV_OUT byte-wise gives the reflected (IEEE) register. */
  CHECK(~model_raw(0xA0U, check9, 9, 8U) == 0xCBF43926U);
}

static void test_software_path(void)
{
  uint8_t buf[2048];
  for (size_t i = 0; i < sizeof buf; ++i) buf[i] = (uint8_t)rnd();
  CHECK(board_crc32_software(0U, check9, 9) == 0xCBF43926U);
  CHECK(board_crc32_software(0x1234U, buf, 0) == 0x1234U);
  for (int t = 0; t < 200; ++t) {
    size_t off = rnd() % 64U, len = rnd() % 1500U;
    uint32_t seed = rnd();
    CHECK(board_crc32_software(seed, buf + off, len) == ref_crc(seed, buf + off, len));
  }
}

static void test_install_and_vectors(void)
{
  model_reset(FAULT_NONE);
  installed_fn = NULL;
  install_calls = 0;
  CHECK(board_crc32_install());
  CHECK(install_calls == 1 && installed_fn == board_crc32);
  CHECK(g_board_crc32_state == 1U);
  CHECK(clock_on && m_pol == 0x04C11DB7U);
  CHECK(atomic_load(&owner) == NULL);

  CHECK(board_crc32(0U, check9, 9) == 0xCBF43926U);
  /* Chained halves give the same CRC at every split. */
  for (size_t s = 0; s <= 9; ++s) {
    CHECK(board_crc32(board_crc32(0U, check9, s), check9 + s, 9 - s) == 0xCBF43926U);
  }
  /* len == 0 must not read data or touch the unit. */
  unsigned long before = model_ops;
  CHECK(board_crc32(0xDEADBEEFU, (const uint8_t *)(uintptr_t)1, 0) == 0xDEADBEEFU);
  CHECK(board_crc32(0U, NULL, 0) == 0U);
  CHECK(model_ops == before);

  static uint8_t buf[1600] __attribute__((aligned(8)));
  for (size_t i = 0; i < sizeof buf; ++i) buf[i] = (uint8_t)rnd();
  static const size_t lens[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 63, 64, 1500};
  for (size_t off = 0; off < 8; ++off) {
    for (size_t i = 0; i < sizeof lens / sizeof lens[0]; ++i) {
      for (int s = 0; s < 3; ++s) {
        uint32_t seed = s == 0 ? 0U : rnd();
        CHECK(board_crc32(seed, buf + off, lens[i]) == ref_crc(seed, buf + off, lens[i]));
      }
    }
  }
  /* Aligned bulk goes through 32-bit writes; unaligned edges through bytes. */
  dr8_writes = dr32_writes = 0;
  CHECK(board_crc32(0U, buf, 64) == ref_crc(0U, buf, 64));
  CHECK(dr32_writes == 16U && dr8_writes == 0U);
  dr8_writes = dr32_writes = 0;
  CHECK(board_crc32(0U, buf + 1, 66) == ref_crc(0U, buf + 1, 66));
  CHECK(dr32_writes == 15U && dr8_writes == 6U); /* 3 head, 15 words, 3 tail */
  /* Random buffers, starts and multi-chunk chains. */
  for (int t = 0; t < 500; ++t) {
    size_t off = rnd() % 16U, len = rnd() % 1500U;
    uint32_t want = ref_crc(0U, buf + off, len), got = 0U;
    size_t done = 0;
    while (done < len) {
      size_t take = 1U + rnd() % (len - done);
      got = board_crc32(got, buf + off + done, take);
      done += take;
    }
    CHECK(got == want);
  }
}

/* An "ISR" preempting a hardware call mid-sequence. */
static uint8_t isr_buf[300];
static uint32_t isr_result;
static unsigned long isr_model_ops;
static void isr_during_hw_call(void)
{
  unsigned long before = model_ops;
  isr_result = board_crc32(0x55AA55AAU, isr_buf + 1, sizeof isr_buf - 1);
  isr_model_ops = model_ops - before;
}

static void test_contention_falls_back_to_software(void)
{
  model_reset(FAULT_NONE);
  installed_fn = NULL;
  CHECK(board_crc32_install());
  uint8_t buf[257];
  for (size_t i = 0; i < sizeof buf; ++i) buf[i] = (uint8_t)rnd();
  for (size_t i = 0; i < sizeof isr_buf; ++i) isr_buf[i] = (uint8_t)rnd();
  for (int where = 0; where < 3; ++where) {
    isr_result = 0U;
    isr_model_ops = 99U;
    on_dr_write = isr_during_hw_call;
    const uint8_t *p = buf + where;
    uint32_t outer = board_crc32(0x01020304U, p, 200);
    CHECK(on_dr_write == NULL); /* the preemption really happened */
    CHECK(outer == ref_crc(0x01020304U, p, 200));
    CHECK(isr_result == ref_crc(0x55AA55AAU, isr_buf + 1, sizeof isr_buf - 1));
    CHECK(isr_model_ops == 0U); /* nested call never touched the unit */
  }
  /* The lock is released: the next call uses the unit again. */
  dr32_writes = 0;
  CHECK(board_crc32(0U, check9, 9) == 0xCBF43926U);
  CHECK(dr32_writes == 2U);
}

/* Real threads: every result correct, the unit never entered twice (the
 * model aborts on overlap), and both paths exercised. */
#define THREADS 6
#define ITERS 4000
static uint8_t shared_buf[1600];
static atomic_int thread_failures;
static atomic_ulong thread_calls;
static void *worker(void *arg)
{
  uint64_t s = (uint64_t)(uintptr_t)arg * 0x9E3779B97F4A7C15ULL + 1U;
  for (int i = 0; i < ITERS; ++i) {
    s ^= s << 13;
    s ^= s >> 7;
    s ^= s << 17;
    size_t off = s % 16U, len = (s >> 8) % 1500U;
    uint32_t seed = (uint32_t)(s >> 32);
    if (len != 0U) atomic_fetch_add(&thread_calls, 1);
    if (board_crc32(seed, shared_buf + off, len) != ref_crc(seed, shared_buf + off, len)) {
      atomic_fetch_add(&thread_failures, 1);
    }
  }
  return NULL;
}
static void test_threads(void)
{
  model_reset(FAULT_NONE);
  CHECK(board_crc32_install());
  for (size_t i = 0; i < sizeof shared_buf; ++i) shared_buf[i] = (uint8_t)rnd();
  pthread_t t[THREADS];
  hw_calls = 0;
  for (uintptr_t i = 0; i < THREADS; ++i) pthread_create(&t[i], NULL, worker, (void *)(i + 1));
  for (int i = 0; i < THREADS; ++i) pthread_join(t[i], NULL);
  CHECK(thread_failures == 0);
  CHECK(hw_calls > 0U && hw_calls <= thread_calls);
  printf("threads: %lu calls, %lu hardware, %lu software (contended)\n",
         (unsigned long)thread_calls, (unsigned long)hw_calls,
         (unsigned long)(thread_calls - hw_calls));
}

static void test_failed_self_test_installs_nothing(void)
{
  for (int f = FAULT_ABSENT; f < FAULT_COUNT; ++f) {
    model_reset((enum fault)f);
    installed_fn = NULL;
    install_calls = 0;
    bool ok = board_crc32_install();
    if (ok || install_calls != 0 || installed_fn != NULL || g_board_crc32_state != 2U) {
      fprintf(stderr, "fault %d was installed\n", f);
      ++failures;
    }
    CHECK(atomic_load(&owner) == NULL);
  }
}

int main(int argc, char **argv)
{
  const char *only = argc > 1 ? argv[1] : NULL;
#define RUN(name)                                        \
  do {                                                   \
    if (!only || strcmp(only, #name) == 0) {             \
      int before = failures;                             \
      name();                                            \
      printf("%s %s\n", failures == before ? "PASS" : "FAIL", #name); \
    }                                                    \
  } while (0)
  RUN(test_model_matches_stm32_reference);
  RUN(test_software_path);
  RUN(test_install_and_vectors);
  RUN(test_contention_falls_back_to_software);
  RUN(test_failed_self_test_installs_nothing);
  RUN(test_threads);
  return failures != 0;
}
