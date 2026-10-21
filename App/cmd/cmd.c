/**
  ******************************************************************************
  * @file    cmd.c
  * @author  niko（需求） / pi coding agent（实现）
  * @version V1.0.0
  * @date    2026-10-20
  * @brief   串口文本命令解析（整数 + 单字母指令，无浮点、无 sscanf）
  ******************************************************************************
  * @attention
  *
  * 解析是"边收边判"的状态机，不是"收完一整行再 strtok"：
  *
  *   收到 'S' → 进入"参数"状态，开始累加数字（带符号）
  *   收到 '\n'/'\r' → 执行，回到空闲
  *   其它字符 → 丢弃
  *
  * 为什么不用 sscanf("%d")：
  *   · 它要拖进一整块 libc 的格式化扫描代码（Flash 紧张，而且行为对怪输入不明确）
  *   · 它不会告诉你"这行是 S 还是 X"，还得先判断首字符
  *   · 手写累加只有 10 行，而且**溢出可控**（见 CMD_AccumulateDigit）
  *
  ******************************************************************************
  */

#include "cmd.h"

/* ==========================================================================
 * 私有状态
 * ========================================================================== */

typedef enum
{
  CMD_ST_IDLE = 0,      /* 等命令首字符 */
  CMD_ST_NUMBER,        /* 已经在收 S 后面的数字（整数）*/
  CMD_ST_GAIN_FLAG,     /* 收到 K，等 P/I/D */
  CMD_ST_GAIN_NUM       /* 收到 KP/KI/KD，正在收浮点值 */
} CMD_State_e;

static CMD_State_e s_state;
static int32_t     s_target_rpm;
static int32_t     s_pending;        /* 正在累加的数字 */
static uint8_t     s_negative;
static uint8_t     s_has_digit;
static uint8_t     s_line_len;       /* 本行已收字符数（防超长） */
static uint8_t     s_drop_line;      /* 1 = 本行已判废，丢弃到行尾 */
static uint16_t    s_idle_ticks;     /* 半截命令已经停了多少个 5 ms */

static uint8_t     s_stop_req;
static uint8_t     s_status_req;
static uint8_t     s_calib_toggle_req;
static uint8_t     s_autostep_toggle_req;
static uint8_t     s_autostep = CMD_AUTOSTEP_DEFAULT;
static uint8_t     s_vofa = CMD_VOFA_DEFAULT;
static uint8_t     s_vofa_toggle_req;

/* ---- 实时增益（KP/KI/KD 命令 → VOFA+ 滑块）---- */
static uint8_t     s_gain_which;     /* 0=Kp 1=Ki 2=Kd */
static int32_t     s_frac;           /* 小数部分累加值 */
static uint8_t     s_frac_digits;    /* 已收小数位数 */
static uint8_t     s_has_dot;
static float       s_gain_req[3];
static uint8_t     s_gain_valid;     /* 1 = 有新值待取 */
static uint8_t     s_gain_mask;      /* bit0=Kp bit1=Ki bit2=Kd：只回写被命令改过的那个
                                      * （没这个的话，发 KP20 会把 Ki/Kd 一起刷成 0 —— 实测踩过）*/

/* ==========================================================================
 * 私有函数
 * ========================================================================== */

static int32_t CMD_ClampTarget(int32_t rpm)
{
  if (rpm >  CMD_TARGET_MAX) { return  CMD_TARGET_MAX; }
  if (rpm < -CMD_TARGET_MAX) { return -CMD_TARGET_MAX; }
  return rpm;
}

static void CMD_AccumulateDigit(char c)
{
  int32_t digit = (int32_t)(c - '0');

  /* 累加到一半就超过上限时直接卡住，避免 int32 溢出（灌 100 位数字也不会出事） */
  if (s_pending > (CMD_TARGET_MAX * 10))
  {
    s_pending = CMD_TARGET_MAX + 1;   /* 足够大，最后会被限幅 */
    return;
  }

  s_pending = s_pending * 10 + digit;
  s_has_digit = 1U;
}

/** 增益数字的累积：整数部分进 s_pending，小数部分进 s_frac（最多 4 位）*/
static void CMD_AccumulateGainDigit(char c)
{
  int32_t digit = (int32_t)(c - '0');

  if (s_has_dot == 0U)
  {
    if (s_pending < 10000) { s_pending = s_pending * 10 + digit; }
  }
  else if (s_frac_digits < 4U)
  {
    s_frac = s_frac * 10 + digit;
    s_frac_digits++;
  }
  s_has_digit = 1U;
}

/** 把已收的增益数字拼成 float，并按各增益的上限钳位 */
static float CMD_GainValue(float limit)
{
  float v = (float)s_pending;
  float scale = 1.0f;
  uint8_t i;

  for (i = 0U; i < s_frac_digits; i++) { scale *= 10.0f; }
  v += (float)s_frac / scale;

  if (s_negative != 0U) { v = -v; }

  /* 不允许负增益（负 Kp 就是正反馈，直接飞车）与超限值 */
  if (v < 0.0f)   { v = 0.0f; }
  if (v > limit)  { v = limit; }
  return v;
}

/** 执行一条已经收完的命令 */
static void CMD_Execute(void)
{
  if (s_drop_line != 0U)
  {
    return;                           /* 整行作废 */
  }

  switch (s_state)
  {
    case CMD_ST_NUMBER:
      if (s_has_digit != 0U)
      {
        int32_t value = (s_negative != 0U) ? -s_pending : s_pending;
        s_target_rpm = CMD_ClampTarget(value);
        s_autostep   = 0U;            /* 外部给速后，自动阶跃让位（否则会互相打架） */
      }
      break;

    case CMD_ST_GAIN_NUM:
      if (s_has_digit != 0U)
      {
        float limit = (s_gain_which == 0U) ? CMD_KP_MAX
                    : (s_gain_which == 1U) ? CMD_KI_MAX : CMD_KD_MAX;
        s_gain_req[s_gain_which] = CMD_GainValue(limit);
        s_gain_mask |= (uint8_t)(1U << s_gain_which);
        s_gain_valid = 1U;
      }
      break;

    case CMD_ST_IDLE:
    case CMD_ST_GAIN_FLAG:              /* K 后面没跟 P/I/D → 作废 */
    default:
      break;                          /* 单字母指令在 CMD_FeedByte 里已经处理 */
  }
}

/* ==========================================================================
 * 对外接口
 * ========================================================================== */

void CMD_Reset(void)
{
  s_state             = CMD_ST_IDLE;
  s_target_rpm        = 0;
  s_pending           = 0;
  s_negative          = 0U;
  s_has_digit         = 0U;
  s_line_len          = 0U;
  s_drop_line         = 0U;
  s_idle_ticks        = 0U;
  s_stop_req          = 0U;
  s_status_req        = 0U;
  s_calib_toggle_req  = 0U;
  s_gain_which        = 0U;
  s_frac              = 0;
  s_frac_digits       = 0U;
  s_has_dot           = 0U;
  s_gain_valid        = 0U;
  s_gain_mask         = 0U;
  s_autostep_toggle_req = 0U;
  s_autostep          = CMD_AUTOSTEP_DEFAULT;
  s_vofa              = CMD_VOFA_DEFAULT;
  s_vofa_toggle_req   = 0U;
}

void CMD_PollTimeout(void)
{
#if (CMD_FLUSH_TIMEOUT_MS > 0U)
  if ((s_state == CMD_ST_IDLE) && (s_pending == 0) && (s_line_len == 0U))
  {
    s_idle_ticks = 0U;
    return;
  }

  s_idle_ticks++;

  if (s_idle_ticks >= (uint16_t)(CMD_FLUSH_TIMEOUT_MS / 5U))
  {
    CMD_Execute();
    s_state      = CMD_ST_IDLE;
    s_pending    = 0;
    s_negative   = 0U;
    s_has_digit  = 0U;
    s_line_len   = 0U;
    s_drop_line  = 0U;
    s_idle_ticks = 0U;
  }
#endif
}

void CMD_FeedByte(char c)
{
  s_idle_ticks = 0U;
  /* ---- 行尾：结算本行 ---- */
  if ((c == '\n') || (c == '\r'))
  {
    CMD_Execute();
    s_state       = CMD_ST_IDLE;
    s_pending     = 0;
    s_negative    = 0U;
    s_has_digit   = 0U;
    s_line_len    = 0U;
    s_drop_line   = 0U;
    s_frac        = 0;
    s_frac_digits = 0U;
    s_has_dot     = 0U;
    return;
  }

  /* ---- 超长行：本行作废，但继续吞到行尾（不会溢出，也不会半行生效）---- */
  if (s_line_len >= (uint8_t)(CMD_LINE_MAX - 1U))
  {
    s_drop_line = 1U;
    return;
  }
  s_line_len++;

  /* ---- 正在收数字 ---- */
  if (s_state == CMD_ST_NUMBER)
  {
    if ((c >= '0') && (c <= '9'))
    {
      if (s_drop_line == 0U) { CMD_AccumulateDigit(c); }
      return;
    }
    if ((c == '-') && (s_has_digit == 0U))
    {
      s_negative = 1U;
      return;
    }
    /* 数字里混进别的字符 → 整行作废 */
    s_drop_line = 1U;
    return;
  }

  /* ---- 收完 K，等 P / I / D ---- */
  if (s_state == CMD_ST_GAIN_FLAG)
  {
    if ((c == 'P') || (c == 'p'))      { s_gain_which = 0U; }
    else if ((c == 'I') || (c == 'i')) { s_gain_which = 1U; }
    else if ((c == 'D') || (c == 'd')) { s_gain_which = 2U; }
    else { s_drop_line = 1U; return; }

    s_state = CMD_ST_GAIN_NUM;
    return;
  }

  /* ---- 正在收增益值（支持小数）---- */
  if (s_state == CMD_ST_GAIN_NUM)
  {
    if ((c >= '0') && (c <= '9'))
    {
      if (s_drop_line == 0U) { CMD_AccumulateGainDigit(c); }
      return;
    }
    if ((c == '.') && (s_has_dot == 0U))
    {
      s_has_dot = 1U;                 /* 第二个小数点会被判废 */
      return;
    }
    if ((c == '-') && (s_has_digit == 0U) && (s_has_dot == 0U))
    {
      s_negative = 1U;
      return;
    }
    s_drop_line = 1U;
    return;
  }

  /* ---- 空闲态：判首字符（大小写都收）---- */
  if (s_drop_line != 0U) { return; }

  if ((c == 'S') || (c == 's'))
  {
    s_state     = CMD_ST_NUMBER;
    s_pending   = 0;
    s_negative  = 0U;
    s_has_digit = 0U;
    return;
  }

  if ((c == 'V') || (c == 'v'))
  {
    s_vofa_toggle_req = 1U;
    return;
  }

  if ((c == 'X') || (c == 'x'))
  {
    s_target_rpm = 0;
    s_autostep   = 0U;
    s_stop_req   = 1U;
    return;
  }

  if ((c == 'A') || (c == 'a'))
  {
    s_autostep_toggle_req = 1U;
    return;
  }

  if (c == '?')
  {
    s_status_req = 1U;
    return;
  }

  if ((c == 'C') || (c == 'c'))
  {
    s_calib_toggle_req = 1U;
    return;
  }

  if ((c == 'K') || (c == 'k'))
  {
    s_state = CMD_ST_GAIN_FLAG;        /* 后面紧跟 P / I / D */
    return;
  }

  /* 其它字符：忽略（空格、\t、杂字符都不算错） */
}

int32_t CMD_GetTargetRpm(void)
{
  return s_target_rpm;
}

void CMD_SetTargetRpm(int32_t rpm)
{
  s_target_rpm = CMD_ClampTarget(rpm);
}

uint8_t CMD_TakeStopRequest(void)
{
  uint8_t v = s_stop_req;
  s_stop_req = 0U;
  return v;
}

uint8_t CMD_TakeStatusRequest(void)
{
  uint8_t v = s_status_req;
  s_status_req = 0U;
  return v;
}

uint8_t CMD_TakeCalibToggle(void)
{
  uint8_t v = s_calib_toggle_req;
  s_calib_toggle_req = 0U;
  return v;
}

uint8_t CMD_TakeGains(float *kp, float *ki, float *kd)
{
  uint8_t v = s_gain_valid;

  if (v != 0U)
  {
    /* 只回写被改过的字段：没收到 KI 命令时就不能动 ki */
    if ((kp != 0) && ((s_gain_mask & 0x01U) != 0U)) { *kp = s_gain_req[0]; }
    if ((ki != 0) && ((s_gain_mask & 0x02U) != 0U)) { *ki = s_gain_req[1]; }
    if ((kd != 0) && ((s_gain_mask & 0x04U) != 0U)) { *kd = s_gain_req[2]; }
    s_gain_valid = 0U;
    s_gain_mask  = 0U;
  }
  return v;
}

uint8_t CMD_TakeAutoStepToggle(void)
{
  uint8_t v = s_autostep_toggle_req;
  s_autostep_toggle_req = 0U;
  return v;
}

uint8_t CMD_GetAutoStep(void)
{
  return s_autostep;
}

void CMD_SetAutoStep(uint8_t on)
{
  s_autostep = (on != 0U) ? 1U : 0U;
}

uint8_t CMD_GetVofa(void)
{
  return s_vofa;
}

void CMD_SetVofa(uint8_t on)
{
  s_vofa = (on != 0U) ? 1U : 0U;
}

uint8_t CMD_TakeVofaToggle(void)
{
  uint8_t v = s_vofa_toggle_req;
  s_vofa_toggle_req = 0U;
  return v;
}
