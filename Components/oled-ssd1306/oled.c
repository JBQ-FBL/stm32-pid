/**
  * @file    oled.c
  * @brief   0.96 寸 OLED（SSD1306，128x64）驱动 —— 软件模拟 I2C，基于 HAL
  *
  * ── 来源 ─────────────────────────────────────────────────
  * 本项目自研：niko 提出需求、pi coding agent 于 2026-10-13 生成，未拷贝任何第三方代码。
  * 参考：SSD1306 数据手册；江协科技《STM32入门教程-2023版》的接口约定（函数命名、
  *       line/column 坐标系）—— 只是为了方便对照教程看代码。
  * 详见同目录 README「来源与许可」。
  * ────────────────────────────────────────────────────────
  *
  * 代码结构（从上到下，跟江协科技 2023 版教程一致，方便你对照视频）：
  *
  *   1) 底层时序：OLED_I2C_Start / Stop / SendByte   —— I2C 协议本身
  *   2) 命令与数据：OLED_WriteCommand / OLED_WriteDataN —— SSD1306 的两种"包裹"
  *   3) 光标与清屏：OLED_SetCursor / OLED_Clear
  *   4) 初始化：OLED_Init（一段固定的寄存器命令表）
  *   5) 应用层：OLED_ShowChar / String / Printf / Num ...
  *
  * ── 为什么 SDA 要来回切输入/输出 ─────────────────────────────────
  * I2C 的 SDA 是"线与"总线：谁都不拉低时靠上拉电阻变高。
  * 从机回 ACK 的时候，主机必须先松手（把 SDA 变成高阻），再去看电平。
  * 在 STM32F1 上，开漏输出模式下【内部上拉是无效的】——因为 F1 没有独立的
  * 上拉/下拉寄存器，上下拉是用 ODR 位编码的，而输出模式下 ODR 被输出值占用了。
  * 所以这里读 ACK 前把 SDA 切成「上拉输入」（这时内部上拉才真正生效），
  * 读完再切回「开漏输出」。好处：就算模块上没焊上拉电阻，通信也能work。
  * ────────────────────────────────────────────────────────────
  */

#include "oled.h"
#include "oled_font.h"
#include <stdarg.h>
#include <stdio.h>

/* ==========================================================================
 * 0. 私有函数声明
 * ========================================================================== */
static void    OLED_I2C_Delay(void);
static void    OLED_GPIO_Init(void);
static void    OLED_SDA_SetOutput(void);
static void    OLED_SDA_SetInput(void);

static void    OLED_I2C_Start(void);
static void    OLED_I2C_Stop(void);
static uint8_t OLED_I2C_SendByte(uint8_t byte);
static uint8_t OLED_I2C_Probe(void);

static void    OLED_WriteCommand(uint8_t cmd);
static void    OLED_WriteDataN(const uint8_t *data, uint16_t count);
static void    OLED_SetCursor(uint8_t page, uint8_t x);

static uint32_t OLED_Pow(uint32_t base, uint32_t exp);

/* ==========================================================================
 * 1. 底层：SCL / SDA 电平操作
 * ========================================================================== */

/* 拉高 = "释放"总线（开漏输出，实际电平由上拉电阻决定），拉低 = 主动拉低 */
#define OLED_SCL_HIGH()   HAL_GPIO_WritePin(OLED_SCL_GPIO_PORT, OLED_SCL_GPIO_PIN, GPIO_PIN_SET)
#define OLED_SCL_LOW()    HAL_GPIO_WritePin(OLED_SCL_GPIO_PORT, OLED_SCL_GPIO_PIN, GPIO_PIN_RESET)
#define OLED_SDA_HIGH()   HAL_GPIO_WritePin(OLED_SDA_GPIO_PORT, OLED_SDA_GPIO_PIN, GPIO_PIN_SET)
#define OLED_SDA_LOW()    HAL_GPIO_WritePin(OLED_SDA_GPIO_PORT, OLED_SDA_GPIO_PIN, GPIO_PIN_RESET)
#define OLED_SDA_READ()   HAL_GPIO_ReadPin(OLED_SDA_GPIO_PORT, OLED_SDA_GPIO_PIN)

/**
  * @brief  软件 I2C 的半周期延时
  * @note   SSD1306 的 I2C 最快 400kHz（半周期 1.25us），我们远远跑不到，
  *         所以这里不需要精确延时，一个空循环就够稳。主频 8MHz 时约 3~6us。
  *         以后上了 72MHz 主频，这个循环会变快 9 倍，也依然远低于 400kHz。
  */
static void OLED_I2C_Delay(void)
{
	for (volatile uint8_t i = 0; i < 6; i++)
	{
		__NOP();
	}
}

/**
  * @brief  初始化本驱动用到的两个 GPIO（PB8 / PB9）
  * @note   驱动自带引脚初始化，不依赖 CubeMX 里的 GPIO 配置，
  *         这样以后在别的工程里直接拷这两个文件就能用。
  *         只动 PB8/PB9，不会影响其它引脚。
  */
static void OLED_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* 使能 SCL/SDA 所在端口的时钟（换到别的端口时，改 oled.h 里那两个 CLK_ENABLE 宏）*/
	OLED_SCL_GPIO_CLK_ENABLE();
	OLED_SDA_GPIO_CLK_ENABLE();

	/* SCL：全程开漏输出 */
	GPIO_InitStruct.Pin   = OLED_SCL_GPIO_PIN;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;             /* F1 的开漏输出无法内部上拉，靠模块上的电阻 */
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(OLED_SCL_GPIO_PORT, &GPIO_InitStruct);

	/* SDA：默认也是开漏输出（读 ACK 时会临时切成输入，见文件开头的说明） */
	OLED_SDA_SetOutput();

	/* 空闲状态：两根线都释放为高 */
	OLED_SCL_HIGH();
	OLED_SDA_HIGH();
}

/** @brief 把 SDA 切成开漏输出（主机的"说话"状态） */
static void OLED_SDA_SetOutput(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin   = OLED_SDA_GPIO_PIN;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(OLED_SDA_GPIO_PORT, &GPIO_InitStruct);
}

/** @brief 把 SDA 切成上拉输入（读从机 ACK 用；F1 上这才是真正使能内部上拉的唯一办法） */
static void OLED_SDA_SetInput(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin  = OLED_SDA_GPIO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(OLED_SDA_GPIO_PORT, &GPIO_InitStruct);
}

/* ==========================================================================
 * 2. I2C 协议：起始 / 停止 / 发一字节并读应答
 * ========================================================================== */

/**
  * @brief  I2C 起始条件
  * @note   时序：SCL 为高时，SDA 出现【下降沿】
  */
static void OLED_I2C_Start(void)
{
	OLED_SDA_HIGH();
	OLED_SCL_HIGH();
	OLED_I2C_Delay();

	OLED_SDA_LOW();          /* SCL 高 → SDA 拉低 = 起始 */
	OLED_I2C_Delay();

	OLED_SCL_LOW();          /* 钳住 SCL，准备发数据 */
	OLED_I2C_Delay();
}

/**
  * @brief  I2C 停止条件
  * @note   时序：SCL 为高时，SDA 出现【上升沿】
  */
static void OLED_I2C_Stop(void)
{
	OLED_SDA_LOW();
	OLED_SCL_HIGH();
	OLED_I2C_Delay();

	OLED_SDA_HIGH();         /* SCL 高 → SDA 释放为高 = 停止 */
	OLED_I2C_Delay();
}

/**
  * @brief  发送一个字节，并读从机的应答
  * @param  byte 要发的字节（高位在前）
  * @retval 0 = 从机应答（ACK），1 = 无人应答（NACK）
  */
static uint8_t OLED_I2C_SendByte(uint8_t byte)
{
	uint8_t i;
	uint8_t ack;

	/* --- 8 个数据位：SCL 低电平期间改 SDA，高电平期间从机采样 --- */
	for (i = 0; i < 8; i++)
	{
		if (byte & 0x80)
		{
			OLED_SDA_HIGH();
		}
		else
		{
			OLED_SDA_LOW();
		}
		byte <<= 1;

		OLED_I2C_Delay();
		OLED_SCL_HIGH();     /* 从机在这期间读走 SDA */
		OLED_I2C_Delay();
		OLED_SCL_LOW();
		OLED_I2C_Delay();
	}

	/* --- 第 9 个时钟：主机松手，从机拉低 SDA 表示收到 --- */
	OLED_SDA_SetInput();
	OLED_I2C_Delay();

	OLED_SCL_HIGH();
	OLED_I2C_Delay();

	ack = (OLED_SDA_READ() == GPIO_PIN_RESET) ? 0 : 1;

	OLED_SCL_LOW();
	OLED_I2C_Delay();
	OLED_SDA_SetOutput();

	return ack;
}

/**
  * @brief  探测屏在不在总线上（发一个从机地址，看有没有人应答）
  * @retval 0 = 有应答
  */
static uint8_t OLED_I2C_Probe(void)
{
	uint8_t ack;

	OLED_I2C_Start();
	ack = OLED_I2C_SendByte(OLED_I2C_ADDR);
	OLED_I2C_Stop();

	return ack;
}

/* ==========================================================================
 * 3. SSD1306 命令 / 数据
 * ========================================================================== */

/**
  * @brief  写一条命令
  * @note   I2C 帧：Start + 从机地址(写) + 0x00(控制字节:后面是命令) + 命令 + Stop
  */
static void OLED_WriteCommand(uint8_t cmd)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(OLED_I2C_ADDR);
	OLED_I2C_SendByte(0x00);
	OLED_I2C_SendByte(cmd);
	OLED_I2C_Stop();
}

/**
  * @brief  连续写 count 个数据字节（一次 I2C 传输发完，比逐字节省 3 倍时间）
  * @note   I2C 帧：Start + 从机地址(写) + 0x40(控制字节:后面是数据) + data[0..count-1] + Stop
  */
static void OLED_WriteDataN(const uint8_t *data, uint16_t count)
{
	uint16_t i;

	OLED_I2C_Start();
	OLED_I2C_SendByte(OLED_I2C_ADDR);
	OLED_I2C_SendByte(0x40);
	for (i = 0; i < count; i++)
	{
		OLED_I2C_SendByte(data[i]);
	}
	OLED_I2C_Stop();
}

/**
  * @brief  设置光标（页地址 + 列地址）
  * @param  page 页号 0~7（一整页 = 8 像素高）
  * @param  x    列号 0~127（一列 = 1 像素宽）
  */
static void OLED_SetCursor(uint8_t page, uint8_t x)
{
	OLED_WriteCommand(0xB0 | (page & 0x07));              /* 页地址：0xB0 + 页号 */
	OLED_WriteCommand(0x10 | ((x & 0xF0) >> 4));          /* 列地址高 4 位 */
	OLED_WriteCommand(0x00 | (x & 0x0F));                 /* 列地址低 4 位 */
}

/* ==========================================================================
 * 4. 初始化
 * ========================================================================== */

/**
  * @brief  OLED 初始化
  * @retval 0 = 屏有应答，初始化完成；非 0 = 屏没应答（接错线/没插/地址不对）
  */
uint8_t OLED_Init(void)
{
	OLED_GPIO_Init();

	HAL_Delay(20);                        /* 等 SSD1306 内部上电复位稳定 */

	if (OLED_I2C_Probe() != 0)
	{
		return 1;                         /* 屏没应答，别再往下发命令了 */
	}

	/* --- 以下是 SSD1306 128x64 的固定初始化命令表（一行一条，和江协的写法一致）--- */
	OLED_WriteCommand(0xAE);              /* 关显示 */
	OLED_WriteCommand(0xD5);              /* 设置时钟分频 / 振荡频率 */
	OLED_WriteCommand(0x80);
	OLED_WriteCommand(0xA8);              /* 设置多路复用率 = 1/64 */
	OLED_WriteCommand(0x3F);
	OLED_WriteCommand(0xD3);              /* 设置显示偏移 = 0 */
	OLED_WriteCommand(0x00);
	OLED_WriteCommand(0x40);              /* 设置显示起始行 = 0 */
	OLED_WriteCommand(0x8D);              /* 电荷泵设置 */
	OLED_WriteCommand(0x14);              /*   0x14 = 开启（不打开屏不亮！）*/
	OLED_WriteCommand(0x20);              /* 设置寻址模式 */
	OLED_WriteCommand(0x02);              /*   0x02 = 页寻址（我们驱动用的就是这种）*/
	OLED_WriteCommand(0xA1);              /* 段重映射：左右方向 */
	OLED_WriteCommand(0xC8);              /* COM 扫描方向：上下方向 */
	OLED_WriteCommand(0xDA);              /* 设置 COM 引脚硬件配置 */
	OLED_WriteCommand(0x12);
	OLED_WriteCommand(0x81);              /* 设置对比度 */
	OLED_WriteCommand(0xCF);              /*   数值越大越亮，0x00~0xFF */
	OLED_WriteCommand(0xD9);              /* 设置预充电周期 */
	OLED_WriteCommand(0xF1);
	OLED_WriteCommand(0xDB);              /* 设置 VCOMH 取消选择电平 */
	OLED_WriteCommand(0x30);
	OLED_WriteCommand(0xA4);              /* 显示内容 = RAM 内容（0xA5 是整屏点亮）*/
	OLED_WriteCommand(0xA6);              /* 正常显示（0xA7 是反色）*/
	OLED_WriteCommand(0xAF);              /* 开显示 */

	OLED_Clear();

	return 0;
}

/* ==========================================================================
 * 5. 应用层：清屏 / 画字符 / 字符串 / 数字
 * ========================================================================== */

/**
  * @brief  清屏：8 页，每页写 128 个 0x00
  * @note   常量表放 flash（static const），不占 RAM
  */
void OLED_Clear(void)
{
	static const uint8_t zeros[OLED_WIDTH] = {0};
	uint8_t page;

	for (page = 0; page < OLED_HEIGHT / 8; page++)
	{
		OLED_SetCursor(page, 0);
		OLED_WriteDataN(zeros, OLED_WIDTH);
	}
}

/**
  * @brief  在指定位置显示一个字符
  * @param  line   行号 1~4（4 行 = 64 像素，每行 16 像素高）
  * @param  column 列号 1~16（16 列 = 128 像素，每列 8 像素宽）
  * @param  ch     字符，字库里只有 0x20~0x7E，别的字符显示成 '?'
  *
  * 一个字 8x16 = 占满【上下两页】，所以：
  *   上半部分写到第 (line-1)*2 页，下半部分写到第 (line-1)*2+1 页
  */
void OLED_ShowChar(uint8_t line, uint8_t column, char ch)
{
	uint8_t page, x, idx;

	/* 越界保护：超出屏幕就直接不画（不加这个，写越界会踩坏内存里的其它变量）*/
	if (line < 1 || line > 4 || column < 1 || column > 16)
	{
		return;
	}

	if (ch < ' ' || ch > '~')
	{
		ch = '?';                        /* 字库里没有的字符（含中文）统一显示 '?' */
	}
	idx = (uint8_t)(ch - ' ');

	page = (line - 1) * 2;
	x    = (column - 1) * 8;

	OLED_SetCursor(page, x);                                  /* 光标移到上半页 */
	OLED_WriteDataN(&OLED_F8x16[idx][0], 8);                  /* 前 8 字节 = 上半 */

	OLED_SetCursor(page + 1, x);                              /* 光标移到下半页 */
	OLED_WriteDataN(&OLED_F8x16[idx][8], 8);                  /* 后 8 字节 = 下半 */
}

/**
  * @brief  显示字符串（写满一行自动换到下一行，超出屏幕就停）
  */
void OLED_ShowString(uint8_t line, uint8_t column, const char *str)
{
	uint8_t i;

	for (i = 0; str[i] != '\0'; i++)
	{
		OLED_ShowChar(line, column, str[i]);

		column++;
		if (column > 16)                 /* 一行只放得下 16 个字符 */
		{
			column = 1;
			line++;
		}
		if (line > 4)                    /* 屏幕只有 4 行 */
		{
			break;
		}
	}
}

/**
  * @brief  格式化显示（用法和 printf 一样）
  * @note   内部先 sprintf 到 32 字节的缓冲再显示；一行只有 16 个字符，超了会被截断
  */
void OLED_Printf(uint8_t line, uint8_t column, const char *format, ...)
{
	char buffer[32];
	va_list args;

	va_start(args, format);
	vsnprintf(buffer, sizeof(buffer), format, args);
	va_end(args);

	OLED_ShowString(line, column, buffer);
}

/** @brief 算 base 的 exp 次方（给数字显示用） */
static uint32_t OLED_Pow(uint32_t base, uint32_t exp)
{
	uint32_t result = 1;

	while (exp--)
	{
		result *= base;
	}
	return result;
}

/**
  * @brief  显示无符号十进制数，定长 length 位，不足的位补 0
  * @note   例：OLED_ShowNum(1, 1, 12, 3) → 显示 "012"
  *         用定长补 0 是为了覆盖旧内容，避免残影（例如 100 → 99 会留下 "990" ）
  */
void OLED_ShowNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length)
{
	uint8_t i;

	for (i = 0; i < length; i++)
	{
		OLED_ShowChar(line, column + i, (char)(number / OLED_Pow(10, length - i - 1) % 10 + '0'));
	}
}

/**
  * @brief  显示带符号十进制数，length 不含符号位
  * @note   例：OLED_ShowSignedNum(1, 1, -12, 3) → 显示 "-012"
  */
void OLED_ShowSignedNum(uint8_t line, uint8_t column, int32_t number, uint8_t length)
{
	uint32_t absValue;

	if (number >= 0)
	{
		OLED_ShowChar(line, column, '+');
		absValue = (uint32_t)number;
	}
	else
	{
		OLED_ShowChar(line, column, '-');
		absValue = (uint32_t)(-number);
	}
	OLED_ShowNum(line, column + 1, absValue, length);
}

/**
  * @brief  显示十六进制数（大写字母），定长 length 位
  */
void OLED_ShowHexNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length)
{
	uint8_t i, single;

	for (i = 0; i < length; i++)
	{
		single = (uint8_t)(number / OLED_Pow(16, length - i - 1) % 16);
		if (single < 10)
		{
			OLED_ShowChar(line, column + i, (char)(single + '0'));
		}
		else
		{
			OLED_ShowChar(line, column + i, (char)(single - 10 + 'A'));
		}
	}
}

/**
  * @brief  显示二进制数，定长 length 位
  */
void OLED_ShowBinNum(uint8_t line, uint8_t column, uint32_t number, uint8_t length)
{
	uint8_t i;

	for (i = 0; i < length; i++)
	{
		OLED_ShowChar(line, column + i, (char)(number / OLED_Pow(2, length - i - 1) % 2 + '0'));
	}
}

/**
  * @brief  显示浮点数
  * @param  intLength  整数部分位数（不含小数点）
  * @param  fracLength 小数部分位数
  * @note   例：OLED_ShowFloatNum(1, 1, 3.1415, 1, 2) → 显示 "3.14"
  */
void OLED_ShowFloatNum(uint8_t line, uint8_t column, float number, uint8_t intLength, uint8_t fracLength)
{
	float fracPart;

	if (number < 0)
	{
		OLED_ShowChar(line, column, '-');
		column++;
		number = -number;
	}

	/* 整数部分 */
	OLED_ShowNum(line, column, (uint32_t)number, intLength);
	column += intLength;

	/* 小数点 */
	if (fracLength > 0)
	{
		OLED_ShowChar(line, column, '.');
		column++;

		/* 小数部分：四舍五入到 fracLength 位 */
		fracPart = (number - (float)(uint32_t)number) * (float)OLED_Pow(10, fracLength);
		OLED_ShowNum(line, column, (uint32_t)(fracPart + 0.5f), fracLength);
	}
}
