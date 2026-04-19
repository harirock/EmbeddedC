/**
 * @file    main.c
 * @author Nellore Harikrishna<nharikrishna6231@gmail.com>
 * @brief   STM32F4 Discovery — Accelerometer Tilt → LED Demo
 *
 *
 * Hardware:
 *   Board  : STM32F4 Discovery
 *   Sensor : LIS3DSH (ID=0x3F) or LIS302DL (ID=0x3B)
 *   SPI    : SPI1  — PA5=SCK, PA6=MISO, PA7=MOSI
 *   CS     : PE3   — Chip Select (active LOW)
 *   LEDs   : PD12=Green, PD13=Orange, PD14=Red, PD15=Blue
 *
 * Tilt behaviour:
 *   Tilt RIGHT    → Red LED
 *   Tilt LEFT     → Blue LED
 *   Tilt FORWARD  → Green LED
 *   Tilt BACKWARD → Orange LED
 */

#include "stm32f4xx.h"

/* ============================================================
 *  SECTION 1 — PIN & REGISTER CONSTANTS
 * ============================================================ */

/* --- LED pins on GPIOD --- */
#define LED_GREEN_PIN 12U
#define LED_ORANGE_PIN 13U
#define LED_RED_PIN 14U
#define LED_BLUE_PIN 15U

#define LED_GREEN (1U << LED_GREEN_PIN)
#define LED_ORANGE (1U << LED_ORANGE_PIN)
#define LED_RED (1U << LED_RED_PIN)
#define LED_BLUE (1U << LED_BLUE_PIN)
#define LED_ALL (LED_GREEN | LED_ORANGE | LED_RED | LED_BLUE)

/* --- SPI1 pins on GPIOA --- */
#define SPI1_SCK_PIN 5U  /* PA5 */
#define SPI1_MISO_PIN 6U /* PA6 */
#define SPI1_MOSI_PIN 7U /* PA7 */

/* --- Chip Select on GPIOE --- */
#define ACCEL_CS_PIN 3U /* PE3 */
#define ACCEL_CS_HIGH() (GPIOE->ODR |= (1U << ACCEL_CS_PIN))
#define ACCEL_CS_LOW() (GPIOE->ODR &= ~(1U << ACCEL_CS_PIN))

/* --- RCC clock enable bits --- */
#define RCC_GPIOA_EN (1U << 0)
#define RCC_GPIOD_EN (1U << 3)
#define RCC_GPIOE_EN (1U << 4)
#define RCC_SPI1_EN (1U << 12)

/* --- SPI CR1 bit positions --- */
#define SPI_CR1_CPHA (1U << 0)      /* Clock phase */
#define SPI_CR1_CPOL (1U << 1)      /* Clock polarity */
#define SPI_CR1_MSTR (1U << 2)      /* Master mode */
#define SPI_CR1_BR_DIV256 (7U << 3) /* Baud = PCLK/256 */
#define SPI_CR1_SSI (1U << 8)       /* Internal SS */
#define SPI_CR1_SSM (1U << 9)       /* Software SS mgmt */
#define SPI_CR1_SPE (1U << 6)       /* SPI Enable */

/* --- SPI SR status flags --- */
#define SPI_SR_RXNE (1U << 0) /* Receive buffer not empty */
#define SPI_SR_TXE (1U << 1)  /* Transmit buffer empty */

/* ============================================================
 *  SECTION 2 — SENSOR CONSTANTS
 *  WHO_AM_I IDs and register addresses from the datasheets.
 * ============================================================ */

/* WHO_AM_I register (same address on both chips) */
#define ACCEL_REG_WHO_AM_I 0x0F

/* Chip identity codes returned by WHO_AM_I */
#define LIS3DSH_CHIP_ID 0x3F
#define LIS302DL_CHIP_ID 0x3B

/* Control register (same address, different values per chip) */
#define ACCEL_REG_CTRL 0x20

/*
 * LIS3DSH CTRL_REG4 = 0x67
 *   Bits [7:4] = 0110 → ODR 100 Hz
 *   Bit  [2]   = 1    → Z axis on
 *   Bit  [1]   = 1    → Y axis on
 *   Bit  [0]   = 1    → X axis on
 */
#define LIS3DSH_CTRL_100HZ_XYZ 0x67

/*
 * LIS302DL CTRL_REG1 = 0x47
 *   Bit  [6]   = 1    → Power on (PD bit)
 *   Bit  [2]   = 1    → Z axis on
 *   Bit  [1]   = 1    → Y axis on
 *   Bit  [0]   = 1    → X axis on
 */
#define LIS302DL_CTRL_POWER_XYZ 0x47

/* Output data registers — X, Y, Z, each split into LOW and HIGH byte */
#define ACCEL_REG_OUT_X_L 0x28
#define ACCEL_REG_OUT_X_H 0x29
#define ACCEL_REG_OUT_Y_L 0x2A
#define ACCEL_REG_OUT_Y_H 0x2B
#define ACCEL_REG_OUT_Z_L 0x2C
#define ACCEL_REG_OUT_Z_H 0x2D

/*
 * SPI protocol convention for LIS3DSH / LIS302DL:
 *   Bit 7 of the address byte = 1 → READ operation
 *   Bit 7 of the address byte = 0 → WRITE operation
 */
#define ACCEL_READ_FLAG 0x80  /* OR  with address to read  */
#define ACCEL_WRITE_MASK 0x7F /* AND with address to write */
#define ACCEL_DUMMY_BYTE 0xFF /* Sent during read to clock in data */

/* ============================================================
 *  SECTION 3 — TILT DETECTION THRESHOLD
 *  5000 out of ±32767 ≈ 15% of full scale.
 *  Increase to need a sharper tilt, decrease for more sensitivity.
 * ============================================================ */
#define TILT_THRESHOLD 5000

/* ============================================================
 *  SECTION 4 — GLOBAL SENSOR STATE
 *  Kept volatile because they are updated inside a loop and
 *  could be read asynchronously (e.g., a future interrupt).
 * ============================================================ */
static volatile int16_t accel_x = 0;
static volatile int16_t accel_y = 0;
static volatile int16_t accel_z = 0;
static volatile uint8_t sensor_chip_id = 0;

/* ============================================================
 *  SECTION 5 — SIMPLE BUSY-WAIT DELAY
 *  Good enough for a demo. Replace with SysTick or RTOS
 *  delays in a real product to avoid blocking the CPU.
 * ============================================================ */
static void delay_cycles(volatile uint32_t count)
{
    while (count--)
        ;
}

/* Convenience wrappers so callers use meaningful names */
#define DELAY_SHORT() delay_cycles(10U)
#define DELAY_MEDIUM() delay_cycles(100000U)
#define DELAY_LONG() delay_cycles(500000U)
#define DELAY_STARTUP() delay_cycles(2000000U)
#define DELAY_LOOP() delay_cycles(100000U)

/* ============================================================
 *  SECTION 6 — LED DRIVER
 * ============================================================ */

/**
 * @brief Enable GPIOD clock and set LED pins as outputs.
 *        Must be called once before any LED function.
 */
static void LED_Init(void)
{
    /* Step 1: Give GPIOD a clock signal — without this the
     *         register writes below have no effect at all. */
    RCC->AHB1ENR |= RCC_GPIOD_EN;

    /*
     * Step 2: Set MODER bits for each LED pin to 01 (output).
     *   Each pin uses 2 bits in MODER: 00=input 01=output 10=AF 11=analog
     *   Pin N lives at bits [2N+1 : 2N].
     *
     *   PD12 → bits [25:24] → set bit 24
     *   PD13 → bits [27:26] → set bit 26
     *   PD14 → bits [29:28] → set bit 28
     *   PD15 → bits [31:30] → set bit 30
     */
    GPIOD->MODER |= (1U << (LED_GREEN_PIN * 2U)) |
                    (1U << (LED_ORANGE_PIN * 2U)) |
                    (1U << (LED_RED_PIN * 2U)) |
                    (1U << (LED_BLUE_PIN * 2U));
}

/** @brief Turn on all 4 LEDs. */
static void LED_AllOn(void)
{
    GPIOD->ODR |= LED_ALL;
}

/** @brief Turn off all 4 LEDs. */
static void LED_AllOff(void)
{
    GPIOD->ODR &= ~LED_ALL;
}

/** @brief Turn on a specific LED by its bitmask (e.g. LED_RED). */
static void LED_On(uint16_t led_mask)
{
    GPIOD->ODR |= led_mask;
}

/**
 * @brief Flash a single LED a given number of times.
 * @param led_mask  Which LED (LED_GREEN, LED_ORANGE, etc.)
 * @param times     How many blinks
 */
static void LED_Blink(uint16_t led_mask, uint8_t times)
{
    for (uint8_t i = 0; i < times; i++)
    {
        GPIOD->ODR |= led_mask;
        DELAY_LONG();
        GPIOD->ODR &= ~led_mask;
        DELAY_LONG();
    }
}

/* ============================================================
 *  SECTION 7 — SPI1 DRIVER
 * ============================================================ */

/**
 * @brief Configure SPI1 to talk to the accelerometer.
 *
 *  Pin assignments:
 *    PA5 = SCK  (clock)
 *    PA6 = MISO (data from sensor to MCU)
 *    PA7 = MOSI (data from MCU to sensor)
 *    PE3 = CS   (chip select, controlled manually)
 *
 *  SPI mode: CPOL=1, CPHA=1 (Mode 3) — required by LIS3DSH/LIS302DL.
 *  Speed: PCLK2/256 ≈ 328 kHz — slowest, most reliable for debugging.
 */
static void SPI1_Init(void)
{
    /* ── 1. Enable clocks ─────────────────────────────────── */
    RCC->AHB1ENR |= RCC_GPIOA_EN; /* GPIOA for SCK/MISO/MOSI */
    RCC->AHB1ENR |= RCC_GPIOE_EN; /* GPIOE for CS pin         */
    RCC->APB2ENR |= RCC_SPI1_EN;  /* SPI1 peripheral itself   */
    delay_cycles(1000);           /* Let clocks stabilise     */

    /* ── 2. Configure PA5/PA6/PA7 as Alternate Function ───── */
    /*
     * MODER: clear the 2 bits per pin, then set to 10 (AF mode).
     * Three pins × 2 bits = 6 bits, starting at pin 5's position (bit 10).
     */
    GPIOA->MODER &= ~(0x3FU << (SPI1_SCK_PIN * 2U)); /* clear */
    GPIOA->MODER |= (0x2AU << (SPI1_SCK_PIN * 2U));  /* set AF */

    /*
     * AFR[0] selects which AF for pins 0–7.
     * Each pin has 4 bits. AF5 = 0101 = SPI1.
     * Three pins × 4 bits = 12 bits at position pin5*4 = 20.
     */
    GPIOA->AFR[0] &= ~(0xFFFU << (SPI1_SCK_PIN * 4U)); /* clear */
    GPIOA->AFR[0] |= (0x555U << (SPI1_SCK_PIN * 4U));  /* AF5   */

    /* High-speed output for clean clock edges */
    GPIOA->OSPEEDR |= (0x3FU << (SPI1_SCK_PIN * 2U));

    /* ── 3. Configure PE3 as GPIO output for Chip Select ──── */
    GPIOE->MODER &= ~(3U << (ACCEL_CS_PIN * 2U)); /* clear */
    GPIOE->MODER |= (1U << (ACCEL_CS_PIN * 2U));  /* output */
    ACCEL_CS_HIGH();                              /* Deselect sensor — CS starts HIGH */

    delay_cycles(1000);

    /* ── 4. Configure SPI1 ────────────────────────────────── */
    SPI1->CR1 = 0; /* Disable SPI before changing settings */

    SPI1->CR1 = SPI_CR1_CPHA |      /* Data on 2nd clock edge (Mode 3) */
                SPI_CR1_CPOL |      /* Clock idles HIGH         (Mode 3) */
                SPI_CR1_MSTR |      /* STM32 is the master               */
                SPI_CR1_BR_DIV256 | /* Slowest speed for reliability      */
                SPI_CR1_SSI |       /* Prevent 'multi-master' error       */
                SPI_CR1_SSM;        /* We manage CS in software            */

    SPI1->CR1 |= SPI_CR1_SPE; /* Enable SPI — ready to go! */

    delay_cycles(1000);
}

/**
 * @brief Send one byte over SPI and simultaneously receive one byte.
 *
 *  SPI is full-duplex: every clock cycle sends a bit AND receives a bit.
 *  To read from the sensor we must send something (a dummy byte 0xFF).
 *
 * @param tx_byte  Byte to send
 * @return         Byte received from the sensor
 */
static uint8_t SPI1_TransferByte(uint8_t tx_byte)
{
    /* Wait until TX buffer is empty (previous byte has been sent) */
    while (!(SPI1->SR & SPI_SR_TXE))
        ;

    /* Load our byte — hardware starts clocking it out immediately */
    SPI1->DR = tx_byte;

    /* Wait until RX buffer has the sensor's response */
    while (!(SPI1->SR & SPI_SR_RXNE))
        ;

    /* Return the received byte */
    return (uint8_t)(SPI1->DR);
}

/* ============================================================
 *  SECTION 8 — ACCELEROMETER REGISTER ACCESS
 * ============================================================ */

/**
 * @brief Read one register from the accelerometer.
 *
 *  Protocol (per LIS3DSH/LIS302DL datasheet):
 *    1. Pull CS LOW  → select the sensor
 *    2. Send address | 0x80  → tells sensor "this is a READ"
 *    3. Send 0xFF (dummy)    → clocks in the register value
 *    4. Pull CS HIGH → end transaction
 *
 * @param reg_addr  Register address (e.g. ACCEL_REG_WHO_AM_I)
 * @return          Value stored in that register
 */
static uint8_t ACCEL_ReadRegister(uint8_t reg_addr)
{
    uint8_t received_value;

    ACCEL_CS_LOW();
    DELAY_SHORT();

    SPI1_TransferByte(reg_addr | ACCEL_READ_FLAG);        /* Send address + READ bit */
    received_value = SPI1_TransferByte(ACCEL_DUMMY_BYTE); /* Clock in the data  */

    DELAY_SHORT();
    ACCEL_CS_HIGH();

    return received_value;
}

/**
 * @brief Write one byte to a register on the accelerometer.
 *
 *  Protocol:
 *    1. Pull CS LOW
 *    2. Send address & 0x7F  → bit 7 = 0 means WRITE
 *    3. Send the value to store
 *    4. Pull CS HIGH
 *
 * @param reg_addr  Register to write (e.g. ACCEL_REG_CTRL)
 * @param value     Value to store in the register
 */
static void ACCEL_WriteRegister(uint8_t reg_addr, uint8_t value)
{
    ACCEL_CS_LOW();
    DELAY_SHORT();

    SPI1_TransferByte(reg_addr & ACCEL_WRITE_MASK); /* Address + WRITE bit */
    SPI1_TransferByte(value);                       /* Data to write       */

    DELAY_SHORT();
    ACCEL_CS_HIGH();
}

/**
 * @brief Read all three axes from the accelerometer.
 *
 *  Each axis is 16 bits split across two 8-bit registers:
 *    OUT_X_L (low byte) and OUT_X_H (high byte).
 *  We combine them: value = (high << 8) | low
 *
 *  Result is stored in the global accel_x, accel_y, accel_z.
 */
static void ACCEL_ReadXYZ(void)
{
    uint8_t low_byte, high_byte;

    /* X axis */
    low_byte = ACCEL_ReadRegister(ACCEL_REG_OUT_X_L);
    high_byte = ACCEL_ReadRegister(ACCEL_REG_OUT_X_H);
    accel_x = (int16_t)((high_byte << 8) | low_byte);

    /* Y axis */
    low_byte = ACCEL_ReadRegister(ACCEL_REG_OUT_Y_L);
    high_byte = ACCEL_ReadRegister(ACCEL_REG_OUT_Y_H);
    accel_y = (int16_t)((high_byte << 8) | low_byte);

    /* Z axis */
    low_byte = ACCEL_ReadRegister(ACCEL_REG_OUT_Z_L);
    high_byte = ACCEL_ReadRegister(ACCEL_REG_OUT_Z_H);
    accel_z = (int16_t)((high_byte << 8) | low_byte);
}

/* ============================================================
 *  SECTION 9 — SENSOR DETECTION & SETUP
 * ============================================================ */

/**
 * @brief Detect which accelerometer is on the board and enable it.
 *
 *  Reads WHO_AM_I up to 5 times (sensor may not be ready immediately).
 *  Returns 1 if a known sensor was found and configured, 0 if not.
 */
static uint8_t ACCEL_DetectAndEnable(void)
{
    /* Try up to 5 times — sensor may need time to power up */
    for (uint8_t attempt = 0; attempt < 5; attempt++)
    {
        sensor_chip_id = ACCEL_ReadRegister(ACCEL_REG_WHO_AM_I);

        if (sensor_chip_id == LIS3DSH_CHIP_ID ||
            sensor_chip_id == LIS302DL_CHIP_ID)
        {
            break; /* Valid ID found — stop retrying */
        }

        DELAY_MEDIUM();
    }

    if (sensor_chip_id == LIS3DSH_CHIP_ID)
    {
        /* LIS3DSH found — blink Orange LED twice to confirm */
        LED_Blink(LED_ORANGE, 2);

        /* Enable: 100Hz output rate, all 3 axes active */
        ACCEL_WriteRegister(ACCEL_REG_CTRL, LIS3DSH_CTRL_100HZ_XYZ);
        DELAY_MEDIUM();
        return 1;
    }
    else if (sensor_chip_id == LIS302DL_CHIP_ID)
    {
        /* LIS302DL found — blink Blue LED twice to confirm */
        LED_Blink(LED_BLUE, 2);

        /* Enable: power on, all 3 axes active */
        ACCEL_WriteRegister(ACCEL_REG_CTRL, LIS302DL_CTRL_POWER_XYZ);
        DELAY_MEDIUM();
        return 1;
    }

    /* Unknown / no response — signal error with 5 fast blinks */
    for (uint8_t i = 0; i < 10; i++)
    {
        LED_AllOn();
        DELAY_MEDIUM();
        LED_AllOff();
        DELAY_MEDIUM();
    }
    return 0;
}

/* ============================================================
 *  SECTION 10 — TILT → LED MAPPING
 *  Separated into its own function so it's easy to replace
 *  later with an Edge AI gesture classifier.
 * ============================================================ */

/**
 * @brief Update LEDs based on current tilt readings.
 *
 *  Uses the global accel_x and accel_y values.
 *  Multiple LEDs can be on at once (e.g., tilted right AND forward).
 *
 *  To add Edge AI later: replace this function body with
 *  a call to your ML inference function.
 */
static void TILT_UpdateLEDs(void)
{
    /* Reset all LEDs at the start of each evaluation */
    LED_AllOff();

    /* X axis: positive = right, negative = left */
    if (accel_x > TILT_THRESHOLD)
        LED_On(LED_RED); /* Tilting RIGHT */

    if (accel_x < -TILT_THRESHOLD)
        LED_On(LED_BLUE); /* Tilting LEFT  */

    /* Y axis: positive = forward, negative = backward */
    if (accel_y > TILT_THRESHOLD)
        LED_On(LED_GREEN); /* Tilting FORWARD   */

    if (accel_y < -TILT_THRESHOLD)
        LED_On(LED_ORANGE); /* Tilting BACKWARD  */
}

/* ============================================================
 *  SECTION 11 — STARTUP SEQUENCE
 * ============================================================ */

/**
 * @brief Visual boot sequence — confirms the board is alive.
 *
 *  1 long flash  → hardware init completed
 *  3 quick blinks → entering sensor detection
 */
static void Startup_FlashSequence(void)
{
    /* One long flash — "I'm alive!" */
    LED_AllOn();
    DELAY_STARTUP();
    LED_AllOff();
    DELAY_LONG();

    /* Three quick blinks — "checking sensor..." */
    for (uint8_t i = 0; i < 3; i++)
    {
        LED_AllOn();
        delay_cycles(300000U);
        LED_AllOff();
        delay_cycles(300000U);
    }

    DELAY_LONG();
}

/* ============================================================
 *  SECTION 12 — MAIN
 * ============================================================ */

int main(void)
{
    /* ── Hardware initialisation ─────────────────────────── */
    LED_Init();
    SPI1_Init();

    /* ── Boot confirmation flash ─────────────────────────── */
    Startup_FlashSequence();

    /* ── Sensor detection ────────────────────────────────── */
    if (!ACCEL_DetectAndEnable())
    {
        /* Sensor not found — hang here so the error blink
         * stays visible. In a real product you'd log the
         * fault and enter a safe/low-power mode. */
        while (1)
            ;
    }

    /* ── Main loop ───────────────────────────────────────── */
    /*
     * This loop runs forever at ~100Hz (limited by Delay).
     *  1. Read fresh accelerometer data
     *  2. Decide which LEDs to light based on tilt
     *
     *  FUTURE UPGRADE: Replace TILT_UpdateLEDs() with an
     *  Edge AI inference call to classify gestures.
     */
    while (1)
    {
        ACCEL_ReadXYZ();   /* Get latest X, Y, Z values  */
        TILT_UpdateLEDs(); /* Map tilt direction to LEDs */
        DELAY_LOOP();      /* ~10ms pause between reads  */
    }
}
