#include "MKL25Z4.h"
#include <string.h>
#include <stdio.h>

// ========== PINI ==========
#define MOTOR_PIN 1         // PTA1
#define LED_RED_PIN 4       // PTD4
#define LED_GREEN_PIN 5     // PTA5
#define BTN_START_PIN 12    // PTA12
#define BTN_MODE_PIN  4     // PTA4

// FSR pe PTB2
#define FSR_ADC_CHANNEL 12  // ADC0_SE12
#define FSR_PIN 2           // PTB2

#define OLED_I2C_ADDR 0x3C

// Prag presiune
#define PRESSURE_THRESHOLD 3700

// ========== PARAMETRI DEBOUNCE ROBUST ==========
#define DEBOUNCE_TIME_MS 80     // Timp debounce (mărit pentru siguranță)
#define DEBOUNCE_CONFIRM_MS 20  // Timp suplimentar pentru confirmare

// ========== PARAMETRI ALBIRE SWEEP ==========
#define ALBIRE_MIN_SPEED 30
#define ALBIRE_MAX_SPEED 90
#define ALBIRE_SWEEP_STEP 2
#define ALBIRE_SWEEP_INTERVAL 50

// ========== VARIABILE GLOBALE ==========
volatile uint32_t system_millis = 0;

// Timer bazat pe referință de timp
volatile uint32_t timer_start_millis = 0;
uint32_t last_second_update = 0;

// ========== DEBOUNCE ROBUST - VARIABILE ==========
// Stări pentru state machine de debounce
typedef enum {
    BTN_IDLE,           // Așteptăm apăsare
    BTN_DEBOUNCING,     // În curs de debounce, așteptăm timeout
    BTN_WAIT_RELEASE    // Așteptăm eliberarea butonului
} ButtonState;

// Structură pentru fiecare buton
typedef struct {
    volatile ButtonState state;
    volatile uint32_t debounce_start;  // Când a început debounce-ul
    volatile uint8_t event_flag;       // Flag pentru main loop
    uint8_t pin;                       // Numărul pinului
} Button;

// Cele două butoane
volatile Button btn_start = {BTN_IDLE, 0, 0, BTN_START_PIN};
volatile Button btn_mode = {BTN_IDLE, 0, 0, BTN_MODE_PIN};

// Flag-uri Evenimente (pentru compatibilitate cu codul existent)
volatile uint8_t evt_start_click = 0;
volatile uint8_t evt_mode_click = 0;

// ========== FSM STATES ==========
typedef enum {
    STATE_IDLE,
    STATE_BRUSHING,
    STATE_PRESSURE_WARNING,
    STATE_QUADRANT_SWITCH,
    STATE_SESSION_COMPLETE
} SystemState;

volatile SystemState current_state = STATE_IDLE;
SystemState previous_state = STATE_IDLE;

// Forward declarations
uint8_t FSM_Is_Motor_Active(void);
void Button_Debounce_Process(volatile Button* btn);

// Variabile periaj
uint8_t mod_curent = 0;
uint8_t cadran = 1;
uint32_t timer_periaj = 0;

// Haptic
uint8_t haptic_active = 0;
uint32_t haptic_end = 0;

// LED feedback non-blocant
uint8_t led_feedback_active = 0;
uint32_t led_feedback_end = 0;

// FSR
volatile uint16_t adc_pressure = 0;
volatile uint8_t pressure_warning = 0;
uint8_t last_pressure_warning = 0;

// ALBIRE SWEEP
uint8_t albire_speed = ALBIRE_MIN_SPEED;
int8_t albire_direction = 1;
uint32_t last_albire_update = 0;

// OLED update flags - FIX: Inițializare explicită
uint8_t oled_need_update_time = 0;
uint8_t oled_need_update_pressure = 0;
uint8_t oled_need_update_mode = 0;
uint8_t oled_need_update_cadran = 0;

// FIX: Cache pentru ultima stare afișată (previne update-uri redundante)
uint8_t last_displayed_mode = 0xFF;  // Valoare imposibilă pentru forțarea primului update
uint8_t last_displayed_cadran = 0;
uint8_t last_displayed_pressure = 0xFF;
int last_displayed_time = -1;

// FONT OGLINDIT VERTICAL
const uint8_t font[][5] = {
    {0x7C,0x8A,0x92,0xA2,0x7C}, // 0
    {0x00,0x42,0xFE,0x02,0x00}, // 1
    {0x42,0x86,0x8A,0x92,0x62}, // 2
    {0x84,0x82,0xA2,0xD2,0x8C}, // 3
    {0x18,0x28,0x48,0xFE,0x08}, // 4
    {0xE4,0xA2,0xA2,0xA2,0x9C}, // 5
    {0x3C,0x52,0x92,0x92,0x0C}, // 6
    {0x80,0x8E,0x90,0xA0,0xC0}, // 7
    {0x6C,0x92,0x92,0x92,0x6C}, // 8
    {0x60,0x92,0x92,0x94,0x78}, // 9
    {0x00,0x00,0x00,0x00,0x00}, // Space (10)
    {0x00,0x00,0x6C,0x00,0x00}, // : (11)
    {0x04,0x08,0x10,0x20,0x40}, // / (12)
    {0xFE,0x90,0x90,0x90,0x60}, // P (13)
    {0xFE,0x92,0x92,0x92,0x82}, // E (14)
    {0xFE,0x90,0x98,0x94,0x62}, // R (15)
    {0x00,0x82,0xFE,0x82,0x00}, // I (16)
    {0x3E,0x48,0x88,0x48,0x3E}, // A (17)
    {0x04,0x02,0x82,0xFC,0x80}, // J (18)
    {0x7C,0x82,0x82,0x82,0x44}, // C (19)
    {0xFE,0x82,0x82,0x44,0x38}, // D (20)
    {0xFE,0x20,0x10,0x08,0xFE}, // N (21)
    {0xFE,0x40,0x30,0x40,0xFE}, // M (22)
    {0x7C,0x82,0x82,0x82,0x7C}, // O (23)
    {0x62,0x92,0x92,0x92,0x8C}, // S (24)
    {0x80,0x80,0xFE,0x80,0x80}, // T (25)
    {0xFE,0x92,0x92,0x92,0x6C}, // B (26)
    {0xFE,0x02,0x02,0x02,0x02}, // L (27)
    {0xF8,0x04,0x02,0x04,0xF8}, // V (28)
    {0xFC,0x02,0x02,0x02,0xFC}, // U (29)
    {0xC6,0x28,0x10,0x28,0xC6}, // X (30)
    {0x3C,0x42,0x42,0x42,0x24}, // G (31)
};

uint8_t GetCharIdx(char c) {
    if(c >= '0' && c <= '9') return c - '0';
    if(c == ' ') return 10; if(c == ':') return 11; if(c == '/') return 12;
    switch(c) {
        case 'P': return 13; case 'E': return 14; case 'R': return 15;
        case 'I': return 16; case 'A': return 17; case 'J': return 18;
        case 'C': return 19; case 'D': return 20; case 'N': return 21;
        case 'M': return 22; case 'O': return 23; case 'S': return 24;
        case 'T': return 25; case 'B': return 26; case 'L': return 27;
        case 'V': return 28; case 'U': return 29; case 'X': return 30;
        case 'G': return 31;
    }
    return 10;
}

// ========== FUNCȚII HELPER PENTRU ÎNTRERUPERI ==========

// Dezactivează întreruperea pentru un pin specific
void Button_Disable_IRQ(uint8_t pin) {
    // Setează IRQC la 0 (dezactivat) păstrând ceilalți biți
    PORTA->PCR[pin] = (PORTA->PCR[pin] & ~PORT_PCR_IRQC_MASK);
}

// Activează întreruperea falling edge pentru un pin
void Button_Enable_IRQ(uint8_t pin) {
    // Curăță flag-ul ISFR mai întâi
    PORTA->ISFR = (1 << pin);
    // Setează IRQC la 0x0A (falling edge)
    PORTA->PCR[pin] = (PORTA->PCR[pin] & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(0x0A);
}

// Citește starea curentă a unui pin (1 = neapăsat, 0 = apăsat)
uint8_t Button_Read_Pin(uint8_t pin) {
    return (PTA->PDIR & (1 << pin)) ? 1 : 0;
}

// ========== ÎNTRERUPERE SYSTICK ==========
void SysTick_Handler(void) {
    system_millis++;
}

// ========== ÎNTRERUPERE PORT A (BUTOANE) ==========
void PORTA_IRQHandler(void) {
    uint32_t isfr = PORTA->ISFR;

    // ===== BUTON START (PTA12) =====
    if (isfr & (1 << BTN_START_PIN)) {
        // Doar dacă suntem în starea IDLE (așteptăm apăsare)
        if (btn_start.state == BTN_IDLE) {
            // Dezactivează întreruperea pentru acest pin
            Button_Disable_IRQ(BTN_START_PIN);

            // Începe debounce
            btn_start.state = BTN_DEBOUNCING;
            btn_start.debounce_start = system_millis;
        }
        // Curăță flag-ul
        PORTA->ISFR = (1 << BTN_START_PIN);
    }

    // ===== BUTON MODE (PTA4) =====
    if (isfr & (1 << BTN_MODE_PIN)) {
        if (btn_mode.state == BTN_IDLE) {
            Button_Disable_IRQ(BTN_MODE_PIN);

            btn_mode.state = BTN_DEBOUNCING;
            btn_mode.debounce_start = system_millis;
        }
        PORTA->ISFR = (1 << BTN_MODE_PIN);
    }
}

// ========== PROCESARE DEBOUNCE (apelată din main loop) ==========
void Button_Debounce_Process(volatile Button* btn) {
    switch (btn->state) {
        case BTN_IDLE:
            // Nu facem nimic, așteptăm întreruperea
            break;

        case BTN_DEBOUNCING:
            // Verificăm dacă a trecut timpul de debounce
            if (system_millis - btn->debounce_start >= DEBOUNCE_TIME_MS) {
                // Verificăm starea REALĂ a pinului acum
                if (Button_Read_Pin(btn->pin) == 0) {
                    // Pinul este încă LOW = buton confirmat apăsat!
                    btn->event_flag = 1;

                    // Setează flag-ul corespunzător pentru FSM
                    if (btn->pin == BTN_START_PIN) {
                        evt_start_click = 1;
                    } else if (btn->pin == BTN_MODE_PIN) {
                        evt_mode_click = 1;
                    }

                    // Trecem la așteptarea eliberării
                    btn->state = BTN_WAIT_RELEASE;
                } else {
                    // Pinul e HIGH = a fost zgomot, nu apăsare reală
                    // Re-activăm întreruperea și revenim la IDLE
                    btn->state = BTN_IDLE;
                    Button_Enable_IRQ(btn->pin);
                }
            }
            break;

        case BTN_WAIT_RELEASE:
            // Așteptăm să fie eliberat butonul
            if (Button_Read_Pin(btn->pin) == 1) {
                // Butonul a fost eliberat
                // Mic delay suplimentar pentru debounce la eliberare
                if (system_millis - btn->debounce_start >= DEBOUNCE_TIME_MS + DEBOUNCE_CONFIRM_MS) {
                    // Re-activăm întreruperea pentru următoarea apăsare
                    btn->state = BTN_IDLE;
                    btn->event_flag = 0;
                    Button_Enable_IRQ(btn->pin);
                }
            } else {
                // Butonul încă apăsat, resetăm timestamp-ul
                btn->debounce_start = system_millis;
            }
            break;
    }
}

// ========== INIȚIALIZARE HARDWARE ==========
void Init_Hardware(void) {
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000);

    // Activare ceas pentru porturi
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTB_MASK;
    SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;

    // LED-uri ca ieșiri GPIO
    PORTA->PCR[LED_GREEN_PIN] = PORT_PCR_MUX(1);
    PORTD->PCR[LED_RED_PIN] = PORT_PCR_MUX(1);
    PTA->PDDR |= (1 << LED_GREEN_PIN);
    PTD->PDDR |= (1 << LED_RED_PIN);

    // ========== CONFIGURARE BUTOANE CU ÎNTRERUPERI ==========
    // Configurare inițială: GPIO + Pull-up + Falling edge interrupt
    PORTA->PCR[BTN_START_PIN] = PORT_PCR_MUX(1) |
                                 PORT_PCR_PE_MASK |
                                 PORT_PCR_PS_MASK |
                                 PORT_PCR_IRQC(0x0A);

    PORTA->PCR[BTN_MODE_PIN] = PORT_PCR_MUX(1) |
                                PORT_PCR_PE_MASK |
                                PORT_PCR_PS_MASK |
                                PORT_PCR_IRQC(0x0A);

    // Curăță flag-urile de întrerupere
    PORTA->ISFR = (1 << BTN_START_PIN) | (1 << BTN_MODE_PIN);

    // Activare întrerupere în NVIC
    NVIC_SetPriority(PORTA_IRQn, 2);
    NVIC_ClearPendingIRQ(PORTA_IRQn);
    NVIC_EnableIRQ(PORTA_IRQn);

    // Motor PWM
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
    PORTA->PCR[MOTOR_PIN] = PORT_PCR_MUX(3);
    TPM2->SC = 0;
    TPM2->MOD = 1000;
    TPM2->CONTROLS[0].CnSC = 0x28;
    TPM2->CONTROLS[0].CnV = 0;
    TPM2->SC = 0x0C;
}

void Set_Motor(uint8_t speed) {
    if(speed > 100) speed = 100;
    TPM2->CONTROLS[0].CnV = (speed * 1000) / 100;
}

// ========== ADC ==========
void Init_ADC(void) {
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
    PORTB->PCR[FSR_PIN] = PORT_PCR_MUX(0);

    ADC0->CFG1 = ADC_CFG1_ADICLK(0) | ADC_CFG1_MODE(1) | ADC_CFG1_ADIV(0);
    ADC0->CFG2 = 0;
    ADC0->SC2 = ADC_SC2_REFSEL(0);
    ADC0->SC3 = ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(0);

    ADC0->SC3 |= ADC_SC3_CAL_MASK;
    while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK));

    uint16_t cal = ADC0->CLP0 + ADC0->CLP1 + ADC0->CLP2 +
                   ADC0->CLP3 + ADC0->CLP4 + ADC0->CLPS;
    cal = (cal / 2) | 0x8000;
    ADC0->PG = cal;
}

uint16_t Read_ADC(uint8_t channel) {
    ADC0->SC1[0] = ADC_SC1_ADCH(channel);
    while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK));
    return ADC0->R[0];
}

void Update_Pressure_Warning(void) {
    adc_pressure = Read_ADC(FSR_ADC_CHANNEL);

    uint8_t new_warning = (adc_pressure < PRESSURE_THRESHOLD) ? 1 : 0;

    // FIX: Update doar dacă chiar s-a schimbat (previne zgomotul pe OLED)
    if (new_warning != pressure_warning) {
        pressure_warning = new_warning;
        oled_need_update_pressure = 1;
    }

    if (pressure_warning) {
        PTD->PSOR = (1 << LED_RED_PIN);
        PTA->PCOR = (1 << LED_GREEN_PIN);
    } else {
        PTD->PCOR = (1 << LED_RED_PIN);
        if (FSM_Is_Motor_Active()) {
            PTA->PSOR = (1 << LED_GREEN_PIN);
        }
    }
}

// ========== ALBIRE SWEEP ==========
void Update_Albire_Sweep(void) {
    if (!FSM_Is_Motor_Active() || mod_curent != 2 || haptic_active) {
        return;
    }

    if (system_millis - last_albire_update < ALBIRE_SWEEP_INTERVAL) {
        return;
    }
    last_albire_update = system_millis;

    albire_speed += albire_direction * ALBIRE_SWEEP_STEP;

    if (albire_speed >= ALBIRE_MAX_SPEED) {
        albire_speed = ALBIRE_MAX_SPEED;
        albire_direction = -1;
    } else if (albire_speed <= ALBIRE_MIN_SPEED) {
        albire_speed = ALBIRE_MIN_SPEED;
        albire_direction = 1;
    }

    Set_Motor(albire_speed);
}

uint8_t Get_Current_Speed(void) {
    if (mod_curent == 0) return 60;
    if (mod_curent == 1) return 40;
    return albire_speed;
}

void Reset_Albire_Sweep(void) {
    albire_speed = ALBIRE_MIN_SPEED;
    albire_direction = 1;
    last_albire_update = system_millis;
}

// ========== I2C ==========
uint8_t I2C_Write(uint8_t addr, uint8_t* data, uint32_t len) {
    if (I2C0->S & I2C_S_BUSY_MASK) {
        I2C0->C1 &= ~I2C_C1_IICEN_MASK;
        I2C0->C1 |= I2C_C1_IICEN_MASK;
    }

    I2C0->C1 |= I2C_C1_TX_MASK | I2C_C1_MST_MASK;
    I2C0->D = (addr << 1);

    uint32_t t = 2000;
    while(!(I2C0->S & I2C_S_IICIF_MASK)) {
        if(--t == 0) {
            I2C0->C1 &= ~I2C_C1_MST_MASK;
            return 0;
        }
    }
    I2C0->S |= I2C_S_IICIF_MASK;

    if(I2C0->S & I2C_S_RXAK_MASK) {
        I2C0->C1 &= ~I2C_C1_MST_MASK;
        return 0;
    }

    for(uint32_t i=0; i<len; i++) {
        I2C0->D = data[i];
        t = 2000;
        while(!(I2C0->S & I2C_S_IICIF_MASK)) {
            if(--t == 0) {
                I2C0->C1 &= ~I2C_C1_MST_MASK;
                return 0;
            }
        }
        I2C0->S |= I2C_S_IICIF_MASK;
    }

    I2C0->C1 &= ~I2C_C1_MST_MASK;
    I2C0->C1 &= ~I2C_C1_TX_MASK;

    for(volatile int x=0; x<50; x++);
    return 1;
}

void OLED_Cmd(uint8_t c) {
    uint8_t d[2]={0x00,c};
    I2C_Write(OLED_I2C_ADDR,d,2);
}

void OLED_Data(uint8_t d) {
    uint8_t buf[2]={0x40,d};
    I2C_Write(OLED_I2C_ADDR,buf,2);
}

void OLED_Init_Sequence(void) {
    uint8_t init[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x1F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC0, 0xDA, 0x02,
        0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF
    };
    for(int i=0; i<sizeof(init); i++) {
        OLED_Cmd(init[i]);
    }
}

void OLED_Clear(void) {
    for(int p=0; p<4; p++) {
        OLED_Cmd(0xB0+p);
        OLED_Cmd(0x00);
        OLED_Cmd(0x10);
        for(int c=0; c<128; c++) {
            OLED_Data(0x00);
        }
    }
}

void Init_OLED(void) {
    SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;
    PORTB->PCR[0] = PORT_PCR_MUX(2) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTB->PCR[1] = PORT_PCR_MUX(2) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    I2C0->F = 0x14;
    I2C0->C1 = I2C_C1_IICEN_MASK;

    for(volatile int i=0; i<200000; i++);
    OLED_Cmd(0xAE);
    for(volatile int i=0; i<50000; i++);
    OLED_Init_Sequence();
    OLED_Clear();
    OLED_Cmd(0xAF);
}

void OLED_SetCursor(uint8_t page, uint8_t col) {
    OLED_Cmd(0xB0 + page);
    OLED_Cmd(0x00 | (col & 0x0F));
    OLED_Cmd(0x10 | (col >> 4));
}

void OLED_PrintAt(uint8_t page, uint8_t col, const char* str) {
    OLED_SetCursor(page, col);
    uint8_t char_buf[7];
    char_buf[0] = 0x40;

    while(*str) {
        uint8_t idx = GetCharIdx(*str);
        for(int i = 0; i < 5; i++) {
            char_buf[1 + i] = font[idx][i];
        }
        char_buf[6] = 0x00;
        I2C_Write(OLED_I2C_ADDR, char_buf, 7);
        str++;
    }
}

void OLED_ClearLine(uint8_t page) {
    OLED_SetCursor(page, 0);
    for(int c=0; c<128; c++) {
        OLED_Data(0x00);
    }
}

void OLED_PrintLine(uint8_t page, const char* str) {
    OLED_SetCursor(page, 0);
    uint8_t clear_buf[17];
    clear_buf[0] = 0x40;
    for(int i = 1; i < 17; i++) {
        clear_buf[i] = 0x00;
    }
    for(int block = 0; block < 8; block++) {
        I2C_Write(OLED_I2C_ADDR, clear_buf, 17);
    }
    OLED_PrintAt(page, 0, str);
}

// ========== DISPLAY UPDATE FUNCTIONS (CU CACHE ANTI-ZGOMOT) ==========

// FIX: Update doar dacă valoarea chiar s-a schimbat
void Display_Mode(void) {
    if (mod_curent == last_displayed_mode) {
        return;  // Exact aceeași valoare, nu facem update
    }

    last_displayed_mode = mod_curent;

    char buf[20];
    const char* mode_str = (mod_curent == 0) ? "STANDARD" :
                           (mod_curent == 1) ? "SENSIBIL" : "ALBIRE";
    sprintf(buf, "MOD: %-10s", mode_str);
    OLED_PrintLine(0, buf);
}

void Display_Time(int seconds_remaining) {
    if (seconds_remaining == last_displayed_time) {
        return;  // Exact aceeași valoare
    }

    last_displayed_time = seconds_remaining;

    char buf[20];
    sprintf(buf, "TIMP: %d:%02d      ", seconds_remaining/60, seconds_remaining%60);
    OLED_PrintLine(1, buf);
}

void Display_Time_Status(const char* status) {
    // FIX: Invalidăm cache-ul când afișăm status
    last_displayed_time = -1;

    char buf[20];
    sprintf(buf, "TIMP: %-10s", status);
    OLED_PrintLine(1, buf);
}

void Display_Cadran(void) {
    if (cadran == last_displayed_cadran) {
        return;  // Exact aceeași valoare
    }

    last_displayed_cadran = cadran;

    char buf[20];
    sprintf(buf, "CADRAN: %d/4    ", cadran);
    OLED_PrintLine(2, buf);
}

void Display_Pressure(void) {
    if (pressure_warning == last_displayed_pressure) {
        return;  // Exact aceeași valoare
    }

    last_displayed_pressure = pressure_warning;

    if (pressure_warning) {
        OLED_PrintLine(3, "PRES: PREA TARE!");
    } else {
        OLED_PrintLine(3, "PRES: OK        ");
    }
}

void Display_Full_Update(void) {
    // FIX: Această funcție forțează update complet (ignoră cache)
    // Resetăm cache-ul pentru a forța afișarea
    last_displayed_mode = 0xFF;
    last_displayed_cadran = 0;
    last_displayed_pressure = 0xFF;
    last_displayed_time = -1;

    Display_Mode();
    if (FSM_Is_Motor_Active()) {
        int ramas = 120 - timer_periaj;
        if (ramas < 0) ramas = 0;
        Display_Time(ramas);
    } else {
        Display_Time_Status("PAUZA");
    }
    Display_Cadran();
    Display_Pressure();
}

// ========== FSM HELPER ==========
uint8_t FSM_Is_Motor_Active(void) {
    return (current_state == STATE_BRUSHING ||
            current_state == STATE_PRESSURE_WARNING ||
            current_state == STATE_QUADRANT_SWITCH);
}

// ========== FSM TRANSITION ==========
void FSM_Enter_State(SystemState new_state) {
    previous_state = current_state;
    current_state = new_state;

    switch(new_state) {
        case STATE_IDLE:
            Set_Motor(0);
            PTD->PSOR = (1<<LED_RED_PIN);
            PTA->PCOR = (1<<LED_GREEN_PIN);
            Display_Time_Status("PAUZA");
            break;

        case STATE_BRUSHING:
            if (previous_state == STATE_IDLE || previous_state == STATE_SESSION_COMPLETE) {
                timer_periaj = 0;
                cadran = 1;
                timer_start_millis = system_millis;
                last_second_update = system_millis;
                Reset_Albire_Sweep();
                Display_Time(120);
                Display_Cadran();
            }
            if (!pressure_warning) {
                PTD->PCOR = (1<<LED_RED_PIN);
                PTA->PSOR = (1<<LED_GREEN_PIN);
            }
            Set_Motor(Get_Current_Speed());
            break;

        case STATE_PRESSURE_WARNING:
            PTD->PSOR = (1<<LED_RED_PIN);
            PTA->PCOR = (1<<LED_GREEN_PIN);
            break;

        case STATE_QUADRANT_SWITCH:
            haptic_active = 1;
            haptic_end = system_millis + 300;
            Set_Motor(0);
            cadran++;
            Display_Cadran();
            break;

        case STATE_SESSION_COMPLETE:
            Set_Motor(0);
            PTD->PSOR = (1<<LED_RED_PIN);
            PTA->PCOR = (1<<LED_GREEN_PIN);
            Display_Time_Status("GATA!");
            cadran = 4;
            Display_Cadran();
            break;
    }
}

// ========== FSM STATE HANDLERS ==========

void FSM_State_Idle(void) {
    // FIX: Procesăm evenimente doar dacă flag-ul este setat
    if (evt_start_click) {
        evt_start_click = 0;
        FSM_Enter_State(STATE_BRUSHING);
    }

    if (evt_mode_click) {
        evt_mode_click = 0;
        mod_curent++;
        if (mod_curent > 2) mod_curent = 0;
        if (mod_curent == 2) Reset_Albire_Sweep();
        Display_Mode();  // Va afișa doar dacă s-a schimbat
    }
}

void FSM_State_Brushing(void) {
    if (pressure_warning) {
        FSM_Enter_State(STATE_PRESSURE_WARNING);
        return;
    }

    if (system_millis - last_second_update >= 1000) {
        last_second_update += 1000;
        timer_periaj++;

        int ramas = 120 - timer_periaj;
        if (ramas < 0) ramas = 0;
        Display_Time(ramas);  // Va afișa doar dacă s-a schimbat

        if (timer_periaj == 30 || timer_periaj == 60 || timer_periaj == 90) {
            FSM_Enter_State(STATE_QUADRANT_SWITCH);
            return;
        }

        if (timer_periaj >= 120) {
            FSM_Enter_State(STATE_SESSION_COMPLETE);
            return;
        }
    }

    if (evt_start_click) {
        evt_start_click = 0;
        FSM_Enter_State(STATE_IDLE);
        return;
    }

    if (evt_mode_click) {
        evt_mode_click = 0;
        mod_curent++;
        if (mod_curent > 2) mod_curent = 0;
        if (mod_curent == 2) Reset_Albire_Sweep();
        Display_Mode();  // Va afișa doar dacă s-a schimbat
        Set_Motor(Get_Current_Speed());
    }

    Update_Albire_Sweep();
}

void FSM_State_Pressure_Warning(void) {
    if (!pressure_warning) {
        FSM_Enter_State(STATE_BRUSHING);
        return;
    }

    if (system_millis - last_second_update >= 1000) {
        last_second_update += 1000;
        timer_periaj++;

        int ramas = 120 - timer_periaj;
        if (ramas < 0) ramas = 0;
        Display_Time(ramas);  // Va afișa doar dacă s-a schimbat

        if (timer_periaj >= 120) {
            FSM_Enter_State(STATE_SESSION_COMPLETE);
            return;
        }
    }

    if (evt_start_click) {
        evt_start_click = 0;
        FSM_Enter_State(STATE_IDLE);
    }

    Update_Albire_Sweep();
}

void FSM_State_Quadrant_Switch(void) {
    if (system_millis >= haptic_end) {
        haptic_active = 0;
        FSM_Enter_State(STATE_BRUSHING);
        return;
    }

    if (evt_start_click) {
        evt_start_click = 0;
        haptic_active = 0;
        FSM_Enter_State(STATE_IDLE);
    }
}

void FSM_State_Session_Complete(void) {
    if (evt_start_click) {
        evt_start_click = 0;
        FSM_Enter_State(STATE_BRUSHING);
    }

    if (evt_mode_click) {
        evt_mode_click = 0;
        mod_curent++;
        if (mod_curent > 2) mod_curent = 0;
        if (mod_curent == 2) Reset_Albire_Sweep();
        Display_Mode();  // Va afișa doar dacă s-a schimbat
    }
}

// ========== MAIN ==========
int main(void) {
    SIM->COPC = 0;  // Dezactivează watchdog
    Init_Hardware();
    Init_OLED();
    Init_ADC();

    PTD->PSOR = (1<<LED_RED_PIN);
    PTA->PCOR = (1<<LED_GREEN_PIN);
    Set_Motor(0);

    // Afișare inițială (forțează afișarea completă)
    Display_Full_Update();

    uint32_t last_pressure_check = 0;

    while(1) {
        // ========== PROCESARE DEBOUNCE BUTOANE ==========
        Button_Debounce_Process(&btn_start);
        Button_Debounce_Process(&btn_mode);

        // ========== EVENIMENTE COMUNE (toate stările) ==========

        // Verificare presiune la fiecare 200ms
        if (system_millis - last_pressure_check >= 200) {
            last_pressure_check = system_millis;
            Update_Pressure_Warning();
        }

        // FIX: Update OLED presiune doar dacă flag-ul este setat
        if (oled_need_update_pressure) {
            oled_need_update_pressure = 0;
            Display_Pressure();  // Funcția deja verifică cache intern
        }

        // LED feedback end
        if (led_feedback_active && system_millis >= led_feedback_end) {
            led_feedback_active = 0;
            if (FSM_Is_Motor_Active() && !pressure_warning) {
                PTA->PSOR = (1<<LED_GREEN_PIN);
            }
        }

        // ========== FSM DISPATCH ==========
        switch(current_state) {
            case STATE_IDLE:
                FSM_State_Idle();
                break;

            case STATE_BRUSHING:
                FSM_State_Brushing();
                break;

            case STATE_PRESSURE_WARNING:
                FSM_State_Pressure_Warning();
                break;

            case STATE_QUADRANT_SWITCH:
                FSM_State_Quadrant_Switch();
                break;

            case STATE_SESSION_COMPLETE:
                FSM_State_Session_Complete();
                break;
        }

        __WFI();
    }
}
