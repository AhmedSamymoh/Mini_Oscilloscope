#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define REC_LENGTH 200

#define OLED_RESET     -1

#define TWI_FREQ 100000L
#define I2C_ADDRESS 0x3C

#define ADC_CHANNEL 0

#define BUTTON1_PIN PB0
#define BUTTON2_PIN PB1
#define BUTTON3_PIN PB2
#define BUTTON4_PIN PB3
#define BUTTON5_PIN PB4
#define TRIGGER_PIN PB5
#define LED_PIN PB7

#define DISPLAY_TEXT_SIZE 1
#define DISPLAY_TEXT_COLOR WHITE

#define H_RANGE_COUNT 8
#define V_RANGE_COUNT 10

const char vRangeName[V_RANGE_COUNT][5] PROGMEM = {"A50V", "A 5V", " 50V", " 20V", " 10V", "  5V", "  2V", "  1V", "0.5V", "0.2V"};
const char * const vstring_table[] PROGMEM = {vRangeName[0], vRangeName[1], vRangeName[2], vRangeName[3], vRangeName[4], vRangeName[5], vRangeName[6], vRangeName[7], vRangeName[8], vRangeName[9]};
const char hRangeName[H_RANGE_COUNT][6] PROGMEM = {" 50ms", " 20ms", " 10ms", "  5ms", "  2ms", "  1ms", "500us", "200us"};
const char * const hstring_table[] PROGMEM = {hRangeName[0], hRangeName[1], hRangeName[2], hRangeName[3], hRangeName[4], hRangeName[5], hRangeName[6], hRangeName[7]};

volatile int waveBuff[REC_LENGTH];
char chrBuff[10];
char hScale[] = "xxxAs";
char vScale[] = "xxxx";

float lsb5V = 0.0055549;
float lsb50V = 0.051513;

volatile int vRange;
volatile int hRange;
volatile int trigD;
volatile int scopeP;
volatile uint8_t hold = 0;
volatile uint8_t paraChanged = 0;
volatile int saveTimer;
int timeExec;

int dataMin;
int dataMax;
int dataAve;
int rangeMax;
int rangeMin;
int rangeMaxDisp;
int rangeMinDisp;
int trigP;
uint8_t trigSync;
int att10x;

void i2c_init()
{
    TWSR = 0;
    TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;
}

void i2c_start()
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void i2c_stop()
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void i2c_send_byte(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void i2c_send_command(uint8_t command)
{
    i2c_start();
    i2c_send_byte(I2C_ADDRESS << 1);
    i2c_send_byte(0x00);
    i2c_send_byte(command);
    i2c_stop();
}

void i2c_send_data_start()
{
    i2c_start();
    i2c_send_byte(I2C_ADDRESS << 1);
    i2c_send_byte(0x40);
}

void i2c_send_data_byte(uint8_t data)
{
    i2c_send_byte(data);
}

void i2c_send_data_stop()
{
    i2c_stop();
}

void i2c_init_display()
{
    i2c_init();
    _delay_ms(100);
    i2c_send_command(0xAE); // Display OFF
    i2c_send_command(0xD5); // Set display clock divide ratio/oscillator frequency
    i2c_send_command(0x80); // Set divide ratio
    i2c_send_command(0xA8); // Set multiplex ratio
    i2c_send_command(0x3F); // 1/64 duty cycle
    i2c_send_command(0xD3); // Set display offset
    i2c_send_command(0x00); // No offset
    i2c_send_command(0x40); // Set display start line
    i2c_send_command(0x8D); // Charge pump setting
    i2c_send_command(0x14); // Enable charge pump
    i2c_send_command(0x20); // Set memory mode
    i2c_send_command(0x00); // Horizontal addressing mode
    i2c_send_command(0xA1); // Set segment remap
    i2c_send_command(0xC8); // Set COM output scan direction
    i2c_send_command(0xDA); // Set COM pins hardware configuration
    i2c_send_command(0x12); // Alternative COM pin configuration
    i2c_send_command(0x81); // Set contrast control
    i2c_send_command(0xCF); // Set contrast value
    i2c_send_command(0xD9); // Set pre-charge period
    i2c_send_command(0xF1); // Phase 1: 15 DCLK, Phase 2: 1 DCLK
    i2c_send_command(0xDB); // Set VCOMH deselect level
    i2c_send_command(0x40); // 0.77 x VCC
    i2c_send_command(0xA4); // Entire display ON
    i2c_send_command(0xA6); // Normal display (non-inverted)
    i2c_send_command(0xAF); // Display ON
}

void i2c_clear_display()
{
    for (int i = 0; i < SCREEN_HEIGHT / 8; i++) {
        i2c_send_command(0xB0 + i); // Set page address
        i2c_send_command(0x00); // Set lower column address
        i2c_send_command(0x10); // Set higher column address
        i2c_send_data_start();
        for (int j = 0; j < SCREEN_WIDTH; j++) {
            i2c_send_data_byte(0x00); // Clear each column
        }
        i2c_send_data_stop();
    }
}

void i2c_set_pixel(uint8_t x, uint8_t y)
{
    if (x >= SCREEN_WIDTH || y >= SCREEN_HEIGHT) {
        return;
    }

    i2c_send_command(0xB0 + (y / 8)); // Set page address
    i2c_send_command(0x00 + (x & 0x0F)); // Set lower column address
    i2c_send_command(0x10 + ((x >> 4) & 0x0F)); // Set higher column address

    uint8_t data = 1 << (y % 8);
    i2c_send_data_start();
    i2c_send_data_byte(data);
    i2c_send_data_stop();
}

void adc_init()
{
    ADMUX = (1 << REFS0); // Reference voltage on AVCC
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC, set prescaler to 128
}

uint16_t adc_read(uint8_t channel)
{
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); // Select ADC channel
    ADCSRA |= (1 << ADSC); // Start conversion
    while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
    return ADC;
}

void setup()
{
    DDRB = 0x00; // Set PORTB as input
    PORTB = 0xFF; // Enable pull-up resistors

    DDRC = 0xFF; // Set PORTC as output
    PORTC = 0x00; // Set PORTC to LOW

    DDRD = 0x00; // Set PORTD as input
    PORTD = 0xFF; // Enable pull-up resistors

    i2c_init_display();
    i2c_clear_display();

    adc_init();

    sei(); // Enable global interrupts
}

void updateDisplay()
{
    i2c_clear_display();

    // Draw grid
    for (int i = 0; i < SCREEN_HEIGHT / 8; i++) {
        i2c_send_command(0xB0 + i); // Set page address
        i2c_send_command(0x00); // Set lower column address
        i2c_send_command(0x10); // Set higher column address
        i2c_send_data_start();
        for (int j = 0; j < SCREEN_WIDTH; j++) {
            if (j % 10 == 0 || i % 2 == 0) {
                i2c_send_data_byte(0xFF); // Draw grid lines
            } else {
                i2c_send_data_byte(0x00);
            }
        }
        i2c_send_data_stop();
    }

    // Draw waveform
    for (int i = 0; i < REC_LENGTH; i++) {
        int data = waveBuff[i];
        if (data > rangeMaxDisp) {
            data = rangeMaxDisp;
        } else if (data < rangeMinDisp) {
            data = rangeMinDisp;
        }
        int y = SCREEN_HEIGHT - ((data - rangeMinDisp) * SCREEN_HEIGHT) / (rangeMaxDisp - rangeMinDisp + 1);
        i2c_set_pixel(i, y);
    }

    // Draw trigger level
    int trigLevel = SCREEN_HEIGHT - ((trigP - rangeMinDisp) * SCREEN_HEIGHT) / (rangeMaxDisp - rangeMinDisp + 1);
    for (int i = 0; i < SCREEN_WIDTH; i++) {
        i2c_set_pixel(i, trigLevel);
    }

    // Draw scale
    for (int i = 0; i < V_RANGE_COUNT; i++) {
        int x = 3 + (i * 12);
        int y = SCREEN_HEIGHT - 8;
        int range = pgm_read_byte(&vstring_table[i][0]);
        sprintf(vScale, "%d", range);
        i2c_send_command(0xB0 + (y / 8));
        i2c_send_command(0x00 + (x & 0x0F));
        i2c_send_command(0x10 + ((x >> 4) & 0x0F));
        i2c_send_data_start();
        for (int j = 0; j < 4; j++) {
            i2c_send_data_byte(pgm_read_byte(&vstring_table[i][j + 1]));
        }
        i2c_send_data_stop();
    }

    for (int i = 0; i < H_RANGE_COUNT; i++) {
        int x = 110 + (i * 12);
        int y = SCREEN_HEIGHT - 8;
        sprintf(hScale, "%d", pgm_read_byte(&hstring_table[i][1]));
        i2c_send_command(0xB0 + (y / 8));
        i2c_send_command(0x00 + (x & 0x0F));
        i2c_send_command(0x10 + ((x >> 4) & 0x0F));
        i2c_send_data_start();
        for (int j = 0; j < 5; j++) {
            i2c_send_data_byte(pgm_read_byte(&hstring_table[i][j]));
        }
        i2c_send_data_stop();
    }
}

void updateWaveform()
{
    int min = waveBuff[0];
    int max = waveBuff[0];
    int ave = waveBuff[0];
    for (int i = 1; i < REC_LENGTH; i++) {
        if (waveBuff[i] < min) {
            min = waveBuff[i];
        }
        if (waveBuff[i] > max) {
            max = waveBuff[i];
        }
        ave += waveBuff[i];
    }
    dataMin = min;
    dataMax = max;
    dataAve = ave / REC_LENGTH;
    rangeMax = dataMax + 3;
    rangeMin = dataMin - 3;
    rangeMaxDisp = rangeMax;
    rangeMinDisp = rangeMin;
}

void saveWaveform()
{
    for (int i = 0; i < REC_LENGTH; i++) {
        waveBuff[i] = adc_read(ADC_CHANNEL);
    }
    updateWaveform();
}

void updateParameters()
{
    switch (vRange) {
        case 0:
            lsb5V = 0.0055549;
            lsb50V = 0.051513;
            break;
        case 1:
            lsb5V = 0.00055549;
            lsb50V = 0.0051513;
            break;
        case 2:
            lsb5V = 0.000055549;
            lsb50V = 0.00051513;
            break;
        case 3:
            lsb5V = 0.00001111;
            lsb50V = 0.00010303;
            break;
        case 4:
            lsb5V = 0.0000027777;
            lsb50V = 0.000025757;
            break;
        case 5:
            lsb5V = 0.00000055555;
            lsb50V = 0.0000051513;
            break;
        case 6:
            lsb5V = 0.00000013888;
            lsb50V = 0.0000012878;
            break;
        case 7:
            lsb5V = 0.000000027777;
            lsb50V = 0.00000025757;
            break;
        case 8:
            lsb5V = 0.0000000055555;
            lsb50V = 0.000000051513;
            break;
        case 9:
            lsb5V = 0.0000000022222;
            lsb50V = 0.000000020605;
            break;
    }

    switch (hRange) {
        case 0:
            saveTimer = 20;
            break;
        case 1:
            saveTimer = 8;
            break;
        case 2:
            saveTimer = 4;
            break;
        case 3:
            saveTimer = 2;
            break;
        case 4:
            saveTimer = 1;
            break;
        case 5:
            saveTimer = 0;
            break;
        case 6:
            saveTimer = -2;
            break;
        case 7:
            saveTimer = -4;
            break;
    }
}

void updateWaveTrigger()
{
    for (int i = 0; i < REC_LENGTH; i++) {
        if (trigSync) {
            trigD = dataAve;
        } else {
            trigD = trigP;
        }
        if ((waveBuff[i] >= trigD) && (waveBuff[i - 1] < trigD)) {
            scopeP = i;
            break;
        }
    }
    scopeP += saveTimer;
    if (scopeP >= REC_LENGTH) {
        scopeP = REC_LENGTH - 1;
    } else if (scopeP < 0) {
        scopeP = 0;
    }
}

void loop()
{
    if (!(PINB & (1 << BUTTON1_PIN))) {
        if (vRange < V_RANGE_COUNT - 1) {
            vRange++;
            paraChanged = 1;
        }
    }
    if (!(PINB & (1 << BUTTON2_PIN))) {
        if (vRange > 0) {
            vRange--;
            paraChanged = 1;
        }
    }
    if (!(PINB & (1 << BUTTON3_PIN))) {
        if (hRange < H_RANGE_COUNT - 1) {
            hRange++;
            paraChanged = 1;
        }
    }
    if (!(PINB & (1 << BUTTON4_PIN))) {
        if (hRange > 0) {
            hRange--;
            paraChanged = 1;
        }
    }
    if (!(PINB & (1 << BUTTON5_PIN))) {
        if (trigSync) {
            trigSync = 0;
        } else {
            trigSync = 1;
        }
        paraChanged = 1;
    }

    if (paraChanged) {
        paraChanged = 0;
        updateParameters();
        saveWaveform();
    }

    if (!(PIND & (1 << TRIGGER_PIN))) {
        if (!hold) {
            hold = 1;
            saveWaveform();
        }
    } else {
        if (hold) {
            hold = 0;
            saveWaveform();
        }
    }

    updateWaveTrigger();
    updateDisplay();

    _delay_ms(10);
}

int main()
{
    setup();

    while (1) {
        loop();
    }

    return 0;
}
