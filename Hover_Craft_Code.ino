//ENGR 290 Summer 2026 group #6
//Hover Craft
//
//
// MODE OVERVIEW:
//   FORWARD MODE: Servo holds center (2500 ticks = 1250us) with IMU yaw
//                 trim to counteract drift and keep straight.
//   TURN MODE:    Servo sweeps to preset LEFT/RIGHT position.

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>

// =========================================================================
// CONFIGURATION
// =========================================================================
#define BAUD          9600UL
#define UBRR          ((F_CPU)/((BAUD)*(16UL))-1)
#define SCL_CLOCK     100000L

// Servo tick values (Prescaler 8 → 0.5us/tick, ICR1 = 40000 → 20ms/50Hz)
#define SERVO_CENTER  2500   // 1250us — straight ahead
#define SERVO_LEFT    750   // 1000us — full left turn
#define SERVO_RIGHT   4250   // 1500us — full right turn

// Yaw trim gain: ticks of correction per degree of yaw error
// Tune this value — start small to avoid oscillation

/////////////////////////////////////////////////
#define YAW_TRIM_GAIN 13.56f //updated
//if it is overcorrecting too much in forward mode, decrease this value 
/////////////////////////////////////////////////

// IMU
#define IMU_ADDR      0xD0   // MPU6050 write address (0x68 << 1)

//Turning timer 
uint32_t turn_start_ms = 0;

//turn mode duraftion 
#define TURN_DURATION_MS 3000  // 3 seconds in turn mode
//Modify based on track runs 
///////////////////////////////////////////////////


// Ultrasonic p6
//front sensor 
#define TRIG_PIN_1      PB3   
#define ECHO_PIN_1      PD2   

//Ultrasonnic p13
//SIDE
#define TRIG_PIN_2     PB5   
#define ECHO_PIN_2     PD3    

// Fans 
#define BASE_FAN PD5    // Base fan MOSET PIN   
#define THRUST_FAN PD6  // Thrust fan MOSET PIN 

// =========================================================================
// DRIVE MODES
// =========================================================================
//states 
#define St_idle       0
#define St_wait_echo  1
#define St_trig_pulse 2
#define St_measure    3
#define St_process    4
#define St_wait_echo2 5
#define St_measure2   6
#define St_process2   7
#define MODE_FORWARD  8
#define MODE_TURN     9


// =========================================================================
// GLOBALS
// =========================================================================
static volatile uint8_t  g_mode = 0; 
static volatile uint8_t  g_turn_dir  = 0;  // Alternates: 0=left, 1=right
static volatile uint8_t  send_telemetry = 0;
static volatile uint32_t timerMicros = 0;
int state = 0;//for switch cases


uint16_t servo_current = SERVO_CENTER;
uint16_t servo_target  = SERVO_CENTER;
#define SERVO_STEP 15  // ticks per iteration — tune this


// States variables--------------
static volatile uint8_t g_sys_flags = 0; 
static volatile uint8_t g_sensor_state = St_idle; 
static volatile uint8_t g_sensor_state2 = St_idle;


// Ultrasonic Timer variables-------------
static volatile unsigned long g_echo_start = 0;
static volatile unsigned long g_echo_end = 0;
static volatile unsigned long g_echo_start2 = 0;
static volatile unsigned long g_echo_end2 = 0;
static volatile unsigned long g_timeout_start_ms = 0; // Tracks timeout windows safely via millis()


//timer variables 
volatile unsigned long timerMillis = 0;
unsigned long prevMillis = 0;
const unsigned long long_interval = 800; //1 second delay 
unsigned long prevSensorMillis = 0;
unsigned long last_print_ms = 0; 
unsigned long forward_start_ms =0;

// IMU data
int16_t real_ax, real_ay, real_az;
int16_t real_gx, real_gy, real_gz;
float   final_ax, final_ay, final_az;
float   final_gx, final_gy, final_gz;

// Kinematics
const float dt  = 0.02f;
float yaw_angle = 0.0f;
float fi_x      = 0.0f;
float velocity_x = 0.0f;

// serial monitor 
float print_yaw, print_ax, print_ay, print_az, print_x;

char txt[20];

// =========================================================================
// FUNCTION PROTOTYPES
// =========================================================================
void gpio_init(void);
void uart_init(void);
void timer0_us_init(void);
void timer1_servo_init(void);
void twi_init(void);
void i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
void i2c_read_imu(void);
void servo_set(uint8_t postion);
void print(char txtarr[]);
void ultrasonic_trigger(void);
uint16_t ultrasonic_get_distance_cm(void);
unsigned long myMillis(void);
float getDistance(); 
float getDistance2();
void servo_update(void); 

// =========================================================================
// MAIN
// =========================================================================
int main(void) {
    uart_init();
    print ("uart ok\n");
    gpio_init();
    print("gpio ok\n");
    twi_init();
    print("twi_init ok\n");
    timer0_us_init();
    print("timer0 ok\n");
    timer1_servo_init();
    print("timer1_servo_init\n");
    

    _delay_ms(100); //non blocking delay for set up 

    // Wake IMU
    i2c_write_reg(IMU_ADDR, 0x6B, 0x00); // Clear sleep bit
    i2c_write_reg(IMU_ADDR, 0x1C, 0x00); // Accel ±2g
    i2c_write_reg(IMU_ADDR, 0x1B, 0x00); // Gyro ±250 deg/s
    print("i2c ok\n");
    sei();

    unsigned long last_dist_fwd_ms  = 0;  // MODE_FORWARD refresh timer 
    float dist = 0.0f;
    float dist2 = 0.0f; 

    //Fan set up 
    PORTD |=  (1 << BASE_FAN) | (1<<THRUST_FAN);
  
    
    print("Starting in idle mode\r\n"); //for troubleshooting 
 
    while (1) {
        unsigned long currentMillis = myMillis(); //refresh 

        switch (g_mode) {

            ///////////////////////////IDLE//////////////////////////////////
            case St_idle:{
                print("in idle\n");
                if (currentMillis - prevSensorMillis >= 500) {
                    prevSensorMillis = currentMillis;
                    g_mode = MODE_FORWARD;
                    turn_start_ms = myMillis(); //turn delay 
                    print("leaving idle\n");
                } else {
                    print("waiting...\n");
                }
                break;
            }
            //////////////////////FORWARD/////////////////////////////////
            case MODE_FORWARD: {
                        if (currentMillis - last_dist_fwd_ms >= 60) {
                            last_dist_fwd_ms = currentMillis;
                            dist = getDistance(); //forward distance 
                        }

                        i2c_read_imu();
                        if (fabsf(final_gz) < 1.5f) final_gz = 0.0f; //prevents accumulation 

                        // Angular Yaw Tracking Loop (Z-Axis Gyro Rotation)
                        yaw_angle += final_gz * dt;
                        float clamped_yaw = yaw_angle;
                        if (clamped_yaw >  84.0f) clamped_yaw =  84.0f; //from tech_2 range to keep craft straight 
                        if (clamped_yaw < -84.0f) clamped_yaw = -84.0f;

                        int16_t trim   = (int16_t)(clamped_yaw * YAW_TRIM_GAIN);
                        int16_t target = (int16_t)SERVO_CENTER - trim;
                        if (target < SERVO_LEFT)  target = SERVO_LEFT;
                        if (target > SERVO_RIGHT) target = SERVO_RIGHT;
                        OCR1A = (uint16_t)target;


                            //for serial monitor printing 
                            if (currentMillis - last_print_ms >= 300) {
                            last_print_ms = currentMillis;
                            print("Front Distance: ");
                            dtostrf(dist, 5, 2, txt);
                            print(txt);
                            print(" cm\r\n");
                            }

                // Obstacle detection
            
                        if ( dist <= 20.0f) { //20cm threshold for front sensor 
                            dist2 = getDistance2();
                            if (dist2 >= 15.0f) { //15cm threshold for side sensor 
                                servo_set(50);
                                print("Turning Left\n");
                            } 
                            else {
                                servo_set(205);
                                print("Turning Right\n");
                            }
                            g_mode = MODE_TURN;
                            turn_start_ms = myMillis();
                        }
                break;
            }   // end MODE_FORWARD
            
            
            /////////////////////////////TURN//////////////////////////////
            case MODE_TURN: {
                if (currentMillis - turn_start_ms <= TURN_DURATION_MS) { //turn timer 
                    // Still turning — servo already set at entry
                } else {
                    // Duration expired — re-center and return to forward
                    servo_set(127);
                    g_mode = MODE_FORWARD;
                    forward_start_ms = myMillis();
                    yaw_angle = 0.0f; //reset yaw 
                    print(">> FORWARD MODE\r\n");
                }
                break;
            }   // end MODE_TURN

        }   // end switch
    }   // end while(1)
    return 0;
}   // end main
   
/////////////////////////////end main/////////////////////////////////////////
//========================FRONT_DISTANCE=====================================
float getDistance(){
    cli();
    g_echo_start = 0;
    g_echo_end   = 0;
    g_sensor_state = St_idle;
    sei();

    // Trigger pulse
    PORTB |= (1 << TRIG_PIN_1);
    for(volatile uint8_t i = 0; i < 40; i++) { }
    PORTB &= ~(1 << TRIG_PIN_1);

    // Configure INT0 for rising edge
    EICRA &= ~((1 << ISC01) | (1 << ISC00));
    EICRA |=  ((1 << ISC01) | (1 << ISC00));
    EIFR  |= (1 << INTF0);
    EIMSK |= (1 << INT0);
    g_sensor_state = St_wait_echo;

    // Wait for echo to complete (30ms for 5m range)
     unsigned long start = myMillis();
    while (g_sensor_state != St_process) {
        if (myMillis() - start > 30){
            break;
        }  // timeout
    }

  unsigned int pulse_duration = g_echo_end - g_echo_start;

  float distance = (float)pulse_duration / 116.62;

// Ignore readings outside realistic range
if (distance > 400.0f | distance == 0.0 ) return 30.0f;
else 
return distance;  
}
//=========================================================================

//============================SIDE_DISTANCE==============================
float getDistance2(){
    cli();
    g_echo_start2 = 0;
    g_echo_end2   = 0;
    g_sensor_state2 = St_idle;
    sei();

    // Trigger pulse
    PORTB |= (1 << TRIG_PIN_2);
    for(volatile uint8_t i = 0; i < 40; i++) { }
    PORTB &= ~(1 << TRIG_PIN_2);

    // Configure INT0 for rising edge
    EICRA &= ~((1 << ISC11) | (1 << ISC10));  // clear INT1 bits
    EICRA |=  ((1 << ISC11) | (1 << ISC10));  // set rising edge for INT1
    EIFR  |= (1 << INTF1);
    EIMSK |= (1 << INT1);
    g_sensor_state2 = St_wait_echo2;
//print("INT1 armed\n");  // ← does this print?
    // Wait for echo to complete (max ~30ms for 5m range)
    unsigned long start2 = myMillis();
    while (g_sensor_state2 != St_process2) {
        if (myMillis() - start2 > 30){
            //print("timeout2");
            g_sensor_state2 = St_idle;
            
            break;
        }  // timeout
    }

   unsigned int pulse_duration2 = g_echo_end2 - g_echo_start2;

  float distance2 = (float)pulse_duration2 / 116.62;

// Ignore readings outside realistic range
if (distance2 > 400.0f | distance2 == 0.0 ) return 30.0f;
else
return distance2;
}
//==========================================================================
// =========================================================================
// SERVO
// =========================================================================
void servo_set(uint8_t position) {
   OCR1A = SERVO_LEFT + (uint16_t)(((float)(SERVO_RIGHT - SERVO_LEFT) / 255.0f) * (float)position);
}

// =========================================================================
// TIMER 0 — microsecond counter (prescaler 8, OCR0A=15 → 8us per tick)
// =========================================================================
void timer0_us_init(void) {
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01);
    OCR0A  = 15;
    TIMSK0 |= (1 << OCIE0A);
}

unsigned long myMillis(void){
    unsigned long ms;
    cli();
    ms = timerMillis;
    sei();
    return ms;
}

ISR(TIMER0_COMPA_vect) {
    timerMicros += 8;
    static uint16_t accum = 0;
    accum += 8;
    if (accum >= 1000) {
        accum -= 1000;
        timerMillis++;
    }
}

ISR(INT0_vect) {
    if (g_sensor_state == St_wait_echo) {
        g_echo_start = TCNT1;
        g_sensor_state = St_measure;
        EICRA &= ~(1 << ISC00);  
    } else if (g_sensor_state == St_measure) {
        g_echo_end = TCNT1;
        g_sensor_state = St_process;
        EIMSK &= ~(1 << INT0);
    }
}

ISR(INT1_vect) {
    if (g_sensor_state2 == St_wait_echo2) {
        g_echo_start2 = TCNT1;
        g_sensor_state2 = St_measure2;
        EICRA &= ~(1 << ISC10);  
    } else if (g_sensor_state2 == St_measure2) {
        g_echo_end2 = TCNT1;
        g_sensor_state2 = St_process2;
        EIMSK &= ~(1 << INT1);
    }
}

// =========================================================================
// TIMER 1 — Servo PWM (50Hz, prescaler 8, 0.5us/tick)
// =========================================================================
void timer1_servo_init(void) {
    TCCR1A = 0;  // Reset
    TCCR1B = 0;  // Reset
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Mode 14, prescaler 8
    ICR1   = 40000;        // 20ms period (50Hz)
    OCR1A  = SERVO_CENTER; // Start at center  
}



// =========================================================================
// GPIO
// =========================================================================
void gpio_init(void) {
  // 1 = output, // 0 or nothing = input
    // PB1 = OC1A (servo), PB3 = TRIG_1, PB5 = TRIG_2  
    DDRB  = (1 << PB1) |(1 << PB3)| (1 << PB5);
    PORTB = 0;

    // PD1 = UART TX, PD6=Thrust fan, PD5= Base fan   
    DDRD  |= (1 << PD1) | (1 << BASE_FAN) | (1 << THRUST_FAN);
    //PD2 and PD3 are inputs 
    PORTD = (1 << PD1);

    // I2C pullups
    DDRC  = 0;
    PORTC = (1 << PC4) | (1 << PC5);

//External Interrupt Control Register A 
//For Ultrasonic Senosr 
EICRA = (1 << ISC00) | (1 << ISC10);  // any logical change on INT0 and INT1
EIMSK = (1 << INT0)  | (1 << INT1);   // enable both
}

// =========================================================================
// UART
// =========================================================================
void uart_init(void) {
    UBRR0H = (uint8_t)((UBRR) >> 8);
    UBRR0L = (uint8_t)(UBRR);
    UCSR0B = (1 << TXEN0);
    UCSR0C = (3 << UCSZ00);
}

void print(char txtarr[]) {
    int i = 0;
    while (txtarr[i] != '\0') {
        while (!(UCSR0A & (1 << UDRE0)));
        UDR0 = txtarr[i++];
    }
}

// =========================================================================
// I2C / TWI
// =========================================================================
void twi_init(void) {
    TWSR = 0;
    TWBR = (uint8_t)(((F_CPU / SCL_CLOCK) - 16) >> 1);
}

void i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    TWDR = dev_addr;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    TWDR = reg_addr;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void i2c_read_imu(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    TWDR = IMU_ADDR;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    TWDR = 0x3B;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));

    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    TWDR = IMU_ADDR | 0x01;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));

    // Read 14 bytes: Ax, Ay, Az, Temp (skipped at [6,7]), Gx, Gy, Gz
    uint8_t buffer[14];
    for (int i = 0; i < 13; i++) {
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
        while (!(TWCR & (1 << TWINT)));
        buffer[i] = TWDR;
    }

    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    buffer[13] = TWDR;
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);

    real_ax = (buffer[0]  << 8) | buffer[1];
    real_ay = (buffer[2]  << 8) | buffer[3];
    real_az = (buffer[4]  << 8) | buffer[5];
    real_gx = (buffer[8]  << 8) | buffer[9];
    real_gy = (buffer[10] << 8) | buffer[11];
    real_gz = (buffer[12] << 8) | buffer[13];

    final_ax = (float)real_ax / 16384.0f;
    final_ay = (float)real_ay / 16384.0f;
    final_az = (float)real_az / 16384.0f;
    final_gx = (float)real_gx / 131.0f;
    final_gy = (float)real_gy / 131.0f;
    final_gz = (float)real_gz / 131.0f;
}