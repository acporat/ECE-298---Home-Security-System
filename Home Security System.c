//***************************************************************************************
//  ECE 298 LS3G15 Home Security System Protoype Software
//	July 17th 2019
//	Created by David Zhang and Aylon Porat
//***************************************************************************************

#include <msp430.h>
#include "driverlib.h"
#include "stdbool.h"


char hexaKeys[4][3] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

const char digit[10][2] =
{
    {0xFC, 0x28},  /* "0" LCD segments a+b+c+d+e+f+k+q */
    {0x60, 0x20},  /* "1" */
    {0xDB, 0x00},  /* "2" */
    {0xF3, 0x00},  /* "3" */
    {0x67, 0x00},  /* "4" */
    {0xB7, 0x00},  /* "5" */
    {0xBF, 0x00},  /* "6" */
    {0xE4, 0x00},  /* "7" */
    {0xFF, 0x00},  /* "8" */
    {0xF7, 0x00}   /* "9" */
};

// LCD memory map for uppercase letters
const char alphabetBig[26][2] =
{
    {0xEF, 0x00},  /* "A" LCD segments a+b+c+e+f+g+m */
    {0xF1, 0x50},  /* "B" */
    {0x9C, 0x00},  /* "C" */
    {0xF0, 0x50},  /* "D" */
    {0x9F, 0x00},  /* "E" */
    {0x8F, 0x00},  /* "F" */
    {0xBD, 0x00},  /* "G" */
    {0x6F, 0x00},  /* "H" */
    {0x90, 0x50},  /* "I" */
    {0x78, 0x00},  /* "J" */
    {0x0E, 0x22},  /* "K" */
    {0x1C, 0x00},  /* "L" */
    {0x6C, 0xA0},  /* "M" */
    {0x6C, 0x82},  /* "N" */
    {0xFC, 0x00},  /* "O" */
    {0xCF, 0x00},  /* "P" */
    {0xFC, 0x02},  /* "Q" */
    {0xCF, 0x02},  /* "R" */
    {0xB7, 0x00},  /* "S" */
    {0x80, 0x50},  /* "T" */
    {0x7C, 0x00},  /* "U" */
    {0x0C, 0x28},  /* "V" */
    {0x6C, 0x0A},  /* "W" */
    {0x00, 0xAA},  /* "X" */
    {0x00, 0xB0},  /* "Y" */
    {0x90, 0x28}   /* "Z" */
};


#define pos1 4                                                 // Digit A1 - L4
#define pos2 6                                                 // Digit A2 - L6
#define pos3 8                                                 // Digit A3 - L8
#define pos4 10                                                // Digit A4 - L10
#define pos5 2                                                 // Digit A5 - L2
#define pos6 18                                                // Digit A6 - L18
// Define word access definitions to LCD memories
#define LCDMEMW ((int*)LCDMEM)
#define LCDBMEMW ((int*)LCDBMEM)

//SERVO MOTOR
#define completePeriod 511
bool increasing = true;
int highPeriod;
Timer_A_initCompareModeParam initComp2Param = {0};
Timer_A_initUpModeParam param = {0};

//BUZZER
Timer_A_initCompareModeParam initComp1Param = {0};
Timer_A_initUpModeParam param_buzzer = {0};
bool increasing_buzzer = true;
int highPeriod_buzzer;

bool newKey = false;
char hexKey = 'D';

char q = 'q';



uint8_t RowPorts[4] = {GPIO_PORT_P8, GPIO_PORT_P1, GPIO_PORT_P5, GPIO_PORT_P8};
uint8_t RowPins[4] =  {GPIO_PIN2,    GPIO_PIN0,    GPIO_PIN1,    GPIO_PIN0};

uint8_t ColPorts[3] = {GPIO_PORT_P1, GPIO_PORT_P1, GPIO_PORT_P1};
uint8_t ColPins[3] =  {GPIO_PIN3,    GPIO_PIN4,    GPIO_PIN5};

uint8_t servoPort = GPIO_PORT_P8;
uint8_t servoPin = GPIO_PIN3;

uint8_t reedPorts[4] = {GPIO_PORT_P2, GPIO_PORT_P2, GPIO_PORT_P1, GPIO_PORT_P1};
uint8_t reedPins[4] =  {GPIO_PIN7,    GPIO_PIN5,    GPIO_PIN6,    GPIO_PIN1};

uint8_t buzzerPort = GPIO_PORT_P1;
uint8_t buzzerPin = GPIO_PIN7;

uint8_t ledPorts[4] = { GPIO_PORT_P8, GPIO_PORT_P5, GPIO_PORT_P5, GPIO_PORT_P5};
uint8_t ledPins[4] = { GPIO_PIN1, GPIO_PIN0, GPIO_PIN2, GPIO_PIN3};

enum states {OPEN, ARMED};


void keypad_aylon();
void init_LCD_aylon();
void init_reed();
void init_buzzer();
void init_servo();
void init_leds();

void showChar(char c, int position);
void buzzer_sound();

void openDoor ();
void closeDoor ();
void servo_delay();
void display_OPEN();
void display_ARMED();
void set_open();
void set_armed();
void clearLCD();
bool equals(char* one, char* two);
enum states currState = OPEN;
bool reeds[4] = {false, false, false, false};

char password[4] = {'1','2','3','4'};
char userInput[4] = {'a','a','a','a'};
int userInputIndex = 0;

bool firstTime = true;
void moveDown(char* word, char key);

void display_PSSWD();

void init_RTC();



void main(void) {
    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;                   // Disable the GPIO power-on default high-impedance mode
                                            // to activate previously configured port settings

    keypad_aylon();
    init_LCD_aylon();
    init_reed();
    init_buzzer();
    init_servo();
    init_leds();
    __enable_interrupt();

    int k;
    set_open();




    for (;;){


        switch (currState){
        case OPEN:
            if (firstTime){
                display_OPEN();
                firstTime = false;
            }
            if (newKey){
                if (hexKey == '#'){
                    openDoor();
                    servo_delay();
                    closeDoor();
                    newKey = false;
                    firstTime = true;
                }
                else if (hexKey == '*'){
                    newKey = false;
                    set_armed();
                    currState = ARMED;
                }
                else{
                    newKey = false;
                }
             //display_OPEN();
            }
        break;

        case ARMED:
            if (firstTime){
                display_ARMED();
                firstTime = false;
            }
            for (k = 0; k<4; k++){
                if (reeds[k]){
                    GPIO_setOutputHighOnPin(ledPorts[k], ledPins[k]);

                }
            }
            if (reeds[0] || reeds[1] || reeds[2] || reeds[3]){
                buzzer_sound();
            }

            if (newKey){
                moveDown(userInput, hexKey);
                newKey = false;
                display_PSSWD();
                if (equals(userInput, password)){
                    set_open();
                    newKey = false;
                    currState = OPEN;
                    firstTime = true;
                }
                else{
                    newKey = false;
                }

            }
        break;
        };

    }
}


void moveDown(char* word, char key){
    word[0] = word[1];
    word[1] = word[2];
    word[2] = word[3];
    word[3] = key;
}

bool equals(char* one,char* two){
    int i;
    for (i = 0; i<4; i++){
        if (one[i] != two[i]){
            return false;
        }
    }
    return true;
}

void display_PSSWD(char* word){
    clearLCD();
    showChar(word[0], pos1);
    showChar(word[1], pos2);
    showChar(word[2], pos3);
    showChar(word[3], pos4);
}

void clearLCD(){
    LCDMEMCTL |= LCDCLRM | LCDCLRBM;                           // Clear LCD memory
    LCDCSSEL0 = 0x000F;                                        // Configure COMs and SEGs
    LCDCSSEL1 = 0x0000;                                        // L0, L1, L2, L3: COM pins
    LCDCSSEL2 = 0x0000;

    LCDM0 = 0x21;                                              // L0 = COM0, L1 = COM1
    LCDM1 = 0x84;                                              // L2 = COM2, L3 = COM3


    LCDCTL0 |= LCD4MUX | LCDON;                                // Turn on LCD, 4-mux selected

    PMMCTL0_H = PMMPW_H;                                       // Open PMM Registers for write
    PMMCTL0_L |= PMMREGOFF_L;                                  // and set PMMREGOFF

    //__bis_SR_register(LPM3_bits | GIE);                        // Enter LPM3
    __no_operation();                                          // For debugger

    _delay_cycles(2800);
}

void set_open(){
    int q;
    for (q = 0; q<4; q++){
        userInput[q] = 'a';
    }
    display_OPEN();
    int i;
    __disable_interrupt();
    for (i = 0; i<4; i++){
        GPIO_clearInterrupt(reedPorts[i], reedPins[i]);
        GPIO_disableInterrupt(reedPorts[i], reedPins[i]);
        GPIO_setOutputLowOnPin(ledPorts[i], ledPins[i]);
        reeds[i] = false;
    }
    __enable_interrupt();
};

void set_armed(){
    bool zones[4] = {true, true, true, true};
    char word[4] = {'A','R','M','1'};
    display_PSSWD(word);
    while (!newKey){

    }
    if (hexKey == '2'){
        zones[0] = false;
    }
    newKey = false;

    word[3] = '2';
    display_PSSWD(word);
    while (!newKey){

    }
    if (hexKey == '2'){
        zones[1] = false;
    }
    newKey = false;

    word[3] = '3';
    display_PSSWD(word);
    while (!newKey){

    }
    if (hexKey == '2'){
        zones[2] = false;
    }
    newKey = false;

    word[3] = '4';
    display_PSSWD(word);
    while (!newKey){

    }
    if (hexKey == '2'){
        zones[3] = false;
    }
    newKey = false;



    int i;
    __disable_interrupt();
    for (i = 0; i<4; i++){
        if (zones[i]){
            GPIO_clearInterrupt(reedPorts[i], reedPins[i]);
            GPIO_enableInterrupt(reedPorts[i], reedPins[i]);
            GPIO_selectInterruptEdge(reedPorts[i], reedPins[i], GPIO_HIGH_TO_LOW_TRANSITION);
        }
        else{
            GPIO_disableInterrupt(reedPorts[i], reedPins[i]);
            GPIO_clearInterrupt(reedPorts[i], reedPins[i]);

        }
    }
    __enable_interrupt();
    display_ARMED();

};


void init_leds(){
  int j;
  for(j = 0; j<4; j++){
      GPIO_setAsOutputPin(ledPorts[j], ledPins[j]);
      GPIO_setOutputLowOnPin(ledPorts[j], ledPins[j]);
  }
};


void servo_delay(){
    volatile unsigned int i = 10000;
    while (i != 0){
        i--;
    }
};

void keypad_aylon(){

    int j;
    for (j = 0; j<4; j++){
        GPIO_setAsOutputPin(RowPorts[j], RowPins[j]);
        GPIO_setOutputHighOnPin(RowPorts[j], RowPins[j]);
    }

    for (j = 0; j<3; j++){
        GPIO_setAsInputPinWithPullDownResistor(ColPorts[j], ColPins[j]);
        GPIO_enableInterrupt(ColPorts[j], ColPins[j]);
        GPIO_selectInterruptEdge(ColPorts[j], ColPins[j], GPIO_LOW_TO_HIGH_TRANSITION);
        GPIO_clearInterrupt(ColPorts[j], ColPins[j]);
    }

};


void init_LCD_aylon(){
    WDTCTL = WDTPW | WDTHOLD;                                  // Stop watchdog timer

        // Configure XT1 oscillator
        P4SEL0 |= BIT1 | BIT2;                                     // P4.2~P4.1: crystal pins
        do
        {
            CSCTL7 &= ~(XT1OFFG | DCOFFG);                         // Clear XT1 and DCO fault flag
            SFRIFG1 &= ~OFIFG;
        }while (SFRIFG1 & OFIFG);                                  // Test oscillator fault flag
        CSCTL6 = (CSCTL6 & ~(XT1DRIVE_3)) | XT1DRIVE_2;            // Higher drive strength and current consumption for XT1 oscillator


        // Disable the GPIO power-on default high-impedance mode
        // to activate previously configured port settings
        PM5CTL0 &= ~LOCKLPM5;

        // Configure LCD pins
        SYSCFG2 |= LCDPCTL;                                        // R13/R23/R33/LCDCAP0/LCDCAP1 pins selected

        LCDPCTL0 = 0xFFFF;
        LCDPCTL1 = 0x07FF;
        LCDPCTL2 = 0x00F0;                                         // L0~L26 & L36~L39 pins selected

        LCDCTL0 = LCDSSEL_0 | LCDDIV_7;                            // flcd ref freq is xtclk

        // LCD Operation - Mode 3, internal 3.08v, charge pump 256Hz
            LCDVCTL = LCDCPEN | LCDREFEN | VLCD_6 | (LCDCPFSEL0 | LCDCPFSEL1 | LCDCPFSEL2 | LCDCPFSEL3);

        LCDMEMCTL |= LCDCLRM | LCDCLRBM;                           // Clear LCD memory

        LCDCSSEL0 = 0x000F;                                        // Configure COMs and SEGs
        LCDCSSEL1 = 0x0000;                                        // L0, L1, L2, L3: COM pins
        LCDCSSEL2 = 0x0000;

        LCDM0 = 0x21;                                              // L0 = COM0, L1 = COM1
        LCDM1 = 0x84;                                              // L2 = COM2, L3 = COM3


        LCDCTL0 |= LCD4MUX | LCDON;                                // Turn on LCD, 4-mux selected

        PMMCTL0_H = PMMPW_H;                                       // Open PMM Registers for write
        PMMCTL0_L |= PMMREGOFF_L;                                  // and set PMMREGOFF

        //__bis_SR_register(LPM3_bits | GIE);                        // Enter LPM3
        __no_operation();                                          // For debugger
};

void init_servo(){
    increasing = true;          // INCREASE
    highPeriod = 0; // NO LIGHT IN BEGINNING

    //Stop WDT
    WDT_A_hold(WDT_A_BASE);

    //P8.3 as output with module function
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P8, GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    PMM_unlockLPM5();

    //Start timer

    param.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod = completePeriod;
    param.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    param.captureCompareInterruptEnable_CCR0_CCIE =
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE;
    param.timerClear = TIMER_A_DO_CLEAR;
    param.startTimer = true;
    Timer_A_initUpMode(TIMER_A1_BASE, &param);


    //Initialize compare mode to generate PWM

    initComp2Param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2;
    initComp2Param.compareInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE;
    initComp2Param.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;


};

void display_OPEN(){

    clearLCD();
    showChar(' ', pos1);
    showChar('O', pos2);
    showChar('P', pos3);
    showChar('E', pos4);
    showChar('N', pos5);
};

void display_ARMED(){

    clearLCD();
    showChar('A', pos1);
    showChar('R', pos2);
    showChar('M', pos3);
    showChar('E', pos4);
    showChar('D', pos5);
};

void init_reed(){
    int j;
    for (j = 0; j<4; j++){
        GPIO_setAsInputPinWithPullUpResistor(reedPorts[j], reedPins[j]);
        GPIO_disableInterrupt(reedPorts[j], reedPins[j]);
        GPIO_selectInterruptEdge(reedPorts[j], reedPins[j], GPIO_HIGH_TO_LOW_TRANSITION);
        GPIO_clearInterrupt(reedPorts[j], reedPins[j]);
    }
};

void showChar(char c, int position){
    if (c == '*')
        c = 'A';
    if (c == '#')
        c = 'B';


    if (c == ' ')
    {
        // Display space
        LCDMEMW[position/2] = 0;
    }
    else if (c >= '0' && c <= '9')
    {
        // Display digit
        LCDMEMW[position/2] = digit[c-48][0] | (digit[c-48][1] << 8);
    }
    else if (c >= 'A' && c <= 'Z')
    {
        // Display alphabet
        LCDMEMW[position/2] = alphabetBig[c-65][0] | (alphabetBig[c-65][1] << 8);
    }
    else
    {
        // Turn all segments on if character is not a space, digit, or uppercase letter
        LCDMEMW[position/2] = 0xFFFF;
    }
};

void openDoor (){
    while (increasing == true) {
        if (highPeriod == 511){
            increasing = false;
        }
        else{
            highPeriod++;
            initComp2Param.compareValue = highPeriod;
            Timer_A_initCompareMode(TIMER_A1_BASE, &initComp2Param);
            _delay_cycles(2800);
        }
    }
};

void closeDoor (){
    while (increasing == false) {
        if (highPeriod == 0){
            increasing = true;
        }
        else{
            highPeriod--;
            initComp2Param.compareValue = highPeriod;
            Timer_A_initCompareMode(TIMER_A1_BASE, &initComp2Param);
            _delay_cycles(2800);
        }
    }
};

void init_buzzer(){
    increasing_buzzer = true;          // INCREASE
    highPeriod_buzzer = 0; // NO LIGHT IN BEGINNING

    GPIO_setAsPeripheralModuleFunctionOutputPin(buzzerPort, buzzerPin, GPIO_PRIMARY_MODULE_FUNCTION);

    PMM_unlockLPM5();

    //Start timer

    param_buzzer.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    param_buzzer.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param_buzzer.timerPeriod = completePeriod;
    param_buzzer.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    param_buzzer.captureCompareInterruptEnable_CCR0_CCIE =
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE;
    param_buzzer.timerClear = TIMER_A_DO_CLEAR;
    param_buzzer.startTimer = true;
    Timer_A_initUpMode(TIMER_A0_BASE, &param_buzzer);


    //Initialize compare mode to generate PWM

    initComp1Param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    initComp1Param.compareInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE;
    initComp1Param.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
};

void buzzer_sound(){
    while (increasing_buzzer == true) {
            if (highPeriod_buzzer == 511){
                increasing_buzzer = false;
            }
            else{
                highPeriod_buzzer++;
                initComp1Param.compareValue = highPeriod_buzzer;
                Timer_A_initCompareMode(TIMER_A0_BASE, &initComp1Param);
                _delay_cycles(2800);
            }
        }
    GPIO_setOutputLowOnPin(buzzerPort, buzzerPin);
    volatile unsigned int i = 10000;
    while (i != 0){
        i--;
    }
    i = 0;

    while (increasing_buzzer == false) {
        if (highPeriod_buzzer == 0){
            increasing_buzzer = true;
        }
        else{
            highPeriod_buzzer--;
            initComp1Param.compareValue = highPeriod_buzzer;
            Timer_A_initCompareMode(TIMER_A0_BASE, &initComp1Param);
            _delay_cycles(2800);
        }
    }
    GPIO_setOutputLowOnPin(buzzerPort, buzzerPin);

    i = 10000;
    while (i != 0){
        i--;
    }
    i = 0;
};


// Port 1 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT1_VECTOR))) Port_1 (void)
#else
#error Compiler not supported!
#endif
{
    int column = -1;

    int j = 0;
    while (j<3){
        if (GPIO_getInterruptStatus( ColPorts[j], ColPins[j])){
            column = j;
            GPIO_clearInterrupt(ColPorts[j], ColPins[j]);
            j = 3;
        }
        j++;
    }
    int k;
    for (k = 2; k<4; k++){
        if (GPIO_getInterruptStatus(reedPorts[k], reedPins[k])){
            reeds[k] = true;
            GPIO_clearInterrupt(reedPorts[k], reedPins[k]);
        }
    }

    __disable_interrupt();

    if (column != -1){
        j = 0;
        while (j<3){
            GPIO_disableInterrupt(ColPorts[j], ColPins[j]);
            j++;
        }
        j = 0;
        while (j<4){
            GPIO_setOutputLowOnPin(RowPorts[j], RowPins[j]);
            j++;
        }
        j = 0;
        while (j<4){
            GPIO_setOutputHighOnPin(RowPorts[j], RowPins[j]);
            if (GPIO_getInputPinValue(ColPorts[column], ColPins[column])){
                newKey = true;
                hexKey = hexaKeys[j][column];
            }
            GPIO_setOutputLowOnPin(RowPorts[j], RowPins[j]);
            j++;
        }
        j = 0;
        while (j<4){
            GPIO_setOutputHighOnPin(RowPorts[j], RowPins[j]);
            j++;
        }
        j = 0;
        while (j<3){
            GPIO_enableInterrupt(ColPorts[j], ColPins[j]);
            j++;
        }
    }

    bool leave = false;

    while (!leave){
        if (!GPIO_getInputPinValue(ColPorts[0], ColPins[0]) &&
            !GPIO_getInputPinValue(ColPorts[1], ColPins[1]) &&
            !GPIO_getInputPinValue(ColPorts[2], ColPins[2])){
            leave = true;
        }
    }




    __enable_interrupt();



};


// Port 2 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT1_VECTOR))) Port_2 (void)
#else
#error Compiler not supported!
#endif
{
    int k;

    for (k = 0; k<2; k++){
        if (GPIO_getInterruptStatus(reedPorts[k], reedPins[k])){
            reeds[k] = true;
            GPIO_clearInterrupt(reedPorts[k], reedPins[k]);
        }
    }

};


