#define LEFT_FORWARD_PIN 12  // Timer 1 OC1B
#define LEFT_REVERSE_PIN 11  // Timer 1 OC1A
#define RIGHT_FORWARD_PIN 7  // Timer 4 OC4B
#define RIGHT_REVERSE_PIN 6  // Timer 4 OC4A
#define JOINT1_PIN 5  // Timer 3 OC3A
#define JOINT2_PIN 3  // Timer 3 OC3C
#define JOINT3_PIN 2  // Timer 3 OC3B
#define JOINT4_PIN 44  // Timer 5 OC5C
#define JOINT5_PIN 45  // Timer 5 OC5B
#define JOINT6_PIN 46  // Timer 5 OC5A
#define MAX_PWM 255
#define MIN_PWM 0


void processCommand(String input);
void handleDriveCommand(String args);
void handleArmCommand(String args);

// Command mapping structure
struct Command {
    const char *name;
    void (*handler)(String args);
};

// Add new commands easily in this list
Command commands[] = {
    {"DRIVE", handleDriveCommand},
    // Future commands can be added here
};

void setup() {
    Serial.begin(115200);  // Match baud rate with Python script
    
    // Set PWM pins as OUTPUT
    pinMode(LEFT_FORWARD_PIN, OUTPUT);
    pinMode(LEFT_REVERSE_PIN, OUTPUT);
    pinMode(RIGHT_FORWARD_PIN, OUTPUT);
    pinMode(RIGHT_REVERSE_PIN, OUTPUT);

    // Configure 1500 Hz PWM for Timer 1 (Pins 11 & 12)
    setupTimer1();
    
    // Configure 1500 Hz PWM for Timer 4 (Pins 6 & 7)
    setupTimer4();

    // Configure 50 Hz PWM for Timer 3 (Pins 2, 3, 5)
    setupTimer3();
    // Configure 50 Hz PWM for Timer 5 (Pins 44, 45, 46)
    setupTimer5();

    Serial.println("PWM Frequency set to 1500 Hz on all motor pins");
}

void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        processCommand(input);
    }
}

// Parses the command and calls the appropriate function
void processCommand(String input) {
    int start = 0;
    while (start < input.length()) {
        int end = input.indexOf(';', start);
        if (end == -1) end = input.length(); // Handle last command

        String singleCommand = input.substring(start, end);
        singleCommand.trim(); // Remove whitespace

        if (singleCommand.length() > 0) {
            int spaceIndex = singleCommand.indexOf(' ');
            String command = spaceIndex == -1 ? singleCommand : singleCommand.substring(0, spaceIndex);
            String args = spaceIndex == -1 ? "" : singleCommand.substring(spaceIndex + 1);

            bool found = false;
            for (Command cmd : commands) {
                if (command.equalsIgnoreCase(cmd.name)) {
                    cmd.handler(args);
                    found = true;
                    break;
                }
            }

            if (!found) {
                Serial.println("Error: Unknown command");
            }
        }
        start = end + 1; // Move to next command
    }
}


// Handles Drive command input
void handleDriveCommand(String args) {
    int pwm_values[4] = {0};  
    int index = 0;

    // Tokenize using commas
    char *token = strtok((char *)args.c_str(), ",");
    while (token != NULL && index < 4) {
        pwm_values[index] = constrain(atoi(token), MIN_PWM, MAX_PWM);
        token = strtok(NULL, ",");
        index++;
    }

    if (index == 4) {
        setDrivePWM(LEFT_FORWARD_PIN, pwm_values[0]);
        setDrivePWM(LEFT_REVERSE_PIN, pwm_values[1]);
        setDrivePWM(RIGHT_FORWARD_PIN, pwm_values[2]);
        setDrivePWM(RIGHT_REVERSE_PIN, pwm_values[3]);

        Serial.print("Drive PWM Set: ");
        Serial.print(pwm_values[0]); Serial.print(",");
        Serial.print(pwm_values[1]); Serial.print(",");
        Serial.print(pwm_values[2]); Serial.print(",");
        Serial.println(pwm_values[3]); Serial.println();
    } else {
        Serial.println("Error: Invalid Drive command format");
    }
}


void handleArmCommand(String args) {
    int pwm_values[6] = {0};  
    int index = 0;

    // Tokenize using commas
    char *token = strtok((char *)args.c_str(), ",");
    while (token != NULL && index < 6) {
        pwm_values[index] = constrain(atoi(token), MIN_PWM, MAX_PWM);
        token = strtok(NULL, ",");
        index++;
    }

    if (index == 6) {
        setArmPWM(JOINT1_PIN, pwm_values[0]);
        setArmPWM(JOINT2_PIN, pwm_values[1]);
        setArmPWM(JOINT3_PIN, pwm_values[2]);
        setArmPWM(JOINT4_PIN, pwm_values[3]);
        setArmPWM(JOINT5_PIN, pwm_values[4]);
        setArmPWM(JOINT6_PIN, pwm_values[5]);

        Serial.print("Arm PWM Set: ");
        Serial.print(pwm_values[0]); Serial.print(",");
        Serial.print(pwm_values[1]); Serial.print(",");
        Serial.print(pwm_values[2]); Serial.print(",");
        Serial.print(pwm_values[3]); Serial.print(",");
        Serial.print(pwm_values[4]); Serial.print(",");
        Serial.print(pwm_values[5]); Serial.println();
    } else {
        Serial.println("Error: Invalid Arm PWM command format");
    }
}

// Function to precisely set 1500 Hz PWM for Timer 1 (Pins 11 & 12)
void setupTimer1() {
    pinMode(11, OUTPUT);
    pinMode(12, OUTPUT);
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1A = (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1);  // Fast PWM, Clear OC1A/OC1B on Compare Match
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);  // No prescaler, Fast PWM mode 14

    ICR1 = 10666;  // (16,000,000 / (1500 * 1)) - 1 = 10666

    OCR1A = 0;  // Start with 0% duty cycle
    OCR1B = 0;  // Start with 0% duty cycle
}

// Function to precisely set 1500 Hz PWM for Timer 4 (Pins 6 & 7)
void setupTimer4() {
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
    TCCR4A = 0;
    TCCR4B = 0;
    TCCR4A = (1 << WGM41) | (1 << COM4A1) | (1 << COM4B1);  // Fast PWM, Clear OC4A/OC4B on Compare Match
    TCCR4B = (1 << WGM43) | (1 << WGM42) | (1 << CS40);  // No prescaler, Fast PWM mode 14

    ICR4 = 10666;  // (16,000,000 / (1500 * 1)) - 1 = 10666

    OCR4A = 0;  // Start with 0% duty cycle
    OCR4B = 0;  // Start with 0% duty cycle
}

//Function to precisely set 50 Hz PWM for Timer 3 (Pins 2, 3, 5)
void setupTimer3() {
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(5, OUTPUT);
    TCCR3A = 0;
    TCCR3B = 0;
    TCCR3A = (1 << WGM31) | (1 << COM3A1) | (1 << COM3B1) | (1 << COM3C1);  // Fast PWM, Clear OC3A/OC3B/OC3C on Compare Match
    TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS31);  // 8 prescaler, Fast PWM mode 14

    ICR3 = 39999;  // (16,000,000 / (50 * 8)) - 1 = 39999

    OCR3A = 0;  // Start with 0% duty cycle
    OCR3B = 0;  // Start with 0% duty cycle
    OCR3C = 0;  // Start with 0% duty cycle
}

//Function to precisely set 50 Hz PWM for Timer 5 (Pins 44, 45, 46)
void setupTimer5() {
    pinMode(44, OUTPUT);
    pinMode(45, OUTPUT);
    pinMode(46, OUTPUT);
    TCCR5A = 0;
    TCCR5B = 0;
    TCCR5A = (1 << WGM51) | (1 << COM5A1) | (1 << COM5B1) | (1 << COM5C1);  // Fast PWM, Clear OC5A/OC5B/OC5C on Compare Match
    TCCR5B = (1 << WGM53) | (1 << WGM52) | (1 << CS51);  // 8 prescaler, Fast PWM mode 14

    ICR5 = 39999;  // (16,000,000 / (50 * 8)) - 1 = 39999

    OCR5A = 0;  // Start with 0% duty cycle
    OCR5B = 0;  // Start with 0% duty cycle
    OCR5C = 0;  // Start with 0% duty cycle
}

// Function to set Drive PWM duty cycle
void setDrivePWM(int pin, float duty) {
    uint16_t pwm_value = map(duty, 0, 255, 0, 10666);  // Scale 0-255 to ICR1/ICR4 range

    if (pin == 11) OCR1A = pwm_value;
    else if (pin == 12) OCR1B = pwm_value;
    else if (pin == 6) OCR4A = pwm_value;
    else if (pin == 7) OCR4B = pwm_value;
}

// Function to set Arm PWM duty cycle
void setArmPWM(int pin, float duty) {
    uint16_t pwm_value = map(duty, 0, 255, 0, 39999);  // Scale 0-255 to ICR3/ICR5 range

    if (pin == 2) OCR3B = pwm_value;
    else if (pin == 3) OCR3C = pwm_value;
    else if (pin == 5) OCR3A = pwm_value;
    else if (pin == 44) OCR5C = pwm_value;
    else if (pin == 45) OCR5B = pwm_value;
    else if (pin == 46) OCR5A = pwm_value;
}
