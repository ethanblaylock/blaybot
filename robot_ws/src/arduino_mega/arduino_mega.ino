#define LEFT_FORWARD_PIN 12  // Timer 1
#define LEFT_REVERSE_PIN 11  // Timer 1
#define RIGHT_FORWARD_PIN 7  // Timer 4
#define RIGHT_REVERSE_PIN 6  // Timer 4

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

    Serial.println("PWM Frequency set to 1500 Hz on all motor pins");
}

void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');  // Read full command
        input.trim();  // Remove whitespace

        int pwm_values[4];  // Array to store parsed values
        int index = 0;

        // Split input based on commas
        char *token = strtok((char *)input.c_str(), ",");
        while (token != NULL && index < 4) {
            pwm_values[index] = constrain(atoi(token), 0, 255);  // Convert to int and constrain to 0-255
            token = strtok(NULL, ",");
            index++;
        }

        // Ensure 4 values were received
        if (index == 4) {
            setPWM(LEFT_FORWARD_PIN, pwm_values[0]);
            setPWM(LEFT_REVERSE_PIN, pwm_values[1]);
            setPWM(RIGHT_FORWARD_PIN, pwm_values[2]);
            setPWM(RIGHT_REVERSE_PIN, pwm_values[3]);

            // Debug output
            Serial.print("PWM Set: ");
            Serial.print(pwm_values[0]); Serial.print(",");
            Serial.print(pwm_values[1]); Serial.print(",");
            Serial.print(pwm_values[2]); Serial.print(",");
            Serial.println(pwm_values[3]);
        } else {
            Serial.println("Error: Invalid command format");
        }
    }
}

// Function to precisely set 1500 Hz PWM for Timer 1 (Pins 11 & 12)
void setupTimer1() {
    pinMode(11, OUTPUT);
    pinMode(12, OUTPUT);

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

    TCCR4A = (1 << WGM41) | (1 << COM4A1) | (1 << COM4B1);  // Fast PWM, Clear OC4A/OC4B on Compare Match
    TCCR4B = (1 << WGM43) | (1 << WGM42) | (1 << CS40);  // No prescaler, Fast PWM mode 14

    ICR4 = 10666;  // (16,000,000 / (1500 * 1)) - 1 = 10666

    OCR4A = 0;  // Start with 0% duty cycle
    OCR4B = 0;  // Start with 0% duty cycle
}

// Function to set PWM duty cycle
void setPWM(int pin, int duty) {
    uint16_t pwm_value = map(duty, 0, 255, 0, 10666);  // Scale 0-255 to ICR1/ICR4 range

    if (pin == 11) OCR1B = pwm_value;
    else if (pin == 12) OCR1A = pwm_value;
    else if (pin == 6) OCR4B = pwm_value;
    else if (pin == 7) OCR4A = pwm_value;
}
