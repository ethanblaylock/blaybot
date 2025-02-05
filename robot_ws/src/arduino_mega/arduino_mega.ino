#define LEFT_FORWARD_PIN 12
#define LEFT_REVERSE_PIN 11
#define RIGHT_FORWARD_PIN 7
#define RIGHT_REVERSE_PIN 6

void setup() {
    Serial.begin(115200);  // Match baud rate with Python script
    
    // Set PWM pins as OUTPUT
    pinMode(LEFT_FORWARD_PIN, OUTPUT);
    pinMode(LEFT_REVERSE_PIN, OUTPUT);
    pinMode(RIGHT_FORWARD_PIN, OUTPUT);
    pinMode(RIGHT_REVERSE_PIN, OUTPUT);
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
            analogWrite(LEFT_FORWARD_PIN, pwm_values[0]);
            analogWrite(LEFT_REVERSE_PIN, pwm_values[1]);
            analogWrite(RIGHT_FORWARD_PIN, pwm_values[2]);
            analogWrite(RIGHT_REVERSE_PIN, pwm_values[3]);

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
