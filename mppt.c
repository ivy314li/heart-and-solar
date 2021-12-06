float voltage = 0;
float current = 0;
float power = 0;
float max_voltage = 0, max_power = 0;
float pwm = 0;
float delta = 0;
float sensor_value1 = 0, sensor_value2 = 0;
float PWM_MIN;
float PWM_MAX;

void setup() {
    pinMode(6, OUTPUT)
}

void loop() {
    sensor_value1 = analogRead(A0);
    sensor_value2 = analogRead(A1);
    voltage = (sensor_value1*3.3/4096.0);
    current = (sensor_value2 * 3.3 / 4096.0);
    power = voltage * current;
    if (power > max_power) {
        if (voltage > max_voltage) {
            pwm = pwm - delta;
        } else {
            pwm = pwm + delta;
        }
    } else {
        if (voltage > max_voltage) {
            pwm = pwm + delta;
        } else {
            pwm = pwm - delta;
        }
    }
    if (pwm < PWM_MIN){
        pwm = PWM_MIN;
    }
    if (pwm > PWM_MAX) {
        pwm = PWM_MAX;
    }
    analogWrite(6, pwm);
}