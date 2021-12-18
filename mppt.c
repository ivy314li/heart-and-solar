float voltage = 0;
float current = 0;
float power = 0;
float power_max = 0;
float pwm = 0;
float pwm_max_power;
float delta = 0;
float sensor_value1 = 0, sensor_value2 = 0;
const float PWM_MIN = 20;
const float PWM_MAX = 150;
const unsigned long SAMPLE_TIME = 20;

void setup() {
    pinMode(6, OUTPUT);
}

void loop() {
    int count = 0;
    for (pwm = PWM_MIN; pwm < PWM_MAX; pwm += delta) {
        analogWrite(6, pwm);
        sensor_value1 = analogRead(0);
        sensor_value2 = analogRead(1);
        voltage = (sensor_value1*3.3/4096.0);
        current = (sensor_value2 * 3.3 / 4096.0);
        power = voltage * current;
        if (power > power_max) {
            count = 0;
            power_max = power;
            pwm_max_power = pwm;
        } else {
            break;
        }
    }
    analogWrite(6, pwm_max_power);
    delay(SAMPLE_TIME);
}