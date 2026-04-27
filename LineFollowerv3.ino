/*
 * ============================================================
 *  Робот по линии  ESP32-S3 + TB6612FNG + QTR-8RC
 *  Библиотека: QTRSensors 4.0.0
 * ============================================================
 *
 *  РЕЖИМЫ:
 *    1) Калибровка  — при старте (~3 с, робот сам покачивается)
 *    2) Езда        — PID по отклонению от центра линии
 *
 *  ШИМ на ESP32-S3: используем LEDC (частота 20 кГц, 8 бит)
 * ============================================================
 */

#include <Arduino.h>
#include <QTRSensors.h>

// ─────────────────────────────────────────────
//  ПИНЫ МОТОРОВ  (TB6612FNG)
// ─────────────────────────────────────────────
#define STBY_PIN 12

#define PWMA_PIN 13   // Мотор A (левый)
#define AIN1_PIN 14
#define AIN2_PIN 15

#define PWMB_PIN 16   // Мотор B (правый)
#define BIN1_PIN 18
#define BIN2_PIN 17

// ─────────────────────────────────────────────
//  КНОПКА BOOT  (GPIO 0, активный LOW)
// ─────────────────────────────────────────────
#define BOOT_BTN_PIN 0

// ─────────────────────────────────────────────
//  ПИНЫ ДАТЧИКОВ  (QTR-8RC)
// ─────────────────────────────────────────────
const uint8_t SENSOR_COUNT = 8;
const uint8_t SensorPins[SENSOR_COUNT] = {41, 5, 6, 10, 9, 36, 35, 8};

// ─────────────────────────────────────────────
//  LEDC  (PWM на ESP32-S3)
// ─────────────────────────────────────────────
#define LEDC_FREQ      20000   // 20 кГц — тихий ШИМ
#define LEDC_BITS      8       // 0-255

// ─────────────────────────────────────────────
//  ПАРАМЕТРЫ ДВИЖЕНИЯ
// ─────────────────────────────────────────────
#define BASE_SPEED     190     // базовая скорость (0-255)
#define MAX_SPEED      255     // максимальная скорость
#define MIN_SPEED      0       // минимальная скорость

// ─────────────────────────────────────────────
//  PID-КОЭФФИЦИЕНТЫ  (настраивайте под свой робот)
// ─────────────────────────────────────────────
float Kp = 0.07f;
float Ki = 0.0002f;
float Kd = 1.2f;

// ─────────────────────────────────────────────
//  КАЛИБРОВКА
// ─────────────────────────────────────────────
#define CALIB_SAMPLES      400   // больше сэмплов — точнее для серого
#define CALIB_SWING_SPEED   90   // скорость покачивания при калибровке

// ─────────────────────────────────────────────
//  ПОРОГ ОБНАРУЖЕНИЯ ЛИНИИ
//  Для выцветшего/серого трека снижаем порог.
//  0-1000: чем ниже — тем чувствительнее к слабому контрасту.
//  Начните с 80, если слетает — попробуйте 50.
// ─────────────────────────────────────────────
#define LINE_THRESHOLD     80    // минимальное значение датчика над линией
#define MIN_CONTRAST       60    // минимальный разброс min/max среди датчиков
QTRSensors qtr;
uint16_t sensorValues[SENSOR_COUNT];

float  lastError   = 0;
float  integral    = 0;
bool   onLine      = true;
uint32_t lastLineTime = 0;
bool   motorsEnabled = true;   // флаг вкл/выкл моторов (кнопка BOOT)
uint32_t lastBtnTime = 0;      // антидребезг кнопки

// ─────────────────────────────────────────────
//  ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ  —  МОТОРЫ
// ─────────────────────────────────────────────

void motorsInit()
{
    // STBY
    pinMode(STBY_PIN, OUTPUT);
    digitalWrite(STBY_PIN, HIGH);   // выход из STANDBY

    // Направление мотора A
    pinMode(AIN1_PIN, OUTPUT);
    pinMode(AIN2_PIN, OUTPUT);

    // Направление мотора B
    pinMode(BIN1_PIN, OUTPUT);
    pinMode(BIN2_PIN, OUTPUT);

    // PWM — новый API (ESP32 Arduino Core 3.x)
    ledcAttach(PWMA_PIN, LEDC_FREQ, LEDC_BITS);
    ledcAttach(PWMB_PIN, LEDC_FREQ, LEDC_BITS);
}

// speed: -255 … +255  (+ вперёд, - назад)
void setMotorA(int speed)
{
    speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
    if (speed >= 0) {
        digitalWrite(AIN1_PIN, HIGH);
        digitalWrite(AIN2_PIN, LOW);
    } else {
        digitalWrite(AIN1_PIN, LOW);
        digitalWrite(AIN2_PIN, HIGH);
        speed = -speed;
    }
    ledcWrite(PWMA_PIN, speed);
}

void setMotorB(int speed)
{
    speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
    if (speed >= 0) {
        digitalWrite(BIN1_PIN, HIGH);
        digitalWrite(BIN2_PIN, LOW);
    } else {
        digitalWrite(BIN1_PIN, LOW);
        digitalWrite(BIN2_PIN, HIGH);
        speed = -speed;
    }
    ledcWrite(PWMB_PIN, speed);
}

void stopMotors()
{
    setMotorA(0);
    setMotorB(0);
}

// Поворот на месте влево (+) или вправо (-)
void spinInPlace(int speed)
{
    setMotorA(-speed);
    setMotorB( speed);
}

// ─────────────────────────────────────────────
//  КАЛИБРОВКА
// ─────────────────────────────────────────────

void calibrateSensors()
{
    Serial.println("=== CALIBRATION START ===");

    // Покачиваемся влево-вправо во время калибровки
    for (int i = 0; i < CALIB_SAMPLES; i++)
    {
        // Первая четверть — влево
        if (i < CALIB_SAMPLES / 4)
            spinInPlace(-CALIB_SWING_SPEED);
        // Вторая половина — вправо
        else if (i < CALIB_SAMPLES * 3 / 4)
            spinInPlace( CALIB_SWING_SPEED);
        // Последняя четверть — обратно влево
        else
            spinInPlace(-CALIB_SWING_SPEED);

        qtr.calibrate();
        delay(10);
    }

    stopMotors();
    delay(200);

    Serial.println("=== CALIBRATION DONE ===");

    // Вывод диапазонов калибровки
    Serial.print("Min: ");
    for (int i = 0; i < SENSOR_COUNT; i++) {
        Serial.print(qtr.calibrationOn.minimum[i]);
        Serial.print("  ");
    }
    Serial.println();

    Serial.print("Max: ");
    for (int i = 0; i < SENSOR_COUNT; i++) {
        Serial.print(qtr.calibrationOn.maximum[i]);
        Serial.print("  ");
    }
    Serial.println();
}

// ─────────────────────────────────────────────
//  ВОССТАНОВЛЕНИЕ ЛИНИИ (потеряли линию)
// ─────────────────────────────────────────────

void recoverLine()
{
    // Крутимся в сторону последней ошибки
    int dir = (lastError > 0) ? 1 : -1;
    spinInPlace(dir * 100);
    delay(20);
}

// ─────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────

void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println("Line Follower — ESP32-S3");

    // Кнопка BOOT
    pinMode(BOOT_BTN_PIN, INPUT_PULLUP);

    // Инициализация датчиков
    qtr.setTypeRC();
    qtr.setSensorPins(SensorPins, SENSOR_COUNT);
    qtr.setEmitterPin(42);    // пин управления ИК-эмиттерами QTR-8RC

    motorsInit();

    delay(1000);   // пауза перед калибровкой

    calibrateSensors();

    // ── Ждём нажатия кнопки BOOT ──────────────
    Serial.println("=== Нажми BOOT для старта ===");
    while (digitalRead(BOOT_BTN_PIN) == HIGH) {
        delay(10);   // ждём нажатия (LOW = нажата)
    }
    // Ждём отпускания кнопки
    while (digitalRead(BOOT_BTN_PIN) == LOW) {
        delay(10);
    }
    delay(200);   // антидребезг
    // ──────────────────────────────────────────

    Serial.println("=== RUNNING ===");
}

// ─────────────────────────────────────────────
//  LOOP  —  PID
// ─────────────────────────────────────────────

void loop()
{
    // ── Кнопка BOOT — вкл/выкл моторов ──────
    if (digitalRead(BOOT_BTN_PIN) == LOW && millis() - lastBtnTime > 300) {
        lastBtnTime = millis();
        motorsEnabled = !motorsEnabled;
        if (!motorsEnabled) {
            stopMotors();
            Serial.println("=== МОТОРЫ ВЫКЛЮЧЕНЫ ===");
        } else {
            integral  = 0;   // сброс интегратора
            lastError = 0;
            Serial.println("=== МОТОРЫ ВКЛЮЧЕНЫ ===");
        }
        // Ждём отпускания
        while (digitalRead(BOOT_BTN_PIN) == LOW) delay(10);
    }

    if (!motorsEnabled) {
        delay(50);
        return;
    }
    // ─────────────────────────────────────────

    // Читаем позицию (0 = крайний левый, 7000 = крайний правый)
    uint16_t position = qtr.readLineBlack(sensorValues);

    // ── Проверка наличия линии ────────────────
    // Для серого/выцветшего трека смотрим не абсолютное значение,
    // а разброс между минимальным и максимальным датчиком.
    // Даже слабый контраст серого на светлом фоне даст разброс.
    uint16_t minVal = 1000, maxVal = 0;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (sensorValues[i] < minVal) minVal = sensorValues[i];
        if (sensorValues[i] > maxVal) maxVal = sensorValues[i];
    }
    uint16_t contrast = maxVal - minVal;

    if (maxVal < LINE_THRESHOLD || contrast < MIN_CONTRAST) {
        // Линия полностью потеряна
        if (onLine) {
            onLine = false;
            lastLineTime = millis();
        }
        // Пробуем восстановить не дольше 1.5 сек
        if (millis() - lastLineTime < 1500) {
            recoverLine();
        } else {
            stopMotors();   // сдаёмся
        }
        return;
    }

    onLine = true;

    // Центр линии = 3500 для 8 датчиков
    float error = (float)position - 3500.0f;

    // ── PID ──────────────────────────────────
    integral += error;
    // Anti-windup
    integral = constrain(integral, -10000.0f, 10000.0f);

    float derivative = error - lastError;
    float correction  = Kp * error + Ki * integral + Kd * derivative;

    lastError = error;
    // ─────────────────────────────────────────

    int leftSpeed  = (int)(BASE_SPEED + correction);
    int rightSpeed = (int)(BASE_SPEED - correction);

    leftSpeed  = constrain(leftSpeed,  MIN_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

    setMotorA(rightSpeed);
    setMotorB(leftSpeed);

    // Отладка (раскомментируйте при настройке)
    /*
    Serial.print("pos="); Serial.print(position);
    Serial.print("  contrast="); Serial.print(contrast);
    Serial.print("  err="); Serial.print(error);
    Serial.print("  corr="); Serial.print(correction);
    Serial.print("  L="); Serial.print(leftSpeed);
    Serial.print("  R="); Serial.println(rightSpeed);
    */
}
