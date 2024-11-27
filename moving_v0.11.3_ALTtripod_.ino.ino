#include <ESP32Servo.h>

// Пины сервоприводов для 6 ног
const int servoHipPins[6] = {12, 18, 15, 16, 17, 14};
const int servoKneePins[6] = {13, 25, 21, 22, 23, 27};

//const int servoHipPins[6] = {12, 18, 15, 16, 17, 14};
//const int servoKneePins[6] = {13, 25, 21, 22, 23, 27};

// Создаем объекты сервоприводов
Servo servoHips[6];
Servo servoKnees[6];

// Длины сегментов ноги (в см)
const float L1 = 2.5; // Длина бедра
const float L2 = 2.5; // Длина голени

// Параметры шага
const float stepHeight = 3.0;
const float stepLength = 5.0;
const int numSteps = 20;
const int moveDelay = 500;

// Глобальные переменные для диапазонов углов суставов
float hipMinAngle, hipMaxAngle;
float kneeMinAngle, kneeMaxAngle;

// Диапазоны сервоприводов
const float servoHipMinAngle = 0.0;
const float servoHipMaxAngle = 50.0;
const float servoKneeMinAngle = 110.0;
const float servoKneeMaxAngle = 170.0;

// Группы ног для трёхподного гейта
int group1[3] = {0, 3, 4};
int group2[3] = {1, 2, 5};

// Определение стороны для каждой ноги: true — левая, false — правая
const bool isLeftLeg[6] = {true, true, true, false, false, false};

// Переменная для управления отладкой
int debugLegIndex = -1; // -1: все ноги, 0-5: конкретная нога, -2: отладка выключена

void setup() {
  Serial.begin(115200);

  const int servoMinUs = 500;
  const int servoMaxUs = 2500;

  for (int i = 0; i < 6; i++) {
    servoHips[i].attach(servoHipPins[i], servoMinUs, servoMaxUs);
    servoKnees[i].attach(servoKneePins[i], servoMinUs, servoMaxUs);
  }

  calculateAngleRanges();
  Serial.println("Инициализация завершена. Команды: DEBUG:ALL, DEBUG:N (0-5), DEBUG:OFF");
}

void loop() {
  handleSerialInput();
  tripodGait();
}

// Управление трёхподным гейтом
void tripodGait() {
  moveLegs(group1, true);
  moveLegs(group2, false);
  moveLegs(group2, true);
  moveLegs(group1, false);
}

// Управление движением группы ног с учетом стороны
void moveLegs(const int legs[], bool isMovingForward) {
  for (int step = 0; step <= numSteps; step++) {
    float t = (float)step / numSteps;

    // Добавляем отладку текущего шага
    if (debugLegIndex == -1) {
      Serial.print("Обрабатывается шаг: ");
      Serial.println(step);
    }

    for (int i = 0; i < 3; i++) {
      int legIndex = legs[i];
      float X, Y;

      if (isLeftLeg[legIndex]) {
        if (isMovingForward) {
          X = -stepLength * 0.2 + stepLength * t;          // Левая нога движется вперёд
          Y = -L1 - L2 + stepHeight * sin(t * PI);         // Левая нога поднимается и опускается
        } else {
          X = -stepLength * 0.2 + stepLength * (1 - t);    // Левая нога плавно отводится назад
          Y = -L1 - L2;                                    // Левая нога остаётся на земле
        }
      } else { // Правая нога
        if (isMovingForward) {
          X = stepLength * 0.2 - stepLength * t;           // Правая нога движется назад (зеркально)
          Y = -L1 - L2 + stepHeight * sin(t * PI);         // Правая нога поднимается и опускается
        } else {
          X = stepLength * 0.2 - stepLength * (1 - t);     // Правая нога плавно отводится вперёд (зеркально)
          Y = -L1 - L2;                                    // Правая нога остаётся на земле
        }
      }

      // Добавляем отладку текущего шага конкретной ноги
      if (debugLegIndex == -1 || debugLegIndex == legIndex) {
        Serial.print("Нога ");
        Serial.print(legIndex);
        Serial.print(" на шаге ");
        Serial.print(step);
        Serial.print(" -> X = ");
        Serial.print(X);
        Serial.print(", Y = ");
        Serial.println(Y);
      }

      moveSingleLeg(legIndex, X, Y);
    }

    delay(moveDelay);
  }
}


// Управление одной ногой
void moveSingleLeg(int legIndex, float X, float Y) {
  float theta1, theta2;

  if (!inverseKinematics(X, Y, theta1, theta2)) {
    if (debugLegIndex == -1 || debugLegIndex == legIndex) {
      Serial.print("Недостижимая точка для ноги ");
      Serial.println(legIndex);
    }
    return;
  }

  float hipAngle = theta1 * 180.0 / PI;
  float kneeAngle = theta2 * 180.0 / PI;

  float servoHipAngle = mapFloat(hipAngle, hipMinAngle, hipMaxAngle, servoHipMinAngle, servoHipMaxAngle);
  float servoKneeAngle = mapFloat(kneeAngle, kneeMinAngle, kneeMaxAngle, servoKneeMinAngle, servoKneeMaxAngle);

  servoHipAngle = constrain(servoHipAngle, servoHipMinAngle, servoHipMaxAngle);
  servoKneeAngle = constrain(servoKneeAngle, servoKneeMinAngle, servoKneeMaxAngle);

  servoHips[legIndex].write(servoHipAngle);
  servoKnees[legIndex].write(servoKneeAngle);

  if (debugLegIndex == -1 || debugLegIndex == legIndex) {
    Serial.print("Нога ");
    Serial.print(legIndex);
    Serial.print(" -> Углы: Hip ");
    Serial.print(hipAngle);
    Serial.print("°, Knee ");
    Serial.print(kneeAngle);
    Serial.println("°");

    Serial.print("Нога ");
    Serial.print(legIndex);
    Serial.print(" -> Servo Angles: Hip ");
    Serial.print(servoHipAngle);
    Serial.print("°, Knee ");
    Serial.print(servoKneeAngle);
    Serial.println("°");
  }
}



// Обработка ввода через Serial
void handleSerialInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equals("DEBUG:ALL")) {
      debugLegIndex = -1;
      Serial.println("Отладка всех ног включена.");
    } else if (input.equals("DEBUG:OFF")) {
      debugLegIndex = -2;
      Serial.println("Отладка отключена.");
    } else if (input.startsWith("DEBUG:")) {
      int leg = input.substring(6).toInt();
      if (leg >= 0 && leg < 6) {
        debugLegIndex = leg;
        Serial.print("Отладка включена для ноги ");
        Serial.println(leg);
      } else {
        Serial.println("Неверный номер ноги. Используйте DEBUG:N (0-5).");
      }
    } else {
      Serial.println("Неизвестная команда. Используйте DEBUG:ALL, DEBUG:N (0-5), DEBUG:OFF.");
    }
  }
}

// Функции кинематики
bool inverseKinematics(float X, float Y, float &theta1, float &theta2) {
  float dx = X, dy = Y;
  float D = sqrt(dx * dx + dy * dy);

  float cosTheta2 = (dx * dx + dy * dy - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  theta2 = acos(constrain(cosTheta2, -1.0, 1.0));

  float k1 = L1 + L2 * cos(theta2);
  float k2 = L2 * sin(theta2);

  // Управление влиянием X и Y на Theta1
  float weight = 0.5; // Настраиваемый коэффициент
  float horizontalComponent = atan2(dy, dx);        // Горизонтальная составляющая
  float verticalComponent = atan2(k2, k1) * weight; // Вертикальная составляющая (с ослаблением)

  theta1 = horizontalComponent - verticalComponent;

  if (debugLegIndex == -1) {
    Serial.print("Кинематика: X = ");
    Serial.print(X);
    Serial.print(", Y = ");
    Serial.print(Y);
    Serial.print(" -> Theta1 = ");
    Serial.print(theta1 * 180.0 / PI);
    Serial.print("°, Theta2 = ");
    Serial.println(theta2 * 180.0 / PI);
  }

  return true;
}


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  float result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  Serial.print("mapFloat: x = ");
  Serial.print(x);
  Serial.print(", in_min = ");
  Serial.print(in_min);
  Serial.print(", in_max = ");
  Serial.print(in_max);
  Serial.print(", out_min = ");
  Serial.print(out_min);
  Serial.print(", out_max = ");
  Serial.print(out_max);
  Serial.print(" -> result = ");
  Serial.println(result);
  return result;
}

// Диапазоны углов
void calculateAngleRanges() {
  hipMinAngle = 180.0;
  hipMaxAngle = -180.0;
  kneeMinAngle = 180.0;
  kneeMaxAngle = -180.0;

  float X0 = -stepLength * 0.2;
  float Y0 = -L1 - L2;

  for (int i = 0; i <= numSteps; i++) {
    float t = (float)i / numSteps;
    float X_up = X0 + stepLength * t;
    float Y_up = Y0 + stepHeight * sin(t * PI);

    updateAngleRanges(X_up, Y_up);

    float X_down = X0 + stepLength * (1 - t);
    float Y_down = Y0;

    updateAngleRanges(X_down, Y_down);
  }

  Serial.print("Диапазон углов бедра: ");
  Serial.print(hipMinAngle);
  Serial.print("° до ");
  Serial.print(hipMaxAngle);
  Serial.println("°");

  Serial.print("Диапазон углов колена: ");
  Serial.print(kneeMinAngle);
  Serial.print("° до ");
  Serial.print(kneeMaxAngle);
  Serial.println("°");
}


void updateAngleRanges(float X, float Y) {
  float theta1, theta2;
  if (inverseKinematics(X, Y, theta1, theta2)) {
    float hipAngle = theta1 * 180.0 / PI;
    float kneeAngle = theta2 * 180.0 / PI;

    if (hipAngle < hipMinAngle) hipMinAngle = hipAngle;
    if (hipAngle > hipMaxAngle) hipMaxAngle = hipAngle;
    if (kneeAngle < kneeMinAngle) kneeMinAngle = kneeAngle;
    if (kneeAngle > kneeMaxAngle) kneeMaxAngle = kneeAngle;
  }
}
