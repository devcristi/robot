#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// ==================== Definire Pin-uri ====================
// Motoare DC
#define IN3 25
#define IN4 32
#define ENB 33
#define IN1 4
#define IN2 2
#define ENA 15

// Senzor Ultrasonic
#define echoPIN 13
#define trigPIN 12

// Servomotoare
#define SERVO_PIN_360 5   // Pin pentru servo-ul de 360°
#define SERVO_PIN_2 27    // Pin pentru servo-ul standard

// ==================== Obiecte și Variabile ====================
Servo servo360;  // Servo de 360°
Servo servo2;    // Servo standard

// Valoare de oprire pentru servo-ul de 360°
const int servo360Stop = 90;
const int servo360Clockwise = 180;
const int servo360CounterClockwise = 0;

// Credențiale Wi-Fi
const char* ssid = "ESP32_Test_AP";
const char* password = "123456789";

// Creare obiect WebServer pe portul 80
WebServer server(80);

// Variabile pentru senzorul ultrasonic
long duration;
long distance;  // Restaurată

// Flag pentru modul autonom (obstacole)
bool autonomousMode = false;

// Definirea stărilor pentru FSM (Autonom)
enum RobotState {
  LOOK_RIGHT,
  TURN_RIGHT,
  LOOK_FORWARD,
  MOVE_FORWARD,
  TURN_LEFT,
  MOVE_BACKWARD,
  STOP_BACKWARD
};

unsigned long previousMillis = 0;    
const long interval = 1000;          
unsigned long lastMovementTime = 0;  
const long movementDelay = 500;      

bool stopRequested = true;

// ==================== Variabile PID ====================
bool pidMode = false;          // Flag pentru activare/dezactivare PID
float setpoint = 30.0;         // Distanța dorită față de obstacol (în cm)
float Kp = 1.6;                
float Ki = 0.05;
float Kd = 0.1;

// Variabile pentru calcul PID
float error = 0.0;
float lastError = 0.0;
float integral = 0.0;
float derivative = 0.0;
float outputPID = 0.0;
unsigned long lastPIDTime = 0;   // Timp de referință pentru calculul derivat/integral

// Variabilă globală pentru ultima valoare PID
float lastPIDOutput = 0.0;

// Factor de corecție dacă motorul din dreapta e mai puternic
float factorRight = 0.7; // Ajustați la nevoie

// ==================== Prototipuri de funcții ====================
void handleRoot();
void measureDistance();
void updateDistance();
void toggleAutonomousMode();
void stopAutonomousMode();
void obstacleAvoidance();
void checkBackward();

// Control PID
void togglePIDMode();
void setSetpoint();
void updatePID();
void sendPIDDataJSON();

// Funcții de control motoare
void moveForwardLeft();
void moveBackwardLeft();
void stopLeftMotor();
void moveForwardRight();
void moveBackwardRight();
void stopRightMotor();
void moveForwardBoth();
void moveBackwardBoth();
void stopBothMotors();

// Funcții servo 360
void rotateServo360Clockwise();
void rotateServo360CounterClockwise();
void stopServo360();
void rotateServo360InSteps();
void setServo360To0();
void setServo360To90();
void setServo360To135();
void setServo360To180();

// Funcții servo standard
void rotateServo2Clockwise();
void rotateServo2CounterClockwise();
void stopServo2();
void setServo2To0();
void setServo2To90();
void setServo2To135();
void setServo2To180();

// Funcții pentru întoarcere
void turnLeft();
void turnRight();

// ==================== Funcții Handler pentru WebServer ====================
void handleRoot() {
  // Include un plot simplu pentru PID
  String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
        <title>Control Roboți</title>
        <style>
            body { font-family: Arial, sans-serif; text-align: center; margin-top: 20px; }
            button { margin: 5px; padding: 10px; font-size: 15px; cursor: pointer; }
            #pidContainer { margin-top: 30px; }
            #chartWrapper { width: 80%; margin: auto; }
        </style>
    </head>
    <body>
        <h1>Panou de Control</h1>

        <!-- Control Motoare Dreapta -->
        <h2>Motor Dreapta</h2>
        <button onclick="sendCommand('moveForwardRight')">Move Forward</button>
        <button onclick="sendCommand('moveBackwardRight')">Move Backward</button>
        <button onclick="sendCommand('stopRightMotor')">Stop</button>

        <!-- Control Motoare Stânga -->
        <h2>Motor Stanga</h2>
        <button onclick="sendCommand('moveForwardLeft')">Move Forward</button>
        <button onclick="sendCommand('moveBackwardLeft')">Move Backward</button>
        <button onclick="sendCommand('stopLeftMotor')">Stop</button>

        <!-- Control Ambele Motoare -->
        <h2>Ambele Motoare</h2>
        <button onclick="sendCommand('moveForwardBoth')">Move Forward</button>
        <button onclick="sendCommand('moveBackwardBoth')">Move Backward</button>
        <button onclick="sendCommand('stopBothMotors')">Stop</button>

        <!-- Control Servo 360° -->
        <h2>Servo 360°</h2>
        <button onclick="sendCommand('rotateServo360Clockwise')">Rotate CW</button>
        <button onclick="sendCommand('rotateServo360CounterClockwise')">Rotate CCW</button>
        <button onclick="sendCommand('stopServo360')">Stop</button>
        <button onclick="sendCommand('rotateServo360InSteps')">Rotate in Steps</button>
        <br>
        <button onclick="sendCommand('setServo360To0')">Rotate 0</button>
        <button onclick="sendCommand('setServo360To90')">Rotate 90</button>
        <button onclick="sendCommand('setServo360To135')">Rotate 135</button>
        <button onclick="sendCommand('setServo360To180')">Rotate 180</button>

        <!-- Control Servo Standard -->
        <h2>Servo Standard</h2>
        <button onclick="sendCommand('rotateServo2Clockwise')">Rotate CW</button>
        <button onclick="sendCommand('rotateServo2CounterClockwise')">Rotate CCW</button>
        <button onclick="sendCommand('stopServo2')">Stop</button>
        <br>
        <button onclick="sendCommand('setServo2To0')">Servo2 0°</button>
        <button onclick="sendCommand('setServo2To90')">Servo2 90°</button>
        <button onclick="sendCommand('setServo2To135')">Servo2 135</button>
        <button onclick="sendCommand('setServo2To180')">Servo2 180°</button>

        <!-- Mod Autonom -->
        <h2>Mod Autonom (Evitare obstacole)</h2>
        <button onclick="sendCommand('toggleAutonomousMode')">Toggle Autonomous Mode</button>
        <button onclick="sendCommand('stopAutonomousMode')">Stop Autonomous Mode</button>

        <!-- PID Control -->
        <div id="pidContainer">
            <h2>Mod PID (Menținere distanță fixă)</h2>
            <button onclick="togglePIDMode()">Toggle PID Mode</button>
            <p>Setpoint (cm):
                <input type="number" id="setpointInput" name="setpointInput" value="20" min="5" max="200">
                <button onclick="applySetpoint()">Aplică</button>
            </p>
            <div id="chartWrapper">
                <canvas id="pidChart" width="600" height="300"></canvas>
            </div>
            <h3 id="distanceLabel">Distanță: -- cm</h3>
        </div>

        <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
        <script>
            let distanceLabel = document.getElementById('distanceLabel');
            let pidChart;
            let timeData = [];
            let outputData = [];
            let pidMode = false; // Urmărește starea modului PID

            // Inițializează graficul PID
            function initChart() {
                const ctx = document.getElementById('pidChart').getContext('2d');
                pidChart = new Chart(ctx, {
                    type: 'line',
                    data: {
                        labels: timeData,
                        datasets: [
                            {
                                label: 'Output PID',
                                data: outputData,
                                borderColor: 'red',
                                fill: false,
                                tension: 0
                            }
                        ]
                    },
                    options: {
                        responsive: true,
                        animation: false,
                        scales: {
                            x: {
                                title: { display: true, text: 'Citiri' }
                            },
                            y: {
                                title: { display: true, text: 'Valoare' }
                            }
                        }
                    }
                });
            }

            // Actualizează datele graficului
            function updateChart(newOutput) {
                if (newOutput !== undefined && !isNaN(newOutput)) {
                    timeData.push(timeData.length);
                    outputData.push(newOutput);
                    if (timeData.length > 50) {
                        timeData.shift();
                        outputData.shift();
                    }
                    pidChart.update();
                } else {
                    console.log('Valoare PID invalidă:', newOutput);
                }
            }

            // Funcție pentru trimiterea comenzilor către server
            function sendCommand(command) {
                fetch('/' + command)
                .then(response => {
                    if (response.ok) {
                        if (command === 'measureDistance') {
                            return response.text().then(data => {
                                if (!isNaN(data)) {
                                    distanceLabel.innerText = "Distanță: " + data + " cm";
                                } else {
                                    distanceLabel.innerText = data;
                                }
                            });
                        } else {
                            return response.text().then(data => {
                                console.log(data);
                                if (command === 'togglePIDMode') {
                                    pidMode = !pidMode;
                                }
                            });
                        }
                    } else {
                        throw new Error('Reacție de rețea nereușită');
                    }
                })
                .catch(err => console.log('Eroare Fetch:', err));
            }

            // Preia datele PID în format JSON
            function fetchPIDData() {
                fetch('/pidData')
                .then(response => response.json())
                .then(jsonData => {
                    const outPID = jsonData.outputPID;
                    updateChart(outPID);
                })
                .catch(err => console.log('Eroare Preluare Date PID:', err));
            }

            // Setează noul setpoint
            function applySetpoint() {
                let val = document.getElementById('setpointInput').value;
                if(val) {
                    fetch('/setSetpoint?value=' + val)
                    .then(response => response.text())
                    .then(data => {
                        console.log(data);
                    })
                    .catch(err => console.log(err));
                }
            }

            // Toggle PID mode și actualizează starea
            function togglePIDMode() {
                sendCommand('togglePIDMode');
            }

            // Actualizează datele PID la fiecare 500ms
            setInterval(() => {
                // Măsoară distanța și actualizează afișajul
                fetch('/measureDistance')
                .then(response => response.text())
                .then(data => {
                    if(!isNaN(data)) {
                        distanceLabel.innerText = "Distanță: " + data + " cm";
                    } else {
                        distanceLabel.innerText = data;
                    }
                })
                .catch(err => console.log('Eroare Măsurare Distanță:', err));

                if(pidMode) {
                    fetchPIDData();
                }
            }, 500);

            window.onload = function() {
                initChart();
            }
        </script>
    </body>
    </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

// ==================== Măsurare Distanță ====================
void measureDistance() {
  digitalWrite(trigPIN, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPIN, LOW);

  // Măsurarea duratei impulsului returnat de senzor
  duration = pulseIn(echoPIN, HIGH);
  
  // Calcularea distanței în centimetri
  distance = duration / 58.2;  // cm

  server.send(200, "text/plain", String(distance));
}

// ==================== Mod Autonom (Evitare Obstacole) ====================
bool isObstacleTooClose(float distThreshold) {
  measureDistance();
  return (distance < distThreshold);
}

RobotState currentState = LOOK_FORWARD;

void toggleAutonomousMode() {
  autonomousMode = !autonomousMode;
  server.send(200, "text/plain", autonomousMode ? "Mod Autonom ACTIVAT" : "Mod Autonom DEZACTIVAT");
}

void stopAutonomousMode() {
  autonomousMode = false;
  server.send(200, "text/plain", "Mod Autonom Oprit");
}

void obstacleAvoidance() {
  switch (currentState) {
    case LOOK_RIGHT:
      rotateServo2Clockwise(); // Întoarcere servo la dreapta
      delay(500);
      measureDistance();
      if (distance > 40) {
        currentState = TURN_RIGHT;
      } else {
        setServo2To90();
        delay(500);
        currentState = LOOK_FORWARD;
      }
      break;

    case TURN_RIGHT:
      turnRight();
      delay(2000);
      currentState = LOOK_FORWARD;
      break;

    case LOOK_FORWARD:
      setServo2To90();
      delay(500);
      measureDistance();
      if (distance > 40) {
        currentState = MOVE_FORWARD;
      } else {
        currentState = LOOK_RIGHT;
      }
      break;

    case MOVE_FORWARD:
      moveForwardBoth();
      delay(2500);
      stopBothMotors();
      currentState = LOOK_FORWARD;
      break;

    case TURN_LEFT:
      rotateServo2CounterClockwise();
      delay(500);
      measureDistance();
      if (distance > 40) {
        turnLeft();
        delay(2000);
        currentState = LOOK_FORWARD;
      } else {
        setServo2To90();
        delay(500);
        currentState = MOVE_BACKWARD;
      }
      break;

    case MOVE_BACKWARD:
      moveBackwardBoth();
      delay(1500);
      stopBothMotors();
      currentState = LOOK_RIGHT;
      break;

    case STOP_BACKWARD:
      stopBothMotors();
      break;

    default:
      currentState = STOP_BACKWARD;
      break;
  }
}

void checkBackward() {
  rotateServo2CounterClockwise();
  delay(5000);
  measureDistance();
  if (distance > 40) {
    turnRight();
  } else {
    // Orice altă logică dorită
  }
  stopServo2();
}

// ==================== PID Control (Menținere Distanță Fixă) ====================
void togglePIDMode() {
  pidMode = !pidMode;
  server.send(200, "text/plain", pidMode ? "PID Mode ACTIVAT" : "PID Mode DEZACTIVAT");
}

void setSetpoint() {
  if (server.hasArg("value")) {
    setpoint = server.arg("value").toFloat();
    server.send(200, "text/plain", "Setpoint actualizat la " + String(setpoint));
  } else {
    server.send(400, "text/plain", "Parametru 'value' lipsește");
  }
}

void updatePID() {
  // Măsurați distanța la fiecare apel, pentru a avea date proaspete
  measureDistance();

  unsigned long now = millis();
  float deltaT = (now - lastPIDTime) / 1000.0; // în secunde
  if (deltaT <= 0.0) {
    deltaT = 0.001;
  }
  lastPIDTime = now;

  error = distance - setpoint; // Eroare = distanță - setpoint

  const float deadband = 0.5; // zonă moartă (cm)
  if (abs(error) < deadband) {
    integral = 0.0;
    derivative = 0.0;
    outputPID = 0.0;
  } else {
    integral += (error * deltaT);
    derivative = (error - lastError) / deltaT;
    outputPID = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // Limităm valorile PID
    if (outputPID > 200) outputPID = 200;
    if (outputPID < -200) outputPID = -200;
  }

  lastError = error;

  // Limitarea ratei de schimbare a output-ului PID
  const float maxChange = 10.0; 
  if (outputPID > lastPIDOutput + maxChange) {
    outputPID = lastPIDOutput + maxChange;
  } else if (outputPID < lastPIDOutput - maxChange) {
    outputPID = lastPIDOutput - maxChange;
  }
  lastPIDOutput = outputPID;

  // Comandăm motoarele în funcție de semn, aplicând factorRight pe motorul din dreapta
  if (outputPID > 0) {
    // Forward
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, (int)outputPID);

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    // Aplicăm factor reducere motor dreapta
    analogWrite(ENA, (int)(outputPID * factorRight));
  } else if (outputPID < 0) {
    // Backward
    float reverseVal = abs(outputPID);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, (int)reverseVal);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    // Aplicăm factor reducere motor dreapta
    analogWrite(ENA, (int)(reverseVal * factorRight));
  } else {
    stopBothMotors();
  }
}

void sendPIDDataJSON() {
  // Actualizăm PID doar dacă este activ
  if (pidMode) {
    updatePID();
  } else {
    stopBothMotors();
  }

  String jsonResponse = "{";
  jsonResponse += "\"outputPID\":" + String(outputPID);
  jsonResponse += "}";

  server.send(200, "application/json", jsonResponse);
}

// ==================== Funcții de Control al Motoarelor ====================
void moveForwardLeft() {
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 255);
  servo2.write(90);
}

void moveBackwardLeft() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 255);
  servo2.write(90);
}

void stopLeftMotor() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
  servo2.write(90);
}

void moveForwardRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 255);
  servo2.write(90);
}

void moveBackwardRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 255);
  servo2.write(90);
}

void stopRightMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  servo2.write(90);
}

void moveForwardBoth() {
  moveForwardLeft();
  moveForwardRight();
  servo2.write(90);
}

void moveBackwardBoth() {
  moveBackwardLeft();
  moveBackwardRight();
  servo2.write(90);
}

void stopBothMotors() {
  stopLeftMotor();
  stopRightMotor();
  servo2.write(90);
}

// ==================== Funcții de Control al Servomotoarelor ====================
void rotateServo360Clockwise() {
  servo360.write(servo360Clockwise);
  server.send(200, "text/plain", "Servo 360 rotind clockwise");
}

void rotateServo360CounterClockwise() {
  servo360.write(servo360CounterClockwise);
  server.send(200, "text/plain", "Servo 360 rotind counter-clockwise");
}

void stopServo360() {
  stopRequested = true;
  servo360.write(servo360Stop);
  server.send(200, "text/plain", "Servo 360 oprit");
}

void rotateServo360InSteps() {
  const int steps = 20;
  const int stepDelay = 20;
  const int angleDecrement = 15;
  const int maxBackward = 18;
  const int maxForward = 162;
  int currentAngle = maxBackward;

  for (int i = 0; i < steps; i++) {
    if (stopRequested) break;
    currentAngle += angleDecrement;
    if (currentAngle > maxForward) {
      currentAngle = maxForward;
    }
    servo360.write(currentAngle);
    delay(stepDelay);
  }
  servo360.write(servo360Stop);
  server.send(200, "text/plain", "Servo 360 a terminat rotația în pași.");
}

void setServo360To0() {
  servo360.write(0);
  server.send(200, "text/plain", "Servo 360 setat la 0°");
}

void setServo360To90() {
  servo360.write(90);
  server.send(200, "text/plain", "Servo 360 setat la 90°");
}

void setServo360To135() {
  servo360.write(135);
  server.send(200, "text/plain", "Servo 360 setat la 135°");
}

void setServo360To180() {
  servo360.write(180);
  server.send(200, "text/plain", "Servo 360 setat la 180°");
}

// Servo standard
void rotateServo2Clockwise() {
  servo2.write(180);
  server.send(200, "text/plain", "Servo standard rotind clockwise");
}

void rotateServo2CounterClockwise() {
  servo2.write(0);
  server.send(200, "text/plain", "Servo standard rotind counter-clockwise");
}

void stopServo2() {
  servo2.write(90);
  server.send(200, "text/plain", "Servo standard oprit");
}

void setServo2To0() {
  servo2.write(0);
  server.send(200, "text/plain", "Servo standard setat la 0°");
}

void setServo2To90() {
  servo2.write(90);
  server.send(200, "text/plain", "Servo standard setat la 90°");
}

void setServo2To135() {
  servo2.write(135);
  server.send(200, "text/plain", "Servo standard setat la 135°");
}

void setServo2To180() {
  servo2.write(180);
  server.send(200, "text/plain", "Servo standard setat la 180°");
}

// ==================== Funcții pentru Întoarcere ====================
void turnLeft() {
  // Motor stânga backward, motor dreapta forward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  analogWrite(ENB, 255);
  analogWrite(ENA, 255);
  delay(1000);

  stopBothMotors();
  servo2.write(90);
}

void turnRight() {
  // Motor stânga forward, motor dreapta backward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  analogWrite(ENB, 255);
  analogWrite(ENA, 255);
  delay(1000);

  stopBothMotors();
  servo2.write(90);
}

// ==================== Setup ====================
void setup() {
  Serial.begin(115200);

  // Inițializare pini motoare
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Inițializare pini senzor ultrasonic
  pinMode(trigPIN, OUTPUT);
  pinMode(echoPIN, INPUT);

  // Atașare servomotoare
  servo360.attach(SERVO_PIN_360);
  servo2.attach(SERVO_PIN_2);
  servo360.write(servo360Stop);
  servo2.write(90);

  // Configurare Wi-Fi în modul Access Point
  WiFi.softAP(ssid, password);
  Serial.println("Configurația WiFi finalizată.");
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  // Definire rute pentru server
  server.on("/", handleRoot);

  // Rute servo 360
  server.on("/rotateServo360Clockwise", rotateServo360Clockwise);
  server.on("/rotateServo360CounterClockwise", rotateServo360CounterClockwise);
  server.on("/stopServo360", stopServo360);
  server.on("/rotateServo360InSteps", rotateServo360InSteps);
  server.on("/setServo360To0", setServo360To0);
  server.on("/setServo360To90", setServo360To90);
  server.on("/setServo360To135", setServo360To135);
  server.on("/setServo360To180", setServo360To180);

  // Rute servo standard
  server.on("/rotateServo2Clockwise", rotateServo2Clockwise);
  server.on("/rotateServo2CounterClockwise", rotateServo2CounterClockwise);
  server.on("/stopServo2", stopServo2);
  server.on("/setServo2To0", setServo2To0);
  server.on("/setServo2To90", setServo2To90);
  server.on("/setServo2To135", setServo2To135);
  server.on("/setServo2To180", setServo2To180);

  // Motoare
  server.on("/moveForwardRight", moveForwardRight);
  server.on("/moveBackwardRight", moveBackwardRight);
  server.on("/stopRightMotor", stopRightMotor);
  server.on("/moveForwardLeft", moveForwardLeft);
  server.on("/moveBackwardLeft", moveBackwardLeft);
  server.on("/stopLeftMotor", stopLeftMotor);
  server.on("/moveForwardBoth", moveForwardBoth);
  server.on("/moveBackwardBoth", moveBackwardBoth);
  server.on("/stopBothMotors", stopBothMotors);

  // Distanță
  server.on("/measureDistance", measureDistance);
  server.on("/toggleAutonomousMode", toggleAutonomousMode);
  server.on("/stopAutonomousMode", stopAutonomousMode);

  // Rute PID
  server.on("/togglePIDMode", togglePIDMode);
  server.on("/setSetpoint", setSetpoint);
  server.on("/pidData", sendPIDDataJSON);

  // Pornire server
  server.begin();
  Serial.println("Web server a pornit.");

  // Inițializăm pentru PID
  lastPIDTime = millis();
}

// ==================== Loop ====================
void loop() {
  server.handleClient();

  if (autonomousMode) {
    obstacleAvoidance();
  }
  // PID se actualizează în sendPIDDataJSON() când pidMode este activ
}