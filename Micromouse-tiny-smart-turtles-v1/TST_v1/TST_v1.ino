// ============================================================================
// MICROMOUSE TST - Tiny Smart Turtles version 1
// Encoder = PA15/PB3 + PB6/PB7
// ============================================================================

#include <Arduino.h>
#include <stdlib.h>

// ============================================================================
// FLOODFILL CONSTANTS
// ============================================================================
const int TURN_SCORE = 0;
const int TILE_SCORE = 1;
const int STREAK_SCORE = 0;

#define STARTING_TARGET 1
#define STARTING_HEADING NORTH
#define STARTING_X 0
#define STARTING_Y 0

#define LOWER_X_GOAL 7
#define LOWER_Y_GOAL 7
#define UPPER_X_GOAL 8
#define UPPER_Y_GOAL 8

#define MAZE_WIDTH 16
#define MAZE_HEIGHT 16
#define OUT_OF_BOUNDS -2
#define NOT_YET_SET -1

typedef enum Heading { NORTH, WEST, SOUTH, EAST } Heading;
typedef enum Action { LEFT, FORWARD, RIGHT, IDLE } Action;

typedef struct coord {
  int x;
  int y;
} coord;
typedef struct neighbor {
  coord pos;
  Heading heading;
  int streak;
} neighbor;
typedef neighbor item_type;

struct node {
  item_type data;
  struct node *next;
};
struct _queue {
  struct node *head;
  struct node *tail;
  int size;
};
typedef struct _queue *queue;

// ============================================================================
// ROBOT CONFIG
// ============================================================================
int stepMove = 1;

/******** CONFIG ********/
#define BT_BAUD 9600
#define REPORT_INTERVAL_MS 50

uint16_t DELAY_AFTER_CELL = 200;
uint16_t DELAY_AFTER_TURN = 200;

/******** IR ********/
#define PULSE_US 400
const uint32_t EMIT[4] = {PA9, PB8, PA8, PB9};
const uint32_t RECV[4] = {PA4, PA3, PA5, PA2};
uint16_t vOn[4], vOff[4], IRd[4];

#include <HardwareSerial.h>
HardwareSerial BT(PB11, PB10);

/******** IR WALL THRESHOLDS (RENAMED) ********/

// --- LEFT / RIGHT WALL DETECTION ---
const uint16_t SIDE_WALL_ON_MAX = 34;  // was LEFT_WALL_MAX / RIGHT_WALL_MAX
const uint16_t SIDE_WALL_OFF_MIN = 34; // was LEFT_WALL_MIN / RIGHT_WALL_MIN

/******** MOTOR  ********/
#define MLEFT_IN1 PA6
#define MLEFT_IN2 PA7
#define MRIGHT_IN3 PB0
#define MRIGHT_IN4 PB1
#define MAX_PWM 255

void motor_init() {
  pinMode(MLEFT_IN1, OUTPUT);
  pinMode(MLEFT_IN2, OUTPUT);
  pinMode(MRIGHT_IN3, OUTPUT);
  pinMode(MRIGHT_IN4, OUTPUT);

  digitalWrite(MLEFT_IN1, HIGH);
  digitalWrite(MLEFT_IN2, HIGH);
  digitalWrite(MRIGHT_IN3, HIGH);
  digitalWrite(MRIGHT_IN4, HIGH);
}

void motorLeft(int power) {
  bool forward = true;
  if (power < 0) {
    power = -power;
    forward = false;
  }
  if (power > MAX_PWM)
    power = MAX_PWM;

  if (forward) {
    analogWrite(MLEFT_IN1, MAX_PWM - power);
    analogWrite(MLEFT_IN2, MAX_PWM);
  } else {
    analogWrite(MLEFT_IN1, MAX_PWM);
    analogWrite(MLEFT_IN2, MAX_PWM - power);
  }
}

void motorRight(int power) {
  bool forward = true;
  if (power < 0) {
    power = -power;
    forward = false;
  }
  if (power > MAX_PWM)
    power = MAX_PWM;

  if (forward) {
    analogWrite(MRIGHT_IN3, MAX_PWM);
    analogWrite(MRIGHT_IN4, MAX_PWM - power);
  } else {
    analogWrite(MRIGHT_IN3, MAX_PWM - power);
    analogWrite(MRIGHT_IN4, MAX_PWM);
  }
}

void setMotor(int L, int R) {
  motorLeft(L);
  motorRight(R);
}

void motorStop() { setMotor(0, 0); }

void drive_brake() {
  digitalWrite(MLEFT_IN1, HIGH);
  digitalWrite(MLEFT_IN2, HIGH);
  digitalWrite(MRIGHT_IN3, HIGH);
  digitalWrite(MRIGHT_IN4, HIGH);

  analogWrite(MLEFT_IN1, 255);
  analogWrite(MLEFT_IN2, 255);
  analogWrite(MRIGHT_IN3, 255);
  analogWrite(MRIGHT_IN4, 255);
}

/******** ENCODER — QUADRATURE A/B CHUẨN ********/
#define ENCLA PA15
#define ENCLB PB3
#define ENCRA PB6
#define ENCRB PB7

volatile long countLeft = 0;
volatile long countRight = 0;

inline void updateEncoder(volatile long &count, uint8_t A, uint8_t B,
                          bool aChange) {
  bool a = digitalRead(A);
  bool b = digitalRead(B);

  if (aChange) {
    if (a == b)
      count++; // hướng 1
    else
      count--; // hướng 2
  } else {
    if (a != b)
      count++; // hướng 1
    else
      count--; // hướng 2
  }
}

void encoderLeftA() { updateEncoder(countLeft, ENCLA, ENCLB, true); }
void encoderLeftB() { updateEncoder(countLeft, ENCLA, ENCLB, false); }
void encoderRightA() { updateEncoder(countRight, ENCRA, ENCRB, true); }
void encoderRightB() { updateEncoder(countRight, ENCRA, ENCRB, false); }

void resetEncoder() {
  countLeft = 0;
  countRight = 0;
}

/******** TICKS ********/
const int32_t CELL_TICKS = 2150;
const int32_t TURN90_TICKS = 905;
const int32_t TURN180_TICKS = 1810;
const int PWM_FWD = 80;
const int PWM_TURN = 50;

/******** IR median ********/
uint16_t readMedian(uint8_t pin, int samples = 7) {
  uint16_t v[9];
  samples = constrain(samples, 3, 9);
  for (int i = 0; i < samples; i++) {
    v[i] = analogRead(pin);
  }

  // sort
  for (int i = 0; i < samples - 1; i++) {
    int m = i;
    for (int j = i + 1; j < samples; j++)
      if (v[j] < v[m])
        m = j;
    if (m != i) {
      uint16_t t = v[i];
      v[i] = v[m];
      v[m] = t;
    }
  }

  return v[samples / 2];
}

/******** IR read ********/
void ir_init() {
  analogReadResolution(12);
  for (int i = 0; i < 4; i++) {
    pinMode(EMIT[i], OUTPUT);
    digitalWrite(EMIT[i], LOW);
    pinMode(RECV[i], INPUT_ANALOG);
  }
}

void debugPrintIR() {
  char buf[64];
  int n = snprintf(buf, sizeof(buf), "[IR] SL:%u  SR:%u  FL:%u  FR:%u", IRd[0],
                   IRd[1], IRd[2], IRd[3]);
  BT.write(buf, n);
  BT.write('\n');
}

void debugPrintFF(Action act) {
  switch (act) {
  case FORWARD:
    BT.println("[FF] DECISION = FORWARD");
    break;
  case LEFT:
    BT.println("[FF] DECISION = LEFT");
    break;
  case RIGHT:
    BT.println("[FF] DECISION = RIGHT");
    break;
  case IDLE:
    BT.println("[FF] DECISION = IDLE (180 or stuck)");
    break;
  }
}

void debugPrintEncoder() {
  char buf[64];
  int n =
      snprintf(buf, sizeof(buf), "[ENC] L:%ld  R:%ld", countLeft, countRight);
  BT.write(buf, n);
  BT.write('\n');
}

void ir_read_once() {

  // ======================== SL (index 0) =========================
  digitalWrite(EMIT[0], LOW);
  delayMicroseconds(80);
  vOff[0] = readMedian(RECV[0], 7);

  digitalWrite(EMIT[0], HIGH);
  delayMicroseconds(PULSE_US);
  vOn[0] = readMedian(RECV[0], 7);

  digitalWrite(EMIT[0], LOW);

  // ======================== SR (index 1) =========================
  digitalWrite(EMIT[1], LOW);
  delayMicroseconds(80);
  vOff[1] = readMedian(RECV[1], 7);

  digitalWrite(EMIT[1], HIGH);
  delayMicroseconds(PULSE_US);
  vOn[1] = readMedian(RECV[1], 7);

  digitalWrite(EMIT[1], LOW);

  // ======================== FL (index 2) =========================
  digitalWrite(EMIT[2], LOW);
  delayMicroseconds(80);
  vOff[2] = readMedian(RECV[2], 7);

  digitalWrite(EMIT[2], HIGH);
  delayMicroseconds(PULSE_US);
  vOn[2] = readMedian(RECV[2], 7);

  digitalWrite(EMIT[2], LOW);

  // ======================== FR (index 3) =========================
  digitalWrite(EMIT[3], LOW);
  delayMicroseconds(80);
  vOff[3] = readMedian(RECV[3], 7);

  digitalWrite(EMIT[3], HIGH);
  delayMicroseconds(PULSE_US);
  vOn[3] = readMedian(RECV[3], 7);

  digitalWrite(EMIT[3], LOW);

  // ======================= Tính diff từng cảm biến =======================
  int32_t diff0 = vOn[0] - vOff[0];
  if (diff0 < 0)
    diff0 = 0;
  int32_t diff1 = vOn[1] - vOff[1];
  if (diff1 < 0)
    diff1 = 0;
  int32_t diff2 = vOn[2] - vOff[2];
  if (diff2 < 0)
    diff2 = 0;
  int32_t diff3 = vOn[3] - vOff[3];
  if (diff3 < 0)
    diff3 = 0;

  IRd[0] = diff0 / 100; // SL
  IRd[1] = diff1 / 100; // SR
  IRd[2] = diff2 / 100;
  IRd[3] = diff3 / 100;

  // In debug
  debugPrintIR();
}

/******** Wall states ********/
static bool left_wall = false, right_wall = false, front_wall = false;

inline void update_side_wall_states() {
  if (!left_wall) {
    if (IRd[0] >= SIDE_WALL_ON_MAX)
      left_wall = true;
  } else {
    if (IRd[0] <= SIDE_WALL_OFF_MIN)
      left_wall = false;
  }

  if (!right_wall) {
    if (IRd[1] >= SIDE_WALL_ON_MAX)
      right_wall = true;
  } else {
    if (IRd[1] <= SIDE_WALL_OFF_MIN)
      right_wall = false;
  }
}

/******** BT Telemetry ********/
void sendLine(uint16_t d0, uint16_t d1, uint16_t d2, uint16_t d3) {
  char buf[64];
  int n = snprintf(buf, sizeof(buf), "SL:%u SR:%u FL:%u FR:%u", d0, d1, d2, d3);
  BT.write(buf, n);
  BT.write('\n');
}

/******** PID – TỐI ƯU THEO LOGIC XE MẪU ********/
float Kp = 4.0; // 8.5
float Ki = 0.0;
float Kd = 3.0; // 5.5

float error = 0;
float previous_error = 0;
float integral = 0;

float getError() {
  error = IRd[0] - IRd[1];
  error = constrain(error, -50, 50);
  return error;
}
float getErrorL() {
  error = 30 - IRd[0];
  error = constrain(error, -50, 50);
  return error;
}

float getErrorR() {
  error = IRd[1] - 30;
  error = constrain(error, -50, 50);
  return error;
}

float calL() {
  float e = getErrorL();
  if ((e > 0 && integral < 20) || (e < 0 && integral > -20)) {
    integral += e;
  }
  float pid = Kp * e + Kd * (e - previous_error) + Ki * integral;
  previous_error = e;
  return pid;
}

float calR() {
  float e = getErrorR();
  if ((e > 0 && integral < 20) || (e < 0 && integral > -20)) {
    integral += e;
  }
  float pid = Kp * e + Kd * (e - previous_error) + Ki * integral;
  previous_error = e;
  return pid;
}

float calculate_pid() {
  float e = getError();
  if ((e > 0 && integral < 20) || (e < 0 && integral > -20)) {
    integral += e;
  }
  float pid = Kp * e + Kd * (e - previous_error) + Ki * integral;
  previous_error = e;
  return pid;
}

//------------------------------------------

uint32_t tLastIR = 0;

void forward_one_cell() {

  resetEncoder();
  // pid_reset_all();

  uint32_t t_no_progress = millis();
  const int NO_PROGRESS_TIMEOUT = 350; // chống kẹt

  while ((abs(countLeft) + abs(countRight)) / 2 <= CELL_TICKS) {

    ir_read_once();

    // ===== 2) Chống kẹt encoder =====
    static long lastL = 0, lastR = 0;
    if (countLeft == lastL && countRight == lastR) {
      if (millis() - t_no_progress > NO_PROGRESS_TIMEOUT) {
        BT.println("[ACT] ENCODER STUCK → STOP");
        break;
      }
    } else {
      t_no_progress = millis();
    }
    lastL = countLeft;
    lastR = countRight;

    // ===== 3) Tốc độ cơ bản =====
    int speed = PWM_FWD;

    int wallThreshold = 18;
    int noWallThreshold = 14;
    bool leftWall = (IRd[0] >= wallThreshold);
    bool rightWall = (IRd[1] >= wallThreshold);
    bool noLeftWall = (IRd[0] <= noWallThreshold);
    bool noRightWall = (IRd[1] <= noWallThreshold);

    if (leftWall && rightWall) {
      // 1. CÓ CẢ HAI TƯỜNG: Chạy PID giữ tâm
      float pid = calculate_pid();
      int leftSpeed = constrain(speed + pid, 0, 150);
      int rightSpeed = constrain(speed - pid, 0, 150);
      setMotor(leftSpeed, rightSpeed);

    } else if (leftWall && noRightWall) {
      // 2. CHỈ CÓ TƯỜNG TRÁI: Chạy PID bám tường trái
      float pidL = calL();
      int leftSpeed = constrain(speed + pidL, 0, 150);
      int rightSpeed = constrain(speed - pidL, 0, 150);
      setMotor(leftSpeed, rightSpeed);

    } else if (rightWall && noLeftWall) {
      // 3. CHỈ CÓ TƯỜNG PHẢI: Chạy PID bám tường phải
      float pidR = calR();
      int leftSpeed = constrain(speed + pidR, 0, 150);
      int rightSpeed = constrain(speed - pidR, 0, 150);
      setMotor(leftSpeed, rightSpeed);

    } else {
      // 4. KHÔNG CÓ TƯỜNG (hoặc có tường trước mặt): Chạy thẳng
      setMotor(speed, speed);
      // Reset PID khi không dùng để tránh bị "giật" khi thấy tường lại
      integral = 0;
      previous_error = 0;
    }
  }

  // ===== 6) DỪNG CUỐI Ô =====
  drive_brake();
  motorStop();
  delay(DELAY_AFTER_CELL);

  stepMove++;
}

/******** TURNS ********/
void turn_90_left() {
  resetEncoder();
  ir_read_once(); // đọc trước để biết có tường hay không

  bool hasL = (IRd[0] >= SIDE_WALL_ON_MAX);
  bool hasR = (IRd[1] >= SIDE_WALL_ON_MAX);

  // === Nếu 2 bên đều có tường → quay gọn ===
  // === Nếu thiếu tường 1 bên → quay + delay ===
  bool needDelay = !(hasL && hasR);

  while ((abs(countLeft) + abs(countRight)) / 2 <= TURN90_TICKS) {
    setMotor(-PWM_TURN, +PWM_TURN);
  }

  drive_brake();
  motorStop();

  if (needDelay)
    delay(100);
  delay(DELAY_AFTER_TURN);
}

void turn_90_right() {
  resetEncoder();
  ir_read_once();

  bool hasL = (IRd[0] >= SIDE_WALL_ON_MAX);
  bool hasR = (IRd[1] >= SIDE_WALL_ON_MAX);

  bool needDelay = !(hasL && hasR);

  while ((abs(countLeft) + abs(countRight)) / 2 <= TURN90_TICKS) {
    setMotor(+PWM_TURN, -PWM_TURN);
  }

  drive_brake();
  motorStop();

  if (needDelay)
    delay(100);
  delay(DELAY_AFTER_TURN);
}

void turn_180_exact() {
  resetEncoder();
  ir_read_once();

  bool hasL = (IRd[0] >= SIDE_WALL_ON_MAX);
  bool hasR = (IRd[1] >= SIDE_WALL_ON_MAX);

  bool needDelay = !(hasL && hasR);

  while ((abs(countLeft) + abs(countRight)) / 2 <= TURN180_TICKS) {
    setMotor(-PWM_TURN, +PWM_TURN);
  }

  drive_brake();
  motorStop();

  if (needDelay)
    delay(100);
  delay(DELAY_AFTER_TURN);
}

void system_init() {
  BT.begin(BT_BAUD);
  motor_init();
  ir_init();

  pinMode(ENCLA, INPUT_PULLUP);
  pinMode(ENCLB, INPUT_PULLUP);
  pinMode(ENCRA, INPUT_PULLUP);
  pinMode(ENCRB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCLA), encoderLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCLB), encoderLeftB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCRA), encoderRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCRB), encoderRightB, CHANGE);

  BT.println("READY: IR + Encoder + Floodfill");
}

/******** WALL API ********/
bool wallFront() {
  ir_read_once();
  return IRd[2] >= 10;
}
bool wallRight() {
  ir_read_once();
  return IRd[1] >= 10;
}
bool wallLeft() {
  ir_read_once();
  return IRd[0] >= 10;
}

int API_wallFront() { return wallFront(); }
int API_wallRight() { return wallRight(); }
int API_wallLeft() { return wallLeft(); }

void API_moveForward() {
  BT.println("[FF] FORWARD");
  forward_one_cell();
}
void API_turnRight() {
  BT.println("[FF] RIGHT");
  turn_90_right();
}
void API_turnLeft() {
  BT.println("[FF] LEFT");
  turn_90_left();
}
void API_turn180() {
  BT.println("[FF] 180");
  turn_180_exact();
}

//===========flood fill===============
unsigned char target = STARTING_TARGET;
coord currentXY = {STARTING_X, STARTING_Y};
Heading currentHeading = STARTING_HEADING;

int verticalWalls[MAZE_WIDTH + 1][MAZE_HEIGHT] = {{0}};   // tường dọc
int horizontalWalls[MAZE_WIDTH][MAZE_HEIGHT + 1] = {{0}}; // tường ngang
int floodArray[MAZE_WIDTH][MAZE_HEIGHT];                  // giá trị flood

void API_turnLeft();
void API_turnRight();
void API_turn180();
void API_moveForward();
int API_wallFront();
int API_wallLeft();
int API_wallRight();
void printMaze();

// --- Queue helpers ---
void halt_with_error(const char *msg) {
  BT.print("FATAL ERROR: ");
  BT.println(msg);
  motorStop();
  while (1)
    ;
}
queue queue_create() {
  queue q = (queue)malloc(sizeof(struct _queue));
  if (q == NULL) {
    halt_with_error("Insufficient memory.");
  }
  q->head = NULL;
  q->tail = NULL;
  q->size = 0;
  return q;
}
int queue_is_empty(queue q) {
  if (q == NULL) {
    halt_with_error("NULL queue.");
  }
  return q->head == NULL;
}
void queue_push(queue q, item_type elem) {
  struct node *n = (struct node *)malloc(sizeof(struct node));
  if (n == NULL) {
    halt_with_error("Insufficient memory.");
  }
  n->data = elem;
  n->next = NULL;
  if (q->head == NULL) {
    q->head = q->tail = n;
  } else {
    q->tail->next = n;
    q->tail = n;
  }
  q->size += 1;
}
item_type queue_pop(queue q) {
  if (queue_is_empty(q)) {
    halt_with_error("Queue is empty.");
  }
  struct node *head = q->head;
  if (q->head == q->tail) {
    q->head = NULL;
    q->tail = NULL;
  } else {
    q->head = q->head->next;
  }
  q->size -= 1;
  item_type data = head->data;
  free(head);
  return data;
}
void queue_clear(queue q) {
  while (!queue_is_empty(q)) {
    queue_pop(q);
  }
}

// --- Solver helpers ---
void updateFloodArray(coord c, int val) { floodArray[c.x][c.y] = val; }

int isAccessible(coord c1, coord c2) {
  if (c1.x == c2.x) {
    if (c1.y > c2.y)
      return !horizontalWalls[c1.x][c1.y];
    else
      return !horizontalWalls[c1.x][c2.y];
  }
  if (c1.y == c2.y) {
    if (c1.x > c2.x)
      return !verticalWalls[c1.x][c1.y];
    else
      return !verticalWalls[c2.x][c1.y];
  }
  return 0;
}

void generateNeighbor(queue q, neighbor n, Heading h, int streak) {
  coord newCoord = n.pos;
  switch (h) {
  case NORTH:
    newCoord.y++;
    break;
  case WEST:
    newCoord.x--;
    break;
  case SOUTH:
    newCoord.y--;
    break;
  case EAST:
    newCoord.x++;
    break;
  }
  if (newCoord.x < 0 || newCoord.x >= MAZE_WIDTH || newCoord.y < 0 ||
      newCoord.y >= MAZE_HEIGHT)
    return;
  if (!isAccessible(n.pos, newCoord))
    return;

  int floodVal = floodArray[n.pos.x][n.pos.y];
  int newFloodVal = floodArray[newCoord.x][newCoord.y];
  int score = TILE_SCORE + ((n.heading == h) ? 0 : TURN_SCORE) -
              ((n.heading == h) ? STREAK_SCORE : 0);

  if (newFloodVal == NOT_YET_SET || newFloodVal > floodVal + score) {
    updateFloodArray(newCoord, floodVal + score);
    neighbor nxt = {newCoord, h, (n.heading == h) ? streak + 1 : 1};
    queue_push(q, nxt);
  }
}

void floodFill() {
  for (int i = 0; i < MAZE_WIDTH; i++)
    for (int j = 0; j < MAZE_HEIGHT; j++)
      floodArray[i][j] = NOT_YET_SET;

  queue q = queue_create();

  if (target) {
    // flood từ vùng goal
    for (int x = LOWER_X_GOAL; x <= UPPER_X_GOAL; x++)
      for (int y = LOWER_Y_GOAL; y <= UPPER_Y_GOAL; y++) {
        coord g = {x, y};
        updateFloodArray(g, 0);
        neighbor n = {g, NORTH, 0};
        queue_push(q, n);
      }
  } else {
    // flood từ start khi đã đến goal
    coord s = {STARTING_X, STARTING_Y};
    updateFloodArray(s, 0);
    neighbor n = {s, NORTH, 0};
    queue_push(q, n);
  }

  while (!queue_is_empty(q)) {
    neighbor current = queue_pop(q);
    generateNeighbor(q, current, NORTH, current.streak);
    generateNeighbor(q, current, WEST, current.streak);
    generateNeighbor(q, current, SOUTH, current.streak);
    generateNeighbor(q, current, EAST, current.streak);
  }
  queue_clear(q);
  free(q);
}

void updateWalls() {
  if (API_wallFront()) {
    switch (currentHeading) {
    case NORTH:
      horizontalWalls[currentXY.x][currentXY.y + 1] = 1;
      break;
    case WEST:
      verticalWalls[currentXY.x][currentXY.y] = 1;
      break;
    case SOUTH:
      horizontalWalls[currentXY.x][currentXY.y] = 1;
      break;
    case EAST:
      verticalWalls[currentXY.x + 1][currentXY.y] = 1;
      break;
    }
  }
  if (API_wallLeft()) {
    switch (currentHeading) {
    case NORTH:
      verticalWalls[currentXY.x][currentXY.y] = 1;
      break;
    case WEST:
      horizontalWalls[currentXY.x][currentXY.y] = 1;
      break;
    case SOUTH:
      verticalWalls[currentXY.x + 1][currentXY.y] = 1;
      break;
    case EAST:
      horizontalWalls[currentXY.x][currentXY.y + 1] = 1;
      break;
    }
  }
  if (API_wallRight()) {
    switch (currentHeading) {
    case NORTH:
      verticalWalls[currentXY.x + 1][currentXY.y] = 1;
      break;
    case WEST:
      horizontalWalls[currentXY.x][currentXY.y + 1] = 1;
      break;
    case SOUTH:
      verticalWalls[currentXY.x][currentXY.y] = 1;
      break;
    case EAST:
      horizontalWalls[currentXY.x][currentXY.y] = 1;
      break;
    }
  }
}

Action turnLeft() {
  API_turnLeft();
  currentHeading = (Heading)((currentHeading + 1) % 4);
  return LEFT;
}
Action turnRight() {
  API_turnRight();
  currentHeading =
      (Heading)((currentHeading == NORTH) ? EAST : currentHeading - 1);
  return RIGHT;
}

Action nextAction() {
  int currentFlood = floodArray[currentXY.x][currentXY.y];

  int northFlood = (currentXY.y + 1 < MAZE_HEIGHT)
                       ? floodArray[currentXY.x][currentXY.y + 1]
                       : OUT_OF_BOUNDS;
  int westFlood = (currentXY.x - 1 >= 0)
                      ? floodArray[currentXY.x - 1][currentXY.y]
                      : OUT_OF_BOUNDS;
  int southFlood = (currentXY.y - 1 >= 0)
                       ? floodArray[currentXY.x][currentXY.y - 1]
                       : OUT_OF_BOUNDS;
  int eastFlood = (currentXY.x + 1 < MAZE_WIDTH)
                      ? floodArray[currentXY.x + 1][currentXY.y]
                      : OUT_OF_BOUNDS;

  int minFlood = currentFlood;
  Heading newHeading = currentHeading;

  if (northFlood != OUT_OF_BOUNDS &&
      isAccessible(currentXY, (coord){currentXY.x, currentXY.y + 1}) &&
      northFlood < minFlood) {
    minFlood = northFlood;
    newHeading = NORTH;
  }
  if (westFlood != OUT_OF_BOUNDS &&
      isAccessible(currentXY, (coord){currentXY.x - 1, currentXY.y}) &&
      westFlood < minFlood) {
    minFlood = westFlood;
    newHeading = WEST;
  }
  if (southFlood != OUT_OF_BOUNDS &&
      isAccessible(currentXY, (coord){currentXY.x, currentXY.y - 1}) &&
      southFlood < minFlood) {
    minFlood = southFlood;
    newHeading = SOUTH;
  }
  if (eastFlood != OUT_OF_BOUNDS &&
      isAccessible(currentXY, (coord){currentXY.x + 1, currentXY.y}) &&
      eastFlood < minFlood) {
    minFlood = eastFlood;
    newHeading = EAST;
  }

  if (newHeading == currentHeading) {
    API_moveForward();
    switch (currentHeading) {
    case NORTH:
      currentXY.y++;
      break;
    case WEST:
      currentXY.x--;
      break;
    case SOUTH:
      currentXY.y--;
      break;
    case EAST:
      currentXY.x++;
      break;
    }
    return FORWARD;
  }

  if (currentHeading == (newHeading + 3) % 4)
    return turnLeft();
  else if (currentHeading == (newHeading + 1) % 4)
    return turnRight();
  else {
    API_turn180();
    currentHeading = (Heading)((currentHeading + 2) % 4);
    return IDLE;
  }
}

void checkDestination() {
  if (target) {
    // Đang đi tới goal
    if (currentXY.x >= LOWER_X_GOAL && currentXY.x <= UPPER_X_GOAL &&
        currentXY.y >= LOWER_Y_GOAL && currentXY.y <= UPPER_Y_GOAL) {
      target = 0; // chuyển sang mode quay về start
      BT.println("[FF] Reached GOAL region -> target=0 (back to START)");
    }
  } else {
    // Đang quay về start
    if (currentXY.x == STARTING_X && currentXY.y == STARTING_Y) {
      target = 1; // có thể đặt dừng hẳn ở đây
      BT.println("[FF] Returned to START. Stopping.");
      motorStop();
      while (1)
        ; // dừng vô hạn
    }
  }
}

void generateInitialWalls() {
  // Bao tường ngoài maze
  for (int i = 0; i < MAZE_WIDTH; i++) {
    horizontalWalls[i][0] = 1;
    horizontalWalls[i][MAZE_HEIGHT] = 1;
  }
  for (int i = 0; i < MAZE_HEIGHT; i++) {
    verticalWalls[0][i] = 1;
    verticalWalls[MAZE_WIDTH][i] = 1;
  }
}

Action solver() {
  checkDestination();
  updateWalls();
  floodFill();
  return nextAction();
}

// In maze ra BT (ASCII)
void printMaze() {
  BT.print("Toa do: (");
  BT.print(currentXY.x);
  BT.print(", ");
  BT.print(currentXY.y);
  BT.println(")");

  for (int y = MAZE_HEIGHT - 1; y >= 0; y--) {

    // hàng tường ngang trên
    for (int x = 0; x < MAZE_WIDTH; x++) {
      BT.print("+");
      if (horizontalWalls[x][y + 1])
        BT.print("---");
      else
        BT.print("   ");
    }
    BT.println("+");

    // hàng tường dọc + robot
    for (int x = 0; x < MAZE_WIDTH; x++) {
      if (verticalWalls[x][y])
        BT.print("|");
      else
        BT.print(" ");

      if (currentXY.x == x && currentXY.y == y) {
        switch (currentHeading) {
        case NORTH:
          BT.print(" ^ ");
          break;
        case WEST:
          BT.print(" < ");
          break;
        case SOUTH:
          BT.print(" v ");
          break;
        case EAST:
          BT.print(" > ");
          break;
        }
      } else {
        BT.print("   ");
      }
    }
    BT.println("|");
  }

  for (int x = 0; x < MAZE_WIDTH; x++) {
    BT.print("+---");
  }
  BT.println("+");
  BT.println("==============================================");
}

// ============================================================================
// SETUP & LOOP
// ============================================================================
void setup() {
  system_init();
  generateInitialWalls();
  delay(3000);
  currentXY.x = STARTING_X;
  currentXY.y = STARTING_Y;
  currentHeading = STARTING_HEADING;
  target = STARTING_TARGET;

  BT.println("READY");
}

void loop() {
  //solver();
  forward_one_cell();
  //  ir_read_once();
  //  delay(100);                                                                        >
}
