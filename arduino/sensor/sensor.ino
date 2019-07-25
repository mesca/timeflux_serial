#include <CircularBuffer.h>

const int DEFAULT_RATE = 1;
const int SAMPLE_SIZE = 8; // SEQ (2) + TIME (4) + LIGHT (2) + CRC (1)
const int BUFFER_SIZE = 1152;
const int COMMAND_PARAM_MAX = 4;
const char COMMAND_PARAM_SEP = ',';
const char COMMAND_END = '\n';

CircularBuffer<byte, BUFFER_SIZE> buffer;
unsigned long interval = 0;
unsigned long utime = 0;
String command = "";
String params[COMMAND_PARAM_MAX];
int current_param = -1;
unsigned int seq = 0;
bool led = false;
bool streaming = false;

void setup() {
  Serial.begin(115200);
  cmd_rate(DEFAULT_RATE);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.print("ready\n");
}

void loop() {

  // Receive commands from the serial input
  receive();

  // Push data from buffer to the serial output
  send();

  // Run at the defined rate
  if (tick() && streaming) {

    if (buffer.available() < SAMPLE_SIZE) {
      // Buffer is saturated
      led_off();

    } else {
      // Read data from analog port
      unsigned int a0 = analogRead(A0);
      // Push sample to buffer
      push(seq);
      push(utime);
      push(a0);
      push((byte) 0); // Dummy CRC
    }
  }
}

void cmd_rate(int rate) {
  interval = 1e6 / rate;
}

void cmd_start() {
  buffer.clear();
  seq = 65535;
  led_on();
  streaming = true;
}

void cmd_stop() {
  streaming = false;
  led_off();
}

void cmd_ping() {
  Serial.write(micros());
}

void led_on() {
  if (!led) {
    digitalWrite(LED_BUILTIN, HIGH);
    led = true;
  }
}

void led_off() {
  if (led) {
    digitalWrite(LED_BUILTIN, LOW);
    led = false;
  }
}

void exec() {
  if (command == "rate") {
    cmd_rate(params[0].toInt());
  } else if (command == "start") {
    cmd_start();
  } else if (command == "stop") {
    cmd_stop();
  } else if (command == "ping") {
    cmd_ping();
  }
}

bool tick() {
  // TODO: overflow after 70 minutes
  unsigned long now = micros();
  if (now - utime >= interval) {
    utime = now;
    seq ++;
    return true;
  }
  return false;
}

void receive() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    switch (c) {
      case COMMAND_PARAM_SEP:
        current_param ++;
        break;
      case COMMAND_END:
        exec();
        command = "";
        memset(params, 0, sizeof(params));
        current_param = -1;
        break;
      default:
        if (current_param == -1) {
          command += c;
        } else {
          params[current_param] += c;
        }
    }
  }
}

void send() {
  int available = Serial.availableForWrite();
  if (available) {
    for (int i = 0; i < available; i++) {
      if (buffer.isEmpty()) break;
      Serial.write(buffer.shift());
    }
  }
}

template <typename T>
void push(T var) {
  for (int i = 0; i < sizeof(var); i++) {
    buffer.push((byte) (var >> (i * 8)));
  }
}
