#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>

#define PWM_PIN 13
#define GROUND_PIN 12
#define PWM_CHANNEL 0
#define PWM_FREQUENCY 1000
#define PWM_RESOLUTION 8

#define WATER_LEVEL_PIN 33

#define LED_RX_PIN 15
#define LED_TX_PIN 2

#define NUM_OCTAVES 6

#define ROLLING_AVERAGE_NUM 50

char ssid[] = "Mine";  // your network SSID (name)
char pass[] = "12345678";       // your network password

WiFiUDP Udp;                                // A UDP instance to let us send and receive packets over UDP
const IPAddress outIp(172,20,10,3);       // remote IP of your computer
const unsigned int outPort = 9999;          // remote port to receive OSC
const unsigned int localPort = 8888;        // local port to listen for OSC packets (actually not used for sending)

int max_water_level = 0;
int min_water_level = 1E5;

int rolling_average = 0;

int last_water_level = 0;
int last_change = 0;

String notes[NUM_OCTAVES] = {"03C", "03D", "03E", "03G", "04A", "04C"};

void setup() {
  Serial.begin(9600);

  // Connect to WiFi network
  Serial.println("\n\nConnecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Print WiFi info
  Serial.println("\nWiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Print UDP info
  Serial.println("Starting UDP");
  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(localPort);

  // Setup PWM
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0);

  // Set ground pin to LOW (0V)
  pinMode(GROUND_PIN, OUTPUT);
  digitalWrite(GROUND_PIN, LOW);

  // Set up RX/TX LEDs
  pinMode(LED_RX_PIN, OUTPUT);
  pinMode(LED_TX_PIN, OUTPUT);
  digitalWrite(LED_TX_PIN, LOW);
  digitalWrite(LED_RX_PIN, LOW);

  for (int i = 0; i < 50; i++)
  {
    int val = analogRead(WATER_LEVEL_PIN);
    delay(100);
  }

  Serial.println("Setup completed.");
}

void recieveMotorSpeed(OSCMessage &msg) {
  // Serial.print("Received message on /1/faderA: ");

  digitalWrite(LED_RX_PIN, HIGH);

  // Assuming the message contains a float
  float value = msg.getFloat(0);

  // Convert to PWM value (0-256)
  int pwmVal = value * 256;

  // Print to serial
  // Serial.print(value);
  // Serial.print(" -> ");
  // Serial.println(pwmVal);

  // Write PWM to channel
  ledcWrite(PWM_CHANNEL, pwmVal);

  digitalWrite(LED_RX_PIN, LOW);
}

void sendWaterLevel()
{
  digitalWrite(LED_TX_PIN, HIGH);
  // Read water level
  unsigned int water_level = analogRead(WATER_LEVEL_PIN);

  if (rolling_average != 0)
  {
    rolling_average = (rolling_average * (ROLLING_AVERAGE_NUM - 1) + water_level) / ROLLING_AVERAGE_NUM;
  }
  else 
  {
    rolling_average = water_level;
  }

  if (rolling_average > max_water_level)
  {
    max_water_level = rolling_average;
    //Serial.print("Max water level: ");
    //Serial.println(max_water_level);
  }

  if (rolling_average < min_water_level)
  {
    min_water_level = rolling_average;
    //Serial.print("Min water level: ");
    //Serial.println(min_water_level);
  }

  // Convert to octave
  float water_level_norm = float(rolling_average - min_water_level) / float(max_water_level - min_water_level);
  int index = water_level_norm * NUM_OCTAVES;
  String note = notes[index % NUM_OCTAVES];

  int new_change = rolling_average - last_water_level;

  if (signbit(new_change) != signbit(last_change))
  {
    // Send message on UDP
    OSCMessage msg_out("/water_level");
    msg_out.add(water_level_norm);
    Udp.beginPacket(outIp, outPort);
    msg_out.send(Udp);
    Udp.endPacket();
    msg_out.empty();
    digitalWrite(LED_TX_PIN, LOW);

    Serial.println(note);
  }

  // Print to serial
  //Serial.print(min_water_level);
  //Serial.print(" ");
  //Serial.print(max_water_level);
  //Serial.print(" ");
  //Serial.println(rolling_average);

  //Serial.print(" | Norm: ");
  //Serial.print(0);
  //Serial.print(" ");
  //Serial.print(1.0);
  //Serial.print(" ");
  //Serial.println(water_level_norm);

  last_water_level = rolling_average;
  last_change = new_change;
}

void loop() {
  OSCMessage msg_in;
  int size = Udp.parsePacket();

  if (size > 0) {
    while (size--) {
      msg_in.fill(Udp.read());
    }
    if (!msg_in.hasError()) {
      msg_in.dispatch("/1/faderA", recieveMotorSpeed);
    } else {
      OSCErrorCode error = msg_in.getError();
      Serial.print("error: ");
      Serial.println(error);
    }
  }

  sendWaterLevel();
  delay(25);
}
