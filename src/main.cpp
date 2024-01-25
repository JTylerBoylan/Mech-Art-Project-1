#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>

#define WIFI_SSID "Mine"
#define WIFI_PASS "12345678"

#define UDP_LOCAL_PORT 8888

#define UDP_REMOTE_IP 192,168,137,189
#define UDP_REMOTE_PORT 9999

#define PWM_PIN 13
#define PWM_CHANNEL 0
#define PWM_FREQUENCY 1000
#define PWM_RESOLUTION 8

#define WATER_LEVEL_PIN 33

#define MOTOR_SPEED_OSC_TOPIC "/1/faderA"
#define WATER_LEVEL_OSC_TOPIC "/water_level"

#define WATER_LEVEL_SMOOTH_FACTOR 50
#define EXTREMA_SMOOTH_FACTOR 5
#define NUMBER_SKIPS_AFTER_EXTREMA 10
#define SIGMOID_FACTOR 2.0

#define LED_RX_PIN 15
#define LED_TX_PIN 2

WiFiUDP Udp;

// Function declarations
void connectToWiFi(const char* ssid, const char* pass);
void beginLocalUdp(unsigned int localPort);
void startPWM(int pin, int channel, int frequency, int resolution);
void setupLEDs(int rxPin, int txPin);

void readOSC();
void sendOSC(const char* topic, float value);

void recieveMotorSpeedOSC(OSCMessage &msg);

void sendWaterLevelNorm();
void parseWaterLevel(int waterLevel, bool &is_extrema, float &waterLevelNorm);
int applyRollingAverage(const int new_value, const int current_average, const int rollingAverageSize);

// Setup the ESP32
void setup()
{
    // Start serial @ 9600 baud
    Serial.begin(9600);

    // Connect to WiFi
    connectToWiFi(WIFI_SSID, WIFI_PASS);

    // Start UDP connection
    beginLocalUdp(UDP_LOCAL_PORT);

    // Start PWM
    startPWM(PWM_PIN, PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);

    // Setup LEDs
    setupLEDs(LED_RX_PIN, LED_TX_PIN);

    Serial.println("Setup complete");

    delay(5000);
}

// Run the ESP32
void loop()
{
    // Read incoming OSC messages
    readOSC();

    // Send water level
    sendWaterLevelNorm();

    // Delay 25ms
    delay(10);
}

// Connect to a WiFi network
void connectToWiFi(const char* ssid, const char* pass) 
{
    Serial.println("\n\nConnecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, pass);

    while (WiFi.status() != WL_CONNECTED) 
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nWiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

// Start a UDP connection
void beginLocalUdp(unsigned int localPort) 
{
    Serial.println("Starting UDP");
    Udp.begin(localPort);
    Serial.print("Local port: ");
    Serial.println(localPort);
}

// Start a PWM connection
void startPWM(int pin, int channel, int frequency, int resolution) 
{
    ledcSetup(channel, frequency, resolution);
    ledcAttachPin(pin, channel);
    ledcWrite(channel, 0);
}

// Setup LEDs
void setupLEDs(int rxPin, int txPin)
{
    pinMode(rxPin, OUTPUT);
    pinMode(txPin, OUTPUT);
    digitalWrite(txPin, LOW);
    digitalWrite(rxPin, LOW);
}

// Read incoming OSC messages
void readOSC()
{
    OSCMessage msg;
    int size = Udp.parsePacket();
    if (size > 0) 
    {
        while (size--) 
        {
            msg.fill(Udp.read());
        }
        if (!msg.hasError()) 
        {
            msg.dispatch(MOTOR_SPEED_OSC_TOPIC, recieveMotorSpeedOSC);
        } 
        else 
        {
            Serial.print("OSC Error: ");
            Serial.println(msg.getError());
        }
    }
}

void sendOSC(const char* topic, float value)
{
    digitalWrite(LED_TX_PIN, HIGH);

    OSCMessage msg(topic);
    msg.add(value);

    Udp.beginPacket(IPAddress(UDP_REMOTE_IP), UDP_REMOTE_PORT);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();

    delay(10);
    digitalWrite(LED_TX_PIN, LOW);
}

// Parse motor speed OSC message
void recieveMotorSpeedOSC(OSCMessage &msg) {
    digitalWrite(LED_RX_PIN, HIGH);

    // Assuming the message contains a float
    float value = msg.getFloat(0);

    // Convert to PWM value (0-255)
    int pwmVal = int(value * 256);

    // Write PWM to channel
    ledcWrite(PWM_CHANNEL, pwmVal);

    //Serial.print("Motor speed: ");
    //Serial.println(pwmVal);

    delay(10);

    digitalWrite(LED_RX_PIN, LOW);
}

void sendWaterLevelNorm()
{
    static int num_skips = 0;

    // Read current water level
    int waterLevel = analogRead(WATER_LEVEL_PIN);

    // Skip first few readings
    if (num_skips >= NUMBER_SKIPS_AFTER_EXTREMA)
    {
        // Parse water level
        bool is_extrema;
        float waterLevelNorm;
        parseWaterLevel(waterLevel, is_extrema, waterLevelNorm);

        // Send water level OSC message if it is an extrema
        if (is_extrema)
        {
            sendOSC(WATER_LEVEL_OSC_TOPIC, waterLevelNorm);
            num_skips = 0;

            Serial.println(waterLevelNorm);
        }

    }        
    else
    {
        num_skips++;
    }
}

// Parse water level
void parseWaterLevel(int waterLevel, bool &is_extrema, float &waterLevelNorm)
{
    // Static variables
    static int waterLevelSmooth = waterLevel;
    static int waterLevelSmoothPrev = waterLevelSmooth;
    static bool isRisingPrev = false;
    
    static int maxWaterLevel = waterLevel;
    static int minWaterLevel = waterLevel;

    static int lastExtrema = 0;

    // Apply rolling average
    waterLevelSmooth = applyRollingAverage(waterLevel, waterLevelSmooth, WATER_LEVEL_SMOOTH_FACTOR);

    // Get change in water level
    int deltaWaterLevelSmooth = waterLevelSmooth - waterLevelSmoothPrev;

    // Check if it is rising or falling
    bool isRising = deltaWaterLevelSmooth > 0;

    // Check if it is an extrema
    is_extrema = isRisingPrev != isRising;

    // Update max and min water levels
    if (isRisingPrev && !isRising) // True if at peak
    {
        maxWaterLevel = applyRollingAverage(waterLevelSmooth, maxWaterLevel, EXTREMA_SMOOTH_FACTOR);
    }
    else if (!isRisingPrev && isRising) // True if at trough
    {
        minWaterLevel = applyRollingAverage(waterLevelSmooth, minWaterLevel, EXTREMA_SMOOTH_FACTOR);
    }

    // Calculate normalized water level
    waterLevelNorm = float(waterLevelSmooth - minWaterLevel) / float(maxWaterLevel - minWaterLevel);

    // Apply sigmoid function
    waterLevelNorm = 1.0 / (1.0 + exp(-SIGMOID_FACTOR * (waterLevelNorm - 0.5)));

    //Serial.printf("%d %d %d\n", minWaterLevel, maxWaterLevel, waterLevelSmooth);

    // Update static variables
    waterLevelSmoothPrev = waterLevelSmooth;
    isRisingPrev = isRising;
}

// Apply a rolling average to a value
int applyRollingAverage(const int new_value, const int current_average, const int rollingAverageSize)
{
    return (current_average * (rollingAverageSize - 1) + new_value) / rollingAverageSize;
}
