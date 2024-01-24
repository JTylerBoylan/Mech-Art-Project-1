#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>

#define WIFI_SSID "Mine"
#define WIFI_PASS "12345678"

#define UDP_LOCAL_PORT 8888

#define UDP_REMOTE_IP 172,20,10,3
#define UDP_REMOTE_PORT 9999

#define PWM_PIN 13
#define PWM_CHANNEL 0
#define PWM_FREQUENCY 1000
#define PWM_RESOLUTION 8

#define WATER_LEVEL_PIN 33

#define MOTOR_SPEED_OSC_TOPIC "/1/faderA"
#define WATER_LEVEL_OSC_TOPIC "/water_level"

#define WATER_LEVEL_AVG_TIGHT 5
#define WATER_LEVEL_AVG_LOOSE 10

#define NUMBER_SKIPS_AT_START 5

WiFiUDP Udp;

// Function declarations
void connectToWiFi(const char* ssid, const char* pass);
void beginLocalUdp(unsigned int localPort);

void startPWM(int pin, int channel, int frequency, int resolution);

void readOSC();
void sendOSC(const char* topic, float value);

void recieveMotorSpeedOSC(OSCMessage &msg);

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

    Serial.println("Setup complete");
}

// Run the ESP32
void loop()
{
    static int num_skips = 0;

    // Read incoming OSC messages
    readOSC();

    // Read current water level
    int waterLevel = analogRead(WATER_LEVEL_PIN);

    // Parse water level
    bool is_extrema;
    float waterLevelNorm;
    parseWaterLevel(waterLevel, is_extrema, waterLevelNorm);

    // Send water level OSC message
    if (is_extrema)
    {
        // Skip first few readings
        if (num_skips >= NUMBER_SKIPS_AT_START)
        {
            sendOSC(WATER_LEVEL_OSC_TOPIC, waterLevelNorm);

            Serial.print("Water level %: ");
            Serial.println(waterLevelNorm * 100);
        }
        else
        {
            num_skips++;
        }
    }

    // Delay 25ms
    delay(25);
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
    OSCMessage msg(topic);
    msg.add(value);

    Udp.beginPacket(IPAddress(UDP_REMOTE_IP), UDP_REMOTE_PORT);
    msg.send(Udp);
    Udp.endPacket();
}

// Parse motor speed OSC message
void recieveMotorSpeedOSC(OSCMessage &msg) {
    // Assuming the message contains a float
    float value = msg.getFloat(0);

    // Convert to PWM value (0-255)
    int pwmVal = int(value * 256);

    // Write PWM to channel
    ledcWrite(PWM_CHANNEL, pwmVal);

    //Serial.print("Motor speed: ");
    //Serial.println(pwmVal);
}

// Parse water level
void parseWaterLevel(int waterLevel, bool &is_extrema, float &waterLevelNorm)
{
    // Static variables to track rolling averages
    static int waterLevelAvgTight = 0;
    static int waterLevelAvgLoose = 0;
    static bool is_rising_last = false;

    // Static variables to track extrema
    static int maxWaterLevel = 0;
    static int minWaterLevel = 1E6;

    // Static variables to track extrema within a period
    static int maxWaterLevelLocal = 0;
    static int minWaterLevelLocal = 1E6;

    // Update rolling averages (Or set if first reading)
    if (waterLevelAvgLoose == 0)
        waterLevelAvgLoose = waterLevel;
    if (waterLevelAvgTight == 0)
        waterLevelAvgTight = waterLevel;
    waterLevelAvgTight = applyRollingAverage(waterLevel, waterLevelAvgTight, WATER_LEVEL_AVG_TIGHT);
    waterLevelAvgLoose = applyRollingAverage(waterLevel, waterLevelAvgLoose, WATER_LEVEL_AVG_LOOSE);

    // Update global extrema
    if (waterLevelAvgTight < minWaterLevel)
        minWaterLevel = waterLevelAvgTight;
    if (waterLevelAvgTight > maxWaterLevel)
        maxWaterLevel = waterLevelAvgTight;

    // Update local extrema
    bool is_rising = waterLevelAvgTight > waterLevelAvgLoose;
    if (is_rising)
    {
        // If it's rising, find local max
        if (waterLevelAvgTight > maxWaterLevelLocal)
            maxWaterLevelLocal = waterLevelAvgTight;
    }
    else
    {
        // If it's falling, find local min
        if (waterLevelAvgTight < minWaterLevelLocal)
            minWaterLevelLocal = waterLevelAvgTight;
    }

    if (is_rising && !is_rising_last) // True if trough
    {
        // If it's a trough, the norm is calculated from the local min
        waterLevelNorm = float(minWaterLevelLocal - minWaterLevel) / float(maxWaterLevel - minWaterLevel);
        
        // Reset local extrema
        maxWaterLevelLocal = 0;
        is_extrema = true;
    }
    else if (!is_rising && is_rising_last) // True if peak
    {
        // If it's a peak, the norm is calculated from the local max
        waterLevelNorm = float(maxWaterLevelLocal - minWaterLevel) / float(maxWaterLevel - minWaterLevel);
        
        // Reset local extrema
        minWaterLevelLocal = 1E6;
        is_extrema = true;
    }
    else
    {
        // If it's not an extrema, the norm is 0
        waterLevelNorm = 0.0;
        is_extrema = false;
    }

/*
    Serial.print(waterLevel);
    Serial.print(" ");
    Serial.print(waterLevelAvgTight);
    Serial.print(" ");
    Serial.print(waterLevelAvgLoose);
    Serial.print(" ");
    Serial.print(minWaterLevel);
    Serial.print(" ");
    Serial.println(maxWaterLevel);
    //Serial.print(" ");
    //Serial.println(waterLevelNorm);
*/

    is_rising_last = is_rising;
}

// Apply a rolling average to a value
int applyRollingAverage(const int new_value, const int current_average, const int rollingAverageSize)
{
    return (current_average * (rollingAverageSize - 1) + new_value) / rollingAverageSize;
}
