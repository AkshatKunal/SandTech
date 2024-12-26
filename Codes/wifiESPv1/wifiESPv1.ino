#include <WiFi.h>

// Wi-Fi credentials
const char* ssid = "DC_5G";
const char* password = "Since2015";

// GPIO pin for the LED
const int ledPin = 14; // GPIO14 (D5 equivalent)

// Variables to store LED state and brightness
int ledState = LOW;
int brightness = 0; // PWM value (0 to 255)

// PWM channel configuration
const int pwmChannel = 0;
const int pwmFrequency = 5000;
const int pwmResolution = 8;

WiFiServer server(80); // Start a web server on port 80

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);

  // // Configure PWM
  // ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
  // ledcAttachPin(ledPin, pwmChannel);
  // ledcWrite(pwmChannel, brightness); // Set initial brightness

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi Connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.begin(); // Start the server
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    Serial.println("New Client Connected");
    String request = client.readStringUntil('\r');
    Serial.println(request);
    client.flush();

    // Parse the HTTP request
    if (request.indexOf("/led/on") != -1) {
      ledState = HIGH;
      brightness = 255;
    } else if (request.indexOf("/led/off") != -1) {
      ledState = LOW;
      brightness = 0;
    } else if (request.indexOf("/brightness?value=") != -1) {
      int start = request.indexOf("value=") + 6;
      int end = request.indexOf(" ", start);
      brightness = request.substring(start, end).toInt();
      brightness = constrain(brightness, 0, 255); // Keep within bounds
    }

    // Update LED state and brightness
    // ledcWrite(pwmChannel, brightness);

    // HTML response
    String response = R"====(
      <!DOCTYPE html>
      <html>
      <head>
        <title>ESP32 LED Control</title>
        <style>
          body { font-family: Arial; text-align: center; margin-top: 20px; }
          button, input[type="range"] { margin: 10px; padding: 10px; font-size: 18px; }
        </style>
      </head>
      <body>
        <h1>ESP32 LED Control</h1>
        <p>LED is currently: )====";
    response += (ledState == HIGH) ? "ON" : "OFF";
    response += R"====(</p>
        <button onclick="location.href='/led/on'">Turn ON</button>
        <button onclick="location.href='/led/off'">Turn OFF</button>
        <p>Adjust Brightness:</p>
        <input type="range" min="0" max="255" value=")====";
    response += String(brightness);
    response += R"====(" onchange="updateBrightness(this.value)">
        <script>
          function updateBrightness(val) {
            fetch('/brightness?value=' + val);
          }
        </script>
      </body>
      </html>
    )====";

    client.print("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
    client.print(response);
    client.stop();
    Serial.println("Client Disconnected");
  }
}
