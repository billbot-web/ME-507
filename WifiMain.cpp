#include <WiFi.h>
#include <WebServer.h>

// Wi-Fi credentials
const char* ssid = "Buttmunch";
const char* password = "360islay";

// Web server on port 80
WebServer server(80);

// webpage response
void handleRoot() {
  server.send(200, "text/plain", "ESP32 is online!");
}

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Route for “/”
  server.on("/", handleRoot);

  // Start web server
  server.begin();
  Serial.println("Web server started");
}

void loop() {
  server.handleClient(); // Keep server running
}