#include <esp_now.h>
#include <WiFi.h>

// Pin definitions
#define BUTTON_PIN_ON 13    // Pin for "ON" button
#define BUTTON_PIN_OFF 12   // Pin for "OFF" button
#define LED_PIN 14          // Pin for LED output

// Structure for data to be sent/received
typedef struct {
  int button_status;  // Status of the button press
} Message;

// Initialize variables
Message myData;
bool newData = false;  // Flag to keep track of new data to send
bool ledStatus = false;  // Current status of the LED

// Universal MAC Address (broadcast address)
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  // Copy incoming data to our struct
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Button status received: ");
  Serial.println(myData.button_status);
  ledStatus = myData.button_status;  // Update LED status based on received data
  digitalWrite(LED_PIN, ledStatus);  // Set LED accordingly
}

void setup() {
  Serial.begin(115200);  // Start serial communication
  pinMode(BUTTON_PIN_ON, INPUT_PULLUP);   // Configure "ON" button pin
  pinMode(BUTTON_PIN_OFF, INPUT_PULLUP);  // Configure "OFF" button pin
  pinMode(LED_PIN, OUTPUT);   // Configure LED pin

  // Set device as Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register send callback function
  esp_now_register_send_cb(OnDataSent);

  // Register receive callback function
  esp_now_register_recv_cb(OnDataRecv);

  // Add broadcast peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  // Check button states and set data to send
  if (digitalRead(BUTTON_PIN_ON) == LOW && !newData) {
    myData.button_status = 1;  // Set button status to ON
    newData = true;  // Flag that new data is ready
  }
  if (digitalRead(BUTTON_PIN_OFF) == LOW && !newData) {
    myData.button_status = 0;  // Set button status to OFF
    newData = true;  // Flag that new data is ready
  }

  // Send data via ESP-NOW
  if (newData) {
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    if (result == ESP_OK) {
      Serial.println("Data sent successfully");
    } else {
      Serial.println("Error sending data");
    }
    newData = false;  // Reset flag after sending data
  }

  delay(100); // Adjust delay as needed
}
