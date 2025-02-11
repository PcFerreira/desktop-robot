#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <FluxGarage_RoboEyes.h>
#include <WiFi.h>
#include <WebServer.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// Constants
#define EEPROM_SIZE 512
#define LIGHT_SENSOR 34
#define AUDIO_SENSOR 36  // KY-038 AO (Analog Out)
#define BUTTON_PIN 15
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

// Number of levels for each sensor
#define LIGHT_LEVELS 5
#define AUDIO_LEVELS 5
#define WIFI_LEVELS 5
#define BT_LEVELS 5

// Total number of states and actions
#define NUM_STATES (LIGHT_LEVELS * AUDIO_LEVELS * WIFI_LEVELS * BT_LEVELS)
#define NUM_ACTIONS 11

// Learning parameters
int8_t Q_table[NUM_STATES][NUM_ACTIONS] = {0};
int state_visit_counts[NUM_STATES] = {0}; // Track state visit counts for novelty-based rewards
float alpha = 0.5;              // Learning rate
float discount_factor = 0.9;    // Discount factor

// FreeRTOS Task Handles
TaskHandle_t TaskSensorHandle, TaskDecisionHandle, TaskActionHandle, TaskFeedbackHandle, TaskDisplayHandle;

// Shared variables between tasks
volatile int current_state = 0;
volatile int selected_action = 0;
volatile float latest_reward = 0.0;
volatile int last_non_default_action = -1;
volatile bool autoMode = true; // Default to auto mode
volatile bool autoLearn = true; // Default to auto-learning enabled

// Mutex for Q-table access
SemaphoreHandle_t qTableMutex;

// OLED Display Setup
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
roboEyes roboEyes;

// Enum Definitions (Avoiding conflicts with macros)
enum Mood {
    MOOD_DEFAULT,
    MOOD_TIRED,
    MOOD_ANGRY,
    MOOD_HAPPY,
    MOOD_SAD,
    MOOD_SURPRISED,
    MOOD_EXCITED
};

// Function Prototypes
void setMood(Mood mood);
void anim_wink();
void anim_rolling_eyes();

// Wi-Fi and Server Setup
const char* ssid = "SET_WIFI_SSID";
const char* password = "SET_WIFI_PASSWORD";
WebServer server(80);

void connectToWiFi() {
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

void handleAction() {
    if (!server.hasArg("action")) {
        server.send(400, "text/plain", "Missing 'action' parameter");
        return;
    }

    int action = server.arg("action").toInt();
    if (action < 0 || action >= NUM_ACTIONS) {
        server.send(400, "text/plain", "Invalid action");
        return;
    }

    perform_action(action); // Perform the requested action
    server.send(200, "text/plain", "Action performed");
}

void handleFeedback() {
    if (!server.hasArg("reward")) {
        server.send(400, "text/plain", "Missing 'reward' parameter");
        return;
    }

    float reward = server.arg("reward").toFloat();

    xSemaphoreTake(qTableMutex, pdMS_TO_TICKS(100));
    latest_reward = reward;
    update_q_table(current_state, selected_action, current_state, latest_reward);
    xSemaphoreGive(qTableMutex);

    server.send(200, "text/plain", "Feedback received");
}

void handleState() {
    String response = "Current State: " + String(current_state);
    server.send(200, "text/plain", response);
}

void handleMode() {
    if (server.hasArg("mode")) {
        String mode = server.arg("mode");
        if (mode == "auto") {
            autoMode = true;
            server.send(200, "text/plain", "Mode set to AUTO");
        } else if (mode == "manual") {
            autoMode = false;
            server.send(200, "text/plain", "Mode set to MANUAL");
        } else {
            server.send(400, "text/plain", "Invalid mode. Use 'auto' or 'manual'");
        }
    } else {
        // Return current mode
        String response = autoMode ? "AUTO" : "MANUAL";
        server.send(200, "text/plain", response);
    }
}

void handleGetQTable() {
    String qTableData = "";
    for (int i = 0; i < NUM_STATES; i++) {
        for (int j = 0; j < NUM_ACTIONS; j++) {
            qTableData += String(Q_table[i][j]);
            if (j < NUM_ACTIONS - 1 || i < NUM_STATES - 1) {
                qTableData += ",";
            }
        }
    }
    server.send(200, "text/plain", qTableData);
}

void handleUploadQTable() {
    if (!server.hasArg("q_table")) {
        server.send(400, "text/plain", "Missing 'q_table' parameter");
        return;
    }

    String qTableData = server.arg("q_table");
    int index = 0;
    char* token = strtok((char*)qTableData.c_str(), ",");
    while (token != NULL && index < NUM_STATES * NUM_ACTIONS) {
        Q_table[index / NUM_ACTIONS][index % NUM_ACTIONS] = atof(token);
        token = strtok(NULL, ",");
        index++;
    }

    if (index != NUM_STATES * NUM_ACTIONS) {
        server.send(400, "text/plain", "Invalid Q-table size");
    } else {
        server.send(200, "text/plain", "Q-table uploaded successfully");
    }
}

void handleAutoLearn() {
    if (server.hasArg("enable")) {
        String enable = server.arg("enable");
        if (enable == "true") {
            autoLearn = true;
            server.send(200, "text/plain", "Auto-learning enabled");
        } else if (enable == "false") {
            autoLearn = false;
            server.send(200, "text/plain", "Auto-learning disabled");
        } else {
            server.send(400, "text/plain", "Invalid value. Use 'true' or 'false'");
        }
    } else {
        // Return current auto-learning status
        String response = autoLearn ? "ENABLED" : "DISABLED";
        server.send(200, "text/plain", response);
    }
}

void setupServer() {
    server.on("/action", HTTP_GET, handleAction);      // Trigger an action
    server.on("/feedback", HTTP_GET, handleFeedback);  // Provide feedback
    server.on("/state", HTTP_GET, handleState);        // Get current state
    server.on("/mode", HTTP_GET, handleMode);          // Toggle auto/manual mode
    server.on("/qtable", HTTP_GET, handleGetQTable);   // Get Q-table
    server.on("/upload_qtable", HTTP_POST, handleUploadQTable); // Upload Q-table
    server.on("/autolearn", HTTP_GET, handleAutoLearn); // Enable/disable auto-learning
    server.begin();
    Serial.println("HTTP server started");
}

void setup() {
    Serial.begin(115200);
    pinMode(LIGHT_SENSOR, INPUT);
    pinMode(AUDIO_SENSOR, INPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Initialize OLED display
    Wire.begin(21, 22);
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("SSD1306 allocation failed!");
        while (1);
    }
    display.clearDisplay();
    display.display();

    // Initialize Robo Eyes
    roboEyes.setHeight(30,30);
    roboEyes.begin(SCREEN_WIDTH, SCREEN_HEIGHT, 120);
    roboEyes.setPosition(DEFAULT);
    roboEyes.setAutoblinker(ON, 3, 2);

    // Connect to Wi-Fi and start the server
    connectToWiFi();
    setupServer();

    BLEDevice::init("");
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setActiveScan(true);

    // Create FreeRTOS tasks
    qTableMutex = xSemaphoreCreateMutex();
    xTaskCreate(TaskReadSensors, "TaskReadSensors", 2048, NULL, 1, &TaskSensorHandle);
    xTaskCreate(TaskDecisionMaking, "TaskDecisionMaking", 2048, NULL, 2, &TaskDecisionHandle);
    xTaskCreate(TaskPerformAction, "TaskPerformAction", 2048, NULL, 1, &TaskActionHandle);
    xTaskCreate(TaskUserFeedback, "TaskUserFeedback", 2048, NULL, 1, &TaskFeedbackHandle);
    xTaskCreate(TaskUpdateDisplay, "TaskUpdateDisplay", 2048, NULL, 1, &TaskDisplayHandle);
}

// Normalize audio sensor with smoothing and random jitter for frequent variations
int get_audio_level(int audio_temp) {
    static int last_audio_level = 0; // Track the last level for smoothing
    const int SMOOTHING_FACTOR = 2;  // Controls how much smoothing is applied
    const int JITTER_RANGE = 1;      // Introduces small random variations

    // Normalize the raw audio value into AUDIO_LEVELS (e.g., 10 levels)
    int raw_audio_level = map(audio_temp, 0, 4095, 0, AUDIO_LEVELS - 1);

    // Apply smoothing to reduce abrupt changes
    int smoothed_audio_level = (raw_audio_level + last_audio_level * SMOOTHING_FACTOR) / (SMOOTHING_FACTOR + 1);

    // Add random jitter to encourage frequent variations
    int jitter = random(-JITTER_RANGE, JITTER_RANGE + 1); // Random value between -1 and 1
    int final_audio_level = constrain(smoothed_audio_level + jitter, 0, AUDIO_LEVELS - 1);

    // Update the last audio level for the next iteration
    last_audio_level = final_audio_level;

    return final_audio_level;
}

// Normalize Wi-Fi signal into WIFI_LEVELS (e.g., 10 levels)
int get_wifi_level(int wifi_signal) {
    static int last_wifi_level = 0; // Track the last level for smoothing
    const int SMOOTHING_FACTOR = 2; // Controls how much smoothing is applied
    const int JITTER_RANGE = 1;     // Introduces small random variations

    // Define the observed range of Wi-Fi networks (e.g., 0–9)
    const int WIFI_MIN = 0;
    const int WIFI_MAX = 9;

    // Normalize the raw Wi-Fi count into WIFI_LEVELS
    int raw_wifi_level = map(wifi_signal, WIFI_MIN, WIFI_MAX, 0, WIFI_LEVELS - 1);

    // Apply smoothing to reduce abrupt changes
    int smoothed_wifi_level = (raw_wifi_level + last_wifi_level * SMOOTHING_FACTOR) / (SMOOTHING_FACTOR + 1);

    // Add random jitter to encourage frequent variations
    int jitter = random(-JITTER_RANGE, JITTER_RANGE + 1); // Random value between -1 and 1
    int final_wifi_level = constrain(smoothed_wifi_level + jitter, 0, WIFI_LEVELS - 1);

    // Update the last Wi-Fi level for the next iteration
    last_wifi_level = final_wifi_level;

    return final_wifi_level;
}

// Normalize Bluetooth signal into BT_LEVELS (e.g., 10 levels)
int get_bt_level(int bt_signal) {
    static int last_bt_level = 0; // Track the last level for smoothing
    const int SMOOTHING_FACTOR = 2; // Controls how much smoothing is applied
    const int JITTER_RANGE = 1;     // Introduces small random variations

    // Define the observed range of Bluetooth devices (e.g., 0–14)
    const int BT_MIN = 0;
    const int BT_MAX = 14; // add to global variable

    // Normalize the raw Bluetooth count into BT_LEVELS
    int raw_bt_level = map(bt_signal, BT_MIN, BT_MAX, 0, BT_LEVELS - 1);

    // Apply smoothing to reduce abrupt changes
    int smoothed_bt_level = (raw_bt_level + last_bt_level * SMOOTHING_FACTOR) / (SMOOTHING_FACTOR + 1);

    // Add random jitter to encourage frequent variations
    int jitter = random(-JITTER_RANGE, JITTER_RANGE + 1); // Random value between -1 and 1
    int final_bt_level = constrain(smoothed_bt_level + jitter, 0, BT_LEVELS - 1);

    // Update the last Bluetooth level for the next iteration
    last_bt_level = final_bt_level;

    return final_bt_level;
}

// Task: Read sensor data
void TaskReadSensors(void *pvParameters) {
    while (1) {
        int light = analogRead(LIGHT_SENSOR);
        int audio_temp = analogRead(AUDIO_SENSOR);
        int wifi_signal = get_wifi_signal_level();
        int bt_signal = get_bt_signal_level();

        int audio = get_audio_level(audio_temp);
        int wifi_state = get_wifi_level(wifi_signal);
        int bt_state = get_bt_level(bt_signal);

        // Combine all sensor values into a single state
        current_state = get_discrete_state(light, audio, wifi_state, bt_state);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// Map sensor values to discrete states
int get_discrete_state(int light, int audio, int wifi_signal, int bt_signal) {
    // Normalize light sensor (0–4095) into 10 levels
    int light_state = map(light, 0, 4095, 0, LIGHT_LEVELS - 1);
    Serial.print("light_state: ");
    Serial.println(light_state);

    // Normalize audio sensor (0–4095) into 10 levels
    //int audio_state = map(audio, 0, 4095, 0, AUDIO_LEVELS - 1);
    Serial.print("audio_state: ");
    Serial.println(audio);

    Serial.print("wifi_signal: ");
    Serial.println(wifi_signal);

    Serial.print("bt_state: ");
    Serial.println(bt_signal);

    // Combine all states into a single index
    return (light_state * AUDIO_LEVELS * WIFI_LEVELS * BT_LEVELS) +
           (audio * WIFI_LEVELS * BT_LEVELS) +
           (wifi_signal * BT_LEVELS) +
           bt_signal;
}

// Get Wi-Fi signal level
int get_wifi_signal_level() {
    int network_count = WiFi.scanNetworks();
    return network_count;
}

// Get Bluetooth signal level
int get_bt_signal_level() {
    BLEScan* pBLEScan = BLEDevice::getScan();
    BLEScanResults* results = pBLEScan->start(1, false); // Returns a pointer to BLEScanResults
    int device_count = results->getCount();              // Use the pointer to get the count
    pBLEScan->clearResults();                            // Clear the scan results to free memory
    return device_count;
}

// Calculate intrinsic reward based on novelty
float calculate_intrinsic_reward(int state) {
    return 1.0 / (1.0 + state_visit_counts[state]);
}

// Update Q-table using unsupervised learning
void update_q_table_unsupervised(int state, int action) {
    int next_state = current_state; // Assume the next state is the current state

    // Update state visit count
    state_visit_counts[next_state]++;

    // Calculate intrinsic reward
    float intrinsic_reward = calculate_intrinsic_reward(next_state);

    // Find the best Q-value for the next state
    float best_next_q = -9999;
    for (int i = 0; i < NUM_ACTIONS; i++) {
        if (Q_table[next_state][i] > best_next_q) {
            best_next_q = Q_table[next_state][i];
        }
    }

    // Update Q-table using intrinsic reward
    Q_table[state][action] += alpha * (intrinsic_reward + discount_factor * best_next_q - Q_table[state][action]);
}

// Task: Decision Making
void TaskDecisionMaking(void *pvParameters) {
    while (1) {
        if (autoMode) {
            xSemaphoreTake(qTableMutex, pdMS_TO_TICKS(100));
            selected_action = choose_action(current_state);
            xSemaphoreGive(qTableMutex);

            // Update Q-table only if auto-learning is enabled
            if (autoLearn) {
                update_q_table_unsupervised(current_state, selected_action);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Select action (Dynamic Exploration)
int choose_action(int state) {
    static int last_action = -1;
    static int repeat_count = 0;
    static float exploration_rate = 0.5;
    float maxQ = -9999;
    int best_action = 0;

    // Find the action with the highest Q-value
    for (int i = 0; i < NUM_ACTIONS; i++) {
        if (Q_table[state][i] > maxQ) {
            maxQ = Q_table[state][i];
            best_action = i;
        }
    }

    // Penalize repetitive actions
    if (best_action == last_action) {
        repeat_count++;
        if (repeat_count > 3) {
            Q_table[state][best_action] *= 0.9;
            Serial.println("Repetitive action detected, penalizing...");
        }
    } else {
        repeat_count = 0;
    }

    // Dynamic exploration (ε-Greedy)
    if (random(0, 100) < (exploration_rate * 100)) {
        best_action = random(0, NUM_ACTIONS);
    }

    // Gradually reduce exploration rate
    exploration_rate *= 0.995;
    if (exploration_rate < 0.1) exploration_rate = 0.1;
    last_action = best_action;
    return best_action;
}

// Task: Perform Actions
void TaskPerformAction(void *pvParameters) {
    while (1) {
        perform_action(selected_action);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Perform an action based on the selected action
void perform_action(int action) {
    if (action != 0) last_non_default_action = action;
    roboEyes.setCuriosity(OFF);
    switch (action) {
        case 0:
            roboEyes.setMood(DEFAULT);
            roboEyes.setPosition(DEFAULT); // Reset eyes to center
            break;
        case 1:
            roboEyes.setPosition(DEFAULT);
            roboEyes.setMood(ANGRY);
            roboEyes.anim_confused();
            break;
        case 2:
            roboEyes.setPosition(DEFAULT);
            roboEyes.setMood(HAPPY);
            roboEyes.anim_laugh();
            break;
        case 3:
            roboEyes.setPosition(DEFAULT);
            roboEyes.setMood(TIRED);
            anim_sleepy();
            break;
        case 4:
            roboEyes.setPosition(DEFAULT);
            setMood(MOOD_SAD);
            roboEyes.blink();
            break;
        case 5:
            roboEyes.setPosition(DEFAULT);
            setMood(MOOD_SURPRISED);
            anim_wink();
            break;
        case 6:
            roboEyes.setPosition(DEFAULT);
            setMood(MOOD_EXCITED);
            anim_rolling_eyes();
            break;
        case 7:
            roboEyes.setCuriosity(ON);
            roboEyes.setPosition(W); // Look left
            break;
        case 8:
            roboEyes.setCuriosity(ON);
            roboEyes.setPosition(E); // Look right
            break;
        case 9:
            roboEyes.setCuriosity(ON);
            roboEyes.setPosition(N); // Look up
            break;
        case 10:
            roboEyes.setCuriosity(ON);
            roboEyes.setPosition(S); // Look up
            break;
        default:
            Serial.println("Unknown action!");
            roboEyes.anim_confused();
            break;
    }
    roboEyes.update();
}

// Task: Handle User Feedback
void TaskUserFeedback(void *pvParameters) {
    bool lastButtonState = HIGH;
    while (1) {
        bool currentButtonState = digitalRead(BUTTON_PIN);
        if (currentButtonState == LOW && lastButtonState == HIGH) {
            vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
            if (digitalRead(BUTTON_PIN) == LOW) {
                unsigned long pressDuration = millis();
                while (digitalRead(BUTTON_PIN) == LOW); // Wait for release
                pressDuration = millis() - pressDuration;
                // Multi-level feedback
                float reward = (pressDuration > 1000) ? -5.0 : 5.0;
                Serial.print("Button Pressed: Reward = ");
                Serial.println(reward);
                xSemaphoreTake(qTableMutex, pdMS_TO_TICKS(100));
                latest_reward = reward;
                update_q_table(current_state, selected_action, current_state, latest_reward);
                xSemaphoreGive(qTableMutex);
                vTaskDelay(pdMS_TO_TICKS(2000)); // Prevent repeated triggers
            }
        }
        lastButtonState = currentButtonState;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Update Q-table
void update_q_table(int state, int action, int next_state, float reward) {
    float best_next_q = -9999;
    for (int i = 0; i < NUM_ACTIONS; i++) {
        if (Q_table[next_state][i] > best_next_q) {
            best_next_q = Q_table[next_state][i];
        }
    }
    Q_table[state][action] += alpha * (reward + discount_factor * best_next_q - Q_table[state][action]);
}

// Task: Update Display
void TaskUpdateDisplay(void *pvParameters) {
    while (1) {
        roboEyes.update();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Set Mood Function
void setMood(Mood mood) {
    switch (mood) {
        case MOOD_DEFAULT:
            roboEyes.setPosition(DEFAULT);
            roboEyes.open();
            break;
        case MOOD_TIRED:
            roboEyes.setPosition(SW);
            roboEyes.close(1, 0);
            roboEyes.close(0, 1);
            break;
        case MOOD_ANGRY:
            roboEyes.setPosition(N);
            roboEyes.setCuriosity(OFF);
            roboEyes.anim_confused();
            break;
        case MOOD_HAPPY:
            roboEyes.setPosition(S);
            roboEyes.anim_laugh();
            break;
        case MOOD_SAD:
            roboEyes.setPosition(SW);
            roboEyes.blink();
            break;
        case MOOD_SURPRISED:
            roboEyes.setPosition(DEFAULT);
            anim_wink(); // Custom implementation
            break;
        case MOOD_EXCITED:
            roboEyes.setPosition(DEFAULT);
            anim_rolling_eyes(); // Custom implementation
            break;
    }
}

// Custom Animations
void anim_wink() {
    autoMode = false;
    roboEyes.close(1, 0); // Close one eye
    vTaskDelay(pdMS_TO_TICKS(200));
    roboEyes.open();      // Open both eyes
    autoMode = true;
}

void anim_rolling_eyes() {
    autoMode = false;
    roboEyes.setPosition(N);
    vTaskDelay(pdMS_TO_TICKS(100));
    roboEyes.setPosition(NE);
    vTaskDelay(pdMS_TO_TICKS(100));
    roboEyes.setPosition(E);
    vTaskDelay(pdMS_TO_TICKS(100));
    roboEyes.setPosition(SE);
    vTaskDelay(pdMS_TO_TICKS(100));
    roboEyes.setPosition(S);
    vTaskDelay(pdMS_TO_TICKS(100));
    roboEyes.setPosition(SW);
    vTaskDelay(pdMS_TO_TICKS(100));
    roboEyes.setPosition(W);
    vTaskDelay(pdMS_TO_TICKS(100));
    roboEyes.setPosition(NW);
    vTaskDelay(pdMS_TO_TICKS(100));
    roboEyes.setPosition(DEFAULT);
    autoMode = true;
}

void anim_sleepy() {
    autoMode = false;
    for (int i = 0; i < 5; i++) {
        roboEyes.close();
        vTaskDelay(pdMS_TO_TICKS(200));
        roboEyes.open();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    autoMode = true;
}

void loop() {
    server.handleClient(); // Handle incoming HTTP requests
    vTaskDelay(pdMS_TO_TICKS(10)); // Yield to FreeRTOS tasks
}