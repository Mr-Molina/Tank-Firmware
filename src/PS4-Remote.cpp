#include "PS4-Remote.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"

// These connect to the settings in main.cpp
extern int DEADZONE_VALUE;
extern int GYRO_DEADZONE_VALUE;
extern int ACC_DEADZONE_VALUE;
extern int ACC_PRECISION_VALUE;
extern int USE_ACCELEROMETER_VALUE;
extern int EVENTS_VALUE;
extern int DEBUG_PS4_DATA_VALUE;  // Should we show controller data on the screen?

// This helps the callback functions find our PS4Remote object
PS4Remote* PS4Remote::_instance = nullptr;

// Constructor - Sets up a new PS4Remote object
PS4Remote::PS4Remote() {
    // Initialize timestamps
    lastTimeStamp = 0;
    
    // Initialize controller state (all buttons unpressed, joysticks centered)
    memset(&currentState, 0, sizeof(ControllerState));
    memset(&prevState, 0, sizeof(ControllerState));
    
    // Set static instance for callbacks
    setInstance(this);
}

// This helps callback functions find our PS4Remote object
void PS4Remote::setInstance(PS4Remote* instance) {
    _instance = instance;
}

// Begin - Sets up the PS4 controller connection
void PS4Remote::begin() {
    Serial.println("PS4Remote::begin - Starting initialization");
    
    // Set up callback functions that run when controller events happen
    PS4.attach(notifyCallback);
    Serial.println("PS4Remote::begin - Attached notify callback");
    PS4.attachOnConnect(onConnectCallback);
    Serial.println("PS4Remote::begin - Attached connect callback");
    PS4.attachOnDisconnect(onDisconnectCallback);
    Serial.println("PS4Remote::begin - Attached disconnect callback");
    
    // Start the PS4 controller (turns on Bluetooth)
    PS4.begin();
    Serial.println("PS4Remote::begin - PS4 controller initialized");
    
    // Disabled to prevent potential crashes
    // removePairedDevices();
    
    // Show this device's Bluetooth address
    Serial.print("This device MAC is: ");
    printDeviceAddress();
    Serial.println("");

    // Print header for data columns
    Serial.println("\n--- PS4 Controller Data ---");
    Serial.println("PS4Remote::begin - Initialization complete");
}

// Get the current state of all buttons and joysticks
PS4Remote::ControllerState PS4Remote::getState() {
    return currentState;
}

// Static callback functions that forward to instance methods
void PS4Remote::notifyCallback() {
    if (_instance) {
        _instance->update();
    }
}

// This runs when the controller connects
void PS4Remote::onConnectCallback() {
    Serial.println("Connected!");
}

// This runs when the controller disconnects
void PS4Remote::onDisconnectCallback() {
    Serial.println("Disconnected!");
}

// Function to apply deadzone to joystick values
// This ignores small movements when the joystick is near the center
int PS4Remote::applyDeadzone(int value, int deadzone) {
    // If the movement is smaller than the deadzone, ignore it
    if (abs(value) < deadzone) {
        return 0;  // Return 0 (centered position)
    }
    return value;  // Return the actual value
}

// Function to reduce precision of accelerometer values
// This makes tilt readings less jumpy and easier to work with
int PS4Remote::reduceAccPrecision(int value) {
    // First apply deadzone (ignore very small tilts)
    if (abs(value) < DEFAULT_ACC_DEADZONE) {
        return 0;  // Return 0 (no tilt)
    }
    
    // Then reduce precision by dividing and multiplying
    // This rounds the value to the nearest multiple of DEFAULT_ACC_PRECISION
    return (value / DEFAULT_ACC_PRECISION) * DEFAULT_ACC_PRECISION;
}

// Update - Reads the latest data from the controller
void PS4Remote::update() {
    // Reset change flag
    currentState.dataChanged = false;

    // Only process at a reasonable rate (every 50 milliseconds)
    if (millis() - lastTimeStamp > 50) {
        // Get button states - Face buttons
        currentState.square = PS4.Square();
        currentState.triangle = PS4.Triangle();
        currentState.cross = PS4.Cross();
        currentState.circle = PS4.Circle();

        // D-pad buttons
        currentState.up = PS4.Up();
        currentState.down = PS4.Down();
        currentState.left = PS4.Left();
        currentState.right = PS4.Right();

        // Shoulder buttons and triggers
        currentState.l1 = PS4.L1();
        currentState.r1 = PS4.R1();
        currentState.l2 = PS4.L2Value(); // Analog trigger (0-255)
        currentState.r2 = PS4.R2Value(); // Analog trigger (0-255)

        // Thumbstick buttons and other buttons
        currentState.l3 = PS4.L3();
        currentState.r3 = PS4.R3();
        currentState.share = PS4.Share();
        currentState.options = PS4.Options();
        currentState.ps = PS4.PSButton();
        currentState.touchpad = PS4.Touchpad();

        // Get and process joystick values
        currentState.lx = applyDeadzone(PS4.LStickX(), DEADZONE_VALUE);
        currentState.ly = applyDeadzone(PS4.LStickY(), DEADZONE_VALUE);
        currentState.rx = applyDeadzone(PS4.RStickX(), DEADZONE_VALUE);
        currentState.ry = applyDeadzone(PS4.RStickY(), DEADZONE_VALUE);

        // Get and process gyroscope values (rotation)
        currentState.gx = applyDeadzone(PS4.GyrX(), GYRO_DEADZONE_VALUE);
        currentState.gy = applyDeadzone(PS4.GyrY(), GYRO_DEADZONE_VALUE);
        currentState.gz = applyDeadzone(PS4.GyrZ(), GYRO_DEADZONE_VALUE);

        // Get and process accelerometer values (tilt)
        currentState.ax = 0;
        currentState.ay = 0;
        currentState.az = 0;
        if (USE_ACCELEROMETER_VALUE) {
            currentState.ax = reduceAccPrecision(PS4.AccX());
            currentState.ay = reduceAccPrecision(PS4.AccY());
            currentState.az = reduceAccPrecision(PS4.AccZ());
        }

        // Check if any values have changed since last update
        bool valuesChanged = (
            currentState.square != prevState.square || 
            currentState.triangle != prevState.triangle || 
            currentState.cross != prevState.cross || 
            currentState.circle != prevState.circle ||
            currentState.up != prevState.up || 
            currentState.down != prevState.down || 
            currentState.left != prevState.left || 
            currentState.right != prevState.right ||
            currentState.l1 != prevState.l1 || 
            currentState.r1 != prevState.r1 || 
            currentState.l2 != prevState.l2 || 
            currentState.r2 != prevState.r2 ||
            currentState.l3 != prevState.l3 || 
            currentState.r3 != prevState.r3 || 
            currentState.share != prevState.share || 
            currentState.options != prevState.options ||
            currentState.ps != prevState.ps || 
            currentState.touchpad != prevState.touchpad ||
            currentState.lx != prevState.lx || 
            currentState.ly != prevState.ly || 
            currentState.rx != prevState.rx || 
            currentState.ry != prevState.ry ||
            currentState.gx != prevState.gx || 
            currentState.gy != prevState.gy || 
            currentState.gz != prevState.gz
        );

        // Also check accelerometer values if they're enabled
        if (USE_ACCELEROMETER_VALUE) {
            valuesChanged = valuesChanged || (
                currentState.ax != prevState.ax || 
                currentState.ay != prevState.ay || 
                currentState.az != prevState.az
            );
        }

        // If anything changed, update our state and maybe show debug info
        if (valuesChanged) {
            currentState.dataChanged = true;

            // Only print debug data if enabled
            if (DEBUG_PS4_DATA_VALUE) {
                // Print all data in a standardized format with columns
                // Face buttons
                Serial.printf("BTN: SQ:%-3s TR:%-3s X:%-3s O:%-3s | ",
                              currentState.square ? "ON" : "OFF",
                              currentState.triangle ? "ON" : "OFF",
                              currentState.cross ? "ON" : "OFF",
                              currentState.circle ? "ON" : "OFF");
    
                // D-pad
                Serial.printf("DPAD: U:%-3s D:%-3s L:%-3s R:%-3s | ",
                              currentState.up ? "ON" : "OFF",
                              currentState.down ? "ON" : "OFF",
                              currentState.left ? "ON" : "OFF",
                              currentState.right ? "ON" : "OFF");
    
                // Shoulder buttons and triggers
                Serial.printf("SHLD: L1:%-3s R1:%-3s L2:%3d R2:%3d | ",
                              currentState.l1 ? "ON" : "OFF",
                              currentState.r1 ? "ON" : "OFF",
                              currentState.l2,
                              currentState.r2);
    
                // Thumbstick buttons (L3/R3)
                Serial.printf("THUMB: L3:%-3s R3:%-3s | ",
                              currentState.l3 ? "ON" : "OFF",
                              currentState.r3 ? "ON" : "OFF");
    
                // Joystick positions
                Serial.printf("JOY: LX:%4d LY:%4d RX:%4d RY:%4d | ",
                              currentState.lx, currentState.ly, 
                              currentState.rx, currentState.ry);
    
                // Gyroscope (rotation) values
                Serial.printf("GYRO: X:%5d Y:%5d Z:%5d",
                              currentState.gx, currentState.gy, currentState.gz);
    
                // Accelerometer (tilt) values, if enabled
                if (USE_ACCELEROMETER_VALUE) {
                    Serial.printf(" | ACC: X:%5d Y:%5d Z:%5d",
                                  currentState.ax, currentState.ay, currentState.az);
                }
    
                Serial.println(); // End the line
            }

            // Update previous state to remember what changed
            prevState = currentState;
        }

        // Remember when we last updated
        lastTimeStamp = millis();
    }

    // Handle button events (press and release) if enabled
    if (EVENTS_VALUE) {
        // Check which buttons were just pressed or released
        boolean sqd = PS4.event.button_down.square,   // Square button pressed
                squ = PS4.event.button_up.square,     // Square button released
                trd = PS4.event.button_down.triangle, // Triangle button pressed
                tru = PS4.event.button_up.triangle,   // Triangle button released
                crd = PS4.event.button_down.cross,    // Cross button pressed
                cru = PS4.event.button_up.cross,      // Cross button released
                cid = PS4.event.button_down.circle,   // Circle button pressed
                ciu = PS4.event.button_up.circle,     // Circle button released
                upd = PS4.event.button_down.up,       // Up button pressed
                upu = PS4.event.button_up.up,         // Up button released
                dnd = PS4.event.button_down.down,     // Down button pressed
                dnu = PS4.event.button_up.down,       // Down button released
                ltd = PS4.event.button_down.left,     // Left button pressed
                ltu = PS4.event.button_up.left,       // Left button released
                rtd = PS4.event.button_down.right,    // Right button pressed
                rtu = PS4.event.button_up.right,      // Right button released
                l1d = PS4.event.button_down.l1,       // L1 button pressed
                l1u = PS4.event.button_up.l1,         // L1 button released
                r1d = PS4.event.button_down.r1,       // R1 button pressed
                r1u = PS4.event.button_up.r1;         // R1 button released

        // Check thumbstick button events
        boolean l3d = PS4.event.button_down.l3,       // L3 button pressed
                l3u = PS4.event.button_up.l3,         // L3 button released
                r3d = PS4.event.button_down.r3,       // R3 button pressed
                r3u = PS4.event.button_up.r3;         // R3 button released

        // If any button events happened, maybe show debug info
        if (sqd || squ || trd || tru || crd || cru || cid || ciu ||
            upd || upu || dnd || dnu || ltd || ltu || rtd || rtu ||
            l1d || l1u || r1d || r1u || l3d || l3u || r3d || r3u) {
            
            // Only print event debug data if enabled
            if (DEBUG_PS4_DATA_VALUE) {
                Serial.printf("EVENT: ");
                // Face buttons
                if (sqd) Serial.printf("SQUARE DOWN ");
                if (squ) Serial.printf("SQUARE UP ");
                if (trd) Serial.printf("TRIANGLE DOWN ");
                if (tru) Serial.printf("TRIANGLE UP ");
                if (crd) Serial.printf("CROSS DOWN ");
                if (cru) Serial.printf("CROSS UP ");
                if (cid) Serial.printf("CIRCLE DOWN ");
                if (ciu) Serial.printf("CIRCLE UP ");
    
                // D-pad
                if (upd) Serial.printf("UP DOWN ");
                if (upu) Serial.printf("UP UP ");
                if (dnd) Serial.printf("DOWN DOWN ");
                if (dnu) Serial.printf("DOWN UP ");
                if (ltd) Serial.printf("LEFT DOWN ");
                if (ltu) Serial.printf("LEFT UP ");
                if (rtd) Serial.printf("RIGHT DOWN ");
                if (rtu) Serial.printf("RIGHT UP ");
    
                // Shoulder buttons
                if (l1d) Serial.printf("L1 DOWN ");
                if (l1u) Serial.printf("L1 UP ");
                if (r1d) Serial.printf("R1 DOWN ");
                if (r1u) Serial.printf("R1 UP ");
    
                // Thumbstick buttons
                if (l3d) Serial.printf("L3 DOWN ");
                if (l3u) Serial.printf("L3 UP ");
                if (r3d) Serial.printf("R3 DOWN ");
                if (r3u) Serial.printf("R3 UP ");
    
                Serial.println();
            }
        }
    }
}

// Remove any previously paired controllers
// (This is disabled to prevent potential crashes)
void PS4Remote::removePairedDevices() {
    uint8_t pairedDeviceBtAddr[20][6];
    int count = esp_bt_gap_get_bond_device_num();
    esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
    for (int i = 0; i < count; i++) {
        esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
    }
}

// Show this device's Bluetooth address
// This is useful for pairing the controller
void PS4Remote::printDeviceAddress() {
    const uint8_t *point = esp_bt_dev_get_address();
    for (int i = 0; i < 6; i++) {
        char str[3];
        sprintf(str, "%02x", (int)point[i]);
        Serial.print(str);
        if (i < 5) {
            Serial.print(":");
        }
    }
}