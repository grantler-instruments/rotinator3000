// uint8_t broadcastAddress[6] = { 0xCC, 0x8D, 0xA2, 0x91, 0xF0, 0xCC };
uint8_t broadcastAddress[6] = { 0xCC, 0x8D, 0xA2, 0x8B, 0x85, 0x1C };

#define MIDI_CHANNEL 11

const unsigned long sendInterval = 16;  // 10ms interval for sending MIDI data
float speedSensitivity = 360.0;         // Maximum rotation speed in degrees/second

#define NUMBER_OF_BUTTONS 5
int buttonPins[] = { 10, 11, 12, 13, 14 };
#define NUMBER_OF_PIXELS 4
#define PIN_NEOPIXEL 6