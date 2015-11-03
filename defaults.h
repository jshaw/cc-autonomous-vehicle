//Public defaults for the car

#define NUM_SENSORS             6  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             2  // emitter is controlled by digital pin 2

//For the start button
#define START_BUTTON_PIN 8

#define KP .2
#define KD 5
#define M1_DEFAULT_SPEED 80
#define M2_DEFAULT_SPEED 80
#define M1_MAX_SPEED 120
#define M2_MAX_SPEED 120
#define DEBUG 0 // set to 1 if serial debug output needed
