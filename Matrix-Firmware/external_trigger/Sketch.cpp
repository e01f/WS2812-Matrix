#include <Arduino.h>

#include <Adafruit_NeoMatrix.h>
#include <gamma.h>
#include <Fonts/TomThumb.h>

#include <WS2812FX.h>

#define LED_COUNT 176
#define LED_PIN 8
#define ANALOG_PIN A0

double approxRollingAverage(double avg, double new_sample, double N);

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = Neo_Matrix options concerning the layout of the matrix as LEDs
// Parameter 4 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bit stream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bit stream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bit stream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bit stream (NeoPixel RGBW products)
WS2812FX ws2812fx = WS2812FX(16, 11, LED_PIN, NEO_MATRIX_TOP + NEO_MATRIX_RIGHT + NEO_MATRIX_COLUMNS + NEO_MATRIX_PROGRESSIVE, NEO_GRB + NEO_KHZ800);

enum screenowner_e {OWNER_WS2812FX, OWNER_MATRIX};
typedef enum screenowner_e screenowner_t;

enum sysState_e {SYS_INIT, SYS_SHOWCAPTION, SYS_SHOWCAPTION_WAIT, SYS_INFO, SYS_INFO_WAIT, SYS_INFO_DRAW, SYS_GFX, SYS_ANI_WAIT, SYS_ANI};
typedef enum sysState_e sysState_t;

enum aniState_e {ANI_OFF, ANI1, ANI2, ANI_MODE_NUM};
typedef enum aniState_e aniState_t;

unsigned long last_trigger = 0;
unsigned long last_sample = 0;
unsigned long last_draw = 0;
unsigned long last_modechg = 0;
unsigned long last_minmax = 0;
unsigned long last_statusreport = 0;
unsigned long last_btn_evt = 0;

unsigned long lastStateChange = 0;

int modectr = 0;
unsigned long now = 0;
double avgAnalog = 512.0;
int sample;
int min_sample = 1024;
int max_sample = 0;
double avg_min = 300.0;
double avg_max = 500.0;
double avg_trigger_interval = 500.0;
int button_state = HIGH;
uint8_t brightness = 100;

char caption1[6];
char caption2[6];

screenowner_t screenowner;
sysState_t sysState;
aniState_t aniState;

const double trigger_factor = 0.7;

void setup() {
	now = millis();
	
	ws2812fx.init();
	ws2812fx.setFont(&TomThumb);
	ws2812fx.setTextWrap(false);
	ws2812fx.setTextColor(ws2812fx.Color(255, 255, 255));
	ws2812fx.setBrightness(brightness);
	ws2812fx.setMode(FX_MODE_STATIC);
	
	//Serial.begin(9600);
	
	pinMode(3, INPUT_PULLUP);

	lastStateChange = now;
	strcpy_P(caption1, PSTR("Init"));
	strcpy_P(caption2, PSTR("v1.0"));
}

inline double approxRollingAverage(double avg, double new_sample, double N) {
	avg -= avg / N;
	avg += new_sample / N;
	return avg;
}

inline void handleButton() {
	bool transit_owner = false;
	bool ex_transit_owner = false;
	if (now - last_btn_evt > 100) {
		int pb = digitalRead(3);
		if (pb != button_state) {
			button_state = pb;
			if (button_state == HIGH) {
				if (now - last_btn_evt > 2000) {
					// long press
					sysState = SYS_INFO;
				} else {
					// short press
					do {
						ex_transit_owner = transit_owner;
						transit_owner = false;
						if (aniState == ANI_OFF) {
							int mode = ws2812fx.getMode();
							int modecount = ws2812fx.getModeCount();
							bool skip = false;
							if (!ex_transit_owner) do {
								skip = false;
								if (mode <  modecount - 1) {
									mode++;
								} else {
									mode = 0;
									screenowner = OWNER_MATRIX;
									aniState = ANI1;
									transit_owner = true;
								}
								switch (mode) {
									case FX_MODE_STATIC:
									case FX_MODE_BREATH:
									case FX_MODE_COLOR_WIPE:
									case FX_MODE_COLOR_WIPE_INV:
									case FX_MODE_COLOR_WIPE_REV:
									case FX_MODE_COLOR_WIPE_REV_INV:
									case FX_MODE_COLOR_WIPE_RANDOM:
									case FX_MODE_SINGLE_DYNAMIC:
									case FX_MODE_RAINBOW:
									case FX_MODE_RAINBOW_CYCLE:
									case FX_MODE_SCAN:
									case FX_MODE_DUAL_SCAN:
									case FX_MODE_FADE:
									case FX_MODE_THEATER_CHASE:
									case FX_MODE_THEATER_CHASE_RAINBOW:
									case FX_MODE_RUNNING_LIGHTS:
									case FX_MODE_CHASE_WHITE:
									case FX_MODE_CHASE_COLOR:
									case FX_MODE_CHASE_RANDOM:
									case FX_MODE_CHASE_RAINBOW:
									case FX_MODE_CHASE_FLASH:
									case FX_MODE_CHASE_FLASH_RANDOM:
									case FX_MODE_CHASE_RAINBOW_WHITE:
									case FX_MODE_CHASE_BLACKOUT:
									case FX_MODE_CHASE_BLACKOUT_RAINBOW:
									case FX_MODE_COLOR_SWEEP_RANDOM:
									case FX_MODE_RUNNING_COLOR:
									case FX_MODE_RUNNING_RED_BLUE:
									case FX_MODE_RUNNING_RANDOM:
									case FX_MODE_LARSON_SCANNER:
									case FX_MODE_COMET:
									case FX_MODE_FIREWORKS:
									case FX_MODE_FIREWORKS_RANDOM:
									case FX_MODE_FIRE_FLICKER:
									case FX_MODE_FIRE_FLICKER_SOFT:
									case FX_MODE_FIRE_FLICKER_INTENSE:
									case FX_MODE_HALLOWEEN:
									case FX_MODE_BICOLOR_CHASE:
									case FX_MODE_TRICOLOR_CHASE:
									skip = true;
									break;
								}
							} while (skip);
							ws2812fx.setMode(mode);
							itoa(mode, caption2, 10);
							//strncpy_P(caption2, (char*)pgm_read_ptr(ws2812fx.getModeName(mode)), 4);
						} else {
							// ANI Mode, next
							if (!ex_transit_owner) {
								if (aniState < ANI_MODE_NUM - 1) {
									aniState = (aniState_t) (((int) aniState) + 1);
								} else {
									aniState = ANI_OFF;
									screenowner = OWNER_WS2812FX;
									transit_owner = true;
								}
							}
							strcpy_P(caption2, PSTR("A-"));
							itoa(aniState, caption2 + 2, 10);
						}
					} while (transit_owner);
					strcpy_P(caption1, PSTR("Mode"));
					sysState = SYS_SHOWCAPTION;
				}
			}
			last_btn_evt = now;
		}
	}
}

inline void computeMinMax() {
	// conservative: +/-120 to avoid spurious triggers
	avg_min = min(approxRollingAverage(avg_min, min_sample, 4), avgAnalog - 300.0);
	avg_max = max(approxRollingAverage(avg_max, max_sample, 4), avgAnalog + 300.0);
	min_sample = 1024;
	max_sample = 0;
}

inline void sampleInput() {
	sample = analogRead(ANALOG_PIN);
	avgAnalog = approxRollingAverage(avgAnalog, sample, 1000);
	min_sample = min(min_sample, sample);
	max_sample = max(max_sample, sample);
	//Serial.println(avgAnalog);
	int trigger_interval = now - last_trigger;
	if (trigger_interval > 150) {
		double local_trigger_factor = trigger_factor;
		if (abs(trigger_interval - avg_trigger_interval) < 20) {
			// Increase sensitivity if regularity detected
			local_trigger_factor *= 0.8;
		}
		if (abs(sample - avgAnalog) > (((double) (avg_max - avg_min)) / 2.0 * local_trigger_factor)) {
			avg_trigger_interval = approxRollingAverage(avg_trigger_interval, trigger_interval, 4);
			last_trigger = now;
		}
	}
}

inline void refreshScreen() {
	// redraw each 30 ms (approx. 30 fps)
	switch (screenowner) {
		case OWNER_WS2812FX:
			ws2812fx.service();
			break;
		case OWNER_MATRIX:
			ws2812fx.show();
			break;
	}
}

inline void runAnimations() {
	double span;
	double relVal;
	
	span = avg_max - avg_min;
	relVal = min(1.0, abs(avgAnalog - sample) * 2.0 / span);
	
	//ws2812fx.fillScreen(0);
	ws2812fx.fillRect(0, 0, 16, 11, ws2812fx.Color(0,0,0));
	
	switch (aniState) {
		case ANI1:
			// rect, size proportional to sound level
			//ws2812fx.drawRect(round(8.0 - relVal * 8.0), round(5.5 - relVal * 5.5), round(relVal * 16.0), round(relVal * 11.0), ws2812fx.Color(255, 255, 255));
			//break;
		case ANI2:
			// filled rect, size proportional to sound level
			ws2812fx.fillRect(round(8.0 - relVal * 8.0), round(5.5 - relVal * 5.5), round(relVal * 16.0), round(relVal * 11.0), ws2812fx.Color(255, 255, 255));
			break;
	}
}

inline void printDebugInfo() {
	// Serial debug info
	Serial.print("avgAnalog = ");
	Serial.print(avgAnalog);
	Serial.print(", trigger: [");
	Serial.print(avgAnalog - (((double) (avg_max - avg_min)) / 2.0 * trigger_factor));
	Serial.print(", ");
	Serial.print(avgAnalog + (((double) (avg_max - avg_min)) / 2.0 * trigger_factor));
	Serial.print("], was: [");
	Serial.print(avg_min);
	Serial.print(", ");
	Serial.print(avg_max);
	Serial.print("], delta = ");
	Serial.println(avg_max - avg_min);
}

inline void runSystem() {
	sysState_t oldState = sysState;
	int pxAvg;
	int pxMin;
	int pxMax;
	int pxTrgMin;
	int pxTrgMax;
	int pxSpl;

	switch (sysState) {
		case SYS_INIT:
			// ...
			sysState = SYS_SHOWCAPTION;
			break;
		case SYS_SHOWCAPTION:
			// show initial captions...
			screenowner = OWNER_MATRIX;
			ws2812fx.fillScreen(0);
			ws2812fx.setBrightness(25);
			ws2812fx.setCursor(0, 5);
			ws2812fx.print(caption1);
			ws2812fx.drawFastHLine(0, 5, 16, ws2812fx.Color(255,0,0));
			ws2812fx.setCursor(0, 11);
			ws2812fx.print(caption2);
			//ws2812fx.show();
			sysState = SYS_SHOWCAPTION_WAIT;
			break;
		case SYS_SHOWCAPTION_WAIT:
			// wait dly
			if (now - lastStateChange > 1000) {
				ws2812fx.fillScreen(0);
				ws2812fx.setBrightness(brightness);
				if (aniState == ANI_OFF) {
					sysState = SYS_GFX;
				} else {
					sysState = SYS_ANI;
				}
			}
			break;
		case SYS_INFO:
			ws2812fx.fillScreen(0);
			ws2812fx.setBrightness(25);
			ws2812fx.setCursor(0, 5);
			strcpy_P(caption1, PSTR("Line"));
			ws2812fx.print(caption1);
			ws2812fx.drawFastHLine(0, 5, 16, ws2812fx.Color(255,0,0));
			sysState = SYS_INFO_DRAW;
			break;
		case SYS_INFO_WAIT:
			if (now - lastStateChange > 30) {
				sysState = SYS_INFO_DRAW;
			}
			break;
		case SYS_INFO_DRAW:
			screenowner = OWNER_MATRIX;
			ws2812fx.fillRect(0, 7, 16, 3, ws2812fx.Color(0,0,0));
			// Show a line, corresp: 0, 1024, avg, min, max, trigmin, trigmax
			ws2812fx.drawFastHLine(0, 8, 16, ws2812fx.Color(255,255,255));
			pxAvg = round(avgAnalog / 1023.0 * 16.0);
			pxMin = round(avg_min / 1023.0 * 16.0);
			pxMax = round(avg_max / 1023.0 * 16.0);
			pxTrgMin = round((avgAnalog - (((double) (avg_max - avg_min)) / 2.0 * trigger_factor)) / 1023.0 * 16.0);
			pxTrgMax = round((avgAnalog + (((double) (avg_max - avg_min)) / 2.0 * trigger_factor)) / 1023.0 * 16.0);
			pxSpl = round(sample / 1023.0 * 16.0);
			
			ws2812fx.drawPixel(pxAvg, 7, ws2812fx.Color(0,255,0));
			ws2812fx.drawFastHLine(pxAvg - abs(pxAvg - pxSpl), 8, 2 * abs(pxAvg - pxSpl) + 1, ws2812fx.Color(0,255,0));
			//ws2812fx.drawPixel(pxAvg, 8, ws2812fx.Color(0,255,0));
			ws2812fx.drawPixel(pxAvg, 9, ws2812fx.Color(0,255,0));
			
			ws2812fx.drawPixel(pxMin, 7, ws2812fx.Color(0,0,255));
			ws2812fx.drawPixel(pxMax, 7, ws2812fx.Color(255,0,0));
			ws2812fx.drawPixel(pxTrgMin, 9, ws2812fx.Color(0,0,255));			
			ws2812fx.drawPixel(pxTrgMax, 9, ws2812fx.Color(255,0,0));
			
			if (now - last_trigger < 100) {
				ws2812fx.drawPixel(pxAvg, 8, ws2812fx.Color(255,0,255));
			}
			//ws2812fx.show();
			/*if (now - lastStateChange > 20000) {
				ws2812fx.fillScreen(0);
				ws2812fx.setBrightness(brightness);
				sysState = SYS_GFX;
			}*/
			sysState = SYS_INFO_WAIT;
			break;
		case SYS_GFX:
			// show GFX
			screenowner = OWNER_WS2812FX;
			// trigger, if analog value is above threshold
			// this comes in handy, when using a microphone on analog input
			if (now == last_trigger) {
				ws2812fx.trigger();
			}
			break;
		case SYS_ANI_WAIT:
			if (now - lastStateChange > 30) {
				sysState = SYS_ANI;
			}
			break;
		case SYS_ANI:
			runAnimations();
			sysState = SYS_ANI_WAIT;
			break;
	}

	if (oldState != sysState) {
		lastStateChange = now;
	}
}

void loop() {
	now = millis();
	
	if (now - last_draw > 30) {
		refreshScreen();
		last_draw = now;
	}
	
	// trigger on a regular basis
	/*
	if (now - last_trigger > 5000) {
		ws2812fx.trigger();
		last_trigger = now;
	}*/
	
	if (now - last_sample > 1) {
		sampleInput();
		last_sample = now;
	}

	if (now - last_minmax > 2000) {
		computeMinMax();
		last_minmax = now;
	}
	
	runSystem();

	/*if (now - last_statusreport > 2000) {
		printDebugInfo();
		last_statusreport = now;
	};*/

	handleButton();
}
