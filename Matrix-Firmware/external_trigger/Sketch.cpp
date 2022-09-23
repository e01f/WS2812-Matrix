#include <Arduino.h>

#include <Adafruit_NeoMatrix.h>
#include <gamma.h>
#include <Fonts/TomThumb.h>

#include <WS2812FX.h>

#define LED_COUNT 176
#define LED_PIN 8
#define ANALOG_PIN A0

void advanceAniColor();
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
Adafruit_NeoMatrix neoMatrix = Adafruit_NeoMatrix(16, 11, LED_PIN, NEO_MATRIX_TOP + NEO_MATRIX_RIGHT + NEO_MATRIX_COLUMNS + NEO_MATRIX_PROGRESSIVE, NEO_GRB + NEO_KHZ800);

enum sysState_e {SYS_INIT, SYS_SHOWCAPTION, SYS_SHOWCAPTION_WAIT, SYS_INFO, SYS_INFO_WAIT, SYS_INFO_DRAW, SYS_ANI_WAIT, SYS_ANI};
typedef enum sysState_e sysState_t;

enum aniState_e {ANI_OFF, ANI1, ANI2, ANI3, ANI_MODE_NUM};
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

sysState_t sysState;
aniState_t aniState;

int ani_trg_count = 0;
int aniColorIndex = 1;
uint16_t aniColor = 0x0FF0DD;

const double trigger_factor = 0.7;

void setup() {
	now = millis();
	
	neoMatrix.begin();
	neoMatrix.setFont(&TomThumb);
	neoMatrix.setTextWrap(false);
	neoMatrix.setTextColor(neoMatrix.Color(255, 255, 255));
	neoMatrix.setBrightness(brightness);
	neoMatrix.fillScreen(0);
	
	//Serial.begin(9600);
	
	pinMode(3, INPUT_PULLUP);

	lastStateChange = now;
	strcpy_P(caption1, PSTR("Init"));
	strcpy_P(caption2, PSTR("v1.1"));
}

inline double approxRollingAverage(double avg, double new_sample, double N) {
	avg -= avg / N;
	avg += new_sample / N;
	return avg;
}

inline void handleButton() {
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
					if (aniState < ANI_MODE_NUM - 1) {
						aniState = (aniState_t) (((int) aniState) + 1);
					} else {
						aniState = ANI_OFF;
					}
					strcpy_P(caption1, PSTR("Mode"));
					itoa(aniState, caption2, 10);
					sysState = SYS_SHOWCAPTION;
				}
			}
			last_btn_evt = now;
		}
	}
}

inline void handleTrigger() {
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
			ani_trg_count++;
			advanceAniColor();
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
}

inline void refreshScreen() {
	// redraw each 30 ms (approx. 30 fps)
	neoMatrix.show();
}

void advanceAniColor() {
	switch (aniColorIndex) {
		case 1:
			aniColor = neoMatrix.Color(255, 0, 0);
			break;
		case 2:
			aniColor = neoMatrix.Color(0, 255, 0);
			break;
		case 3:
			aniColor = neoMatrix.Color(0, 0, 255);
			break;
		case 4:
			aniColor = neoMatrix.Color(255, 255, 0);
			break;
		case 5:
			aniColor = neoMatrix.Color(255, 0, 255);
			break;
		case 6:
			aniColor = neoMatrix.Color(0, 255, 255);
			aniColorIndex = 0;
			break;
	}
	aniColorIndex++;
}

inline void runAnimations() {
	double span;
	double relVal;
	
	span = avg_max - avg_min;
	relVal = min(1.0, abs(avgAnalog - sample) * 2.0 / span);
	
	neoMatrix.fillScreen(0);
	
	switch (aniState) {
		case ANI1:
			// rect, size proportional to sound level
			if (ani_trg_count > neoMatrix.width()) {
				ani_trg_count = 0;
			}
			for (int i = 0; i < ani_trg_count; i++) {
				neoMatrix.drawFastVLine(i, 0, neoMatrix.height(), neoMatrix.Color(255, 255, 255));
			}
			//neoMatrix.drawRect(round(8.0 - relVal * 8.0), round(5.5 - relVal * 5.5), round(relVal * 16.0), round(relVal * 11.0), neoMatrix.Color(255, 255, 255));
			break;
		case ANI2:
			// filled rect, size proportional to sound level
			neoMatrix.fillRect(round(8.0 - relVal * 8.0), round(5.5 - relVal * 5.5), round(relVal * 16.0), round(relVal * 11.0), aniColor);
			break;
		case ANI3:
			// mini squares, alternating colors
			uint16_t oldAniColor = aniColor;
			for (int x = 0; x < neoMatrix.width(); x += 3) {
				for (int y = 0; y < neoMatrix.height(); y += 3) {
					neoMatrix.fillRect(x, y, 3, 3, aniColor);
					advanceAniColor();
				}
			}
			aniColor = oldAniColor;
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
			neoMatrix.fillScreen(0);
			neoMatrix.setBrightness(25);
			neoMatrix.setCursor(0, 5);
			neoMatrix.print(caption1);
			neoMatrix.drawFastHLine(0, 5, 16, neoMatrix.Color(255,0,0));
			neoMatrix.setCursor(0, 11);
			neoMatrix.print(caption2);
			sysState = SYS_SHOWCAPTION_WAIT;
			break;
		case SYS_SHOWCAPTION_WAIT:
			if (now - lastStateChange > 1000) {
				neoMatrix.fillScreen(0);
				neoMatrix.setBrightness(brightness);
				sysState = SYS_ANI;
			}
			break;
		case SYS_INFO:
			neoMatrix.fillScreen(0);
			neoMatrix.setBrightness(25);
			neoMatrix.setCursor(0, 5);
			strcpy_P(caption1, PSTR("Line"));
			neoMatrix.print(caption1);
			neoMatrix.drawFastHLine(0, 5, 16, neoMatrix.Color(255,0,0));
			sysState = SYS_INFO_DRAW;
			break;
		case SYS_INFO_WAIT:
			if (now - lastStateChange > 30) {
				sysState = SYS_INFO_DRAW;
			}
			break;
		case SYS_INFO_DRAW:
			neoMatrix.fillRect(0, 7, 16, 3, neoMatrix.Color(0,0,0));
			// Show a line, corresp: 0, 1024, avg, min, max, trigmin, trigmax
			neoMatrix.drawFastHLine(0, 8, 16, neoMatrix.Color(255,255,255));
			pxAvg = round(avgAnalog / 1023.0 * 16.0);
			pxMin = round(avg_min / 1023.0 * 16.0);
			pxMax = round(avg_max / 1023.0 * 16.0);
			pxTrgMin = round((avgAnalog - (((double) (avg_max - avg_min)) / 2.0 * trigger_factor)) / 1023.0 * 16.0);
			pxTrgMax = round((avgAnalog + (((double) (avg_max - avg_min)) / 2.0 * trigger_factor)) / 1023.0 * 16.0);
			pxSpl = round(sample / 1023.0 * 16.0);
			
			neoMatrix.drawPixel(pxAvg, 7, neoMatrix.Color(0,255,0));
			neoMatrix.drawFastHLine(pxAvg - abs(pxAvg - pxSpl), 8, 2 * abs(pxAvg - pxSpl) + 1, neoMatrix.Color(0,255,0));
			//ws2812fx.drawPixel(pxAvg, 8, ws2812fx.Color(0,255,0));
			neoMatrix.drawPixel(pxAvg, 9, neoMatrix.Color(0,255,0));
			
			neoMatrix.drawPixel(pxMin, 7, neoMatrix.Color(0,0,255));
			neoMatrix.drawPixel(pxMax, 7, neoMatrix.Color(255,0,0));
			neoMatrix.drawPixel(pxTrgMin, 9, neoMatrix.Color(0,0,255));			
			neoMatrix.drawPixel(pxTrgMax, 9, neoMatrix.Color(255,0,0));
			
			if (now - last_trigger < 100) {
				neoMatrix.drawPixel(pxAvg, 8, neoMatrix.Color(255,0,255));
			}
			sysState = SYS_INFO_WAIT;
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
	
	if (now - last_sample > 1) {
		sampleInput();
		last_sample = now;
	}
	
	if (now - last_minmax > 2000) {
		computeMinMax();
		last_minmax = now;
	}
	
	handleTrigger();
	
	if (now - last_draw > 30) {
		refreshScreen();
		last_draw = now;
	}
	
	runSystem();

	/*if (now - last_statusreport > 2000) {
		printDebugInfo();
		last_statusreport = now;
	};*/

	handleButton();
}
