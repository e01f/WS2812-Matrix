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
bool calculateBallPath();

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

enum aniState_e {ANI_OFF, ANI1, ANI2, ANI3, ANI4, ANI5, ANI_MODE_NUM};
typedef enum aniState_e aniState_t;

struct pointXY_s {
	double x;
	double y;
};
typedef struct pointXY_s pointXY_t;

struct ballVector_s {
	pointXY_t start;
	double k;
	pointXY_t v;
	uint8_t quadrant;
	double angle;
};
typedef struct ballVector_s ballVector_t;

struct waypoint_s {
	double distance;
	ballVector_t bV;	
};
typedef struct waypoint_s waypoint_t;

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
int last_ani_trg_count = 0;
int aniColorIndex = 1;
uint16_t aniColor = 0x0FF0DD;
uint16_t aniParams[6];
uint16_t lastAniParams[6];

double ballLeftY = 5.0;
double ballRightY = 6.0;
double paddleHitOffset = 0.0;
ballVector_t ball;
pointXY_t ballPos;
// start -> [top/bottom 1 -> [top/bottom 2 ->]] sidewall
waypoint_t ballPath[4];
uint8_t ballPathLength;

double vuValues[16];

const double trigger_factor = 0.7;

ballVector_t ballVector(pointXY_t p, double angle);

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
	
	ballPathLength = 1;
	ballPath[0].distance = 0.0;
	ballPath[0].bV = ballVector({0, 4}, radians(-5.0));
		
	ball = ballPath[0].bV;
}

double approxRollingAverage(double avg, double new_sample, double N) {
	avg -= avg / N;
	avg += new_sample / N;
	return avg;
}

void handleButton() {
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

void handleTrigger() {
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

void computeMinMax() {
	// conservative: +/-120 to avoid spurious triggers
	avg_min = min(approxRollingAverage(avg_min, min_sample, 4), avgAnalog - 300.0);
	avg_max = max(approxRollingAverage(avg_max, max_sample, 4), avgAnalog + 300.0);
	min_sample = 1024;
	max_sample = 0;
}

void sampleInput() {
	sample = analogRead(ANALOG_PIN);
	avgAnalog = approxRollingAverage(avgAnalog, sample, 1000);
	min_sample = min(min_sample, sample);
	max_sample = max(max_sample, sample);
	//Serial.println(avgAnalog);
}

bool _compareAniParams(uint16_t *current, uint16_t *last, int length, bool aggResult) {
	aggResult &= *current == *last;
	if (length <= 1) return aggResult;
	return _compareAniParams(current + 1, last + 1, length - 1, aggResult);
}

bool compareAniParams(uint16_t *current, uint16_t *last, int length) {
	return _compareAniParams(current, last, length, true);
}

bool screenUpdateRequired() {
	// Diagnostic state, no need to cut down on updates
	if (sysState != SYS_ANI && sysState != SYS_ANI_WAIT) return true;
	// aniParams != lastAniParams
	return !compareAniParams(aniParams, lastAniParams, sizeof(aniParams) / sizeof(aniParams[0]));
}

void refreshScreen() {
	// redraw each 30 ms (approx. 30 fps), whenever this is needed
	if (screenUpdateRequired()) {
		neoMatrix.show();
		for (int i = 0; i < sizeof(aniParams) / sizeof(aniParams[0]); i++) {
			lastAniParams[i] = aniParams[i];
		}
	}
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

void saveAniParams(uint16_t param1, uint16_t param2, uint16_t param3, uint16_t param4, uint16_t param5) {
	aniParams[1] = param1;
	aniParams[2] = param2;
	aniParams[3] = param3;
	aniParams[4] = param4;
	aniParams[5] = param5;
}

void saveAniParams(uint16_t param1, uint16_t param2, uint16_t param3) {
	saveAniParams(param1, param2, param3, 0xFFFF, 0xFFFF);
}

void saveAniParams(uint16_t param) {
	saveAniParams(param, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF);
}

void saveAniState() {
	aniParams[0] = aniState;
}

bool isPointInMatrix(pointXY_t p) {
	if (p.x < 0) return false;
	if (p.x > neoMatrix.width() - 1) return false;
	if (p.y < 0) return false;
	if (p.y > neoMatrix.height() - 1) return false;
	return true;
}

double euclidDist(pointXY_t a, pointXY_t b) {
	return sqrt(sq(a.x - b.x) + sq(a.y - b.y));
}

ballVector_t yIntersectBallVector(ballVector_t v, pointXY_t intersectPoint) {
	// intersect ball vector horizontally at y
	intersectPoint.x = (intersectPoint.y - v.start.y + v.k * v.start.x) / v.k;
	return ballVector(intersectPoint, TWO_PI - v.angle);
}

ballVector_t xIntersectBallVector(ballVector_t v, pointXY_t intersectPoint) {
	// intersect ball vector vertically at x
	intersectPoint.y = intersectPoint.x * v.k - v.k * v.start.x + v.start.y;
	return ballVector(intersectPoint, PI - v.angle);
}

ballVector_t ballVector(pointXY_t p, double angle) {
	// create new ball vector from p at horz. angle angle
	ballVector_t newBall;
	newBall.start.x = p.x;
	newBall.start.y = p.y;
	newBall.angle = angle;
	
	// angle is to the left, from right horizon
	if (angle < HALF_PI) newBall.quadrant = 1;
	else if (angle < PI) newBall.quadrant = 2;
	else if (angle < PI + HALF_PI) newBall.quadrant = 3;
	else newBall.quadrant = 4;
	
	// k > 0 rising, k < 0 falling (valuewise)
	newBall.k = tan(angle);
	
	// compute the normalized (length == 1) vector
	// special case: 0 and 180 degrees (1, 0) and (-1, 0)
	if (angle == 0) {
		newBall.v.x = 1.0;
		newBall.v.y = 0;
	} else if (angle == PI) {
		newBall.v.x = -1.0;
		newBall.v.y = 0;
	} else {	
		// the normalized vector stems from the linear equation, which is now valid
		newBall.v.x = 1.0;
		newBall.v.y = newBall.k;
	
		switch (newBall.quadrant) {
			case 2:
			case 3:
			// down, left and up, left need special treatment
			newBall.v.x *= -1.0;
			newBall.v.y *= -1.0;
			break;
		}
	
		// normalize the vector
		double norm = euclidDist({0.0, 0.0}, newBall.v);
		newBall.v.x *= (1 / norm);
		newBall.v.y *= (1 / norm);
	}
	
	return newBall;
}

void addWaypoint(waypoint_t *path, uint8_t *pathIndex, double distance, ballVector_t bV) {
	path[*pathIndex].distance = distance;
	path[*pathIndex].bV = bV;
	(*pathIndex)++;
}

bool calculateBallPath() {
	// return ball path, starting at ball
	double pathLength = 0.0;
	ballVector_t particle = ball;
	pointXY_t intersectPoint = particle.start;
	ballVector_t intersect;
	bool sideHit;
	uint8_t pathIndex = 0;
	// add start to path
	addWaypoint(ballPath, &pathIndex, pathLength, particle);
	do {
		ballVector_t xIntersect;
		ballVector_t yIntersect;
		// Prepare intersect box
		switch (particle.quadrant) {
			case 1:
				// down, right
				xIntersect.start.x = neoMatrix.width() - 1;
				yIntersect.start.y = neoMatrix.height() - 1;
				break;
			case 2:
				// down, left
				xIntersect.start.x = 0;
				yIntersect.start.y = neoMatrix.height() - 1;
				break;
			case 3:
				// up, left
				xIntersect.start.x = 0;
				yIntersect.start.y = 0;
				break;
			case 4:
				// up, right
				xIntersect.start.x = neoMatrix.width() - 1;
				yIntersect.start.y = 0;
				break;
		}
		
		// calculate intersection point, calculate bounce
		xIntersect = xIntersectBallVector(particle, xIntersect.start);
		yIntersect = yIntersectBallVector(particle, yIntersect.start);
		
		// The ball hits first whichever point lies still within the box
		sideHit = isPointInMatrix(xIntersect.start);
		if (sideHit) {
			// Hits side
			intersect = xIntersect;
		} else if (isPointInMatrix(yIntersect.start)) {
			// Hits top or bottom
			intersect = yIntersect;
		}

		// path length then sums distance traveled to this intersection point
		pathLength += euclidDist(particle.start, intersect.start);
		
		// apply new solution
		particle = intersect;
		
		// add to path
		addWaypoint(ballPath, &pathIndex, pathLength, particle);
	} while (!sideHit && pathIndex < sizeof(ballPath) / sizeof(ballPath[0]));
	
	// save the length
	ballPathLength = pathIndex;
	
	// TODO maybe convert all the vector madness to pointers to save space and time for copies
	
	return sideHit;
}

pointXY_t alongBallPath(double progress) {
	// return where the ball is, according to the progress
	uint8_t pathIndex = 1;
	
	// get ball position at
	double distance = ballPath[ballPathLength - 1].distance * progress;
	
	// advance until preceding section contains ball
	while (ballPath[pathIndex].distance < distance && pathIndex < ballPathLength) pathIndex++;
	
	// Adjust distance to be from the precinct waypoint
	distance = distance - ballPath[pathIndex - 1].distance;
	
	// apply unit vector scaling, i.e. project path
	return {ballPath[pathIndex - 1].bV.start.x + ballPath[pathIndex - 1].bV.v.x * distance, ballPath[pathIndex - 1].bV.start.y + ballPath[pathIndex - 1].bV.v.y * distance};
}

void runAnimations() {
	double span;
	double relVal;

	uint16_t oldAniColor;

	double ballProgress;
	
	span = avg_max - avg_min;
	relVal = min(1.0, abs(avgAnalog - sample) * 2.0 / span);
	
	neoMatrix.fillScreen(0);
	
	switch (aniState) {
		case ANI1:
			// bars that arrange to sound triggers
			if (ani_trg_count > neoMatrix.width()) {
				ani_trg_count = 0;
			}
			for (int i = 0; i < ani_trg_count; i++) {
				neoMatrix.drawFastVLine(i, 0, neoMatrix.height(), neoMatrix.Color(255, 255, 255));
			}
			saveAniParams(ani_trg_count);
			break;
		case ANI2:
			// filled rect, size proportional to sound level
			//neoMatrix.drawRect(round(8.0 - relVal * 8.0), round(5.5 - relVal * 5.5), round(relVal * 16.0), round(relVal * 11.0), neoMatrix.Color(255, 255, 255));
			neoMatrix.fillRect(round(8.0 - relVal * 8.0), round(5.5 - relVal * 5.5), round(relVal * 16.0), round(relVal * 11.0), aniColor);
			saveAniParams(round(8.0 - relVal * 8.0), round(5.5 - relVal * 5.5), round(relVal * 16.0), round(relVal * 11.0), aniColor);
			break;
		case ANI3:
			// mini squares, alternating colors
			oldAniColor = aniColor;
			for (int x = 0; x < neoMatrix.width(); x += 3) {
				for (int y = 0; y < neoMatrix.height(); y += 3) {
					neoMatrix.fillRect(x, y, 3, 3, aniColor);
					advanceAniColor();
				}
			}
			aniColor = oldAniColor;
			saveAniParams(aniColor);
			break;
		case ANI4:
			// Pong-like animation, ball hits when average-of-four trigger is expected next
			// Expect next trigger at: last_trigger + avg_trigger_interval
			if (last_ani_trg_count != ani_trg_count) {
				// A trigger has happened since the last time
				last_ani_trg_count = ani_trg_count;
				
				// Set ball at the intersection and calculate new angle, add random "paddle" offset
				// angle needs to be between +- 36 degrees
				double rndDeg = (double) random(72);
				if (ballPath[ballPathLength - 1].bV.start.x < 0.5) {
					// left side
					if (rndDeg < 36.0) rndDeg += 324;
					else rndDeg -= 36;
				} else {
					// right side
					rndDeg += 144;
				}
				ball = ballVector(ballPath[ballPathLength - 1].bV.start, radians(rndDeg));
				
				// calculate the new path
				calculateBallPath();
			}
			// Net
			neoMatrix.drawFastVLine(7, 0, neoMatrix.height(), neoMatrix.Color(100, 100, 100));
			neoMatrix.drawFastVLine(8, 0, neoMatrix.height(), neoMatrix.Color(100, 100, 100));
			// Players
			neoMatrix.drawLine(round(ballPath[0].bV.start.x), round(ballPath[0].bV.start.y - 1), round(ballPath[0].bV.start.x), round(ballPath[0].bV.start.y + 1), neoMatrix.Color(255, 255, 255));
			neoMatrix.drawLine(round(ballPath[ballPathLength - 1].bV.start.x), round(ballPath[ballPathLength - 1].bV.start.y - 1), round(ballPath[ballPathLength - 1].bV.start.x), round(ballPath[ballPathLength - 1].bV.start.y + 1), neoMatrix.Color(255, 255, 255));
			// Ball
			ballProgress = (now - last_trigger) / ((double) avg_trigger_interval);
			ballProgress = min(1.0, ballProgress);
			ballPos = alongBallPath(ballProgress);
			neoMatrix.drawPixel(round(ballPos.x), round(ballPos.y), aniColor);
			saveAniParams(round(ballPos.x), round(ballPos.y), aniColor);
			break;
		case ANI5:
			// VU-Meter animation
			// each trigger sets high and generate random peaks and lows 30..100%
			if (last_ani_trg_count != ani_trg_count) {
				// A trigger has happened since the last time
				last_ani_trg_count = ani_trg_count;
				for (int i = 0; i < sizeof(vuValues) / sizeof(vuValues[0]); i++) {
					vuValues[i] = 0.3 + ((double) random(100)) / 100.0;
					// so it has to do with sound
					vuValues[i] *= 0.75 + relVal / 4.0;
					//vuValues[i] = log10(1.0 + vuValues[i] * 9.0);
				}
			}
			//double decay = 1.0 - (now - last_trigger) / ((double) avg_trigger_interval);
			double decay = 1.0 - log10(1 + (now - last_trigger) / ((double) avg_trigger_interval));
			decay = max(0.0, decay);
			int j = 0;
			for (int i = 0; i < sizeof(vuValues) / sizeof(vuValues[0]); i++) {
				int vuValue = round(vuValues[i] * decay * neoMatrix.height());
				if (i >= 1 && i <= 5) {
					// saveAniParams(), simplified...
					aniParams[i] = vuValue;
				}
				//j++;
				if (vuValue <= 0) continue;
				// 3 red, 3 yellow, 5 green
				uint8_t h = neoMatrix.height() - 1;
				neoMatrix.drawLine(i + j, h, i + j, h - min(5, vuValue) + 1, neoMatrix.Color(0, 255, 0));
				//neoMatrix.drawLine(i + j + 1, h, i + j + 1, h - min(5, vuValue) + 1, neoMatrix.Color(0, 255, 0));
				h -= 5;
				vuValue -= 5;
				if (vuValue <= 0) continue;
				neoMatrix.drawLine(i + j, h, i + j, h - min(3, vuValue) + 1, neoMatrix.Color(255, 200, 0));
				//neoMatrix.drawLine(i + j + 1, h, i + j + 1, h - min(3, vuValue) + 1, neoMatrix.Color(255, 200, 0));
				h -= 3;
				vuValue -= 3;
				if (vuValue <= 0) continue;
				neoMatrix.drawLine(i + j, h, i + j, h - min(3, vuValue) + 1, neoMatrix.Color(255, 0, 0));
				//neoMatrix.drawLine(i + j + 1, h, i + j + 1, h - min(3, vuValue) + 1, neoMatrix.Color(255, 0, 0));
				
			}
			break;
	}
	saveAniState();
}

void printDebugInfo() {
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

void runSystem() {
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
