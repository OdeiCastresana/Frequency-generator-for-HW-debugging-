/*
 * One-button frequency sequencer on a single pin (D9)
 * Board: Arduino Uno / Nano (ATmega328P @ 16 MHz)
 *
 * Sequence per press:
 *   OFF -> 15.625 kHz -> 31.25 kHz -> 62.5 kHz -> OFF ...
 *
 * Implementation: Timer1 Fast PWM (Mode 14), TOP = ICR1
 * Frequencies (prescaler = 1, f_clk = 16 MHz):
 *   f_pwm = 16e6 / (1 * (ICR1 + 1))
 *   15.625 kHz: ICR1 = 1023  -> f = 16e6 / 1024 = 15625 Hz
 *   31.25  kHz: ICR1 = 511   -> f = 16e6 / 512  = 31250 Hz
 *   62.5   kHz: ICR1 = 255   -> f = 16e6 / 256  = 62500 Hz
 *
 * Duty (non-inverting): D = (OCR1A + 1) / (ICR1 + 1)
 * For 50% duty and these odd TOP values, OCR1A = ICR1 / 2 gives exact 50%.
 *
 * Button wiring: momentary switch between D2 and GND (uses internal pull-up).
 */

const uint8_t BTN_PIN = 2;   // Button to GND (internal pull-up)
const uint8_t OUT_PIN = 9;   // D9 = OC1A (Timer1 output)

enum Mode : uint8_t {
  MODE_OFF = 0,
  MODE_15K6,
  MODE_31K25,
  MODE_62K5
};

Mode mode = MODE_OFF;

void setupTimer1_Mode14_ICR1() {
  // Clear Timer1
  TCCR1A = 0;
  TCCR1B = 0;

  // Mode 14: Fast PWM with TOP = ICR1 (WGM13:0 = 1110)
  TCCR1A |= (1 << WGM11);                  // WGM11 = 1
  TCCR1B |= (1 << WGM12) | (1 << WGM13);   // WGM12 = 1, WGM13 = 1

  // Non-inverting PWM on OC1A (D9)
  TCCR1A |= (1 << COM1A1);

  // Prescaler = 1 (fastest, exact target freqs)
  TCCR1B |= (1 << CS10);
}

inline void setFreq50pct_on_D9(uint16_t topICR1) {
  // Set TOP then duty; for 50% duty with non-inverting, OCR1A = ICR1 / 2
  ICR1  = topICR1;
  OCR1A = topICR1 / 2;   // exact 50% because ICR1 is odd (255/511/1023)
}

void enableD9PWM() {
  TCCR1A |= (1 << COM1A1);     // ensure OC1A is driven by compare unit
}

void disableD9PWM_andDriveLow() {
  // Disconnect PWM from OC1A and stop Timer1 clock
  TCCR1A &= ~(1 << COM1A1);
  TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
  // Drive pin low (0 V)
  digitalWrite(OUT_PIN, LOW);
}

void applyMode(Mode m) {
  switch (m) {
    case MODE_OFF:
      disableD9PWM_andDriveLow();
      break;

    case MODE_15K6:   // 15.625 kHz
      setupTimer1_Mode14_ICR1();
      setFreq50pct_on_D9(1023);
      enableD9PWM();
      break;

    case MODE_31K25:  // 31.25 kHz
      setupTimer1_Mode14_ICR1();
      setFreq50pct_on_D9(511);
      enableD9PWM();
      break;

    case MODE_62K5:   // 62.5 kHz
      setupTimer1_Mode14_ICR1();
      setFreq50pct_on_D9(255);
      enableD9PWM();
      break;
  }
}

void setup() {
  pinMode(BTN_PIN, INPUT_PULLUP); // Button to GND
  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, LOW);     // Ensure low at startup

  applyMode(MODE_OFF);
}

void loop() {
  static uint8_t prev = HIGH;
  uint8_t cur = digitalRead(BTN_PIN);

  if (prev == HIGH && cur == LOW) {
    // Debounce
    delay(15);
    if (digitalRead(BTN_PIN) == LOW) {
      // Advance sequence
      switch (mode) {
        default:
        case MODE_OFF:   mode = MODE_15K6;  break;
        case MODE_15K6:  mode = MODE_31K25; break;
        case MODE_31K25: mode = MODE_62K5;  break;
        case MODE_62K5:  mode = MODE_OFF;   break;
      }
      applyMode(mode);

      // Wait for release
      while (digitalRead(BTN_PIN) == LOW) { /* wait */ }
      delay(15);
    }
  }
  prev = cur;
}