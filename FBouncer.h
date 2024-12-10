#define SHORT_PRESS 10000
#define LONG_PRESS 200000

#define SHORT_PRESS_MILLIS 50
#define LONG_PRESS_MILLIS 1000

class FBouncer {
public:
  FBouncer(int b_pin, int ncyc = SHORT_PRESS) {
    button_pin = b_pin;
    button_counter = 0;
    ncycles = ncyc;
    extraReturn = false;
    prev_millis = 0;
    button_counter_millis = 0;

    pinMode(button_pin, INPUT_PULLUP);
  }

  void setNCycle(int cyc) {
    ncycles = cyc;
  }

  bool readButton() {
    int button = digitalRead(button_pin);

    if (button == LOW) {
      button_counter++;

      if (button_counter >= ncycles) {
        button_counter = 0;
        return true;
      }
    } else {
      button_counter = 0;
    }

    return false;
  }

  int readButtonTime() {
    int button = digitalRead(button_pin);
    int ret = 0;

    if (button == LOW) {
      button_counter++;
      if (button_counter >= LONG_PRESS) {
        button_counter = 0;
        extraReturn = true;  // вышли удерживая кнопку
        return LONG_PRESS;
      }
    } else {
      if (extraReturn) {  // был выход с удержанной кнопкой
        extraReturn = false;
      } else {
        if (button_counter >= SHORT_PRESS) {
          ret = SHORT_PRESS;
        }
      }

      button_counter = 0;
    }

    return ret;
  }

  int readButtonTimeMillis() {
    int button = digitalRead(button_pin);
    int ret = 0;

    if (button == LOW) {
      int now = millis();
      if (prev_millis > 0) {
        button_counter_millis += now - prev_millis;
      }
      prev_millis = now;

      if (button_counter_millis >= LONG_PRESS_MILLIS) {
        button_counter_millis = 0;
        prev_millis = 0;
        extraReturn = true;  // вышли удерживая кнопку
        return LONG_PRESS;
      }
    } else {
      if (extraReturn) {  // был выход с удержанной кнопкой
        extraReturn = false;
      } else {
        if (button_counter_millis >= SHORT_PRESS_MILLIS) {
          ret = SHORT_PRESS;
        }
      }

      button_counter_millis = 0;
      prev_millis = 0;
    }

    return ret;
  }

private:
  bool extraReturn;
  int button_pin;
  int button_counter;
  int button_counter_millis;
  int prev_millis;
  int ncycles;
};
