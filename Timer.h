#ifndef Timer_H
#define Timer_H

class Timer {
  private:
    unsigned long duration;
    unsigned long start_time;
    bool is_running;

  public:
    Timer(unsigned long duration) {
      this->duration = duration;
      this->is_running = false;
      this->start_time = 0;
    }

    void start() {
      this->start_time = millis();
      this->is_running = true;
    }

    void stop() {
      this->is_running = false;
    }

    bool running() {
      return this->is_running;
    }

    bool expired() {
      if (this->is_running && millis() - this->start_time >= this->duration) {
        this->is_running = false;
        return true;
      }
      return false;
    }
};

#endif

/*

Example Implimentation:

Timer my_timer(5000); // Create a 5-second timer
my_timer.start(); // Start the timer
while (true) {
  if (my_timer.expired()) {
    // Do something when the timer expires
    break;
  }
  // Do something else while the timer is running
}

*/
