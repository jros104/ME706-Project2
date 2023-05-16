#ifndef Fan_H
#define Fan_H


class Fan{
	
	private:

		int _pin;

	public:

		Fan(int pin){
			this->_pin = pin;
			pinMode(this->_pin, OUTPUT);
			digitalWrite(this->_pin, HIGH);
		}

		void Toggle(bool On){
			Serial.print("Fan: ");
			Serial.println(On ? "ON" : "OFF");
			digitalWrite(this->_pin, On ? LOW : HIGH);
		}
};

#endif 