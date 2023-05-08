#ifndef Fan_H
#define Fan_H


class Fan{
	
	private:

		int _pin;

	public:

		Fan(int pin){

			this->_pin = pin;
			pinMode(pin, OUTPUT);
			digitalWrite(pin, HIGH);

		}

		void Toggle(bool On){
		
			digitalWrite(pin, On ? LOW : HIGH);

		}
};

#endif 