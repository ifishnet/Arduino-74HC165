/* ShiftIn.h - Arduino library that reads in values from an 8 bit shift register (74HC165).
 * You can daisy-chain several shift register in order to read in up to 64 buttons by only using 4 Arduino pins.
 *
 * Created by 江小鱼, July 17, 2019
 *
LICENSE
The MIT License (MIT)

Copyright (c) 2019 江小鱼 QQ81130753

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */
#ifndef ShiftIn_h
#define ShiftIn_h

#include "Arduino.h"

template<byte chipCount, typename ShiftType>
class _ShiftIn {
private:
	byte ploadPin;
	byte clockEnablePin;
	byte dataPin;
	byte clockPin;

	const uint16_t dataWidth;
	uint8_t pulseWidth;

	ShiftType lastState;
	ShiftType currentState;
	
	ShiftType Switch[chipCount];
public:
	_ShiftIn() : dataWidth(chipCount * 8), pulseWidth(5), lastState(0), currentState(0) {}
	
	// setup all pins
	void begin(int pload, int clockEN, int data, int clock) {
		pinMode(ploadPin = pload, OUTPUT);
		pinMode(clockEnablePin = clockEN, OUTPUT);
		pinMode(dataPin = data, INPUT);
		pinMode(clockPin = clock, OUTPUT); 
		
		digitalWrite(clockPin, LOW);
        digitalWrite(ploadPin, HIGH);
	}
	
	inline uint8_t getPulseWidth() { return pulseWidth; }
	inline void setPulseWidth(uint8_t value) { pulseWidth = value; }
	inline uint16_t getDataWidth() { return dataWidth; }
	// whether some value has changed
	inline boolean hasChanged() { return lastState != currentState; }
	// whether the value with index 'id' has changed
	inline boolean hasChanged(int id) { return state(id) != last(id); }
	// returns the state from the last frame
	inline ShiftType getLast() { return lastState; }
	// returns the current state
	inline ShiftType getCurrent() { return currentState; }
	// whether button 'id' is pressed or not
	inline boolean state(int id) { return bitRead(currentState, id); }
	// whether button 'id' was pressed in the last frame
	inline boolean last(int id) { return bitRead(lastState, id); }
	// whether button 'id' is now pressed, but wasn't pressed in the last frame
	inline boolean pressed(int id) { return !last(id) && state(id); }
	// whether button 'id' is now released, but was pressed in the last frame
	inline boolean released(int id) { return last(id) && !state(id); }
	// Either HIGH (1) or LOW (0)
	inline boolean getsta(int id) { return (Switch[id / 8] >> (id % 8)) & 1; }
	
	// read in data from shift register and return the new value
	ShiftType read() {
		lastState = currentState;
		long bitVal;
		ShiftType bytesVal = 0;

		digitalWrite(clockEnablePin, HIGH);
		digitalWrite(ploadPin, LOW);
		delayMicroseconds(pulseWidth);
		digitalWrite(ploadPin, HIGH);
		digitalWrite(clockEnablePin, LOW);

		for(int i = 0; i < dataWidth; i++) {
			bitVal = digitalRead(dataPin);
			setNoUpdate(i, bitVal);
			bytesVal |= (bitVal << ((dataWidth - 1) - i)); 
			digitalWrite(clockPin, HIGH);
			delayMicroseconds(pulseWidth);
			digitalWrite(clockPin, LOW);
		}
		currentState = bytesVal;
		return(bytesVal);
	}
	
	ShiftType setNoUpdate(int pin, uint8_t bitVal)
	{
		if (bitVal == 1) {
			Switch[pin / 8] |= (1 << (((dataWidth - 1) - pin) % 8));
		}
		else {
			Switch[pin / 8] &= ~(1 << (((dataWidth - 1) - pin) % 8));
		}
	}
	
	ShiftType getAsc(){
		ShiftType _bitVal[chipCount];
		ShiftType _bytesVal = 0;
		for(int i=0; i < chipCount; i++)
		{
		   _bitVal[i] = (currentState >> (i*8))& 0xFF; 
		}
		
		for(int j=0; j < chipCount; j++)
		{ 
		   _bytesVal = ((_bytesVal << j*8) | _bitVal[j]);
		}
		
		return _bytesVal;
		
	}
	
	
	// same as read, but it returns whether something has changed or not
	boolean update() {
		return read() != lastState;
	}
};

// fallback with 64 bit state (up to 8 shift registers)
template<byte chipCount>
class ShiftIn : public _ShiftIn<chipCount, unsigned int> {};
// single shift register (8 bit state)
template<>
class ShiftIn<1> : public _ShiftIn<1, unsigned int> {};
// two shift registers (16 bit state)
template<>
class ShiftIn<2> : public _ShiftIn<2, unsigned int> {};
// three shift registers (32 bit state)
template<>
class ShiftIn<3> : public _ShiftIn<3, unsigned int> {};
// four shift registers (32 bit state)
template<>
class ShiftIn<4> : public _ShiftIn<4, unsigned int> {};

#endif