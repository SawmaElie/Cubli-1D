
#include "filters.h"


float Moving_Average_Filter(float newVal){
	static float buffer[10];
	static int index = 0;
	buffer[index] = newVal;
	index = (index + 1) % 10;
	newVal = 0;
	for(int i = 0; i < 10; i++){
		newVal = newVal + buffer[i];
	}
	return newVal / 10;
}

float Moving_Average_Filter1(float newVal){
	static float buffer[10];
	static int index = 0;
	buffer[index] = newVal;
	index = (index + 1) % 10;
	newVal = 0;
	for(int i = 0; i < 10; i++){
		newVal = newVal + buffer[i];
	}
	return newVal / 10;
}

float Moving_Average_Filter2(float newVal){
	static float buffer[10];
	static int index = 0;
	buffer[index] = newVal;
	index = (index + 1) % 10;
	newVal = 0;
	for(int i = 0; i < 10; i++){
		newVal = newVal + buffer[i];
	}
	return newVal / 10;
}

float Moving_Average_Filter3(float newVal){
	static float buffer[10];
	static int index = 0;
	buffer[index] = newVal;
	index = (index + 1) % 10;
	newVal = 0;
	for(int i = 0; i < 10; i++){
		newVal = newVal + buffer[i];
	}
	return newVal / 10;
}
float Moving_Average_Filter4(float newVal){
	static float buffer[10];
	static int index = 0;
	buffer[index] = newVal;
	index = (index + 1) % 10;
	newVal = 0;
	for(int i = 0; i < 10; i++){
		newVal = newVal + buffer[i];
	}
	return newVal / 10;
}
float Moving_Average_Filter5(float newVal){
	static float buffer[10];
	static int index = 0;
	buffer[index] = newVal;
	index = (index + 1) % 10;
	newVal = 0;
	for(int i = 0; i < 10; i++){
		newVal = newVal + buffer[i];
	}
	return newVal / 10;
}

