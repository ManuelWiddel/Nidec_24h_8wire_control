#ifndef INC_LCDFUNCTIONS_H_

#include <stdio.h>
#include <math.h>
#define INC_LCDFUNCTIONS_H_

#define TIME_DELAY_BEFORE_EN 1
#define TIME_DELAY_BEFORE_DISABLE 2


extern char StringNumber[12];


#endif /* INC_LCDFUNCTIONS_H_ */


void SendBitToPortAndPin(GPIO_TypeDef *port, uint16_t pinMask, uint8_t bitState) {
	if (bitState) {
		HAL_GPIO_WritePin(port, pinMask, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(port, pinMask, GPIO_PIN_RESET);
	}
}

void LCDSendAByteToDataPins(char character) {
	SendBitToPortAndPin(LCD_D0_GPIO_Port, LCD_D0_Pin, character & 0b00000001);
	SendBitToPortAndPin(LCD_D1_GPIO_Port, LCD_D1_Pin, character & 0b00000010);
	SendBitToPortAndPin(LCD_D2_GPIO_Port, LCD_D2_Pin, character & 0b00000100);
	SendBitToPortAndPin(LCD_D3_GPIO_Port, LCD_D3_Pin, character & 0b00001000);
	SendBitToPortAndPin(LCD_D4_GPIO_Port, LCD_D4_Pin, character & 0b00010000);
	SendBitToPortAndPin(LCD_D5_GPIO_Port, LCD_D5_Pin, character & 0b00100000);
	SendBitToPortAndPin(LCD_D6_GPIO_Port, LCD_D6_Pin, character & 0b01000000);
	SendBitToPortAndPin(LCD_D7_GPIO_Port, LCD_D7_Pin, character & 0b10000000);
	HAL_Delay(TIME_DELAY_BEFORE_DISABLE);
	SendBitToPortAndPin(LCD_EN_GPIO_Port, LCD_EN_Pin, 0);
}

void LCDEnable() {
	HAL_Delay(TIME_DELAY_BEFORE_EN);
	SendBitToPortAndPin(LCD_EN_GPIO_Port, LCD_EN_Pin, 1);
}
void LCDSetToWrite() {
	SendBitToPortAndPin(LCD_RW_GPIO_Port, LCD_RW_Pin, 0);
}

void LCDSetToRead() {
	SendBitToPortAndPin(LCD_RW_GPIO_Port, LCD_RW_Pin, 1);
}

void LCDInstructionMode() {
	SendBitToPortAndPin(LCD_RS_GPIO_Port, LCD_RS_Pin, 0);
}

void LCDCharacterMode() {
	SendBitToPortAndPin(LCD_RS_GPIO_Port, LCD_RS_Pin, 1);
}

void LCDSendInstruction(char character) {
	LCDSetToWrite();
	LCDInstructionMode();
	LCDEnable();
	LCDSendAByteToDataPins(character);
}

void LCDSendACharacter(char character) {
	LCDSetToWrite();
	LCDCharacterMode();
	LCDEnable();
	LCDSendAByteToDataPins(character);
}

void LCDSendAString(char *StringOfCharacters) {

	while(*StringOfCharacters) {
		LCDSendACharacter(*StringOfCharacters++);
	}

}

void LCDSendInteger(uint32_t IntNumber) {

	snprintf(StringNumber, sizeof(StringNumber), "%d", IntNumber);
	LCDSendAString(StringNumber);
}

void LCDSendFloat3d(float FloatNumber) {
	int negative = FloatNumber < 0.0f;
	    float value = fabsf(FloatNumber);

	    int whole = (int)value;
	    int decimal = (int)((value - whole) * 1000.0f);

	    char str[12];

	    snprintf(str, sizeof(str), "%s%d.%03d",
	             negative ? "-" : "",
	             whole,
	             decimal);

	    LCDSendAString(str);
	}

void LCDSetCursor(uint8_t row, uint8_t column)
{
    uint8_t address;

    if (row == 0)
        address = 0x00 + column;
    else
        address = 0x40 + column;
    if (((row == 0) | (row == 1)) & ((0 <= column) & (column <= 15))) {
    	LCDSendInstruction(0x80 | address);
    }
}

void LCDInit() {
	LCDSendInstruction(0b00000001); // Clear display
	LCDSendInstruction(0b10000000);
	LCDSendInstruction(0b00111000); // Set to 8-bit mode and 2 line display
	LCDSendInstruction(0b00001110); // Turn on display and cursor
	LCDSendInstruction(0b00000110); // Set the mode to increment address by one
}
