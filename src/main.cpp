#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <cstdint>
#include <cmath>
#include <cstdint>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>

struct
{
	std::bitset<32> inputs;
	std::bitset<32> senderInputs = 0xffffffff;
	SemaphoreHandle_t mutex; // LAB 2 task 1
	int knob3Rotation;		 // LAB 2 task 2
	int knob2Rotation;		 // octave number
	int volume;
	uint8_t RX_Message[8] = {0};
	uint32_t adjstepSizes[12] = {0};
	bool receiver = false;
} sysState;
QueueHandle_t msgInQ;

// Constants
const uint32_t interval = 100; // Display update interval

// Pin definitions
// Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

// Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

// Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

// Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

// Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

// Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

// the mutex
SemaphoreHandle_t dataMutex;

// Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value)
{
	digitalWrite(REN_PIN, LOW);
	digitalWrite(RA0_PIN, bitIdx & 0x01);
	digitalWrite(RA1_PIN, bitIdx & 0x02);
	digitalWrite(RA2_PIN, bitIdx & 0x04);
	digitalWrite(OUT_PIN, value);
	digitalWrite(REN_PIN, HIGH);
	delayMicroseconds(2);
	digitalWrite(REN_PIN, LOW);
}

void CAN_RX_ISR(void)
{
	uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

const uint32_t stepSizes[] = {51076056, 54113197, 57330935, 60740010, 64351798, 68178356, 72232452, 76527617, 81078186, 85899345, 91007186, 96418755};

// part 3 step 3 - interrupt to send generate sound
volatile uint32_t currentStepSize;
volatile uint32_t localCurrentStepSize;

void sampleISR()
{
	static uint32_t phaseAcc[12] = {0}; // Initialize phase accumulators for each note
	static int32_t mixedOutput = 0;		// Initialize mixed output for all active notes
	int activeNoteCount = 0;			// Initialize counter for active notes

	// Iterate through all possible notes
	for (int i = 0; i < 12; ++i)
	{
		if ((sysState.senderInputs[i] && sysState.inputs[i]) == 0) // Check if the note is active
		{
			// Calculate the step size for the note based on its frequency
			uint32_t stepSize = sysState.adjstepSizes[i];

			// Increment phase accumulator for the note with its respective step size
			phaseAcc[i] += stepSize;

			// Calculate the output voltage for the note
			int32_t Vout = (phaseAcc[i] >> 24) - 128;

			// Adjust the volume for the note
			Vout = Vout >> (8 - sysState.volume);

			// Accumulate the output voltage for the mixed output
			mixedOutput += Vout;

			// Increment active note count
			activeNoteCount++;
		}
	}

	// Divide the mixed output by the number of active notes to scale it down
	if (activeNoteCount > 0)
	{
		if (activeNoteCount <= 2)
		{
			mixedOutput /= 4;
		}
		else
		{
			mixedOutput /= (activeNoteCount * 1.3);
		}
	}
	else
	{
		mixedOutput = 0; // If no active notes, set mixedOutput to 0
	}

	// Output the mixed voltage to the analog pin
	analogWrite(OUTR_PIN, mixedOutput + 128);
}

std::bitset<4> readCols()
{
	// Set each row select address (RA0, RA1, RA2) low
	// digitalWrite(RA2_PIN, LOW);
	// digitalWrite(RA1_PIN, LOW);
	// digitalWrite(RA0_PIN, LOW);

	std::bitset<4> result;
	result[0] = digitalRead(C0_PIN);
	result[1] = digitalRead(C1_PIN);
	result[2] = digitalRead(C2_PIN);
	result[3] = digitalRead(C3_PIN);

	return result;

	// Set row select enable (REN) high
	// digitalWrite(REN_PIN, HIGH);

	// // Read the inputs from the four columns (C0, C1, C2, C3)
	// std::bitset<4> result;
	// result[0] = digitalRead(C0_PIN);
	// result[1] = digitalRead(C1_PIN);
	// result[2] = digitalRead(C2_PIN);
	// result[3] = digitalRead(C3_PIN);

	// // Reset row select enable (REN) to low for the next read
	// digitalWrite(REN_PIN, LOW);
}

// Function to select a given row of the switch matrix
void setRow(uint8_t rowIdx)
{
	// Disable row select enable
	digitalWrite(REN_PIN, LOW);

	// Set the row select address pins based on rowIdx
	digitalWrite(RA2_PIN, (rowIdx & 0x04) ? HIGH : LOW);
	digitalWrite(RA1_PIN, (rowIdx & 0x02) ? HIGH : LOW);
	digitalWrite(RA0_PIN, (rowIdx & 0x01) ? HIGH : LOW);

	// Enable row select enable
	digitalWrite(REN_PIN, HIGH);
}

// part 2 step 2 - fucnion to check state of input and return pressed keys
const char *getKeysFromInput(std::bitset<32> inputs)
{
	// Define the keys and their corresponding input values
	const char *keys[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
	// const uint32_t keyValues[] = {
	//     0b111111111111, // FFF
	//     0b111111111110, // FFE
	//     0b111111111101, // FFD
	//     0b111111111011, // FFB
	//     0b111111110111, // FF7
	//     0b111111101111, // FEF
	//     0b111111011111, // FDF
	//     0b111110111111, // FBF
	//     0b111101111111, // F7F
	//     0b111011111111, // EFF
	//     0b110111111111, // DFF
	//     0b101111111111, // BFF
	//     0b011111111111  // 7FF
	// };

	// Check if all inputs are zero (FFF)
	if (inputs.to_ulong() == 0b111111111111)
	{
		return "Nothing";
	}

	// Initialize a character array to store pressed keys
	static char pressedKeys[256];				 // Assuming a maximum length of 256 characters
	memset(pressedKeys, 0, sizeof(pressedKeys)); // Clear the array

	// Index to track the next character position in pressedKeys array
	int keyIndex = 0;
	bool isFirstKey = true;

	// Iterate through the key values
	for (int i = 0; i < 12; ++i)
	{
		// Check if the corresponding bit in inputs is 0
		if ((inputs.to_ulong() & (1 << i)) == 0)
		{
			// Add the pressed key to the array
			if (!isFirstKey)
			{
				pressedKeys[keyIndex++] = ',';
				pressedKeys[keyIndex++] = ' ';
			}
			isFirstKey = false;
			strcpy(&pressedKeys[keyIndex], keys[i]);
			keyIndex += strlen(keys[i]);
		}
	}

	// Null-terminate the array
	pressedKeys[keyIndex] = '\0';

	// Return the array of pressed keys
	return pressedKeys;
}

// part 3 step 2 - fucnion to check state of input and return corresponding stepsize

uint32_t getStepSizeFromInput(std::bitset<32> inputs)
{
	// Check if all inputs are 1 (FFF)
	if (inputs.to_ulong() == 0b111111111111)
	{
		return 0; // Return 0 if no keys are pressed
	}

	// Iterate through the key values
	for (int i = 0; i < 12; ++i)
	{
		// Check if the corresponding bit in inputs is 0
		if ((inputs.to_ulong() & (1 << i)) == 0)
		{
			// Return the step size corresponding to the pressed key
			return stepSizes[i];
		}
	}

	// Default case: return 0 if no keys are pressed (should not reach here)
	return 0;
}

void decodeTask(void *pvParameters)
{
	uint8_t local[8] = {0};
	while (1)
	{
		xQueueReceive(msgInQ, local, portMAX_DELAY);
		if (local[5] == 'S'){
			sysState.receiver=false;
			sysState.volume=local[4];
		}

		if (local[0] == 'R')
		{

			sysState.senderInputs[local[2]] = 1;
		}
		if (local[0] == 'P')
		{
			sysState.senderInputs[local[2]] = 0;
			// sysState.inputs[local[1]]=0;
			// Serial.println("Received");
			for (int j = 0; j < 12; ++j)
			{
				if (local[1] >= 4)
				{
					// Serial.println("oct > 4");
					sysState.adjstepSizes[j] = stepSizes[j] << (local[1] - 4);
				}
				else if (sysState.knob2Rotation < 4)
				{
					sysState.adjstepSizes[j] = stepSizes[j] >> (4 - local[1]);
				}
			}
		}
		xSemaphoreTake(sysState.mutex, portMAX_DELAY);
		memcpy(sysState.RX_Message, local, sizeof(local));
		xSemaphoreGive(sysState.mutex);
	}
}

// new trask which finds the input and also sets the values for the step size and current stepo size thing
void scanKeysTask(void *pvParameters)
{
	const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
	TickType_t xLastWakeTime = xTaskGetTickCount();

	static std::bitset<32> prevInputs = 0; // Store previous inputs

	int prevA = 0;
	int prevB = 0;

	int prev2A = 0;
	int prev2B = 0;

	while (1)
	{
		// Open mutex
		if (xSemaphoreTake(sysState.mutex, portMAX_DELAY) == pdTRUE)
		{
			// Key scanning loop
			for (uint8_t rowIdx = 0; rowIdx < 6; ++rowIdx)
			{
				// Set the row select address
				setRow(rowIdx);

				// Add a small delay to allow for the switch matrix columns to settle
				delayMicroseconds(3);

				// Read the columns and copy the results into the inputs bitset
				std::bitset<4> rowInputs = readCols();
				for (uint8_t colIdx = 0; colIdx < 4; ++colIdx)
				{
					sysState.inputs[rowIdx * 4 + colIdx] = rowInputs[colIdx];
				}

				// Knob 3 decode (LAB 2 task 2)
				if (rowIdx == 3)
				{
					// Read the knob 3 input
					int currentA = rowInputs[0];
					int currentB = rowInputs[1];

					if (prevA != currentA || prevB != currentB)
					{
						delayMicroseconds(500);
						if (prevA == 0 && prevB == 0)
						{
							if (currentA == 1 && currentB == 0)
							{
								sysState.knob3Rotation = (sysState.knob3Rotation + 1) % 9;
								// Serial.println("Clockwise");
							}
							else if (currentA == 0 && currentB == 1)
							{
								sysState.knob3Rotation = (sysState.knob3Rotation + 8) % 9;
								// Serial.println("Counter-clockwise");
							}
						}
					}
					sysState.knob3Rotation = constrain(sysState.knob3Rotation, 0, 8);
					prevA = currentA;
					prevB = currentB;

					// Read the knob 2 input
					int current2A = rowInputs[2];
					int current2B = rowInputs[3];

					if (prev2A != current2A || prev2B != current2B)
					{
						delayMicroseconds(500);
						if (prev2A == 0 && prev2B == 0)
						{
							if (current2A == 1 && current2B == 0)
							{
								sysState.knob2Rotation = (sysState.knob2Rotation + 1) % 9;
								Serial.println("Clockwise");
							}
							else if (current2A == 0 && current2B == 1)
							{
								sysState.knob2Rotation = (sysState.knob2Rotation + 8) % 9;
								Serial.println("Counter-clockwise");
							}
						}
					}
					sysState.knob2Rotation = constrain(sysState.knob2Rotation, 0, 8);
					prev2A = current2A;
					prev2B = current2B;
				}

				
				if (rowIdx == 5)
				{
					if (!rowInputs[1]) {
						uint8_t TX_Message[8] = {0};
						// HERE ADD CAN MESSAGE TO TURN ALL OTHER BOARDS INTO SENDERS (RECEIVER = FALSE)
						TX_Message[5] = 'S' ;
						CAN_TX(0x123, TX_Message);

						// IN DECODE IF 'S' RECEIVER = FALSE
						sysState.receiver = true;
					}
				}
				if (sysState.receiver)
				{
					uint8_t TX_Message[8] = {0};
					sysState.volume=sysState.knob3Rotation;
					TX_Message[4] = sysState.knob3Rotation;
					CAN_TX(0x123, TX_Message);
					for (int j = 0; j < 12; ++j)
					{
						if (sysState.knob2Rotation > 4)
						{
							sysState.adjstepSizes[j] = stepSizes[j] << (sysState.knob2Rotation - 4);
						}
						else if (sysState.knob2Rotation < 4)
						{
							sysState.adjstepSizes[j] = stepSizes[j] >> (4 - sysState.knob2Rotation);
						}
					}
				}
				else
				{
					uint8_t TX_Message[8] = {0};
					// Compare current inputs with previous inputs
					for (int i = 0; i < 12; ++i)
					{
						if (sysState.inputs[i] != prevInputs[i])
						{
							// Key state changed, update TX_Message
							TX_Message[0] = sysState.inputs[i] ? 'R' : 'P'; // 'P' for pressed, 'R' for released
							TX_Message[1] = sysState.knob2Rotation;			// Octave number
							TX_Message[2] = i ;							// Note number
							CAN_TX(0x123, TX_Message);
							break; // Exit loop after handling the first changed key
						}
					}
				}
			}

			// uint8_t TX_Message[8] = {0};
			// // Compare current inputs with previous inputs
			// for (int i = 0; i < 32; ++i) {
			//     if (sysState.inputs[i] != prevInputs[i]) {
			//         // Key state changed, update TX_Message
			//         TX_Message[0] = sysState.inputs[i] ? 'R' : 'P'; // 'P' for pressed, 'R' for released
			//         TX_Message[1] = sysState.knob2Rotation; // Octave number
			//         TX_Message[2] = i % 12; // Note number
			// 		CAN_TX(0x123, TX_Message);
			//         break; // Exit loop after handling the first changed key
			//     }
			// }

			// Store current inputs for next iteration
			prevInputs = sysState.inputs;

			// Release mutex
			xSemaphoreGive(sysState.mutex);
		}
		localCurrentStepSize = getStepSizeFromInput(sysState.inputs);
		__atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);

		// Delay until the next execution time
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

void displayUpdateTask(void *pvParameters)
{
	const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
	TickType_t xLastWakeTime = xTaskGetTickCount();

	while (1)
	{
		// Update display
		if (xSemaphoreTake(sysState.mutex, portMAX_DELAY) == pdTRUE)
		{
			// Display TX_Message in the serial monitor
			// Serial.print("TX_Message: ");
			// for (int i = 0; i < 8; ++i) {
			//    Serial.print(TX_Message[i]);
			//    Serial.print(" ");
			//}
			// Serial.println();

			// Update display content
			u8g2.clearBuffer();
			u8g2.setFont(u8g2_font_ncenB08_tr);
			u8g2.setCursor(2, 10);
			u8g2.print("Inputs: ");
			u8g2.print(sysState.inputs.to_ulong(), HEX);

			u8g2.setCursor(80, 10);
			u8g2.print("Oct: ");
			u8g2.print(sysState.knob2Rotation);

			u8g2.setCursor(80, 20);
			u8g2.print("pls :)");

			u8g2.setCursor(2, 20);
			u8g2.print("Keys: ");
			u8g2.print(getKeysFromInput(sysState.inputs));

			u8g2.setCursor(2, 30);
			u8g2.print("Volume: ");
			u8g2.print(sysState.volume);

			// u8g2.setCursor(66,30);
			// u8g2.print((char) TX_Message[0]);
			// u8g2.print(TX_Message[1]);
			// u8g2.print(TX_Message[2]);

			// Poll for received messages

			u8g2.setCursor(66, 30);
			u8g2.print((char)sysState.RX_Message[0]);
			u8g2.print(sysState.RX_Message[1]);
			u8g2.print(sysState.RX_Message[2]);
			u8g2.print(sysState.RX_Message[4]);
			u8g2.print((char)sysState.RX_Message[5]);

			u8g2.sendBuffer(); // Transfer internal memory to the display

			// Toggle LED
			digitalToggle(LED_BUILTIN);

			xSemaphoreGive(sysState.mutex);
		}

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

void setup()
{
	// put your setup code here, to run once:
	CAN_Init(sysState.receiver);
	setCANFilter(0x123, 0x7ff);
	CAN_RegisterRX_ISR(CAN_RX_ISR);
	CAN_Start();
	msgInQ = xQueueCreate(36, 8);
	// Initialize the mutex
	dataMutex = xSemaphoreCreateMutex();
	if (dataMutex == NULL)
	{
		// Failed to create mutex
		while (1)
			;
	}
	// rtos scheduler setup
	TaskHandle_t scanKeysHandle = NULL;
	xTaskCreate(
		scanKeysTask,	  /* Function that implements the task */
		"scanKeys",		  /* Text name for the task */
		64,				  /* Stack size in words, not bytes */
		NULL,			  /* Parameter passed into the task */
		2,				  /* Task priority */
		&scanKeysHandle); /* Pointer to store the task handle */

	xTaskCreate(
		displayUpdateTask, /* Function that implements the task */
		"displayUpdate",   /* Text name for the task */
		256,			   /* Stack size in words, not bytes */
		NULL,			   /* Parameter passed into the task */
		1,				   /* Task priority (higher priority) */
		&scanKeysHandle	   /* Pointer to store the task handle */
	);

	xTaskCreate(
		decodeTask,
		"decode",
		256,
		NULL,
		1,
		&scanKeysHandle);

	// interrupt timer setup
	TIM_TypeDef *Instance = TIM1;
	HardwareTimer *sampleTimer = new HardwareTimer(Instance);
	// interrupt setup
	sampleTimer->setOverflow(22000, HERTZ_FORMAT);
	sampleTimer->attachInterrupt(sampleISR);
	sampleTimer->resume();
	// Set pin directions
	pinMode(RA0_PIN, OUTPUT);
	pinMode(RA1_PIN, OUTPUT);
	pinMode(RA2_PIN, OUTPUT);
	pinMode(REN_PIN, OUTPUT);
	pinMode(OUT_PIN, OUTPUT);
	pinMode(OUTL_PIN, OUTPUT);
	pinMode(OUTR_PIN, OUTPUT);
	pinMode(LED_BUILTIN, OUTPUT);

	pinMode(C0_PIN, INPUT);
	pinMode(C1_PIN, INPUT);
	pinMode(C2_PIN, INPUT);
	pinMode(C3_PIN, INPUT);
	pinMode(JOYX_PIN, INPUT);
	pinMode(JOYY_PIN, INPUT);

	// Initialise display
	setOutMuxBit(DRST_BIT, LOW); // Assert display logic reset
	delayMicroseconds(2);
	setOutMuxBit(DRST_BIT, HIGH); // Release display logic reset
	u8g2.begin();
	setOutMuxBit(DEN_BIT, HIGH); // Enable display power supply

	// Initialise UART
	Serial.begin(9600);
	Serial.println("sysstateinput:");
	Serial.println(sysState.inputs.to_ulong(), HEX);
	Serial.println("step:");
	Serial.println(currentStepSize);

	sysState.mutex = xSemaphoreCreateMutex();

	// rtos scheduler start
	vTaskStartScheduler();
}

void loop()
{

	//   // Acquire the mutex
	//   if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
	//       // Access and modify shared resources (sysState.inputs and currentStepSize)

	//       // Release the mutex
	//       xSemaphoreGive(dataMutex);
	//   }

	// // put your main code here, to run repeatedly:
	// static uint32_t next = millis();
	// static uint32_t count = 0;

	// while (millis() < next)
	// 	; // Wait for the next interval

	// next += interval;

	// // Update display
	// u8g2.clearBuffer();					// clear the internal memory
	// u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
	// u8g2.setCursor(2, 10);
	// u8g2.print("Inputs: ");
	// u8g2.print(sysState.inputs.to_ulong(), HEX);

	// u8g2.setCursor(2, 20);
	// u8g2.print("Keys: ");
	// u8g2.print(getKeysFromInput(sysState.inputs));

	// u8g2.setCursor(2, 30);
	// u8g2.print("step: ");
	// u8g2.print(currentStepSize);

	// u8g2.sendBuffer(); // transfer internal memory to the display

	// // Toggle LED
	// digitalToggle(LED_BUILTIN);
}