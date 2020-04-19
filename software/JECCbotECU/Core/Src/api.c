/*
 * api.c
 *
 *  Created on: Mar 26, 2020
 *      Author: jonas
 */
#include "api.h"

uint16_t apiMemory[API_MEMORY_SIZE];

bool apiWriteables[API_MEMORY_SIZE];

bool apiLocked;

ApiInstruction apiInstruction;

void apiInit()
{
	for(int i = 0; i < API_MEMORY_SIZE; i++)
	{
		apiMemory[i] = 0;
		apiWriteables[i] = true;
	}

	apiInstruction.set = false;
	apiLocked = false;
}

void apiSetInstruction(char *instruction)
{
	apiInstruction.commandLen = strlen(instruction)+1;
	strncpy(apiInstruction.command, instruction, apiInstruction.commandLen);
	apiInstruction.set = true;
}

ApiInstruction apiUpdate()
{
	apiInstruction.responseLen = 0;

		if(apiInstruction.set)
		{
			if(':' == apiInstruction.command[0] && '\n' == apiInstruction.command[apiInstruction.commandLen - 2])
			{
				char instructorStr[3];
				char addressStr[5];

				uint8_t instructor;
				uint16_t address;

				strncpy(instructorStr, &apiInstruction.command[1], 2);
				instructorStr[2] = '\0';
				strncpy(addressStr, &apiInstruction.command[3], 4);
				addressStr[4] = '\0';


				instructor = strtol(instructorStr, NULL, 16);
				address = strtol(addressStr, NULL, 16);


				if(address < API_MEMORY_SIZE)
				{
//					while(apiLocked){}
//					apiLocked = true;
					if(API_INSTRUCTION_READ == instructor)
					{
						apiInstruction.responseLen = sprintf(apiInstruction.response, "%04x%04x\n", address, apiMemory[address]);
					}
					else if(API_INSTRUCTION_WRITE == instructor)
					{
						if(apiWriteables[address])
						{
							char valueStr[5];
							uint16_t value;

							strncpy(valueStr, &apiInstruction.command[7], 4);
							valueStr[4] = '\0';

							value = strtol(valueStr, NULL, 16);

							apiMemory[address] = value;

							apiInstruction.responseLen = sprintf(apiInstruction.response, "%04x%04x\n", address, apiMemory[address]);
						}
						else
						{
							apiInstruction.responseLen = sprintf(apiInstruction.response, "e%04x\n", API_ERROR_ACCESS_DENIED);
						}
					}
					else
					{
						apiInstruction.responseLen = sprintf(apiInstruction.response, "e%04x\n", API_ERROR_WRONG_INSTRUCTOR);
					}
//					apiLocked = false;
				}
				else
				{
					apiInstruction.responseLen = sprintf(apiInstruction.response, "e%04x\n", API_ERROR_INVALID_ADDRESS);
				}

			}
			else
			{
				apiInstruction.responseLen = sprintf(apiInstruction.response, "e%04x\n", API_ERROR_WRONG_FORMAT);
			}
		}


	apiInstruction.set = false;

	return apiInstruction;
}

bool isApiAddressValid(int address)
{
	if(address > 0 && address < API_MEMORY_SIZE)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool apiWrite16(int address, int16_t value)
{
	if(isApiAddressValid(address))
	{
		memcpy(&apiMemory[address], &value, 2);
		return true;
	}
	else
	{
		return false;
	}
}

int16_t apiRead16(int address)
{
	int16_t val = -1;

	if(isApiAddressValid(address))
	{
		memcpy(&val, &apiMemory[address], 4);
	}

	return val;
}

bool apiWrite32(int address, int32_t value)
{
	if(isApiAddressValid(address))
	{
		memcpy(&apiMemory[address], &value, 4);
		return true;
	}
	else
	{
		return false;
	}
}

int32_t apiRead32(int address)
{
	int32_t val = -1;

	if(isApiAddressValid(address))
	{
		memcpy(&val, &apiMemory[address], 4);
	}

	return val;
}


bool apiWriteFloat(int address, float value)
{
	if(isApiAddressValid(address))
	{
		memcpy(&apiMemory[address], &value, 4);
		return true;
	}
	else
	{
		return false;
	}
}

float apiReadFloat(int address)
{
	float f =  -1.0;
	if(isApiAddressValid(address))
	{
		memcpy(&f, &apiMemory[address], 4);
	}

	return f;
}

///*
// * api.c
// *
// *  Created on: Mar 26, 2020
// *      Author: jonas
// */
//#include "api.h"
//
//uint16_t apiMemory[API_MEMORY_SIZE];
//
//bool apiWriteables[API_MEMORY_SIZE];
//
//bool apiLocked;
//
//void apiInit()
//{
//	for(int i = 0; i < API_MEMORY_SIZE; i++)
//	{
//		apiMemory[i] = 0;
//		apiWriteables[i] = true;
//	}
//	apiLocked = false;
//}
//
//void apiDoInstruction(char *instruction, char *responseMessage)
//{
//	apiLocked = true;
//	int len = strlen(instruction)+1;
//	char ins[len];
//	strcpy(ins, instruction);
//
//	if(':' == ins[0] && '\n' == ins[len - 2])
//	{
//		char instructorStr[3];
//		char addressStr[5];
//
//		uint8_t instructor;
//		uint16_t address;
//
//
//
//		strncpy(instructorStr, &ins[1], 2);
//		instructorStr[2] = '\0';
//		strncpy(addressStr, &ins[3], 4);
//		addressStr[4] = '\0';
//
//
//		instructor = strtol(instructorStr, NULL, 16);
//		address = strtol(addressStr, NULL, 16);
//
//
//		if(address < API_MEMORY_SIZE)
//		{
//			if(API_INSTRUCTION_READ == instructor)
//			{
//				sprintf(responseMessage, ":%04x%04x\n", address, apiMemory[address]);
//			}
//			else if(API_INSTRUCTION_WRITE == instructor)
//			{
//				if(apiWriteables[address])
//				{
//					char valueStr[5];
//					uint16_t value;
//
//					strncpy(valueStr, &ins[7], 4);
//					valueStr[4] = '\0';
//
//					value = strtol(valueStr, NULL, 16);
//
//					apiMemory[address] = value;
//
//					sprintf(responseMessage, ":%04x%04x\n", address, apiMemory[address]);
//				}
//				else
//				{
//					sprintf(responseMessage, ":e%04x\n", API_ERROR_ACCESS_DENIED);
//				}
//			}
//			else
//			{
//				sprintf(responseMessage, ":e%04x\n", API_ERROR_WRONG_INSTRUCTOR);
//			}
//		}
//		else
//		{
//			sprintf(responseMessage, ":e%04x\n", API_ERROR_INVALID_ADDRESS);
//		}
//
//	}
//	else
//	{
//		sprintf(responseMessage, ":e%04x\n", API_ERROR_WRONG_FORMAT);
//	}
//	apiLocked = false;
//}
//
//bool isApiAddressValid(int address)
//{
//	if(address > 0 && address < API_MEMORY_SIZE)
//	{
//		return true;
//	}
//	else
//	{
//		return false;
//	}
//}
//
//bool apiWrite16(int address, int16_t value)
//{
//	if(isApiAddressValid(address))
//	{
//		memcpy(&apiMemory[address], &value, 2);
//		return true;
//	}
//	else
//	{
//		return false;
//	}
//}
//
//int16_t apiRead16(int address)
//{
//	int16_t val = -1;
//
//	if(isApiAddressValid(address))
//	{
//		memcpy(&val, &apiMemory[address], 4);
//	}
//
//	return val;
//}
//
//bool apiWrite32(int address, int32_t value)
//{
//	if(isApiAddressValid(address))
//	{
//		memcpy(&apiMemory[address], &value, 4);
//		return true;
//	}
//	else
//	{
//		return false;
//	}
//}
//
//int32_t apiRead32(int address)
//{
//	int32_t val = -1;
//
//	if(isApiAddressValid(address))
//	{
//		memcpy(&val, &apiMemory[address], 4);
//	}
//
//	return val;
//}
//
//
//bool apiWriteFloat(int address, float value)
//{
//	if(isApiAddressValid(address))
//	{
//		memcpy(&apiMemory[address], &value, 4);
//		return true;
//	}
//	else
//	{
//		return false;
//	}
//}
//
//float apiReadFloat(int address)
//{
//	float f =  -1.0;
//	if(isApiAddressValid(address))
//	{
//		memcpy(&f, &apiMemory[address], 4);
//	}
//
//	return f;
//}
