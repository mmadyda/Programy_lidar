/*
 *  @author KKest
 *		@created 19.01.2022
 *	
 * Types for rpLidar library
 *
 */
 
#include "rpLidarTypes.h"

rp_descriptor_t resp_descriptor[]={{0xA5,0x5A,0x54,0x00,0x00,0x40,0x82},//Legacy Version
								{0xA5,0x5A,0x84,0x00,0x00,0x40,0x84},//Extended Version
								{0xA5,0x5A,0x54,0x00,0x00,0x40,0x85},//Dense Version
								{0xA5,0x5A,0x05,0x00,0x00,0x40,0x81},//StartScan
								{0xA5,0x5A,0x05,0x00,0x00,0x40,0x81},//Force Scan
								{0xA5,0x5A,0x14,0x00,0x00,0x00,0x04},//Get Device Info
								{0xA5,0x5A,0x03,0x00,0x00,0x00,0x06},//Get Health Info
								{0xA5,0x5A,0x04,0x00,0x00,0x00,0x15},//Get sample rate
								{0xA5,0x5A,0x04,0x00,0x00,0x00,0x15}};//Device configuration

rq_message_t req_message[]={{0xA5,0x25}, //Stop
							{0xA5,0x40},	//Reset
							{0xA5,0x20},//Scan
							{0xA5,0x82}, //Express scan
							{0xA5,0x21}, //Force Scan
							{0xA5,0x50}, //Get Info
							{0xA5,0x52}, //Get Health
							{0xA5,0x59}, //Get Samplerate
							{0xA5,0x84}};//Get device Conf
							
rq_Packet_t req_Express[]={{0xA5,0x82,0x05,0x00,0x00,0x00,0x00,0x00,0x22}, //Legacy Mode
							{0xA5,0x82,0x05,0x00,0x00,0x00,0x00,0x00,0x22}, //Extended (Byte 4 = ScanMode ID)
							{0xA5,0x82,0x05,0x00,0x00,0x00,0x00,0x00,0x22}}; //Dense Mode