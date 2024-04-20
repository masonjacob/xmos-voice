/* Speech Recognition HMM Grammar Vocabulary Description file
 * Copyright (C) 2003-2022 Sensory, Inc. All Rights Reserved.
 * 
 *
 *            source: ./tmp/CZZ8fAlesnWxzYpcwv3qRaLzQGu2/1597893098/trecs-en_US_12-13-5-0_5d7ff8bee216504e5a10a964d25db6545f719006.snsr
 *           created: Thu Oct 19 11:35:08 2023
 *   min lib version: 6.4.0
 *   operating point: 10
 *  production ready: NO - development only
 *       license key: yes
 * recognition limit: 107
 *    duration limit: 11.43 hours
 *
 * Created by VoiceHub 2.3.6
 * Project: XMOS_Command_Set_command
 *
 * This model will stop working after a preset number of recognition events
 * and/or a after a preset number of audio "bricks" have been processed.
 *
 * ------------------------- DO NOT USE IN A PRODUCT -------------------------
 */

//extern u32 gs_command_grammarLabel;
#ifndef NETLABEL
#define NETLABEL
//extern u32 dnn_command_netLabel;
#endif

/* The following phrases (Hex format) correspond to the word IDs emitted by the recognizer. */
#define COMMAND_PHRASE_COUNT 9
#define COMMAND_PHRASE_0 "SILENCE"	/* Legacy system phrase */
#define COMMAND_PHRASE_1 "\x6D\x6F\x76\x65\x5F\x72\x69\x67\x68\x74"	/* Phrase: move_right */
#define COMMAND_PHRASE_2 "\x6D\x6F\x76\x65\x5F\x6C\x65\x66\x74"	/* Phrase: move_left */
#define COMMAND_PHRASE_3 "\x6D\x6F\x76\x65\x5F\x75\x70"	/* Phrase: move_up */
#define COMMAND_PHRASE_4 "\x6D\x6F\x76\x65\x5F\x64\x6F\x77\x6E"	/* Phrase: move_down */
#define COMMAND_PHRASE_5 "\x62\x6C\x69\x6E\x6B\x5F\x6F\x6E\x63\x65"	/* Phrase: blink_once */
#define COMMAND_PHRASE_6 "\x62\x6C\x69\x6E\x6B\x5F\x66\x69\x76\x65\x5F\x74\x69\x6D\x65\x73"	/* Phrase: blink_five_times */
#define COMMAND_PHRASE_7 "\x73\x5F\x6F\x5F\x73"	/* Phrase: s_o_s */
#define COMMAND_PHRASE_8 "nota"	/* Legacy system phrase */
