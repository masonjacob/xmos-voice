/* Speech Recognition HMM Grammar Vocabulary Description file
 * Copyright (C) 2003-2022 Sensory, Inc. All Rights Reserved.
 * 
 *
 *            source: ./tmp/CZZ8fAlesnWxzYpcwv3qRaLzQGu2/1597893098/trecs-en_US_12-13-5-0_8b429f88cf5af7bcff77a080d6e8b22f02735840.snsr
 *           created: Thu Oct 19 11:35:08 2023
 *   min lib version: 7.0.0
 *   operating point: 10
 *  production ready: NO - development only
 *       license key: yes
 * recognition limit: 107
 *    duration limit: 11.43 hours
 *
 * Created by VoiceHub 2.3.6
 * Project: XMOS_Command_Set_wakeword
 *
 * This model will stop working after a preset number of recognition events
 * and/or a after a preset number of audio "bricks" have been processed.
 *
 * ------------------------- DO NOT USE IN A PRODUCT -------------------------
 */

//extern u32 gs_wakeword_grammarLabel;
#ifndef NETLABEL
#define NETLABEL
//extern u32 dnn_wakeword_netLabel;
#endif

/* The following phrases (Hex format) correspond to the word IDs emitted by the recognizer. */
#define WAKEWORD_PHRASE_COUNT 3
#define WAKEWORD_PHRASE_0 "SILENCE"	/* Legacy system phrase */
#define WAKEWORD_PHRASE_1 "\x78\x6D\x6F\x73\x5F\x62\x6F\x61\x72\x64"	/* Phrase: xmos_board */
#define WAKEWORD_PHRASE_2 "nota"	/* Legacy system phrase */
