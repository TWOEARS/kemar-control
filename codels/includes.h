#include <stdio.h>
#include <err.h>
#include <stdlib.h>
#include <stdbool.h>

#include "elmo-axis/socketcan.h"
#include "elmo-axis/harmonica.h"
#include "elmo-axis/kemar.h"

#define pi 3.14159265359

CAN_HARMONICA_STR *h;
KEMAR_POS_VEL_STR *k;
CAN_DRIVE_PARAMS *p;

double globalTarget;
/*Flags for Homing Function*/
int flagH, stepH;
/*Flag for Current Position*/
int flagCP, showCP;
/*Flag for Move Absoulte Position*/
int flagMAP;
/*Flag for Move Relative Position*/
int flagMRP;
/*Flag for Control in Speed*/
int flagCIS;
/*Flags for State Control*/
int flagC;
