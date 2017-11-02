#ifndef Variables_h

#define Variables_h
#define _USE_MATH_DEFINES

//#include "btBulletDynamicsCommon.h"
#include <BulletDynamics/btBulletDynamicsCommon.h>
#include <stdio.h>
#include <stdlib.h>
//#include <GL/glut.h>
#include <GLUT/GLUT.h>
#include <OpenGL/OpenGL.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <map>
#include <random>
//#include <OpenGL/DemoApplication.h>

/*卒論で使用*/
#define ARRAY_LENGTH 100
#define NUM_LEGS 5
#define CONECT_LENGTH 5
#define POOL_SIZE 100
#define MAX_CYCLE 5000
#define MIN_CYCLE 1500
#define MAX_ANGLE M_PI
#define MIN_ANGLE M_PI/3


#define FPS 60.f
#define RADIAN 180/M_PI
#define SECOND 120//管足振る周期の半分//kannsokufurushuukinohannbunn

#define M_BODY 0
#define M_ARM 0
#define M_TF 1

#define LENGTH 12
#define RADIUS 4
#define INIT_POS_Y 24
#define NUM_GROUND 2

/*以下、要調整*/
#define ANGLE M_PI/3//管足の限度//kannsokunogenndo
#define ANGLE_ATTACH M_PI/8//管足、地面との吸着判定角度上限//kannsoku,jimenntonokyuutyakuhannteikakudojougenn
#define ANGLE_DETACH -ANGLE/2//管足、地面からの離脱判定角度下限//kannsoku,jimenntonoridatuhannteikakudojougenn
#define ANGLE_VELOCITY_TF ANGLE*60/SECOND//管足振る角速度//kannsokufurukakusokudo
#define ANGLE_VELOCITY_GROUND ANGLE//管足地面間の振る角速度//kannsokujimennkannnofurukakusokudo

#endif
