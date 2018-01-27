#ifndef Variables_h

#define Variables_h
#define _USE_MATH_DEFINES

//#include "btBulletDynamicsCommon.h"
#include <BulletDynamics/btBulletDynamicsCommon.h>
#include <stdio.h>
#include <stdlib.h>
//#include <GL/glut.h>
//#include <GLUT/GLUT.h>
#include <GL/freeglut.h>
#include <OpenGL/OpenGL.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <map>
#include <random>
#include <string>
//#include <OpenGL/DemoApplication.h>

enum CollisionGroup{
    RX_COL_NOTHING = 0, // 0000
    RX_COL_GROUND = 1, // 0001
    RX_COL_BODY = 2,  // 0010
    RX_COL_TF = 4,  // 0100
    RX_COL_AMP = 8   // 1000
};

/*卒論で使用*/
#define ARRAY_LENGTH (NUM_JOINT+2)*NUM_LEGS
#define ARRAY_LENGTH_2 NUM_TF*NUM_LEGS
#define NUM_LEGS 5
#define CONECT_LENGTH 2*NUM_LEGS*NUM_LEGS
#define POOL_SIZE 100
#define MAX_CYCLE 1000
#define MIN_CYCLE 100
#define MAX_ANGLE M_PI_2
#define MIN_ANGLE -M_PI_2
#define MAX_CYCLE_2 500
#define MIN_CYCLE_2 200
#define MAX_ANGLE_2 M_PI/3
#define MIN_ANGLE_2 -M_PI/3
#define MAX_ANGLE2_2 M_PI
#define MIN_ANGLE2_2 -M_PI
#define NUM_JOINT 1
#define MAX_MOTOR_TORQUE 0.8f//出力[W] ＝ ( 2 * M_PI / 60 ) × T[N・m] × θ[rad/min]
#define MAX_MOTOR_TORQUE_B 800000000000.f//出力[W] ＝ ( 2 * M_PI / 60 ) × T[N・m] × θ[rad/min]

#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480


#define BODYPART_COUNT (NUM_JOINT+1) * NUM_LEGS + 1 //+ NUM_LEGS //最後のプラスはturn判定センサ
#define JOINT_COUNT BODYPART_COUNT - 1 //+ NUM_LEGS
#define NUM_TURN 2
#define SWING_ANGLE M_PI
#define FRICTION 5.0
#define M_OBJ 0.5f
#define M_OBJ0 0.5f
#define M_TF 5.f

#define FPS 60.f
#define RADIAN 180/M_PI
#define SECONDS 120//管足振る周期の半分//kannsokufurushuukinohannbunn

#define FBODY_SIZE 1.5f
#define FLEG_LENGTH 2.4f/NUM_JOINT
#define FLEG_WIDTH 0.9f
#define FHEIGHT 1.5f
#define RADIUS_TF 0.25f
#define LENGTH 0.9f

#define RADIUS 1

#define INIT_POS_Y LENGTH + RADIUS_TF*3 + 2
#define NUM_GROUND 7
#define NUM_TF_UNIT 8//>=4
#define NUM_TF NUM_TF_UNIT*NUM_JOINT
#define DL_TIME 120
#define RE_TIME 120
#define THRESH_VEL 5
#define THRESH_TURN M_PI
#define THRESHOLD_ROT M_PI/2
#define TF_PERCENT 80
/*以下、要調整*/
#define ANGLE M_PI/3//管足の限度//kannsokunogenndo
#define ANGLE_ATTACH M_PI/20//管足、地面との吸着判定角度上限//kannsoku,jimenntonokyuutyakuhannteikakudojougenn
#define ANGLE_ATTACH_Z_foward M_PI_2+ANGLE_ATTACH//管足、地面との吸着判定角度上限//kannsoku,jimenntonokyuutyakuhannteikakudojougenn
#define ANGLE_ATTACH_Z_back M_PI_2-ANGLE_ATTACH//管足、地面との吸着判定角度上限//kannsoku,jimenntonokyuutyakuhannteikakudojougenn
#define ANGLE_DETACH -ANGLE/2//管足、地面からの離脱判定角度下限//kannsoku,jimenntonoridatuhannteikakudojougenn
#define ANGLE_VELOCITY_TF ANGLE*60/SECONDS//管足振る角速度//kannsokufurukakusokudo
#define ANGLE_VELOCITY_GROUND ANGLE//管足地面間の振る角速度//kannsokujimennkannnofurukakusokudo

//*******************************//
#define NUM_GENARATION 100
#define SIMULATION_TIME_STEP 1200
#define GA 0
#define NT 0
#define VERSION_1_3 0
#define SINGLE 1

#define VERSION 3
#define WALL 1

//ver3 3個発生源
#define FIRST_STEP 2000
#define SECOND_STEP 5000
//*******************************//
#endif
