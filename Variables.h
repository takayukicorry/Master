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
#define MAX_CYCLE 2000
#define MIN_CYCLE 300
#define MAX_ANGLE M_PI_2
#define MIN_ANGLE -M_PI_2
#define MAX_CYCLE_2 2000
#define MIN_CYCLE_2 200
#define MAX_ANGLE_2 M_PI/3
#define MIN_ANGLE_2 -M_PI/3
#define MAX_ANGLE2_2 M_PI
#define MIN_ANGLE2_2 -M_PI
#define NUM_JOINT 2

#define MAX_MOTOR_TORQUE 500.f//出力[W] ＝ ( 2 * M_PI / 60 ) × T[N・m] × θ[rad/min]

#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480


#define BODYPART_COUNT (NUM_JOINT+1) * NUM_LEGS + 1 //+ NUM_LEGS //最後のプラスはturn判定センサ
#define JOINT_COUNT BODYPART_COUNT - 1 //+ NUM_LEGS
#define NUM_TURN 2
#define SWING_ANGLE M_PI
#define FRICTION 5.0
#define M_OBJ 0.5f
#define M_OBJ0 0.5f
#define M_TF 0.5f

#define FPS 60.f
#define RADIAN 180/M_PI
#define SECONDS 120//管足振る周期の半分//kannsokufurushuukinohannbunn

#define FBODY_SIZE 15.f
#define FLEG_LENGTH 22.5f/NUM_JOINT
#define FLEG_WIDTH 10.f
#define FHEIGHT 15.f
#define RADIUS 3
#define LENGTH 10
#define RADIUS_TF 1
#define INIT_POS_Y LENGTH + RADIUS_TF*3 + 2
#define NUM_GROUND 3
#define NUM_TF_UNIT 8//>=4
#define NUM_TF NUM_TF_UNIT*NUM_JOINT
#define DL_TIME 120
#define RE_TIME 120
#define THRESH_VEL 5
#define THRESH_TURN 0.8
#define THRESHOLD_ROT M_PI/2
#define TF_PERCENT 70
/*以下、要調整*/
#define ANGLE M_PI/3//管足の限度//kannsokunogenndo
#define ANGLE_ATTACH M_PI/8//管足、地面との吸着判定角度上限//kannsoku,jimenntonokyuutyakuhannteikakudojougenn
#define ANGLE_ATTACH_Z_foward M_PI_2+ANGLE_ATTACH//管足、地面との吸着判定角度上限//kannsoku,jimenntonokyuutyakuhannteikakudojougenn
#define ANGLE_ATTACH_Z_back M_PI_2-ANGLE_ATTACH//管足、地面との吸着判定角度上限//kannsoku,jimenntonokyuutyakuhannteikakudojougenn
#define ANGLE_DETACH -ANGLE/2//管足、地面からの離脱判定角度下限//kannsoku,jimenntonoridatuhannteikakudojougenn
#define ANGLE_VELOCITY_TF ANGLE*60/SECONDS//管足振る角速度//kannsokufurukakusokudo
#define ANGLE_VELOCITY_GROUND ANGLE//管足地面間の振る角速度//kannsokujimennkannnofurukakusokudo

//*******************************//
#define NUM_GENARATION 100
#define SIMULATION_TIME_STEP 1200
#define GA 0
#define NT 1
#define VERSION_1_3 0
#define SINGLE 1

#define VERSION 1
#define WALL 0
//*******************************//
#endif
