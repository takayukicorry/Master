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

/*色など*/
GLfloat light0pos[] = { 300.0, 300.0, 300.0, 1.0 };
GLfloat light1pos[] = { -300.0, 300.0, 300.0, 1.0 };
struct MaterialStruct {
    GLfloat ambient[4];
    GLfloat diffuse[4];
    GLfloat specular[4];
    GLfloat shininess;
};
MaterialStruct ms_jade = {
    { 0.135, 0.2225, 0.1575, 1.0 },
    { 0.54, 0.89, 0.63, 1.0 },
    { 0.316228, 0.316228, 0.316228, 1.0 },
    12.8 };
MaterialStruct ms_ruby = {
    { 0.1745, 0.01175, 0.01175, 1.0 },
    { 0.61424, 0.04136, 0.04136, 1.0 },
    { 0.727811, 0.626959, 0.626959, 1.0 },
    76.8 };
GLfloat red[] = { 0.8, 0.2, 0.2, 1.0 };
GLfloat green[] = { 0.2, 0.8, 0.2, 1.0 };
GLfloat blue[] = { 0.2, 0.2, 0.8, 1.0 };
GLfloat yellow[] = { 0.8, 0.8, 0.2, 1.0 };
GLfloat white[] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat shininess = 30.0;
enum CollisionGroup{
    RX_COL_NOTHING = 0, // 0000
    RX_COL_GROUND = 1, // 0001
    RX_COL_BODY = 2,  // 0010
    RX_COL_TF = 4,  // 0100
    RX_COL_AMP = 8   // 1000
};
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
