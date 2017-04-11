#ifndef Variables_h

#define Variables_h
#define _USE_MATH_DEFINES

#define FPS 60.f

#define RADIAN 180/M_PI
#define SECOND 120//管足振る周期の半分

#define M_BODY 0
#define M_ARM 0
#define M_TF 1

#define LENGTH 12
#define RADIUS 4
#define INIT_POS_Y 24

/*以下、要調整*/
#define ANGLE M_PI/3//管足の限度
#define ANGLE_ATTACH -M_PI/8//管足、地面との吸着判定角度上限
#define ANGLE_DETACH ANGLE/2//管足、地面からの離脱判定角度下限
#define ANGLE_VELOCITY_TF ANGLE*60/SECOND//管足振る角速度
#define ANGLE_VELOCITY_GROUND ANGLE//管足地面間の振る角速度

#endif
