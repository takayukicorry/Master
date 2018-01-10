//
//  MasterStuff.cpp
//  Master
//
//  Created by 増田貴行 on 2017/11/02.
//  Copyright © 2017年 増田貴行. All rights reserved.
//

#include "MasterStuff.hpp"

static Master* stuff = 0;

static void Render() {
    stuff->Render();
}

static void Key(unsigned char key, int x, int y) {
    stuff->keyboard(key, x, y);
}

static void idle() {
    stuff->idle();
}

static void idleDemo() {
    stuff->idleDemo();
}

void mastermain(int argc, char** argv, Master* master, bool single) {
    stuff = master;
    
    master->createGround();
    master->createStarfish();
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
    glutCreateWindow(argv[0]);
    glutDisplayFunc(Render);
    glutKeyboardFunc(Key);
    if (single) {
        glutIdleFunc(idleDemo);
    } else {
        glutIdleFunc(idle);
    }
    master->init();
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,GLUT_ACTION_GLUTMAINLOOP_RETURNS);
    glutMainLoop();
    
    master->CleanupBullet();
}


