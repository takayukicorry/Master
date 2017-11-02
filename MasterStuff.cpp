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

static void idle() {
    stuff->idle();
}

void mastermain(int argc, char** argv, Master* master) {
    stuff = master;
    
    master->createGround();
    master->createStarfish();
    
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(640, 480);
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
    glutCreateWindow(argv[0]);
    glutDisplayFunc(Render);
    glutIdleFunc(idle);
    master->init();
    glutMainLoop();
    
    master->CleanupBullet();
}
