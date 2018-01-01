//
//  main.cpp
//  Master
//
//  Created by 増田貴行 on 2017/10/31.
//  Copyright © 2017年 増田貴行. All rights reserved.
//
#define DEMO 1
#define VERSION 1
#include <stdio.h>
#include "MasterStuff.hpp"
#include "Master.hpp"

int main (int argc, char** argv) {
    GAmanager manager = *new GAmanager(VERSION);
    Master master = *new Master();
    std::map<int, Starfish*> Ss;
    
#if DEMO
    for (int i = 0; i < 10; i++) {
        manager.CreateNext();
        std::cout << "第" << i << "世代　最優秀個体:" << manager.evalue(manager.pool[0]) <<std::endl;
    }
#endif
    
    Ss[1] = new Ophiuroid(manager.pool[0]);
    Ss[2] = new Ophiuroid2(manager.pool[0]);
    Ss[VERSION]->kCheck_first = true;
    
    master.setStarfish(Ss[VERSION]);
    mastermain(argc, argv, &master, DEMO);
    
    return 0;
}

