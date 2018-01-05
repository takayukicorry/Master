//
//  main.cpp
//  Master
//
//  Created by 増田貴行 on 2017/10/31.
//  Copyright © 2017年 増田貴行. All rights reserved.
//
#include <stdio.h>
#include "MasterStuff.hpp"
#include "Master.hpp"
#include "GAMain.hpp"

int main (int argc, char** argv) {
    srand((unsigned)time(NULL));
    
    GAmanager manager = *new GAmanager(VERSION);
    Master master = *new Master();
    std::map<int, Starfish*> Ss;
    
#if NT
    for (int i = 0; i < NUM_GENARATION; i++) {
        
        std::cout << "第" << i << "世代　最優秀個体:" <<  <<std::endl;
    }
#elseif GA
    for (int i = 0; i < NUM_GENARATION; i++) {
        manager.CreateNext();
        std::cout << "第" << i << "世代　最優秀個体:" << manager.evalue(manager.pool[0]) <<std::endl;
    }
#endif
    
    Ss[1] = new Ophiuroid(manager.pool[0]);
    Ss[2] = new Ophiuroid2(manager.pool[0]);
    Ss[3] = new Ophiuroid3(manager.pool[0]);
    Ss[VERSION]->kCheck_first = true;
    
    master.setStarfish(Ss[VERSION]);
    mastermain(argc, argv, &master, GA);
    
    return 0;
}

