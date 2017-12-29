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

int main (int argc, char** argv) {
    GAmanager manager = *new GAmanager(1);//どっちの挙動のGAやるか
    Starfish* oph = new Ophiuroid(manager.pool[0]);
    Starfish* oph2 = new Ophiuroid2(manager.pool[0]);
    Master master = *new Master();//世界観作成
    
#if 0
    master.setStarfish(oph);
    master.setParameter(manager.pool[0]);
    mastermain(argc, argv, &master);
#else
    for (int i = 0; i < NUM_GENARATION; i++) {
        manager.CreateNext();
        std::cout << "第" << i << "世代　最優秀個体:" << manager.evalue(manager.pool[0]) <<std::endl;
    }
    
#endif
    return 0;
}

