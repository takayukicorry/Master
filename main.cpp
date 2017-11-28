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
    GAmanager manager = *new GAmanager();
    //Starfish* oph = new Ophiuroid(manager.pool[0]);
    Starfish* oph2 = new Ophiuroid2(manager.pool[0]);
    Master master = *new Master();
    
    master.setStarfish(oph2);
    master.setManager(manager);
    mastermain(argc, argv, &master);
    return 0;
}
