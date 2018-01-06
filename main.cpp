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
    Population* pop;
    
    char date[64];
    
#if NT
    pop = oph_test_realtime(&manager);
#elif GA
    for (int i = 0; i < NUM_GENARATION; i++) {
        time_t t = time(NULL);
        strftime(date, sizeof(date), "%Y/%m/%d %a %H:%M:%S", localtime(&t));
        printf("第%d世代 %s\n", i, date);
        
        manager.CreateNext();
        std::cout << "最優秀個体:" << manager.evalue(manager.pool[0]) <<std::endl;
    }
#endif
    Ss[1] = new Ophiuroid(manager.pool[0]);
    Ss[2] = new Ophiuroid2(manager.pool[0]);
    Ss[3] = new Ophiuroid3(manager.pool[0]);
    Ss[VERSION]->kCheck_first = true;
    
#if NT
    std::vector<Organism*>::iterator curorg;
    double max_fitness=0;
    Organism *champ;
    champ=*(pop->organisms.begin()); //Make sure at least something is chosen
    //Find the population champ
    for(curorg = pop->organisms.begin(); curorg != pop->organisms.end(); ++curorg) {
        if ((*curorg)->fitness > max_fitness){
            champ=(*curorg);
            max_fitness=champ->fitness;
        }
    }
    Ss[VERSION]->setNet(champ->net);
#endif
    master.setStarfish(Ss[VERSION]);
    mastermain(argc, argv, &master, SINGLE);
    
    return 0;
}

