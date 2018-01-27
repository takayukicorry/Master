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
    
    char date[64];
    time_t t = time(NULL);
    strftime(date, sizeof(date), "%m%d_%H%M%S", localtime(&t));
    std::string fnstr{date};
    std::string fnstr_v = std::to_string(VERSION);
    std::string fnstr_avi, fnstr_data;
    
    fnstr_avi = "/Users/masudatakayuki/M1/修士論文/動画/"+fnstr+"_"+fnstr_v+".avi";
    const char *fn_avi = fnstr_avi.c_str();
    fnstr_data = "/Users/masudatakayuki/M1/修士論文/データ/"+fnstr+"_"+fnstr_v;
    const char *fn_data = fnstr_data.c_str();
    
    
    GAmanager manager = *new GAmanager(VERSION);
    Master master = *new Master(fn_avi);
    std::map<int, Starfish*> Ss;
    Population* pop;
    
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
    std::cout << ".....now evaluating CHAMP....." <<std::endl;
    for(curorg = pop->organisms.begin(); curorg != pop->organisms.end(); ++curorg) {
        double fitness = oph_evaluate(*curorg, manager.pool[0]);
        if (fitness > max_fitness){
            champ=*curorg;
            max_fitness=fitness;
        }
    }
    std::cout << max_fitness <<std::endl;

    Ss[VERSION]->setNet(champ->net);
    
    NEAT::print_Genome_tofile(champ->gnome, fn_data);
#endif
    master.setStarfish(Ss[VERSION]);
    mastermain(argc, argv, &master, SINGLE);
    
    return 0;
}

