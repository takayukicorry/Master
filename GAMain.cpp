//
//  GAMain.cpp
//  Master
//
//  Created by 増田貴行 on 2018/01/05.
//  Copyright © 2018年 増田貴行. All rights reserved.
//

#include "GAMain.hpp"

Population *oph_test_realtime(GAmanager* m) {
    //
    //set param
    //
    const char *filename = "/Users/masudatakayuki/M1/Master/Master/param.txt";
    NEAT::load_neat_params(filename, false);
    
    char date[64];
    Population *pop=0;
    Genome *start_genome;
    //char curword[20];
    //int id;
    //std::ostringstream *fnamebuf;
    int gen;
    int expcount;
    int status;
    int runs[NEAT::num_runs];
    int totalevals;
    int samples;
    
    memset (runs, 0, NEAT::num_runs * sizeof(int));
    
    //
    //start
    //
    start_genome = new Genome(NUM_LEGS, NUM_LEGS, NUM_LEGS*NUM_LEGS, 1);
    for(expcount=0;expcount<NEAT::num_runs;expcount++) {
        pop= new Population(start_genome,NEAT::pop_size);
        pop->verify();
        
        for (gen=1;gen<=NUM_GENARATION;gen++) {
            std::cout << "ただいま第" << gen << "世代" << std::endl;
            time_t t = time(NULL);
            strftime(date, sizeof(date), "%Y/%m/%d %a %H:%M:%S", localtime(&t));
            printf("%s\n", date);
            
            status = oph_epoch(pop,gen,m);
            
            if (status) {
                
            }
            
        }
        
        if (expcount<NEAT::num_runs-1) delete pop;
    }
    
    totalevals=0;
    samples=0;
    for(expcount=0;expcount<NEAT::num_runs;expcount++) {
        std::cout<<runs[expcount]<<std::endl;
        if (runs[expcount]>0)
        {
            totalevals+=runs[expcount];
            samples++;
        }
    }
    
    std::cout<<"Failures: "<<(NEAT::num_runs-samples)<<" out of "<<NEAT::num_runs<<" runs"<<std::endl;
    std::cout<<"Average evals: "<<(samples>0 ? (double)totalevals/samples : 0)<<std::endl;
    
    return pop;
}

int oph_epoch(Population *pop,int generation, GAmanager* m) {
    std::vector<Organism*>::iterator curorg;
    std::vector<Species*>::iterator curspecies;
    int i = 0;
    
    for(curorg=(pop->organisms).begin();curorg!=(pop->organisms).end();++curorg) {
        m->v[i] = oph_evaluate(*curorg, m->pool[i]);
        i++;
    }
    
    for(curspecies=(pop->species).begin();curspecies!=(pop->species).end();++curspecies) {
        (*curspecies)->compute_average_fitness();
        (*curspecies)->compute_max_fitness();
    }
    
    //Create the next generation
    pop->epoch(generation);
    m->CreateNext();
    
    return 0;
}

double oph_evaluate(Organism *org, GAparameter p) {
    double v;
    Network *net = org->net;
    
    if (VERSION==1) {
        Ophiuroid oph(p);
        v = oph.evalue_NEAT(net);
    } else if (VERSION==3) {
        Ophiuroid3 oph(p);
        v = oph.evalue_NEAT(net);
    }
    
    org->fitness = v;
    org->winner = false;
    
    return v;
}
