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
    NEAT::pop_size = 100;
    
    //
    //start
    //
    Genome *start_genome = new Genome(NUM_LEGS, NUM_LEGS, NUM_LEGS*NUM_LEGS, 1);
    Population *pop = new Population(start_genome,NEAT::pop_size);
    pop->verify();
    
    Starfish* oph = new Ophiuroid3(m->pool[0]);
    oph_realtime_loop(pop, oph, m);
    
    return pop;
}

bool oph_evaluate(Organism *org, Starfish *oph) {
    Network *net = org->net;
    
    org->fitness = oph->evalue_NEAT(net);
    
    org->winner = false;
    
    return false;
}

void oph_realtime_loop(Population *pop, Starfish *oph, GAmanager* m) {
    std::vector<Organism*>::iterator curorg;
    std::vector<Species*>::iterator curspecies;
    std::vector<Species*> sorted_species;
    bool win=false;

    //We try to keep the number of species constant at this number
    int num_species_target=NEAT::pop_size/15;
    
    //This is where we determine the frequency of compatibility threshold adjustment
    int compat_adjust_frequency = NEAT::pop_size/10;
    if (compat_adjust_frequency < 1)
        compat_adjust_frequency = 1;
    
    //Initially, we evaluate the whole population
    //Evaluate each organism on a test
    for(curorg=(pop->organisms).begin();curorg!=(pop->organisms).end();++curorg) {
        //shouldn't happen
        if (((*curorg)->gnome)==0) {
            std::cout<<"ERROR EMPTY GEMOME!"<<std::endl;
            //cin>>pause;
        }
        oph->setParam(m->pool[(*curorg)->species->id]);
        if (oph_evaluate((*curorg),oph)) win=true;
        
    }

    //Get ready for real-time loop
    
    //Rank all the organisms from best to worst in each species
    pop->rank_within_species();
    
    //Assign each species an average fitness
    //This average must be kept up-to-date by rtNEAT in order to select species probabailistically for reproduction
    pop->estimate_all_averages();
    
    //Real-time evolution variables
    int offspring_count;
    Organism *new_org;
    
    //Now create offspring one at a time, testing each offspring,
    // and replacing the worst with the new offspring if its better
    for (offspring_count=0;offspring_count<20000;offspring_count++) {
        
        
        //Every pop_size reproductions, adjust the compat_thresh to better match the num_species_targer
        //and reassign the population to new species
        if (offspring_count % compat_adjust_frequency == 0) {
            
            int num_species = (int)pop->species.size();
            double compat_mod=0.1;  //Modify compat thresh to control speciation
            
            // This tinkers with the compatibility threshold
            if (num_species < num_species_target) {
                NEAT::compat_threshold -= compat_mod;
            }
            else if (num_species > num_species_target)
                NEAT::compat_threshold += compat_mod;
            
            if (NEAT::compat_threshold < 0.3)
                NEAT::compat_threshold = 0.3;
            
            std::cout<<"compat_thresh = "<<NEAT::compat_threshold<<std::endl;
            
            //Go through entire population, reassigning organisms to new species
            for (curorg = (pop->organisms).begin(); curorg != pop->organisms.end(); ++curorg) {
                pop->reassign_species(*curorg);
            }
        }
        
        std::cout<<"Pop size: "<<pop->organisms.size()<<std::endl;
        
        //Here we call two rtNEAT calls:
        //1) choose_parent_species() decides which species should produce the next offspring
        //2) reproduce_one(...) creates a single offspring fromt the chosen species
        new_org=(pop->choose_parent_species())->reproduce_one(offspring_count,pop,pop->species);
        
        //Now we evaluate the new individual
        //Note that in a true real-time simulation, evaluation would be happening to all individuals at all times.
        //That is, this call would not appear here in a true online simulation.
        std::cout<<"Evaluating new baby: "<<std::endl;
        
        oph->setParam(m->pool[new_org->species->id]);
        if (oph_evaluate(new_org,oph)) win=true;
        
        if (win) {
            std::cout<<"WINNER"<<std::endl;
            break;
        }
        
        //Now we reestimate the baby's species' fitness
        new_org->species->estimate_average();
        
        //Remove the worst organism
        pop->remove_worst();
        
    }
    
}
