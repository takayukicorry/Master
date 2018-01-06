//
//  GAMain.hpp
//  Master
//
//  Created by 増田貴行 on 2018/01/05.
//  Copyright © 2018年 増田貴行. All rights reserved.
//

#ifndef GAMain_hpp
#define GAMain_hpp

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <list>
#include <vector>
#include <algorithm>
#include <cmath>
#include <string>
#include "neat.hpp"
#include "network.hpp"
#include "population.hpp"
#include "organism.hpp"
#include "genome.hpp"
#include "species.hpp"
#include "Ophiuroid.hpp"
#include "Ophiuroid2.hpp"
#include "Ophiuroid3.hpp"
#include "GA2.h"

using namespace NEAT;

Population *oph_test_realtime(GAmanager*);
int oph_epoch(Population*, int, GAmanager*);
double oph_evaluate(Organism*, GAparameter);

#endif /* GAMain_hpp */
