/* dartwrap - dart simulator interface for torch
   Copyright (c) 2017, Friedrich Beckmann, Hochschule Augsburg

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>. */

#include "dartwrap.h"
#include <stdlib.h>

DartSim *dart_new(){
  DartSim *ds = malloc(sizeof(DartSim));
  ds->simtime = 0.0;
  return ds;
}

void dart_gc(DartSim *ds){
  if (ds)
    free(ds);
}

double dart_act(DartSim *ds,
                int action){
  ds->simtime += (double) action;
  return ds->simtime;
}


