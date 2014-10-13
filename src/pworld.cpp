/*************************************************************
 * pworld.cpp
 * ---------------
 * Source file to implement pworld.h
 *
 * Last Revision: Oct. 13 2014
 *
 * TO DO: - Continue following tutorial to fill this out.
 *************************************************************/

#include "pworld.h"

namespace marballs
{
    void ParticleWorld::startFrame()
    {
        ParticleList *plist = firstParticle;

        while (plist)
        {
            // Removes forces from force accumulator
            plist->particle->ClearAccumulator();

            // Get next particle in list
            plist = plist->next;
        }
    }
}// end marballs namespace
