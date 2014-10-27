#include <cstdlib>
#include "world.h"

using namespace marballs;

//World::World(unsigned maxContacts, unsigned iterations)
//:
//firstBody(NULL),
//resolver(iterations),
//firstContactGen(NULL),
//maxContacts(maxContacts)
//{
//    contacts = new Contact[maxContacts];
//    calculateIterations = (iterations == 0);
//}

//World::~World()
//{
//    delete[] contacts;
//}

void World::startFrame()
{
    BodyRegistration *reg = firstBody;
    while (reg)
    {
        // Remove all forces from the accumulator
        reg->body->ClearAccumulators();
        reg->body->CalculateDerivedData();

        // Get the next registration
        reg = reg->next;
    }
}

//unsigned World::generateContacts()
//{
//    unsigned limit = maxContacts;
//    Contact *nextContact = contacts;
//
//    ContactGenRegistration * reg = firstContactGen;
//    while (reg)
//    {
//        unsigned used = reg->gen->addContact(nextContact, limit);
//        limit -= used;
//        nextContact += used;
//
//        // We're missing contacts.
//        if (limit <= 0) break;
//
//        reg = reg->next;
//    }
//
//    // Return the number of contacts used.
//    return maxContacts - limit;
//}

void World::runPhysics(marb duration)
{
    // First apply the force generators
    //registry.updateForces(duration);

    // Then integrate the objects
    BodyRegistration *reg = firstBody;
    while (reg)
    {
        // Remove all forces from the accumulator
        reg->body->Integrate(duration);

        // Get the next registration
        reg = reg->next;
    }

//    // Generate contacts
//    unsigned usedContacts = generateContacts();
//
//    // And process them
//    if (calculateIterations) resolver.setIterations(usedContacts * 4);
//    resolver.resolveContacts(contacts, usedContacts, duration);
}
