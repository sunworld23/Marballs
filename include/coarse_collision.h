/*************************************************************
 * coarse_collision.h
 * -------------
 * Header file for the coarse collision system.
 *
 * Last Revision: Nov. 30, 2014
 *
 * TO DO: - Continue following tutorial to fill this out
 *          (Page 241, section 12.3.2)
 *************************************************************/

#ifndef COARSE_COLLISION_INCLUDED
#define COARSE_COLLISION_INCLUDED

#include "decimal_precision.h"
#include "body.h"

namespace marballs
{
    struct PotentialContact // Stores a potential contact.
    {
        RigidBody* body[2]; // Holds the potential contacts
    };

    /* Class BVHNode - Base class for nodes in a bound hierarchy.
     * Uses a binary tree to store the bounding volumes. */
     template<class BoundingVolumeClass>
     class BVHNode
     {
        public:
            /*********************************
            *     Variable Declarations
            *********************************/
            BVHNode* children[2]; // Holds the children value of the tree
            BoundingVolumeClass volume; // Holds a single bounding volume for all nodes in tree
            RigidBody* body; // Rigid body at this node of the tree

            /*********************************
            *     Function Declarations
            *********************************/

            // IsLeaf - returns whether or not current node is a leaf node.
            bool IsLeaf() const{
                return (body != NULL);
            }

            // GetPotentialContacts - Checks contacts from this node downward
            // in the tree hierarchy then stores them into an array. Returns
            // number of potential contacts found.
            unsigned GetPotentialContacts(PotentialContact* contacts, unsigned limit) const;

     }; // End BVHNode class

    template<class BoundingVolumeClass>
    bool BVHNode<BoundingVolumeClass>::Overlaps(const BVHNode<BoundingVolumeClass> * other) const{
        return volume->Overlaps(other->volume);
    }

    template<class BoundingVolumeClass>
    unsigned BVHNode<BoundingVolumeClass>::GetPotentialContacts(PotentialContact* contacts, unsigned limit) const{
        // Quit early if we don’t have the room for contacts, or if we are at a leaf node
        if (isLeaf() || limit == 0) return 0;

        // Get the potential contacts of one of our children with the other
        return children[0]->GetPotentialContactsWith(children[1], contacts, limit);
    }

    template<class BoundingVolumeClass>
    unsigned BVHNode<BoundingVolumeClass>::GetPotentialContactsWith(const BVHNode<BoundingVolumeClass> *other,
                                                                    PotentialContact* contacts,
                                                                    unsigned limit) const{
        // Quit early if we don’t overlap or if we have no room
        // to report contacts.
        if (!Overlaps(other) || limit == 0) return 0;

        // If we’re both at leaf nodes, then we have a potential contact.
        if (IsLeaf() && other->IsLeaf())
        {
            contacts->body[0] = body;
            contacts->body[1] = other->body;
            return 1;
        }

        // Determine which node to descend into. If either is
        // a leaf, then we descend the other. If both are branches,
        // then we use the one with the largest size.
        if (other->IsLeaf() || (!IsLeaf() && volume->GetSize() >= other->volume->GetSize()))
        {
            // Recurse into ourself
            unsigned count = children[0]->GetPotentialContactsWith(other, contacts, limit);

            // Check whether we have enough slots to do the other side too.
            if (limit > count) {
                return count + children[1]->GetPotentialContactsWith(other, contacts+count, limit-count);
            }
            else
            {
                return count;
            }
        }
        else
        {
            // Recurse into the other node.
            unsigned count = GetPotentialContactsWith(other->children[0], contacts, limit);

            // Check whether we have enough slots to do the other side too.
            if (limit > count) {
                return count + GetPotentialContactsWith(other->children[1], contacts+count, limit-count);
            }
            else
            {
                return count;
            }
        }
    }

    struct BoundingSphere // Bounding sphere to test for overlap
    {
        Vector3 center;
        marb radius;

        public:
            // BoundingSphere - Constructor for bounding sphere
            BoundingSphere(const Vector3 &center, marb radius);

            // BoundingSphere - Different type of constructor
            BoundingSphere(const BoundingSphere &one, const BoundingSphere &two);

            // Overlaps - checks if bounding sphere overlaps with other given bounding sphere
            bool Overlaps(const BoundingSphere *other) const;
    };

    bool BoundingSphere::Overlaps(const){
        marb distanceSquared = (center - other->center).SquareMagnitude();
        return distanceSquared < (radius+other->radius)*(radius+other->radius);
    }

}// End marballs namespace
#endif // COARSE_COLLISION_INCLUDED
