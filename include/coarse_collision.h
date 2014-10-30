/*************************************************************
 * coarse_collision.h
 * -------------
 * Header file for the coarse collision system.
 *
 * Last Revision: Nov. 30, 2014
 *
 * TO DO: - Continue following tutorial to fill this out
 *          (Page 245)
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

            // Insert - inserts given rigid body and given bounding volume
            // into the hierarchy (binary tree)
            void Insert(RigidBody* body, const BoundingVolumeClass &volume);

            // Deletes the node and removes it from the hierarchy.
            // This includes removing its associated rigid body and
            // preceding children nodes. Forces volume to recalculate
            ~BVHNode();

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

    template<class BoundingVolumeClass>
    void BVHNode<BoundingVolumeClass>::Insert(RigidBody* newBody, const BoundingVolumeClass &newVolume){
        // If we are a leaf, then the only option is to spawn two
        // new children and place the new body in one.
        if (isLeaf())
        {
            // Child one is a copy of us.
            children[0] = new BVHNode<BoundingVolumeClass>(this, volume, body)

            // Child two holds the new body
            children[1] = new BVHNode<BoundingVolumeClass>(this, newVolume, newBody);

            // And we now loosen the body (we’re no longer a leaf).
            this->body = NULL;

            // We need to recalculate our bounding volume.
            RecalculateBoundingVolume();
        }
        // Otherwise we need to work out which child gets to keep
        // the inserted body. We give it to whoever would grow the
        // least to incorporate it.
        else
        {
            if (children[0]->volume.GetGrowth(newVolume) < children[1]->volume.GetGrowth(newVolume))
            {
                children[0]->Insert(newBody, newVolume);
            }
            else
            {
                children[1]->Insert(newBody, newVolume);
            }
        }
    }

    template<class BoundingVolumeClass>
    BVHNode<BoundingVolumeClass>::~BVHNode<BoundingVolumeClass>(){
        // If we don’t have a parent, then we ignore the sibling processing.
        if (parent)
        {
            // Find our sibling.
            BVHNode<BoundingVolumeClass> *sibling;
            if (parent->children[0] == this) sibling = parent->children[1];
            else sibling = parent->children[0];

            // Write its data to our parent.
            parent->volume = sibling->volume;
            parent->body = sibling->body;
            parent->children[0] = sibling->children[0];
            parent->children[1] = sibling->children[1];

            // Delete the sibling (we blank its parent and
            // children to avoid processing/deleting them).
            sibling->parent = NULL;
            sibling->body = NULL;
            sibling->children[0] = NULL;
            sibling->children[1] = NULL;
            delete sibling;

            // Recalculate the parent’s bounding volume.
            parent->recalculateBoundingVolume();
        }

        // Delete our children (again we remove their parent data so
        // we don’t try to process their siblings as they are deleted).
        if (children[0]) {
            children[0]->parent = NULL;
            delete children[0];
        }
        if (children[1]) {
            children[1]->parent = NULL;
            delete children[0];
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
