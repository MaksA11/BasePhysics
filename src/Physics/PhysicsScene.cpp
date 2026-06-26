#include <BasePhysics/Physics/PhysicsScene.hpp>

namespace bp
{
    PhysicsScene::PhysicsScene() : gravity(0.0f, -9.81f), hashGrid(3.0f), checkCounter(0), aabbCollisionCheckCount(0), satCollisionCheckCount(0)
    {
        contacts.reserve(1024);
    }
    PhysicsScene::PhysicsScene(Vec2 gravity, float cellSize) : gravity(gravity), hashGrid(cellSize), checkCounter(0), aabbCollisionCheckCount(0), satCollisionCheckCount(0)
    {
        contacts.reserve(1024);
    }
    PhysicsScene::~PhysicsScene()
    {
        for(Rigidbody *rb : bodies)
        {
            delete rb;
        }
        for(Joint *joint : joints)
        {
            delete joint;
        }

        bodies.clear();
        joints.clear();
    }

    Rigidbody *PhysicsScene::AddRigidbody(Rigidbody *rb)
    {
        bodies.push_back(rb);
        return rb;
    }
    Rigidbody *PhysicsScene::AddRigidbody(BodyPreset preset)
    {
        Rigidbody *rb = Rigidbody::CreateRigidbody(preset);
        bodies.push_back(rb);
        return rb;
    }
    Rigidbody *PhysicsScene::AddRigidbody(Vec2 position, float rotation, ColliderShape shape, float mass, float linearDamping, float angularDamping,
        float restitution, float friction, bool isStatic, bool lockRotation, bool usesGravity, bool isSensor)
    {
        BodyPreset preset = BodyPreset(position, rotation, shape, mass, linearDamping, angularDamping, restitution, friction, isStatic, usesGravity, isSensor, lockRotation);
        Rigidbody *rb = Rigidbody::CreateRigidbody(preset);
        bodies.push_back(rb);
        return rb;
    }

    void PhysicsScene::RemoveRigidbody(Rigidbody *rb)
    {
        for(Joint *joint : joints)
        {
            if(joint->GetRigidbody1() == rb || joint->GetRigidbody2() == rb)
                Joint::DeleteJoint(joint, joints);
        }
        Rigidbody::DeleteRigidbody(rb, bodies);
    }
    void PhysicsScene::RemoveRigidbody(int index)
    {
        for(Joint *joint : joints)
        {
            if(joint->GetRigidbody1() == bodies[index] || joint->GetRigidbody2() == bodies[index])
                Joint::DeleteJoint(joint, joints);
        }
        Rigidbody::DeleteRigidbody(bodies[index], bodies);
    }

    Joint *PhysicsScene::CreateJoint(Joint *joint)
    {
        joints.push_back(joint);
        return joint;
    }
    Joint *PhysicsScene::CreateJoint(JointPreset preset)
    {
        Joint *joint = Joint::CreateJoint(preset);
        joints.push_back(joint);
        return joint;
    }
    Joint *PhysicsScene::CreateJoint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, bool disableCollision, JointType jointType)
    {
        JointPreset preset = JointPreset(rb1, rb2, localAnchor1, localAnchor2, disableCollision, jointType);
        Joint *joint = Joint::CreateJoint(preset);
        joints.push_back(joint);
        return joint;
    }

    void PhysicsScene::RemoveJoint(Joint *joint)
    {
        Joint::DeleteJoint(joint, joints);
    }
    void PhysicsScene::RemoveJoint(int index)
    {
        Joint::DeleteJoint(joints[index], joints);
    }

    void PhysicsScene::Step(float deltaTime, unsigned int substeps, unsigned int iterations)
    {
        deltaTime /= (float)substeps;

        for(int substep = 0; substep < substeps; substep++)
        {
            for(Rigidbody *rb : bodies)
            {
                rb->IntegrateVelocity(deltaTime, gravity);
                rb->ApplyDamping(deltaTime);
                rb->IntegratePosition(deltaTime);
                rb->GetCollider().UpdateWorldGeometry(rb->GetPosition(), rb->GetRotation());
            }
            
            DetectCollisions();
            for(int iter = 0; iter < iterations; iter++)
            {
                for(Joint *joint : joints)
                {
                    joint->SolveVelocity(deltaTime, iterations);
                }

                for(const ContactManifold &contact : contacts)
                {
                    ResolveCollision(contact);
                }
            }
            
            for(const ContactManifold &contact : contacts)
            {
                SeparateBodies(contact);
            }

            for(Joint *joint : joints)
            {
                joint->SolvePosition();
            }

            for(Rigidbody *rb : bodies)
            {
                rb->GetCollider().UpdateWorldGeometry(rb->GetPosition(), rb->GetRotation());
            }
        }
    }

    void PhysicsScene::DetectCollisions()
    {
        aabbCollisionCheckCount = 0;
        satCollisionCheckCount = 0;

        contacts.clear();
        hashGrid.Refresh(bodies);

        int n = (int)bodies.size();
        if(pairCheckMatrix.size() < (size_t)n * n)
        {
            pairCheckMatrix.assign((size_t)n * n, 0);
            checkCounter = 0;
        }
        checkCounter++;
        if(checkCounter >= std::numeric_limits<int>::max() - 1)
        {
            std::fill(pairCheckMatrix.begin(), pairCheckMatrix.end(), 0);
            checkCounter = 1;
        }

        for(int hash = 0; hash < HashGrid::GetGridSize(); hash++)
        {
            for(int i = hashGrid.GetHead(hash); i != -1; i = hashGrid.GetEntry(i).next)
            {
                for(int j = hashGrid.GetEntry(i).next; j != -1; j = hashGrid.GetEntry(j).next)
                {
                    int rb1Index = hashGrid.GetEntry(i).rbIndex;
                    int rb2Index = hashGrid.GetEntry(j).rbIndex;

                    if(rb1Index == rb2Index)
                        continue;

                    int minIndex = std::min(rb1Index, rb2Index);
                    int maxIndex = std::max(rb1Index, rb2Index);
                    int matrixIndex = minIndex * n + maxIndex;

                    if(pairCheckMatrix[matrixIndex] == checkCounter)
                        continue;
                    pairCheckMatrix[matrixIndex] = checkCounter;

                    aabbCollisionCheckCount++;

                    Rigidbody *rb1 = bodies[rb1Index];
                    Rigidbody *rb2 = bodies[rb2Index];

                    if((rb1->IsStatic() && rb2->IsStatic()) || (rb1->GetCollider().IsSensor() || rb2->GetCollider().IsSensor()))
                        continue;

                    if(!collisions::IntersectAABBs(rb1->GetCollider().GetAABB(rb1->GetPosition(), rb1->GetRotation()),
                        rb2->GetCollider().GetAABB(rb2->GetPosition(), rb2->GetRotation())))
                        continue;

                    bool skipCollision = false;

                    for(Joint *joint : joints)
                    {
                        if(((joint->GetRigidbody1() == rb1 && joint->GetRigidbody2() == rb2) || (joint->GetRigidbody1() == rb2 && joint->GetRigidbody2() == rb1)) && joint->IsCollisionDisabled())
                            skipCollision = true;
                    }

                    if(skipCollision)
                        continue;

                    satCollisionCheckCount++;

                    Vec2 normal;
                    float depth;
                    std::vector<Vec2> contactPoints;

                    if(collisions::Collide(rb1, rb2, normal, depth, contactPoints))
                    {
                        if(!contactPoints.empty())
                        {
                            contacts.emplace_back(rb1Index, rb2Index, normal, depth, contactPoints[0]);
                            if(contactPoints.size() > 1) 
                                contacts.back().contactPoints.push_back(contactPoints[1]);
                        }
                    }
                }
            }
        }

        // for(int i = 0; i < (int)bodies.size() - 1; i++)
        // {
        //     for(int j = i + 1; j < (int)bodies.size(); j++)
        //     {
        //         aabbCollisionCheckCount++;

        //         if((bodies[i]->IsStatic() && bodies[j]->IsStatic()) || (bodies[i]->GetCollider().IsSensor() || bodies[j]->GetCollider().IsSensor()))
        //             continue;

        //         if(!collisions::IntersectAABBs(bodies[i]->GetCollider().GetAABB(bodies[i]->GetPosition(), bodies[i]->GetRotation()),
        //             bodies[j]->GetCollider().GetAABB(bodies[j]->GetPosition(), bodies[j]->GetRotation())))
		// 		    continue;

        //         satCollisionCheckCount++;
                
        //         Vec2 normal;
        //         float depth;
        //         std::vector<Vec2> contactPoints;

        //         if(collisions::Collide(bodies[i], bodies[j], normal, depth, contactPoints))
        //         {
        //             if(contactPoints.size() == 1)
        //                 contacts.push_back(ContactManifold(i, j, normal, depth, contactPoints[0]));
        //             if(contactPoints.size() == 2)
        //                 contacts.push_back(ContactManifold(i, j, normal, depth, contactPoints[0], contactPoints[1]));
        //         }
        //     }
        // }
    }
    void PhysicsScene::SeparateBodies(const ContactManifold &contact)
    {
        Rigidbody *rb1 = bodies[contact.rbIndex1];
        Rigidbody *rb2 = bodies[contact.rbIndex2];
        
        const float percent = 0.8f;
        const float slop = 0.01f;

        float invMassSum = rb1->GetInverseMass() + rb2->GetInverseMass();
        if(invMassSum == 0.0f)
            return;

        float correctionMag = std::max(contact.depth - slop, 0.0f) / invMassSum * percent;
        Vec2 correction = contact.normal * correctionMag;

        if(!rb1->IsStatic())
            rb1->Move(-correction * rb1->GetInverseMass());
        if(!rb2->IsStatic())
            rb2->Move(correction * rb2->GetInverseMass());
    }
    void PhysicsScene::ResolveCollision(const ContactManifold &contact)
    {
        Rigidbody *rb1 = bodies[contact.rbIndex1];
        Rigidbody *rb2 = bodies[contact.rbIndex2];

        const Vec2 &normal = contact.normal;
        const std::vector<Vec2> &contacts = contact.contactPoints;
        int contactCount = contacts.size();

        float e = std::min(rb1->GetCollider().GetRestitution(), rb2->GetCollider().GetRestitution());
        float sf = std::sqrt(rb1->GetCollider().GetFriction() * rb2->GetCollider().GetFriction());
        float df = 0.8f * sf;

        for(const Vec2 &contact : contacts)
        {
            Vec2 relativeVel = rb2->GetVelocityAtWorldPoint(contact) - rb1->GetVelocityAtWorldPoint(contact);
            float contactVelMag = math::Dot(relativeVel, normal);
            
            if(contactVelMag > 0.0f)
                continue;

            Vec2 r1 = contact - rb1->GetPosition();
            Vec2 r2 = contact - rb2->GetPosition();
            float r1n = math::Cross(r1, normal);
            float r2n = math::Cross(r2, normal);

            float j = (-(1.0f + e) * contactVelMag) / (rb1->GetInverseMass() + rb2->GetInverseMass() + (r1n * r1n) * rb1->GetInverseInertia() + (r2n * r2n) * rb2->GetInverseInertia());
            j /= (float)contactCount;

            Vec2 impulse = normal * j;

            rb1->ApplyImpulseAtWorldPoint(-impulse, contact);
            rb2->ApplyImpulseAtWorldPoint(impulse, contact);
            
            relativeVel = rb2->GetVelocityAtWorldPoint(contact) - rb1->GetVelocityAtWorldPoint(contact);

            Vec2 tangent = relativeVel - normal * math::Dot(relativeVel, normal);

            if(utils::NearlyEqual(tangent, Vec2::Zero()))
                continue;
            
            tangent.Normalize();

            float r1t = math::Cross(r1, tangent);
            float r2t = math::Cross(r2, tangent);
            
            float jt = -math::Dot(relativeVel, tangent) / (rb1->GetInverseMass() + rb2->GetInverseMass() + (r1t * r1t) * rb1->GetInverseInertia() + (r2t * r2t) * rb2->GetInverseInertia());
            jt /= (float)contactCount;

            Vec2 frictionImpulse;
            if(std::abs(jt) <= j * sf)
                frictionImpulse = tangent * jt;
            else
                frictionImpulse = tangent * (-j * df);

            rb1->ApplyImpulseAtWorldPoint(-frictionImpulse, contact);
            rb2->ApplyImpulseAtWorldPoint(frictionImpulse, contact);
        }
    }

    void PhysicsScene::SetGravity(Vec2 val)
    {
        gravity = val;
    }
    void PhysicsScene::Clear()
    {
        for(Rigidbody *rb : bodies)
        {
            delete rb;
        }
        for(Joint *joint : joints)
        {
            delete joint;
        }

        bodies.clear();
        joints.clear();
        contacts.clear();
    }

    const std::vector<Rigidbody *> &PhysicsScene::GetBodies() const
    {
        return bodies;
    }
    const std::vector<Joint *> &PhysicsScene::GetJoints() const
    {
        return joints;
    }
    const std::vector<ContactManifold> &PhysicsScene::GetContacts() const
    {
        return contacts;
    }
    const HashGrid &PhysicsScene::GetHashGrid() const
    {
        return hashGrid;
    }

    int PhysicsScene::GetAABBCollisionCheckCount() const
    {
        return aabbCollisionCheckCount;
    }
    int PhysicsScene::GetSATCollisionCheckCount() const
    {
        return satCollisionCheckCount;
    }
}