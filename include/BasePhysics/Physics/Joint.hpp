#pragma once

#include <BasePhysics/Core/Vec2.hpp>
#include <BasePhysics/Physics/Rigidbody.hpp>
#include <BasePhysics/Physics/JointTypes.hpp>

namespace bp
{
    class Joint
    {
        private:
            Joint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, JointType jointType)
            : rb1(rb1), rb2(rb2), localAnchor1(localAnchor1), localAnchor2(localAnchor2), jointType(jointType) {}

            Rigidbody *rb1;
            Rigidbody *rb2;

            Vec2 localAnchor1;
            Vec2 localAnchor2;

            JointType jointType;
        
        public:
            static Joint *CreateJoint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, JointType jointType);
            static Joint *CreateWeldJoint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, float referenceAngle);
            static Joint *CreateDistanceJoint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, float distance);
            static Joint *CreateSpringJoint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, float restDistance, float stiffness, float damping);
            static Joint *CreateHingeJoint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, float lowerLimit, float upperLimit);
            static Joint *CreateRopeJoint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, float maxDistance);

            static void DeleteJoint(Joint *joint, std::vector<Joint *> &joints);

            void SolveVelocity();
            void SolvePosition();

            const Rigidbody *GetRigidbody1() const;
            const Rigidbody *GetRigidbody2() const;

            Vec2 GetAnchor1() const;
            Vec2 GetAnchor2() const;
    };
}
