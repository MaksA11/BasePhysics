#pragma once

#include <BasePhysics/Core/Vec2.hpp>
#include <BasePhysics/Physics/Rigidbody.hpp>
#include <BasePhysics/Physics/JointTypes.hpp>

namespace bp
{
    class Joint
    {
        private:
            Joint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchorPoint1, Vec2 localAnchorPoint2, JointType jointType)
            : rb1(rb1), rb2(rb2), localAnchorPoint1(localAnchorPoint1), localAnchorPoint2(localAnchorPoint2), jointType(jointType) {}

            Rigidbody *rb1;
            Rigidbody *rb2;

            Vec2 localAnchorPoint1;
            Vec2 localAnchorPoint2;


            JointType jointType;
        
        public:
            static Joint *CreateJoint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, JointType jointType);
            static Joint *CreateWeldJoint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, float referenceAngle);
            static Joint *CreateDistanceJoint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, float distance);
            static Joint *CreateSpringJoint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, float restDistance, float stiffness, float damping);
            static Joint *CreateHingeJoint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, float lowerLimit, float upperLimit);
            static Joint *CreateRopeJoint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, float maxDistance);

            static void DeleteJoint(Joint *joint, std::vector<Joint *> &joints);

            bool IsWeld() const;
            bool IsDistance() const;
            bool IsSpring() const;
            bool IsHinge() const;
            bool IsRope() const;

            const WeldJoint *GetWeld() const;
            const DistanceJoint *GetDistance() const;
            const SpringJoint *GetSpring() const;
            const HingeJoint *GetHinge() const;
            const RopeJoint *GetRope() const;

            void SolveVelocity(float deltaTime, unsigned int iterations);
            void SolvePosition();

            const Rigidbody *GetRigidbody1() const;
            Rigidbody *GetRigidbody1();
            const Rigidbody *GetRigidbody2() const;
            Rigidbody *GetRigidbody2();

            Vec2 GetLocalAnchor1() const;
            Vec2 GetLocalAnchor2() const;

            Vec2 GetWorldAnchor1() const;
            Vec2 GetWorldAnchor2() const;
    };
}
