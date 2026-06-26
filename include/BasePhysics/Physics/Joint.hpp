#pragma once

#include <BasePhysics/Core/Vec2.hpp>
#include <BasePhysics/Physics/Rigidbody.hpp>
#include <BasePhysics/Physics/JointTypes.hpp>
#include <BasePhysics/Physics/JointPreset.hpp>

namespace bp
{
    class Joint
    {
        private:
            Joint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchorPoint1, Vec2 localAnchorPoint2, bool disableCollision, JointType jointType)
                : rb1(rb1), rb2(rb2), localAnchorPoint1(localAnchorPoint1), localAnchorPoint2(localAnchorPoint2), disableCollision(disableCollision), jointType(jointType) {}
            Joint(JointPreset preset)
                : rb1(preset.rb1), rb2(preset.rb2), localAnchorPoint1(preset.localAnchorPoint1), localAnchorPoint2(preset.localAnchorPoint2),
                disableCollision(preset.disableCollision), jointType(preset.jointType) {}

            Rigidbody *rb1;
            Rigidbody *rb2;

            Vec2 localAnchorPoint1;
            Vec2 localAnchorPoint2;

            bool disableCollision;

            JointType jointType;
        
        public:
            static Joint *CreateJoint(JointPreset preset);
            static void DeleteJoint(Joint *joint, std::vector<Joint *> &joints);

            bool IsWeld() const;
            bool IsDistance() const;
            bool IsSpring() const;
            bool IsRevolute() const;
            bool IsRope() const;

            const WeldJoint *GetWeld() const;
            const DistanceJoint *GetDistance() const;
            const SpringJoint *GetSpring() const;
            const RevoluteJoint *GetRevolute() const;
            const RopeJoint *GetRope() const;

            void SolveVelocity(float deltaTime, size_t iterations);
            void SolvePosition();

            const Rigidbody *GetRigidbody1() const;
            Rigidbody *GetRigidbody1();
            const Rigidbody *GetRigidbody2() const;
            Rigidbody *GetRigidbody2();
            
            Vec2 GetLocalAnchor1() const;
            Vec2 GetLocalAnchor2() const;
            Vec2 GetWorldAnchor1() const;
            Vec2 GetWorldAnchor2() const;

            bool IsCollisionDisabled() const;
            void DisableCollision(bool value);
        };
}
