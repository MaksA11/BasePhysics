#pragma once

#include <BasePhysics/Core/Vec2.hpp>
#include <BasePhysics/Physics/Rigidbody.hpp>
#include <BasePhysics/Physics/JointTypes.hpp>

namespace bp
{
    struct JointPreset
    {
        Rigidbody *rb1;
        Rigidbody *rb2;

        Vec2 localAnchorPoint1;
        Vec2 localAnchorPoint2;

        bool disableCollision;

        JointType jointType;

        JointPreset() : rb1(rb1), rb2(rb2), localAnchorPoint1(localAnchorPoint1), localAnchorPoint2(localAnchorPoint2), disableCollision(disableCollision), jointType(jointType) {}
        JointPreset(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchorPoint1, Vec2 localAnchorPoint2, bool disableCollision, JointType jointType)
            : rb1(rb1), rb2(rb2), localAnchorPoint1(localAnchorPoint1), localAnchorPoint2(localAnchorPoint2), disableCollision(disableCollision), jointType(jointType) {}
    };
}