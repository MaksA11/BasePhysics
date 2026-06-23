#pragma once

#include <variant>

#include <BasePhysics/Core/Vec2.hpp>

namespace bp
{
    struct WeldJoint
    {
        float referenceAngle;

        WeldJoint(float referenceAngle) : referenceAngle(referenceAngle) {}
    };
    struct DistanceJoint
    {
        float distance;

        DistanceJoint(float distance) : distance(distance) {}
    };
    struct SpringJoint
    {
        float restDistance;
        float stiffness;
        float damping;

        SpringJoint(float restDistance, float stiffness, float damping) : restDistance(restDistance), stiffness(stiffness), damping(damping) {}
    };
    struct HingeJoint
    {
        float referenceAngle;
        float lowerLimit;
        float upperLimit;

        HingeJoint(float referenceAngle, float lowerLimit, float upperLimit) : referenceAngle(referenceAngle), lowerLimit(lowerLimit), upperLimit(upperLimit) {}
    };
    struct RopeJoint
    {
        float maxDistance;

        RopeJoint(float maxDistance) : maxDistance(maxDistance) {}
    };

    using JointType = std::variant<WeldJoint, DistanceJoint, SpringJoint, HingeJoint, RopeJoint>;
}