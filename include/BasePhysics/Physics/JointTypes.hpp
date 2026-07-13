#pragma once

#include <variant>
#include <limits>

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
    struct SliderJoint
    {
        Vec2 localAxis;
        float referenceAngle;
        float lowerLimit;
        float upperLimit;

        SliderJoint(Vec2 localAxis, float referenceAngle, float lowerLimit = -std::numeric_limits<float>::max(), float upperLimit = std::numeric_limits<float>::max())
            : localAxis(localAxis), referenceAngle(referenceAngle), lowerLimit(lowerLimit), upperLimit(upperLimit) {}
    };
    struct RopeJoint
    {
        float maxDistance;

        RopeJoint(float maxDistance) : maxDistance(maxDistance) {}
    };
    struct SpringJoint
    {
        float restDistance;
        float stiffness;
        float damping;

        SpringJoint(float restDistance, float stiffness, float damping) : restDistance(restDistance), stiffness(stiffness), damping(damping) {}
    };
    struct RevoluteJoint
    {
        float referenceAngle;
        float lowerLimit;
        float upperLimit;

        RevoluteJoint(float referenceAngle, float lowerLimit, float upperLimit) : referenceAngle(referenceAngle), lowerLimit(lowerLimit), upperLimit(upperLimit) {}
    };
    struct TorsionSpringJoint
    {
        float restAngle;
        float stiffness;
        float damping;

        TorsionSpringJoint(float restAngle, float stiffness, float damping) : restAngle(restAngle), stiffness(stiffness), damping(damping) {}
    };

    using JointType = std::variant<WeldJoint, DistanceJoint, SpringJoint, SliderJoint, RevoluteJoint, TorsionSpringJoint, RopeJoint>;
}