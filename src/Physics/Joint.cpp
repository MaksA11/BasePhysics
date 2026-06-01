#include <BasePhysics/Physics/Joint.hpp>

namespace bp
{
    Joint *Joint::CreateJoint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, JointType jointType)
    {
        return new Joint(rb1, rb2, localAnchor1, localAnchor2, jointType);
    }
    Joint *Joint::CreateWeldJoint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, float referenceAngle)
    {
        JointType jointType = WeldJoint(referenceAngle);
        return new Joint(rb1, rb2, localAnchor1, localAnchor2, jointType);
    }
    Joint *Joint::CreateDistanceJoint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, float distance)
    {
        JointType jointType = DistanceJoint(distance);
        return new Joint(rb1, rb2, localAnchor1, localAnchor2, jointType);
    }
    Joint *Joint::CreateSpringJoint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, float restDistance, float stiffness, float damping)
    {
        JointType jointType = SpringJoint(restDistance, stiffness, damping);
        return new Joint(rb1, rb2, localAnchor1, localAnchor2, jointType);
    }
    Joint *Joint::CreateHingeJoint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, float lowerLimit, float upperLimit)
    {
        JointType jointType = HingeJoint(lowerLimit, upperLimit);
        return new Joint(rb1, rb2, localAnchor1, localAnchor2, jointType);
    }
    Joint *Joint::CreateRopeJoint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, float maxDistance)
    {
        JointType jointType = RopeJoint(maxDistance);
        return new Joint(rb1, rb2, localAnchor1, localAnchor2, jointType);
    }

    void Joint::DeleteJoint(Joint *joint, std::vector<Joint *> &joints)
    {
        auto it = std::find(joints.begin(), joints.end(), joint);

        if(it != joints.end())
        {
            delete joint;
            joints.erase(it);
        }
    }

    void Joint::SolvePosition()
    {

    }

    void Joint::SolveVelocity()
    {

    }

    const Rigidbody *Joint::GetRigidbody1() const
    {
        return rb1;
    }
    Rigidbody *Joint::GetRigidbody1()
    {
        return rb1;
    }
    const Rigidbody *Joint::GetRigidbody2() const
    {
        return rb2;
    }
    Rigidbody *Joint::GetRigidbody2()
    {
        return rb2;
    }

    Vec2 Joint::GetLocalAnchor1() const
    {
        return localAnchor1;
    }
    Vec2 Joint::GetLocalAnchor2() const
    {
        return localAnchor2;
    }

    Vec2 Joint::GetWorldAnchor1() const
    {
        return bp::math::Transform(localAnchor1, rb1->GetPosition(), rb1->GetRotation());
    }
    Vec2 Joint::GetWorldAnchor2() const
    {
        return bp::math::Transform(localAnchor2, rb2->GetPosition(), rb2->GetRotation());
    }
}