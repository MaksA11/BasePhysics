#include <BasePhysics/Physics/Joint.hpp>

namespace bp
{
    Joint *Joint::CreateJoint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, bool disableCollision, JointType jointType)
    {
        return new Joint(rb1, rb2, localAnchor1, localAnchor2, disableCollision, jointType);
    }
    Joint *Joint::CreateWeldJoint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, bool disableCollision, float referenceAngle)
    {
        JointType jointType = WeldJoint(referenceAngle);
        return new Joint(rb1, rb2, localAnchor1, localAnchor2, disableCollision, jointType);
    }
    Joint *Joint::CreateDistanceJoint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, bool disableCollision, float distance)
    {
        JointType jointType = DistanceJoint(distance);
        return new Joint(rb1, rb2, localAnchor1, localAnchor2, disableCollision, jointType);
    }
    Joint *Joint::CreateSpringJoint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, bool disableCollision, float restDistance, float stiffness, float damping)
    {
        JointType jointType = SpringJoint(restDistance, stiffness, damping);
        return new Joint(rb1, rb2, localAnchor1, localAnchor2, disableCollision, jointType);
    }
    Joint *Joint::CreateHingeJoint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, bool disableCollision, float lowerLimit, float upperLimit)
    {
        JointType jointType = HingeJoint(lowerLimit, upperLimit);
        return new Joint(rb1, rb2, localAnchor1, localAnchor2, disableCollision, jointType);
    }
    Joint *Joint::CreateRopeJoint(Rigidbody *rb1, Rigidbody *rb2, Vec2 localAnchor1, Vec2 localAnchor2, bool disableCollision, float maxDistance)
    {
        JointType jointType = RopeJoint(maxDistance);
        return new Joint(rb1, rb2, localAnchor1, localAnchor2, disableCollision, jointType);
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

    bool Joint::IsWeld() const
    {
        return std::holds_alternative<WeldJoint>(this->jointType);
    }
    bool Joint::IsDistance() const
    {
        return std::holds_alternative<DistanceJoint>(this->jointType);
    }
    bool Joint::IsSpring() const
    {
        return std::holds_alternative<SpringJoint>(this->jointType);
    }
    bool Joint::IsHinge() const
    {
        return std::holds_alternative<HingeJoint>(this->jointType);
    }
    bool Joint::IsRope() const
    {
        return std::holds_alternative<RopeJoint>(this->jointType);
    }

    const WeldJoint *Joint::GetWeld() const
    {
        return std::get_if<WeldJoint>(&this->jointType);
    }
    const DistanceJoint *Joint::GetDistance() const
    {
        return std::get_if<DistanceJoint>(&this->jointType);
    }
    const SpringJoint *Joint::GetSpring() const
    {
        return std::get_if<SpringJoint>(&this->jointType);
    }
    const HingeJoint *Joint::GetHinge() const
    {
        return std::get_if<HingeJoint>(&this->jointType);
    }
    const RopeJoint *Joint::GetRope() const
    {
        return std::get_if<RopeJoint>(&this->jointType);
    }

    void Joint::SolveVelocity(float deltaTime, unsigned int iterations)
    {
        if(IsWeld())
        {
            Vec2 anchorPoint = (GetWorldAnchor1() + GetWorldAnchor2()) / 2;

            Vec2 relativeVel = rb2->GetVelocityAtWorldPoint(anchorPoint) - rb1->GetVelocityAtWorldPoint(anchorPoint);

            Vec2 r1 = anchorPoint - rb1->GetPosition();
            Vec2 r2 = anchorPoint - rb2->GetPosition();

            float r1x = math::Cross(r1, Vec2::Right());
            float r2x = math::Cross(r2, Vec2::Right());

            float r1y = math::Cross(r1, Vec2::Up());
            float r2y = math::Cross(r2, Vec2::Up());

            float jx = -relativeVel.x / (rb1->GetInverseMass() + rb2->GetInverseMass() + (r1x * r1x) * rb1->GetInverseInertia() + (r2x * r2x) * rb2->GetInverseInertia());
            float jy = -relativeVel.y / (rb1->GetInverseMass() + rb2->GetInverseMass() + (r1y * r1y) * rb1->GetInverseInertia() + (r2y * r2y) * rb2->GetInverseInertia());

            Vec2 impulse = Vec2(jx, jy);

            rb1->ApplyImpulseAtWorldPoint(-impulse, anchorPoint);
            rb2->ApplyImpulseAtWorldPoint(impulse, anchorPoint);

            float relativeAngVel = rb2->GetAngularVelocity() - rb1->GetAngularVelocity();
            float angImpulse = -relativeAngVel / (rb1->GetInverseInertia() + rb2->GetInverseInertia());

            rb1->ApplyAngularImpulse(-angImpulse);
            rb2->ApplyAngularImpulse(angImpulse);
        }
        else if(IsDistance())
        {
            Vec2 point1 = GetWorldAnchor1();
            Vec2 point2 = GetWorldAnchor2();

            Vec2 d = point2 - point1;
            Vec2 n = d.Normalized();

            Vec2 relativeVel = rb2->GetVelocityAtWorldPoint(point2) - rb1->GetVelocityAtWorldPoint(point1);

            float vn = math::Dot(relativeVel, n);

            float r1n = math::Cross((point1 - rb1->GetPosition()), n);
            float r2n = math::Cross((point2 - rb2->GetPosition()), n);

            float j = -vn / (rb1->GetInverseMass() + rb2->GetInverseMass() + (r1n * r1n) * rb1->GetInverseInertia() + (r2n * r2n) * rb2->GetInverseInertia());

            Vec2 impulse = n * j;

            rb1->ApplyImpulseAtWorldPoint(-impulse, point1);
            rb2->ApplyImpulseAtWorldPoint(impulse, point2);
        }
        else if(IsSpring())
        {
            Vec2 point1 = GetWorldAnchor1();
            Vec2 point2 = GetWorldAnchor2();

            Vec2 d = point2 - point1;
            Vec2 n = d.Normalized();

            Vec2 relativeVel = rb2->GetVelocityAtWorldPoint(point2) - rb1->GetVelocityAtWorldPoint(point1);

            float vn = math::Dot(relativeVel, n);

            float x = d.Magnitude() - GetSpring()->restDistance;
            float F = -(GetSpring()->stiffness * x) - (GetSpring()->damping * vn);

            float j = (F * deltaTime) / (float)iterations;

            Vec2 impulse = n * j;

            rb1->ApplyImpulseAtWorldPoint(-impulse, point1);
            rb2->ApplyImpulseAtWorldPoint(impulse, point2);
        }
        else if(IsHinge())
        {
            Vec2 anchorPoint = (GetWorldAnchor1() + GetWorldAnchor2()) / 2;

            Vec2 relativeVel = rb2->GetVelocityAtWorldPoint(anchorPoint) - rb1->GetVelocityAtWorldPoint(anchorPoint);

            Vec2 r1 = anchorPoint - rb1->GetPosition();
            Vec2 r2 = anchorPoint - rb2->GetPosition();

            float r1x = math::Cross(r1, Vec2::Right());
            float r2x = math::Cross(r2, Vec2::Right());

            float r1y = math::Cross(r1, Vec2::Up());
            float r2y = math::Cross(r2, Vec2::Up());

            float jx = -relativeVel.x / (rb1->GetInverseMass() + rb2->GetInverseMass() + (r1x * r1x) * rb1->GetInverseInertia() + (r2x * r2x) * rb2->GetInverseInertia());
            float jy = -relativeVel.y / (rb1->GetInverseMass() + rb2->GetInverseMass() + (r1y * r1y) * rb1->GetInverseInertia() + (r2y * r2y) * rb2->GetInverseInertia());

            Vec2 impulse = Vec2(jx, jy);

            rb1->ApplyImpulseAtWorldPoint(-impulse, anchorPoint);
            rb2->ApplyImpulseAtWorldPoint(impulse, anchorPoint);
        }
        else if(IsRope())
        {
            Vec2 point1 = GetWorldAnchor1();
            Vec2 point2 = GetWorldAnchor2();

            Vec2 d = point2 - point1;
            Vec2 n = d.Normalized();

            if(d.MagnitudeSquared() < GetRope()->maxDistance * GetRope()->maxDistance)
                return;

            Vec2 relativeVel = rb2->GetVelocityAtWorldPoint(point2) - rb1->GetVelocityAtWorldPoint(point1);

            float vn = math::Dot(relativeVel, n);

            if(vn <= 0)
                return;

            float r1n = math::Cross((point1 - rb1->GetPosition()), n);
            float r2n = math::Cross((point2 - rb2->GetPosition()), n);

            float j = -vn / (rb1->GetInverseMass() + rb2->GetInverseMass() + (r1n * r1n) * rb1->GetInverseInertia() + (r2n * r2n) * rb2->GetInverseInertia());

            Vec2 impulse = n * j;

            rb1->ApplyImpulseAtWorldPoint(-impulse, point1);
            rb2->ApplyImpulseAtWorldPoint(impulse, point2);
        }
    }

    // TODO: implement
    void Joint::SolvePosition()
    {
        if(IsWeld())
        {
            return;
        }
        else if(IsDistance())
        {
            return;
        }
        else if(IsSpring())
        {
            return;
        }
        else if(IsHinge())
        {
            return;
        }
        else if(IsRope())
        {
            return;
        }
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

    bool Joint::IsCollisionDisabled() const
    {
        return disableCollision;
    }

    Vec2 Joint::GetLocalAnchor1() const
    {
        return localAnchorPoint1;
    }
    Vec2 Joint::GetLocalAnchor2() const
    {
        return localAnchorPoint2;
    }

    Vec2 Joint::GetWorldAnchor1() const
    {
        return bp::math::Transform(localAnchorPoint1, rb1->GetPosition(), rb1->GetRotation());
    }
    Vec2 Joint::GetWorldAnchor2() const
    {
        return bp::math::Transform(localAnchorPoint2, rb2->GetPosition(), rb2->GetRotation());
    }
}