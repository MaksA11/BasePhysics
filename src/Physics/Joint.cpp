#include <BasePhysics/Physics/Joint.hpp>

namespace bp
{
    Joint *Joint::CreateJoint(JointPreset preset)
    {
        return new Joint(preset);
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
    bool Joint::IsSlider() const
    {
        return std::holds_alternative<SliderJoint>(this->jointType);
    }
    bool Joint::IsRope() const
    {
        return std::holds_alternative<RopeJoint>(this->jointType);
    }
    bool Joint::IsSpring() const
    {
        return std::holds_alternative<SpringJoint>(this->jointType);
    }
    bool Joint::IsRevolute() const
    {
        return std::holds_alternative<RevoluteJoint>(this->jointType);
    }
    bool Joint::IsTorsionSpring() const
    {
        return std::holds_alternative<TorsionSpringJoint>(this->jointType);
    }

    const WeldJoint *Joint::GetWeld() const
    {
        return std::get_if<WeldJoint>(&this->jointType);
    }
    const DistanceJoint *Joint::GetDistance() const
    {
        return std::get_if<DistanceJoint>(&this->jointType);
    }
    const SliderJoint *Joint::GetSlider() const
    {
        return std::get_if<SliderJoint>(&this->jointType);
    }
    const RopeJoint *Joint::GetRope() const
    {
        return std::get_if<RopeJoint>(&this->jointType);
    }
    const SpringJoint *Joint::GetSpring() const
    {
        return std::get_if<SpringJoint>(&this->jointType);
    }
    const RevoluteJoint *Joint::GetRevolute() const
    {
        return std::get_if<RevoluteJoint>(&this->jointType);
    }
    const TorsionSpringJoint *Joint::GetTorsionSpring() const
    {
        return std::get_if<TorsionSpringJoint>(&this->jointType);
    }

    void Joint::SolveVelocity(float timestep, size_t iterations)
    {
        if(IsWeld())
        {
            Vec2 anchorPoint = (GetWorldAnchor1() + GetWorldAnchor2()) * 0.5f;

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

            Vec2 delta = point2 - point1;
            Vec2 normal = delta.Normalized();

            Vec2 relativeVel = rb2->GetVelocityAtWorldPoint(point2) - rb1->GetVelocityAtWorldPoint(point1);

            float vn = math::Dot(relativeVel, normal);

            float r1n = math::Cross((point1 - rb1->GetPosition()), normal);
            float r2n = math::Cross((point2 - rb2->GetPosition()), normal);

            float j = -vn / (rb1->GetInverseMass() + rb2->GetInverseMass() + (r1n * r1n) * rb1->GetInverseInertia() + (r2n * r2n) * rb2->GetInverseInertia());

            Vec2 impulse = normal * j;

            rb1->ApplyImpulseAtWorldPoint(-impulse, point1);
            rb2->ApplyImpulseAtWorldPoint(impulse, point2);
        }
        else if(IsSlider())
        {
            Vec2 anchorPoint = (GetWorldAnchor1() + GetWorldAnchor2()) * 0.5f;

            Vec2 worldAxis = math::Rotate(GetSlider()->localAxis, rb1->GetRotation());
            Vec2 perpAxis = math::Perpendicular(worldAxis);

            Vec2 relativeVel = rb2->GetVelocityAtWorldPoint(anchorPoint) - rb1->GetVelocityAtWorldPoint(anchorPoint);
            float vPerp = math::Dot(relativeVel, perpAxis);

            Vec2 r1 = anchorPoint - rb1->GetPosition();
            Vec2 r2 = anchorPoint - rb2->GetPosition();

            float r1p = math::Cross(r1, perpAxis);
            float r2p = math::Cross(r2, perpAxis);

            float lateralImpulse = -vPerp / (rb1->GetInverseMass() + rb2->GetInverseMass() + (r1p * r1p) * rb1->GetInverseInertia() + (r2p * r2p) * rb2->GetInverseInertia());
            Vec2 impulse = perpAxis * lateralImpulse;

            rb1->ApplyImpulseAtWorldPoint(-impulse, anchorPoint);
            rb2->ApplyImpulseAtWorldPoint(impulse, anchorPoint);
            
            float relativeAngVel = rb2->GetAngularVelocity() - rb1->GetAngularVelocity();
            float angImpulse = -relativeAngVel / (rb1->GetInverseInertia() + rb2->GetInverseInertia());

            rb1->ApplyAngularImpulse(-angImpulse);
            rb2->ApplyAngularImpulse(angImpulse);

            Vec2 delta = GetWorldAnchor2() - GetWorldAnchor1();
            float currentTranslation = math::Dot(delta, worldAxis);
            float vAxis = math::Dot(relativeVel, worldAxis);

            float limitImpulseMag = 0.0f;

            if((currentTranslation <= GetSlider()->lowerLimit && vAxis < 0.0f) || (currentTranslation >= GetSlider()->upperLimit && vAxis > 0.0f))
            {
                float r1a = math::Cross(r1, worldAxis);
                float r2a = math::Cross(r2, worldAxis);
                float K = rb1->GetInverseMass() + rb2->GetInverseMass() + (r1a * r1a) * rb1->GetInverseInertia() + (r2a * r2a) * rb2->GetInverseInertia();
                
                if(K > 0.0f)
                    limitImpulseMag = -vAxis / K;
            }

            Vec2 limitImpulse = worldAxis * limitImpulseMag;

            rb1->ApplyImpulseAtWorldPoint(-limitImpulse, anchorPoint);
            rb2->ApplyImpulseAtWorldPoint(limitImpulse, anchorPoint);
        }
        else if(IsRope())
        {
            Vec2 point1 = GetWorldAnchor1();
            Vec2 point2 = GetWorldAnchor2();

            Vec2 delta = point2 - point1;
            Vec2 normal = delta.Normalized();

            if(delta.MagnitudeSquared() < GetRope()->maxDistance * GetRope()->maxDistance)
                return;

            Vec2 relativeVel = rb2->GetVelocityAtWorldPoint(point2) - rb1->GetVelocityAtWorldPoint(point1);

            float vn = math::Dot(relativeVel, normal);

            if(vn <= 0.0f)
                return;

            float r1n = math::Cross((point1 - rb1->GetPosition()), normal);
            float r2n = math::Cross((point2 - rb2->GetPosition()), normal);

            float j = -vn / (rb1->GetInverseMass() + rb2->GetInverseMass() + (r1n * r1n) * rb1->GetInverseInertia() + (r2n * r2n) * rb2->GetInverseInertia());

            Vec2 impulse = normal * j;

            rb1->ApplyImpulseAtWorldPoint(-impulse, point1);
            rb2->ApplyImpulseAtWorldPoint(impulse, point2);
        }
        else if(IsSpring())
        {
            Vec2 point1 = GetWorldAnchor1();
            Vec2 point2 = GetWorldAnchor2();

            Vec2 delta = point2 - point1;
            Vec2 normal = delta.Normalized();

            Vec2 relativeVel = rb2->GetVelocityAtWorldPoint(point2) - rb1->GetVelocityAtWorldPoint(point1);

            float vn = math::Dot(relativeVel, normal);

            float x = delta.Magnitude() - GetSpring()->restDistance;
            float F = -(GetSpring()->stiffness * x) - (GetSpring()->damping * vn);

            float j = (F * timestep) / (float)iterations;
            
            Vec2 impulse = normal * j;
            
            rb1->ApplyImpulseAtWorldPoint(-impulse, point1);
            rb2->ApplyImpulseAtWorldPoint(impulse, point2);
        }
        else if(IsRevolute())
        {
            Vec2 anchorPoint = (GetWorldAnchor1() + GetWorldAnchor2()) * 0.5f;

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

            float currentRelativeAngle = rb2->GetRotation() - rb1->GetRotation();
            float relativeAngVel = rb2->GetAngularVelocity() - rb1->GetAngularVelocity();

            float angleDiff = currentRelativeAngle - GetRevolute()->referenceAngle;
            angleDiff = math::NormalizeAngle(angleDiff);

            float angImpulse = 0.0f;

            if((angleDiff <= GetRevolute()->lowerLimit && relativeAngVel < 0) || (angleDiff >= GetRevolute()->upperLimit && relativeAngVel > 0))
                angImpulse = -relativeAngVel / (rb1->GetInverseInertia() + rb2->GetInverseInertia());
            
            rb1->ApplyAngularImpulse(-angImpulse);
            rb2->ApplyAngularImpulse(angImpulse);
        }
        else if(IsTorsionSpring())
        {
            Vec2 anchorPoint = (GetWorldAnchor1() + GetWorldAnchor2()) * 0.5f;

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

            float currentRelativeAngle = rb2->GetRotation() - rb1->GetRotation();
            float relativeAngVel = rb2->GetAngularVelocity() - rb1->GetAngularVelocity();

            float angError = currentRelativeAngle - GetTorsionSpring()->restAngle;
            angError = math::NormalizeAngle(angError);

            float T = -(GetTorsionSpring()->stiffness * angError) - (GetTorsionSpring()->damping * relativeAngVel);
            float angImpulse = (T * timestep) / (float)iterations;

            rb1->ApplyAngularImpulse(-angImpulse);
            rb2->ApplyAngularImpulse(angImpulse);
        }
    }

    void Joint::SolvePosition()
    {
        if(IsWeld())
        {
            Vec2 point1 = GetWorldAnchor1();
            Vec2 point2 = GetWorldAnchor2();

            Vec2 error = point2 - point1;
            float invMassSum = rb1->GetInverseMass() + rb2->GetInverseMass();

            const float percent = 0.4f;
            const float slop = 0.005f;

            float errorMag = error.Magnitude();

            Vec2 correction = Vec2::Zero();

            if(invMassSum > 0.0f && errorMag > slop)
                correction = (error / errorMag) * (errorMag - slop) * percent / invMassSum;
            
            rb1->Move(correction * rb1->GetInverseMass());
            rb2->Move(-correction * rb2->GetInverseMass());

            float currentRelativeAngle = rb2->GetRotation() - rb1->GetRotation();
            float angError = currentRelativeAngle - GetWeld()->referenceAngle;
            
            angError = math::NormalizeAngle(angError);

            float invInertiaSum = rb1->GetInverseInertia() + rb2->GetInverseInertia();

            if(invInertiaSum <= 0.0f)
                return;

            const float angPercent = 0.2f;
            float angCorrection = (angError / invInertiaSum) * angPercent;

            rb1->Rotate(angCorrection * rb1->GetInverseInertia());
            rb2->Rotate(-angCorrection * rb2->GetInverseInertia());
        }
        else if(IsDistance())
        {
            Vec2 point1 = GetWorldAnchor1();
            Vec2 point2 = GetWorldAnchor2();

            Vec2 delta = point2 - point1;
            float currentDistance = delta.Magnitude();

            Vec2 normal;
            if(currentDistance > 0.0001f)
                normal = delta / currentDistance;
            else
                normal = Vec2::Right();

            float error = currentDistance - GetDistance()->distance;
            float invMassSum = rb1->GetInverseMass() + rb2->GetInverseMass();

            if(invMassSum <= 0.0f)
                return;

            const float percent = 0.4f;
            const float slop = 0.005f;

            if(std::abs(error) < slop)
                return;

            float correctionMag = (error > 0.0f ? error - slop : error + slop) * percent;
            Vec2 correction = (delta / currentDistance) * (correctionMag / invMassSum);

            rb1->Move(correction * rb1->GetInverseMass());
            rb2->Move(-correction * rb2->GetInverseMass());
        }
        else if(IsSlider())
        {
            Vec2 delta = GetWorldAnchor2() - GetWorldAnchor1();

            Vec2 worldAxis = math::Rotate(GetSlider()->localAxis, rb1->GetRotation());
            Vec2 perpAxis = math::Perpendicular(worldAxis);

            float error = math::Dot(delta, perpAxis);
            float invMassSum = rb1->GetInverseMass() + rb2->GetInverseMass();

            const float percent = 0.4f;
            const float slop = 0.005f;

            Vec2 correction = Vec2::Zero();

            if(invMassSum > 0.0f && std::abs(error) > slop)
                correction = perpAxis * ((error - slop) * percent / invMassSum);

            rb1->Move(correction * rb1->GetInverseMass());
            rb2->Move(-correction * rb2->GetInverseMass());

            float currentRelativeAngle = rb2->GetRotation() - rb1->GetRotation();
            float angError = currentRelativeAngle - GetSlider()->referenceAngle;
            
            angError = math::NormalizeAngle(angError);

            float invInertiaSum = rb1->GetInverseInertia() + rb2->GetInverseInertia();

            if(invInertiaSum <= 0.0f)
                return;

            const float angPercent = 0.2f;
            float angCorrection = (angError / invInertiaSum) * angPercent;

            rb1->Rotate(angCorrection * rb1->GetInverseInertia());
            rb2->Rotate(-angCorrection * rb2->GetInverseInertia());

            float currentTranslation = math::Dot(delta, worldAxis);
            float limitError = 0.0f;

            if(currentTranslation <= GetSlider()->lowerLimit)
                limitError = currentTranslation - GetSlider()->lowerLimit;
            else if(currentTranslation >= GetSlider()->upperLimit)
                limitError = currentTranslation - GetSlider()->upperLimit;

            float limitCorrectionMag = (limitError > 0.0f ? limitError - slop : limitError + slop) * percent;
            Vec2 limitCorrection = worldAxis * (limitCorrectionMag / invMassSum);
            
            rb1->Move(limitCorrection * rb1->GetInverseMass());
            rb2->Move(-limitCorrection * rb2->GetInverseMass());
        }
        else if(IsRope())
        {
            Vec2 point1 = GetWorldAnchor1();
            Vec2 point2 = GetWorldAnchor2();

            Vec2 delta = point2 - point1;
            float currentDistance = delta.Magnitude();

            Vec2 normal;
            if(currentDistance > 0.0001f)
                normal = delta / currentDistance;
            else
                normal = Vec2::Right();

            float error = currentDistance - GetRope()->maxDistance;
            float invMassSum = rb1->GetInverseMass() + rb2->GetInverseMass();

            if(invMassSum <= 0.0f)
                return;

            const float percent = 0.4f;
            const float slop = 0.005f;

            if(std::abs(error) < slop)
                return;

            if(error < 0.0f)
                return;

            float correctionMag = (error > 0.0f ? error - slop : error + slop) * percent;
            Vec2 correction = (delta / currentDistance) * (correctionMag / invMassSum);

            rb1->Move(correction * rb1->GetInverseMass());
            rb2->Move(-correction * rb2->GetInverseMass());
        }
        else if(IsSpring())
        {
            return;
        }
        else if(IsRevolute())
        {
            Vec2 point1 = GetWorldAnchor1();
            Vec2 point2 = GetWorldAnchor2();

            Vec2 error = point2 - point1;
            float invMassSum = rb1->GetInverseMass() + rb2->GetInverseMass();

            const float percent = 0.4f;
            const float slop = 0.005f;

            float errorMag = error.Magnitude();

            Vec2 correction = Vec2::Zero();

            if(invMassSum > 0.0f && errorMag > slop)
                correction = (error / errorMag) * (errorMag - slop) * percent / invMassSum;

            rb1->Move(correction * rb1->GetInverseMass());
            rb2->Move(-correction * rb2->GetInverseMass());

            float currentRelativeAngle = rb2->GetRotation() - rb1->GetRotation();
            float relativeAngVel = rb2->GetAngularVelocity() - rb1->GetAngularVelocity();

            float angleDiff = currentRelativeAngle - GetRevolute()->referenceAngle;
            angleDiff = math::NormalizeAngle(angleDiff);

            float angError = 0.0f;

            if(angleDiff <= GetRevolute()->lowerLimit && relativeAngVel < 0)
                angError = angleDiff - GetRevolute()->lowerLimit;
            if(angleDiff >= GetRevolute()->upperLimit && relativeAngVel > 0)
                angError = angleDiff - GetRevolute()->upperLimit;

            if(std::abs(angError) <= 0.0f)
                return;

            float invInertiaSum = rb1->GetInverseInertia() + rb2->GetInverseInertia();

            if(invInertiaSum <= 0.0f)
                return;

            const float angPercent = 0.2f;
            float angCorrection = (angError * angPercent) / invInertiaSum;

            rb1->Rotate(angCorrection * rb1->GetInverseInertia());
            rb2->Rotate(-angCorrection * rb2->GetInverseInertia());
        }
        else if(IsTorsionSpring())
        {
            Vec2 point1 = GetWorldAnchor1();
            Vec2 point2 = GetWorldAnchor2();

            Vec2 error = point2 - point1;
            float invMassSum = rb1->GetInverseMass() + rb2->GetInverseMass();

            if(invMassSum <= 0.0f)
                return;

            const float percent = 0.4f;
            const float slop = 0.005f;

            float errorMag = error.Magnitude();

            if(errorMag < slop)
                return;

            Vec2 correction = (error / errorMag) * (errorMag - slop) * percent / invMassSum;

            rb1->Move(correction * rb1->GetInverseMass());
            rb2->Move(-correction * rb2->GetInverseMass());
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
        
    bool Joint::IsCollisionDisabled() const
    {
        return disableCollision;
    }

    void Joint::DisableCollision(bool value)
    {
        disableCollision = value;
    }
}