#pragma once

#include <cmath>

namespace bp
{
    struct Vec2
    {
        float x, y;
        
        Vec2()
        {
            x = 0.0f;
            y = 0.0f;
        }
        Vec2(float x, float y)
        {
            this->x = x;
            this->y = y;
        }

        static Vec2 Zero()
        {
            return Vec2(0.0f, 0.0f);
        }
        static Vec2 One()
        {
            return Vec2(1.0f, 1.0f);
        }
        static Vec2 Up()
        {
            return Vec2(0.0f, 1.0f);
        }
        static Vec2 Right()
        {
            return Vec2(1.0f, 0.0f);
        }

        float Magnitude() const
        {
            return std::sqrt(MagnitudeSquared());
        }
        float MagnitudeSquared() const
        {
            return (x * x) + (y * y);
        }
        Vec2 Normalized() const
        {
            float mag = Magnitude();

            if(mag == 0)
                return Zero();

            return Vec2(x / mag, y / mag);
        }

        void Normalize()
        {
            float mag = Magnitude();

            if(mag == 0)
                return;

            x /= mag;
            y /= mag;
        }

        Vec2 operator + (const Vec2 &vec) const
        {
            return Vec2(x + vec.x, y + vec.y);
        }
        Vec2 operator - (const Vec2 &vec) const
        {
            return Vec2(x - vec.x, y - vec.y);
        }
        Vec2 operator * (const float scalar) const
        {
            return Vec2(x * scalar, y * scalar);
        }
        Vec2 operator / (const float scalar) const
        {
            if(scalar == 0)
                return Zero();
            else
                return Vec2(x / scalar, y / scalar);
        }

        Vec2 &operator += (const Vec2 &vec)
        {
            x += vec.x;
            y += vec.y;

            return *this;
        }
        Vec2 &operator -= (const Vec2 &vec)
        {
            x -= vec.x;
            y -= vec.y;

            return *this;
        }
        Vec2 &operator *= (float scalar)
        {
            x *= scalar;
            y *= scalar;

            return *this;
        }
        Vec2 &operator /= (float scalar)
        {
            if(scalar == 0)
            {
                x = 0;
                y = 0;
            }
            else
            {
                x /= scalar;
                y /= scalar;
            }

            return *this;
        }

        Vec2 operator - () const
        {
            return Vec2(-x, -y);
        }
    };
}