#pragma once

#include <vector>

#include "../Core/Vec2.h"

namespace bp
{
    class Rigidbody
    {
        private:
            Rigidbody(Vec2 position, float rotation, Vec2 size);

            Vec2 size;

            Vec2 position;
            float rotation;

        public:
            static Rigidbody *CreateRigidbody(Vec2 position, float rotation, Vec2 size);
            static void DeleteRigidbody(Rigidbody *rb, std::vector<Rigidbody *> &bodies);

            Vec2 GetPosition();
            Vec2 GetSize();
            float GetRotation();
    };
}