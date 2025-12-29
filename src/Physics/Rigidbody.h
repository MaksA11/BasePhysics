#pragma once

#include <vector>

#include "../Core/Vec2.h"
#include "../Collision/Collider.h"

namespace bp
{
    class Rigidbody
    {
        private:
            Rigidbody(Vec2 position, float rotation, Collider collider) : position(position), rotation(rotation), collider(collider) {}

            Vec2 position;
            float rotation;

            Collider collider;



        public:
            static Rigidbody *CreateCircleBody(Vec2 position, float rotation, float radius, float restitution, float friciton);
            static Rigidbody *CreateBoxBody(Vec2 position, float rotation, Vec2 size, float restitution, float friciton);
            static Rigidbody *CreatePolygonBody(Vec2 position, float rotation, std::vector<Vec2> vertices, float restitution, float friciton);
            static void DeleteRigidbody(Rigidbody *rb, std::vector<Rigidbody *> &bodies);

            Vec2 GetPosition();
            float GetRotation();

            Collider GetCollider();
    };
}