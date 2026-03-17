#pragma once

// Core utilities
#include <BasePhysics/Core/Vec2.hpp>
#include <BasePhysics/Core/Math.hpp>
#include <BasePhysics/Core/Utils.hpp>
#include <BasePhysics/Core/Geometry.hpp>

// Collision system
#include <BasePhysics/Collision/ColliderShapes.hpp>
#include <BasePhysics/Collision/Collider.hpp>
#include <BasePhysics/Collision/BroadPhase/AABB.hpp>
#include <BasePhysics/Collision/BroadPhase/HashGrid.hpp>
#include <BasePhysics/Collision/NarrowPhase/ContactManifold.hpp>
#include <BasePhysics/Collision/NarrowPhase/Collisions.hpp>

// Physics system
#include <BasePhysics/Physics/BodyPreset.hpp>
#include <BasePhysics/Physics/Rigidbody.hpp>
#include <BasePhysics/Physics/PhysicsScene.hpp>