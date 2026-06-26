#pragma once

#include <vector>

#include <BasePhysics/Core/Vec2.hpp>
#include <BasePhysics/Physics/Rigidbody.hpp>
#include <BasePhysics/Collision/BroadPhase/GridEntry.hpp>
#include <BasePhysics/Collision/BroadPhase/AABB.hpp>

namespace bp
{
    class HashGrid
    {
        private:
            float cellSize;
            std::vector<size_t> heads;
            std::vector<GridEntry> entries;

            static constexpr size_t gridSize = 4096;
            static constexpr size_t prime1 = 73856093;
            static constexpr size_t prime2 = 19349663;

            inline size_t GenerateHash(int x, int y) const;

            void Clear();
            void MapBodyToCells(size_t rbIndex, const AABB &aabb);

        public:
            HashGrid();
            HashGrid(float cellSize);

            void Refresh(const std::vector<Rigidbody *> &bodies);
            
            size_t GetHead(size_t hash) const;
            const GridEntry &GetEntry(size_t index) const;

            static size_t GetGridSize();
            float GetCellSize() const;
    };
}