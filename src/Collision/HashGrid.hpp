#pragma once

#include <unordered_map>
#include <vector>

#include "../Core/Vec2.hpp"
#include "../Physics/Rigidbody.hpp"
#include "AABB.hpp"

namespace bp
{
    class HashGrid
    {
        private:
            float cellSize;
            std::unordered_map<size_t, std::vector<int>> grid;

            static constexpr int gridSize = 10000;
            static constexpr int prime1 = 73856093;
            static constexpr int prime2 = 19349663;

            void Clear();
            void MapBodyToCells(int rbIndex, const AABB &aabb);

        public:
            HashGrid() : cellSize(2.5f) {};
            HashGrid(float cellSize) : cellSize(cellSize) {};

            void Refresh(const std::vector<Rigidbody *> &bodies);
            const std::unordered_map<size_t, std::vector<int>> &GetGrid() const;

            float GetCellSize() const;
    };
}