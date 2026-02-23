#pragma once

#include <vector>

#include "../Core/Vec2.hpp"
#include "../Physics/Rigidbody.hpp"
#include "GridEntry.hpp"
#include "AABB.hpp"

namespace bp
{
    class HashGrid
    {
        private:
            float cellSize;
            std::vector<int> heads;
            std::vector<GridEntry> entries;

            static constexpr int gridSize = 4096;
            static constexpr int prime1 = 73856093;
            static constexpr int prime2 = 19349663;

            inline size_t GenerateHash(int x, int y) const;

            void Clear();
            void MapBodyToCells(int rbIndex, const AABB &aabb);

        public:
            HashGrid();
            HashGrid(float cellSize);

            void Refresh(const std::vector<Rigidbody *> &bodies);
            
            int GetHead(int hash) const;
            const GridEntry &GetEntry(int index) const;

            static int GetGridSize();
            float GetCellSize() const;
    };
}