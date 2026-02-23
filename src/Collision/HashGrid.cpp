#include "HashGrid.hpp"

namespace bp
{
    size_t HashGrid::GenerateHash(Vec2 pos)
    {
        int x = static_cast<int>(std::floor(pos.x / cellSize));
        int y = static_cast<int>(std::floor(pos.y / cellSize));

        return ((static_cast<size_t>(x) * prime1) ^ (static_cast<size_t>(y) * prime2)) % gridSize;
    }

    void HashGrid::Clear()
    {
        for(auto &[key, cell] : grid)
        {
            cell.clear();
        }
    }
    void HashGrid::MapBodiesToCells(int rbIndex, const AABB &aabb)
    {
        int minX = static_cast<int>(std::floor(aabb.min.x / cellSize));
        int maxX = static_cast<int>(std::floor(aabb.max.x / cellSize));
        int minY = static_cast<int>(std::floor(aabb.min.y / cellSize));
        int maxY = static_cast<int>(std::floor(aabb.max.y / cellSize));

        for(int x = minX; x <= maxX; ++x)
        {
            for(int y = minY; y <= maxY; ++y)
            {
                size_t hash = ((static_cast<size_t>(x) * prime1) ^ (static_cast<size_t>(y) * prime2)) % gridSize;
                grid[hash].push_back(rbIndex);
            }
        }
    }

    void HashGrid::Refresh(const std::vector<Rigidbody*> &bodies)
    {
        Clear();
        for(int i = 0; i < (int)bodies.size(); i++)
        {
            Rigidbody *rb = bodies[i];
            AABB aabb = rb->GetCollider().GetAABB(rb->GetPosition(), rb->GetRotation());

            MapBodiesToCells(i, aabb);
        }
    }

    const std::unordered_map<size_t, std::vector<int>> &HashGrid::GetGrid() const
    {
        return grid;
    }

    float HashGrid::GetCellSize() const
    {
        return cellSize;
    }
}