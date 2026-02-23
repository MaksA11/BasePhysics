#include "HashGrid.hpp"

namespace bp
{
    HashGrid::HashGrid() : cellSize(2.5f)
    {
        heads.assign(gridSize, -1);
        entries.reserve(4096);
    }
    HashGrid::HashGrid(float cellSize) : cellSize(cellSize)
    {
        heads.assign(gridSize, -1);
        entries.reserve(4096);
    }

    inline size_t HashGrid::GenerateHash(int x, int y) const
    {
        return static_cast<size_t>((static_cast<size_t>(x) * prime1 ^ static_cast<size_t>(y) * prime2) & (gridSize - 1));
    }

    void HashGrid::Clear()
    {
        std::fill(heads.begin(), heads.end(), -1);
        entries.clear();
    }
    void HashGrid::MapBodyToCells(int rbIndex, const AABB &aabb)
    {
        int minX = static_cast<int>(std::floor(aabb.min.x / cellSize));
        int maxX = static_cast<int>(std::floor(aabb.max.x / cellSize));
        int minY = static_cast<int>(std::floor(aabb.min.y / cellSize));
        int maxY = static_cast<int>(std::floor(aabb.max.y / cellSize));

        for(int x = minX; x <= maxX; ++x)
        {
            for(int y = minY; y <= maxY; ++y)
            {
                int hash = GenerateHash(x, y);
                entries.push_back({rbIndex, heads[hash]});
                heads[hash] = (int)entries.size() - 1;
            }
        }
    }

    void HashGrid::Refresh(const std::vector<Rigidbody *> &bodies)
    {
        Clear();
        for(int i = 0; i < (int)bodies.size(); i++)
        {
            Rigidbody *rb = bodies[i];
            AABB aabb = rb->GetCollider().GetAABB(rb->GetPosition(), rb->GetRotation());

            MapBodyToCells(i, aabb);
        }
    }

    int HashGrid::GetHead(int hash) const
    {
        return heads[hash];
    }
    const GridEntry &HashGrid::GetEntry(int index) const
    {
        return entries[index];
    }
    int HashGrid::GetGridSize()
    {
        return gridSize;
    }

    float HashGrid::GetCellSize() const
    {
        return cellSize;
    }
}