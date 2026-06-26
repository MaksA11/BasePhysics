#include <BasePhysics/Collision/BroadPhase/HashGrid.hpp>

namespace bp
{
    HashGrid::HashGrid() : cellSize(2.5f)
    {
        heads.assign(gridSize, std::numeric_limits<size_t>::max());
        entries.reserve(4096);
    }
    HashGrid::HashGrid(float cellSize) : cellSize(cellSize)
    {
        heads.assign(gridSize, std::numeric_limits<size_t>::max());
        entries.reserve(4096);
    }

    inline size_t HashGrid::GenerateHash(int x, int y) const
    {
        return static_cast<size_t>((static_cast<size_t>(x) * prime1 ^ static_cast<size_t>(y) * prime2) & (gridSize - 1));
    }

    void HashGrid::Clear()
    {
        std::fill(heads.begin(), heads.end(), std::numeric_limits<size_t>::max());
        entries.clear();
    }
    void HashGrid::MapBodyToCells(size_t rbIndex, const AABB &aabb)
    {
        int minX = static_cast<int>(std::floor(aabb.min.x / cellSize));
        int maxX = static_cast<int>(std::floor(aabb.max.x / cellSize));
        int minY = static_cast<int>(std::floor(aabb.min.y / cellSize));
        int maxY = static_cast<int>(std::floor(aabb.max.y / cellSize));

        for(int x = minX; x <= maxX; ++x)
        {
            for(int y = minY; y <= maxY; ++y)
            {
                size_t hash = GenerateHash(x, y);
                entries.push_back(GridEntry(rbIndex, heads[hash]));
                heads[hash] = entries.size() - 1;
            }
        }
    }

    void HashGrid::Refresh(const std::vector<Rigidbody *> &bodies)
    {
        Clear();
        for(size_t i = 0; i < bodies.size(); i++)
        {
            Rigidbody *rb = bodies[i];
            AABB aabb = rb->GetCollider().GetAABB(rb->GetPosition(), rb->GetRotation());

            MapBodyToCells(i, aabb);
        }
    }

    size_t HashGrid::GetHead(size_t hash) const
    {
        return heads[hash];
    }
    const GridEntry &HashGrid::GetEntry(size_t index) const
    {
        return entries[index];
    }
    size_t HashGrid::GetGridSize()
    {
        return gridSize;
    }

    float HashGrid::GetCellSize() const
    {
        return cellSize;
    }
}