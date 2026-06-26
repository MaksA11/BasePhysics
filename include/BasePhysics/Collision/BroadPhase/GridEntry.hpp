#pragma once

namespace bp
{
    struct GridEntry
    {
        size_t rbIndex;
        size_t next;

        GridEntry(size_t rbIndex, size_t next) : rbIndex(rbIndex), next(next) {}
    };
}