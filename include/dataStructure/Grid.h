#pragma once
#include <array>
#include <vector>

namespace DeltaVins {

template <typename T, int ROW, int COL>
struct Grid {
    Grid(int row_stride, int col_stride) {
        this->row_stride = row_stride;
        this->col_stride = col_stride;
    }

    void Add(const T& t, int x, int y) {
        int row = x / row_stride;
        int col = y / col_stride;
        grid[row * COL + col].push_back(t);
    }

    std::vector<T>& Get(int row, int col) { return grid[row * COL + col]; }

    std::vector<T>& Get(int index) { return grid[index]; }

    void Clear() {
        for (auto& v : grid) {
            v.clear();
        }
    }

    std::array<std::vector<T>, COL * ROW> grid;
    int row_stride = 0;
    int col_stride = 0;
};

}  // namespace DeltaVins