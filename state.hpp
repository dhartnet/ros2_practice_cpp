#pragma once

struct State {
    double x;
    double y;

    bool operator==(const State& other) const {
        return x == other.x && y == other.y;
    }
};