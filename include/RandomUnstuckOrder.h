#pragma once

static constexpr float LATERAL_SHIFT_TIME_MULTIPLIER = 2.0f;
static constexpr int REQUIRED_CONSECUTIVE_STUCK_CHECKS = 2;

enum RandomUnstuckMove {
    RANDOM_UNSTUCK_FORWARD_RIGHT = 0,
    RANDOM_UNSTUCK_FORWARD_LEFT = 1,
    RANDOM_UNSTUCK_BACK_RIGHT = 2,
    RANDOM_UNSTUCK_BACK_LEFT = 3
};

class RandomUnstuckOrder {
private:
    int moves[4];
    int currentIndex;
    int moveCount;

    unsigned int nextRandom(unsigned int& seed) {
        seed = seed * 1103515245u + 12345u;
        return seed;
    }

public:
    RandomUnstuckOrder()
        : currentIndex(0),
          moveCount(4)
    {
        moves[0] = RANDOM_UNSTUCK_FORWARD_RIGHT;
        moves[1] = RANDOM_UNSTUCK_FORWARD_LEFT;
        moves[2] = RANDOM_UNSTUCK_BACK_RIGHT;
        moves[3] = RANDOM_UNSTUCK_BACK_LEFT;
    }

    void reset(unsigned int seed) {
        moves[0] = RANDOM_UNSTUCK_FORWARD_RIGHT;
        moves[1] = RANDOM_UNSTUCK_FORWARD_LEFT;
        moves[2] = RANDOM_UNSTUCK_BACK_RIGHT;
        moves[3] = RANDOM_UNSTUCK_BACK_LEFT;
        currentIndex = 0;
        moveCount = 4;

        if (seed == 0) {
            seed = 1;
        }

        for (int i = 3; i > 0; i--) {
            int swapIndex = static_cast<int>(nextRandom(seed) % static_cast<unsigned int>(i + 1));
            int temp = moves[i];
            moves[i] = moves[swapIndex];
            moves[swapIndex] = temp;
        }
    }

    // Travel opposite the stuck direction, then switch the turn direction
    // so the robot ends parallel but shifted to one side.
    void resetLateralShift(unsigned int seed, bool wasMovingForward) {
        if (seed == 0) {
            seed = 1;
        }

        bool shiftLeftFirst = (nextRandom(seed) % 2u) == 0u;

        if (wasMovingForward) {
            moves[0] = shiftLeftFirst
                ? RANDOM_UNSTUCK_BACK_LEFT
                : RANDOM_UNSTUCK_BACK_RIGHT;
            moves[1] = shiftLeftFirst
                ? RANDOM_UNSTUCK_BACK_RIGHT
                : RANDOM_UNSTUCK_BACK_LEFT;
        } else {
            moves[0] = shiftLeftFirst
                ? RANDOM_UNSTUCK_FORWARD_LEFT
                : RANDOM_UNSTUCK_FORWARD_RIGHT;
            moves[1] = shiftLeftFirst
                ? RANDOM_UNSTUCK_FORWARD_RIGHT
                : RANDOM_UNSTUCK_FORWARD_LEFT;
        }

        currentIndex = 0;
        moveCount = 2;
    }

    int current() const {
        return moves[currentIndex];
    }

    bool advance() {
        currentIndex++;
        return currentIndex < moveCount;
    }
};
