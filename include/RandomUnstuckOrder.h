#pragma once

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

    unsigned int nextRandom(unsigned int& seed) {
        seed = seed * 1103515245u + 12345u;
        return seed;
    }

public:
    RandomUnstuckOrder()
        : currentIndex(0)
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

    int current() const {
        return moves[currentIndex];
    }

    bool advance() {
        currentIndex++;
        return currentIndex < 4;
    }
};
