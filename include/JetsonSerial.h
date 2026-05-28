// #pragma once
// #include <string>

// struct FindBlockRawConfig{
//     int numSequentialBlocks;
//     int maxDifferenceDistance;
// };

// struct FindBlockRawVar{
//     int sequentialBlocksCount;

//     int lastBlockXPos;
//     int lastBlockYPos;
//     int lastBlockXDistance;
//     int lastBlockYDistance;

//     bool x_is_stable;
//     bool y_is_stable;
// };

// struct TrackBlockRawConfig{
//     int cameraCenterX;
//     int cameraCenterY;

//     int maxTrackingXJump;
//     int maxTrackingYJump;

//     int maxLostFrames;
// };

// struct TrackBlockRawVar{
//     int xError;
//     int yError;

//     int lastBlockXPos;
//     int lastBlockYPos;

//     int xJump;
//     int yJump;

//     int lostFrameCount;

//     bool targetVisible;
//     bool trackingLocked;

//     bool xJumpValid;
//     bool yJumpValid;
// };

// class JetsonSerial {
//     private:
//         char buffer[128] = {0};

//         FILE* serialPTR = nullptr;

//         size_t len;
//         std::string cmd_str;
        
//         char pos_string[10];
//         int comma_index;
//         int null_index;

//         std::string tempX;
//         std::string tempY;

//         FindBlockRawConfig findBlockRawConfig;
//         TrackBlockRawConfig trackBlockRawConfig;
//     public:
//         int block_x_pos;
//         int block_y_pos;

//         void JetsonSerialSetup();
//         void update_block_pose();

//         void find_block_raw_init(FindBlockRawConfig config);
//         bool find_block_raw_step();
//         FindBlockRawVar findBlockRawVar;

//         void track_block_raw_init(TrackBlockRawConfig config);
//         bool track_block_raw_step();
//         TrackBlockRawVar trackBlockRawVar;

//         void print_block_pos_on_screen();

//         int getXError();

//         int getYError();
// };

#pragma once
#include <string>
#include <stdio.h>

struct FindBlockRawConfig{
    int numSequentialBlocks;
    int maxDifferenceDistance;
};

struct FindBlockRawVar{
    int sequentialBlocksCount;

    int lastBlockXPos;
    int lastBlockYPos;
    int lastBlockXDistance;
    int lastBlockYDistance;

    bool x_is_stable;
    bool y_is_stable;
};

struct TrackBlockRawConfig{
    int cameraCenterX;
    int cameraCenterY;

    int maxTrackingXJump;
    int maxTrackingYJump;

    int maxLostFrames;
};

struct TrackBlockRawVar{
    int xError;
    int yError;

    int lastBlockXPos;
    int lastBlockYPos;

    int xJump;
    int yJump;

    int lostFrameCount;

    bool targetVisible;
    bool trackingLocked;

    bool xJumpValid;
    bool yJumpValid;
};

class JetsonSerial {
    public:
        static const int MAX_BLOCKS = 8;

    private:
        char buffer[128] = {0};

        FILE* serialPTR = nullptr;

        size_t len;
        std::string cmd_str;

        FindBlockRawConfig findBlockRawConfig;
        TrackBlockRawConfig trackBlockRawConfig;

        void clear_blocks();
        bool parse_xy_pair(const std::string& pair, int& x, int& y);
        bool select_block_index(int index);
        bool select_block_closest_to(int targetX, int targetY);

    public:
        // The currently selected block.
        // Your command code can keep using these like before.
        int block_x_pos;
        int block_y_pos;

        // All red blocks from the latest Jetson message.
        // Message format from Python:
        // 0
        // or
        // count;x1,y1;x2,y2;...
        int block_count;
        int block_x_positions[MAX_BLOCKS];
        int block_y_positions[MAX_BLOCKS];

        void JetsonSerialSetup();
        void update_block_pose();

        void find_block_raw_init(FindBlockRawConfig config);
        bool find_block_raw_step();
        FindBlockRawVar findBlockRawVar;

        void track_block_raw_init(TrackBlockRawConfig config);
        bool track_block_raw_step();
        TrackBlockRawVar trackBlockRawVar;

        void print_block_pos_on_screen();

        int getXError();
        int getYError();
};
