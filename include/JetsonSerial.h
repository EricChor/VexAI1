#pragma once
#include <string>

class JetsonSerial {
    private:
        int block_x_pos;
        int block_y_pos;

        int track_block_num_ticks_identification;
        int track_block_max_distance_identification;

        bool tracking_block;
        char buffer[128] = {0};

        FILE* serialPTR;

        size_t len;
        std::string cmd_str;
        
        char pos_string[10];
        int comma_index;
        int null_index;

        std::string tempX;
        std::string tempY;

    public:
        void JetsonSerialSetup();
        void update_block_pose();

        void find_block_init();
        bool find_block_step();

        void track_block_init();
        bool track_block_step();

        void print_block_pos_on_screen();
};