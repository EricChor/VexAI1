#pragma once
#include "Command.h"
#include <vector>

class Scheduler {
    private:
        std::vector<Command*> commands;
        std::vector<bool> initialized;

        void removeCommand(int index) {
            commands.erase(commands.begin() + index);
            initialized.erase(initialized.begin() + index);
        }

    public:
        Scheduler() {
            commands.reserve(20);
            initialized.reserve(20);
        }

        void schedule(Command* command) {
            if (command == nullptr) {
                return;
            }

            commands.push_back(command);
            initialized.push_back(false);
        }

        void run() {
            for (int i = 0; i < static_cast<int>(commands.size()); i++) {
                Command* command = commands[i];

                if (command == nullptr) {
                    removeCommand(i);
                    i--;
                    continue;
                }

                if (!initialized[i]) {
                    command->initialize();
                    initialized[i] = true;
                }

                command->execute();

                if (command->isFinished()) {
                    command->end();
                    removeCommand(i);
                    i--;
                }
            }
        }

        void cancelAll() {
            for (int i = 0; i < static_cast<int>(commands.size()); i++) {
                if (commands[i] != nullptr && initialized[i]) {
                    commands[i]->end();
                }
            }

            commands.clear();
            initialized.clear();
        }

        bool isEmpty() const {
            return commands.empty();
        }
};