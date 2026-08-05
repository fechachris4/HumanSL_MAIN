#include <iostream>
#include <vector>
#include "BridgeMain.h"
int main(int argc, char** argv) {
    return RunBridge(std::vector<std::string>(argv + 1, argv + argc),
                     std::cout, std::cerr);
}
