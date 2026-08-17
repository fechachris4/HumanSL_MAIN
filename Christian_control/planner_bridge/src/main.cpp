#include <algorithm>
#include <iostream>
#include <vector>

#include "BridgeMain.h"

int main(int argc, char** argv) {
    const std::vector<std::string> args(argv + 1, argv + argc);
    return RunBridge(args, std::cout, std::cerr);
}
