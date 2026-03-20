#include "task_parameters.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <stdexcept>

std::unordered_map<std::string, double> TaskParameters::parameters;
bool TaskParameters::isLoaded = false;

bool TaskParameters::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open parameter file: " << filename << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        line.erase(std::remove_if(line.begin(), line.end(), [](unsigned char ch) {
            return ch == '\r' || ch == '\n';
        }), line.end());

        if (line.empty() || line[0] == '#') {
            continue;
        }

        size_t equalPos = line.find('=');
        if (equalPos != std::string::npos) {
            std::string key = line.substr(0, equalPos);
            std::string valueStr = line.substr(equalPos + 1);

            key.erase(0, key.find_first_not_of(" \t"));
            key.erase(key.find_last_not_of(" \t") + 1);
            valueStr.erase(0, valueStr.find_first_not_of(" \t"));
            valueStr.erase(valueStr.find_last_not_of(" \t") + 1);

            try {
                double value = std::stod(valueStr);
                parameters[key] = value;
            } catch (const std::exception& e) {
                std::cerr << "Warning: Failed to parse value for key '" << key << "': " << valueStr << std::endl;
            }
        }
    }

    file.close();
    isLoaded = true;

    std::cout << "Successfully loaded " << parameters.size() << " parameters from " << filename << std::endl;
    return true;
}

double TaskParameters::get(const std::string& key) {
    if (!isLoaded) {
        std::cerr << "Error: Parameters not loaded. Cannot get parameter: " << key << std::endl;
        throw std::runtime_error("Parameters not loaded");
    }

    auto it = parameters.find(key);
    if (it != parameters.end()) {
        return it->second;
    } else {
        std::cerr << "Error: Parameter '" << key << "' not found in configuration file" << std::endl;
        throw std::runtime_error("Parameter not found: " + key);
    }
}

void TaskParameters::printAll() {
    std::cout << "Loaded parameters:" << std::endl;
    for (const auto& pair : parameters) {
        std::cout << "  " << pair.first << " = " << pair.second << std::endl;
    }
}