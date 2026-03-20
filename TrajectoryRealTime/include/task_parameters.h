#ifndef TASK_PARAMETERS_H
#define TASK_PARAMETERS_H

#include <string>
#include <unordered_map>

class TaskParameters {
private:
    static std::unordered_map<std::string, double> parameters;
    static bool isLoaded;

public:
    static bool loadFromFile(const std::string& filename);
    static double get(const std::string& key);
    static void printAll();
};

#endif