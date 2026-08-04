// Process-wide exclusive ownership of one controller resource.

#pragma once

#include <string>

class ProcessLock
{
public:
    ProcessLock(const std::string& path,
                const std::string& already_locked_message);
    ~ProcessLock();

    ProcessLock(const ProcessLock&) = delete;
    ProcessLock& operator=(const ProcessLock&) = delete;

private:
    int fd_ = -1;
};
