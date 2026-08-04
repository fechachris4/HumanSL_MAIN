// Linux advisory-file-lock implementation for ProcessLock.

#include "ProcessLock.h"

#include <cerrno>
#include <fcntl.h>
#include <stdexcept>
#include <system_error>

#include <sys/file.h>
#include <unistd.h>

ProcessLock::ProcessLock(const std::string& path,
                         const std::string& already_locked_message)
{
    fd_ = ::open(path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0600);
    if (fd_ < 0)
        throw std::system_error(errno, std::generic_category(),
                                "cannot open process lock '" + path + "'");

    if (::flock(fd_, LOCK_EX | LOCK_NB) == 0)
        return;

    const int lock_error = errno;
    ::close(fd_);
    fd_ = -1;
    if (lock_error == EWOULDBLOCK || lock_error == EAGAIN)
        throw std::runtime_error(already_locked_message);
    throw std::system_error(lock_error, std::generic_category(),
                            "cannot acquire process lock '" + path + "'");
}

ProcessLock::~ProcessLock()
{
    if (fd_ >= 0)
        // Closing the last descriptor releases flock, including when the
        // kernel closes descriptors after a crash. Keep the file itself:
        // unlinking it would permit two processes to lock different inodes.
        ::close(fd_);
}
