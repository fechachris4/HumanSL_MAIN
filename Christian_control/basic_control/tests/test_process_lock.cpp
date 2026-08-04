// Hardware-free regression tests for the process-wide controller lock.

#include <cerrno>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <system_error>

#include <unistd.h>

#include "ProcessLock.h"

namespace
{
    int failures = 0;

    void Check(bool ok, const std::string& what)
    {
        if (!ok) {
            std::cout << "FAIL: " << what << "\n";
            ++failures;
        }
    }

    class TemporaryLockFile
    {
    public:
        TemporaryLockFile()
        {
            char path_template[] = "/tmp/basic_control_process_lock_test.XXXXXX";
            const int fd = ::mkstemp(path_template);
            if (fd < 0)
                throw std::system_error(errno, std::generic_category(),
                                        "mkstemp for process-lock test");
            ::close(fd);
            path_ = path_template;
        }

        ~TemporaryLockFile()
        {
            if (!path_.empty())
                ::unlink(path_.c_str());
        }

        const std::string& path() const { return path_; }

    private:
        std::string path_;
    };

    void TestSecondIndependentOpenIsRejected()
    {
        TemporaryLockFile file;
        const std::string busy_message =
            "another test controller already owns this resource";
        ProcessLock first(file.path(), busy_message);

        bool rejected = false;
        try {
            ProcessLock second(file.path(), busy_message);
        } catch (const std::runtime_error& error) {
            rejected = true;
            Check(error.what() == busy_message,
                  "duplicate acquisition reports the supplied clear message");
        }
        Check(rejected,
              "a second independent open in the same process cannot acquire the lock");
    }

    void TestDestructionLeavesNoStaleLock()
    {
        TemporaryLockFile file;
        const std::string busy_message = "process lock is already held";
        {
            ProcessLock first(file.path(), busy_message);
        }

        Check(::access(file.path().c_str(), F_OK) == 0,
              "the persistent lock file still exists after release");
        try {
            ProcessLock reacquired(file.path(), busy_message);
        } catch (const std::exception& error) {
            Check(false, "destruction permits reacquisition without a stale lock: " +
                             std::string(error.what()));
        }
    }
} // namespace

int main()
{
    try {
        TestSecondIndependentOpenIsRejected();
        TestDestructionLeavesNoStaleLock();
    } catch (const std::exception& error) {
        std::cout << "FAIL: unexpected exception: " << error.what() << "\n";
        return 1;
    }

    if (failures == 0) {
        std::cout << "all process-lock tests passed\n";
        return 0;
    }
    std::cout << failures << " test(s) failed\n";
    return 1;
}
