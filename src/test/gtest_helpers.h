#include <gtest/gtest.h>

#if defined(GTestColor) // ((ROS_VERSION_MAJOR == 1) && (ROS_VERSION_MINOR <= 13))
namespace testing
{
    namespace internal
    {
        enum GTestColor {
            COLOR_DEFAULT,
            COLOR_RED,
            COLOR_GREEN,
            COLOR_YELLOW
        };

        extern void ColoredPrintf(GTestColor color, const char* fmt, ...);
    }
}
#define PRINTF(...)  do { testing::internal::ColoredPrintf(testing::internal::COLOR_GREEN, "[          ] "); testing::internal::ColoredPrintf(testing::internal::COLOR_YELLOW, __VA_ARGS__); } while(0)
#else
#define PRINTF(...)
#endif

// C++ stream interface
class TestCout : public std::stringstream
{
public:
    ~TestCout()
    {
        PRINTF("%s",str().c_str());
    }
};

#define TEST_COUT  TestCout()
