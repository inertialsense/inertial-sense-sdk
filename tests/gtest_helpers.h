#include <gtest/gtest.h>
#ifndef GTEST_HELPERS_H
#define GTEST_HELPERS_H

#include <chrono>

#if defined(GTestColor)
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

#define ANSI_BLACK       "\u001b[30m"
#define ANSI_DARKGRAY    "\u001b[1,30m"
#define ANSI_DARKRED     "\u001b[31m"
#define ANSI_RED         "\u001b[1,31m"
#define ANSI_DARKGREEN   "\u001b[32m"
#define ANSI_GREEN       "\u001b[1,32m"
#define ANSI_BROWN       "\u001b[33m"
#define ANSI_YELLOW      "\u001b[1,33m"
#define ANSI_DARKBLUE    "\u001b[34m"
#define ANSI_BLUE        "\u001b[1,34m"
#define ANSI_DARKMAGENTA "\u001b[35m"
#define ANSI_MAGENTA     "\u001b[1,35m"
#define ANSI_DARKCYAN    "\u001b[36m"
#define ANSI_CYAN        "\u001b[1,36m"
#define ANSI_GRAY        "\u001b[37m"
#define ANSI_WHITE       "\u001b[1,37m"
#define ANSI_DEFAULT     "\u001b[39m"
#define ANSI_RESET       "\u001b[0m"

#define TEST_PRINTF(...)  do { testing::internal::ColoredPrintf(testing::internal::COLOR_GREEN, "[          ] "); testing::internal::ColoredPrintf(testing::internal::COLOR_YELLOW, __VA_ARGS__); } while (0)
#else
#define ANSI_BLACK       ""
#define ANSI_DARKGRAY    ""
#define ANSI_DARKRED     ""
#define ANSI_RED         ""
#define ANSI_DARKGREEN   ""
#define ANSI_GREEN       ""
#define ANSI_BROWN       ""
#define ANSI_YELLOW      ""
#define ANSI_DARKBLUE    ""
#define ANSI_BLUE        ""
#define ANSI_DARKMAGENTA ""
#define ANSI_MAGENTA     ""
#define ANSI_DARKCYAN    ""
#define ANSI_CYAN        ""
#define ANSI_GRAY        ""
#define ANSI_WHITE       ""
#define ANSI_DEFAULT     ""
#define ANSI_RESET       ""

#define TEST_PRINTF(...)        do { printf("[%10c] ", ' '); printf(__VA_ARGS__); fflush(stdout); } while (0);

#define TEST_PRINTF_TS(...)     do { auto now = std::chrono::system_clock::now();                                                   \
                                     auto ms = duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;             \
                                     std::time_t now_c = std::chrono::system_clock::to_time_t(now);                                 \
                                     std::tm* local_tm = std::localtime(&now_c);                                                    \
                                     std::stringstream fs; fs << std::put_time(local_tm, "%H:%M:%S");                               \
                                     printf("[%s.%03lld]", fs.str().c_str(), static_cast<long long>(ms.count()));                  \
                                     printf(__VA_ARGS__); fflush(stdout); } while (0);
#endif

// C++ stream interface
class TestCout : public std::stringstream
{
public:
    ~TestCout() { TEST_PRINTF("%s", str().c_str()); }
};

#define TEST_COUT  TestCout()


// C++ stream interface
class TestCoutTs : public std::stringstream
{
public:
    ~TestCoutTs() { TEST_PRINTF_TS("%s", str().c_str()); }
};

#define TEST_COUT_TS  TestCoutTs()

#endif // GTEST_HELPERS_H