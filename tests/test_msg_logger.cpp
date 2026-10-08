/**
 * @file test_msg_logger.cpp
 * @brief Run-time log output redirection: IS_LOG_SET_OUTPUT_PATH(), IS_LOG_GET_OUTPUT_PATH() and IS_LOG_OUTPUT().
 *
 * Applications reconfigure the SDK log while it is in use (EvalTool's Application Log Settings
 * dialog, SN-8777), so a switch must close the file the logger opened, must leave the current
 * output intact when the new file cannot be opened, and must not lose or split lines being
 * written by other threads.
 */

#include <gtest/gtest.h>

#include "msg_logger.h"

#include <atomic>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

namespace {

/** Reads a text file into lines. */
std::vector<std::string> readLines(const fs::path& p) {
    std::vector<std::string> lines;
    std::ifstream in(p);
    for (std::string line; std::getline(in, line);)
        lines.push_back(line);
    return lines;
}

/** Counts the lines of p that contain needle. */
size_t countContaining(const fs::path& p, const std::string& needle) {
    size_t n = 0;
    for (const auto& line : readLines(p))
        if (line.find(needle) != std::string::npos)
            n++;
    return n;
}

#if defined(__linux__)
/** Returns whether this process holds an open descriptor on p. */
bool processHasOpen(const fs::path& p) {
    std::error_code ec;
    const fs::path target = fs::canonical(p, ec);
    for (const auto& entry : fs::directory_iterator("/proc/self/fd", ec)) {
        std::error_code lec;
        if (fs::read_symlink(entry.path(), lec) == target)
            return true;
    }
    return false;
}
#endif

class MsgLoggerOutputTest : public ::testing::Test {
protected:
    void SetUp() override {
        savedPath = IS_LOG_GET_OUTPUT_PATH();
        savedLevel = IS_GET_LOG_LEVEL();
        IS_SET_LOG_LEVEL(IS_LOG_LEVEL_INFO);
        // Deliberately not keyed on UnitTest::random_seed(): in GoogleTest >= 1.14, that seed is
        // derived from the current time in milliseconds even when --gtest_shuffle is off (earlier
        // than 1.14 it was a constant 0 unless shuffling), so it is NOT guaranteed to match between
        // this (parent) process and the freshly re-exec'd child of a "threadsafe"-style death test
        // below -- which would make the parent and child compute two different directories for the
        // same logical test. The test name alone is stable across processes and is already unique
        // within a single run, which is all that's needed here.
        dir = fs::temp_directory_path() / (std::string("is_msg_logger_") +
                                           ::testing::UnitTest::GetInstance()->current_test_info()->name());
        fs::remove_all(dir);
        fs::create_directories(dir);
    }

    void TearDown() override {
        // Restore the logger before removing the directory, so the temp files are closed first.
        if (savedPath.empty() || IS_LOG_SET_OUTPUT_PATH(savedPath.c_str(), 1) != 0)
            IS_LOG_SET_OUTPUT_PATH("STDOUT", 1);
        IS_SET_LOG_LEVEL(savedLevel);
        std::error_code ec;
        fs::remove_all(dir, ec);
    }

    std::string savedPath;
    eLogLevel savedLevel = IS_LOG_LEVEL_INFO;
    fs::path dir;
};

}  // namespace

TEST_F(MsgLoggerOutputTest, SetOutputPath_WritesToFileAndReportsPath) {
    const fs::path a = dir / "a.log";
    ASSERT_EQ(0, IS_LOG_SET_OUTPUT_PATH(a.string().c_str(), 0));
    EXPECT_EQ(a.string(), IS_LOG_GET_OUTPUT_PATH());

    log_info(IS_LOG_FACILITY_NONE, "marker-one");
    EXPECT_EQ(1u, countContaining(a, "marker-one"));
}

TEST_F(MsgLoggerOutputTest, SwitchingOutput_ClosesPreviousFileAndRoutesNewLinesOnlyToNewFile) {
    const fs::path a = dir / "a.log";
    const fs::path b = dir / "b.log";

    ASSERT_EQ(0, IS_LOG_SET_OUTPUT_PATH(a.string().c_str(), 0));
    log_info(IS_LOG_FACILITY_NONE, "before-switch");
    ASSERT_EQ(0, IS_LOG_SET_OUTPUT_PATH(b.string().c_str(), 0));
    log_info(IS_LOG_FACILITY_NONE, "after-switch");

    EXPECT_EQ(1u, countContaining(a, "before-switch"));
    EXPECT_EQ(0u, countContaining(a, "after-switch"));
    EXPECT_EQ(0u, countContaining(b, "before-switch"));
    EXPECT_EQ(1u, countContaining(b, "after-switch"));
#if defined(__linux__)
    EXPECT_FALSE(processHasOpen(a)) << "previous log file was not closed";
    EXPECT_TRUE(processHasOpen(b));
#endif
}

TEST_F(MsgLoggerOutputTest, AppendFlag_PreservesOrTruncatesExistingContent) {
    const fs::path a = dir / "a.log";
    { std::ofstream(a) << "existing-line\n"; }

    ASSERT_EQ(0, IS_LOG_SET_OUTPUT_PATH(a.string().c_str(), 1));
    log_info(IS_LOG_FACILITY_NONE, "appended");
    EXPECT_EQ(1u, countContaining(a, "existing-line"));
    EXPECT_EQ(1u, countContaining(a, "appended"));

    ASSERT_EQ(0, IS_LOG_SET_OUTPUT_PATH(a.string().c_str(), 0));
    log_info(IS_LOG_FACILITY_NONE, "truncated");
    EXPECT_EQ(0u, countContaining(a, "existing-line"));
    EXPECT_EQ(1u, countContaining(a, "truncated"));
}

TEST_F(MsgLoggerOutputTest, UnopenablePath_FailsAndLeavesCurrentOutputActive) {
    const fs::path a = dir / "a.log";
    ASSERT_EQ(0, IS_LOG_SET_OUTPUT_PATH(a.string().c_str(), 0));

    const fs::path bad = dir / "no-such-dir" / "x.log";
    EXPECT_EQ(-1, IS_LOG_SET_OUTPUT_PATH(bad.string().c_str(), 0));
    EXPECT_EQ(a.string(), IS_LOG_GET_OUTPUT_PATH());

    log_info(IS_LOG_FACILITY_NONE, "still-here");
    EXPECT_EQ(1u, countContaining(a, "still-here"));
}

TEST_F(MsgLoggerOutputTest, StandardStreamNames_AreCaseInsensitiveAndReportedCanonically) {
    ASSERT_EQ(0, IS_LOG_SET_OUTPUT_PATH("stdout", 1));
    EXPECT_STREQ("STDOUT", IS_LOG_GET_OUTPUT_PATH());
    ASSERT_EQ(0, IS_LOG_SET_OUTPUT_PATH("StdErr", 1));
    EXPECT_STREQ("STDERR", IS_LOG_GET_OUTPUT_PATH());
}

TEST_F(MsgLoggerOutputTest, LogOutputStream_ClosesFileOpenedByPath) {
    const fs::path a = dir / "a.log";
    ASSERT_EQ(0, IS_LOG_SET_OUTPUT_PATH(a.string().c_str(), 0));

    IS_LOG_OUTPUT(stdout);
    EXPECT_STREQ("STDOUT", IS_LOG_GET_OUTPUT_PATH());
#if defined(__linux__)
    EXPECT_FALSE(processHasOpen(a));
#endif
}

TEST_F(MsgLoggerOutputTest, SwitchingWhileOtherThreadsLog_LosesAndSplitsNoLines) {
    const fs::path a = dir / "a.log";
    const fs::path b = dir / "b.log";
    ASSERT_EQ(0, IS_LOG_SET_OUTPUT_PATH(a.string().c_str(), 1));

    constexpr int kThreads = 4;
    constexpr int kMessagesPerThread = 2000;
    std::atomic<bool> go{false};
    std::vector<std::thread> writers;
    for (int t = 0; t < kThreads; t++) {
        writers.emplace_back([&go, t] {
            while (!go.load()) {}
            for (int i = 0; i < kMessagesPerThread; i++)
                log_info(IS_LOG_FACILITY_NONE, "concurrent t=%d i=%d end", t, i);
        });
    }

    go = true;
    for (int s = 0; s < 200; s++)
        ASSERT_EQ(0, IS_LOG_SET_OUTPUT_PATH(((s % 2) ? a : b).string().c_str(), 1));
    for (auto& w : writers)
        w.join();
    IS_LOG_SET_OUTPUT_PATH("STDOUT", 1);

    size_t total = 0;
    for (const auto& p : {a, b}) {
        for (const auto& line : readLines(p)) {
            if (line.find("concurrent") == std::string::npos)
                continue;
            total++;
            EXPECT_EQ(line.size() - 3, line.rfind("end")) << "split or interleaved line: " << line;
        }
    }
    EXPECT_EQ(static_cast<size_t>(kThreads * kMessagesPerThread), total);
}

TEST_F(MsgLoggerOutputTest, SwitchingToSamePathDestructively_WhileOtherThreadsLog_LosesAndSplitsNoLines) {
    const fs::path a = dir / "a.log";
    ASSERT_EQ(0, IS_LOG_SET_OUTPUT_PATH(a.string().c_str(), 1));

    constexpr int kThreads = 4;
    constexpr int kMessagesPerThread = 2000;
    std::atomic<bool> go{false};
    std::vector<std::thread> writers;
    for (int t = 0; t < kThreads; t++) {
        writers.emplace_back([&go, t] {
            while (!go.load()) {}
            for (int i = 0; i < kMessagesPerThread; i++)
                log_info(IS_LOG_FACILITY_NONE, "concurrent t=%d i=%d end", t, i);
        });
    }

    go = true;
    // Repeatedly re-truncate the SAME path the writers are actively logging to. Before the fix,
    // the destructive fopen(path, "w") happened before log_mutex was taken, so this truncation
    // could land while another thread was mid-record on the old stream (same underlying file),
    // losing a prefix or leaving a stale-offset gap. With the fix, the open is deferred until
    // log_mutex is held, serializing it against every write+flush (which also hold the lock for
    // their full duration), so a same-path switch can never tear a record in progress.
    for (int s = 0; s < 200; s++)
        ASSERT_EQ(0, IS_LOG_SET_OUTPUT_PATH(a.string().c_str(), 0));
    for (auto& w : writers)
        w.join();
    IS_LOG_SET_OUTPUT_PATH("STDOUT", 1);

    // Repeated same-path truncation is expected to discard most prior content -- that's the
    // intended behavior of a destructive switch -- so unlike the cross-file test above we can't
    // assert a total count. We can and do assert that whatever lines survive are intact (never
    // split or corrupted by a truncation landing mid-write), and that at least some logging
    // actually interleaved with the truncations so the race window was genuinely exercised.
    size_t total = 0;
    for (const auto& line : readLines(a)) {
        if (line.find("concurrent") == std::string::npos)
            continue;
        total++;
        EXPECT_EQ(line.size() - 3, line.rfind("end")) << "split or truncated-mid-record line: " << line;
    }
    EXPECT_GT(total, 0u);
}

#if !defined(_WIN32)
/** Logs from an atexit handler that runs after the logger's own exit handler, as a static destructor would. */
static void logFromLateExitHandler() {
    log_info(IS_LOG_FACILITY_NONE, "logged-during-exit");
}

TEST_F(MsgLoggerOutputTest, MessagesLoggedDuringExit_GoToConfiguredOutput) {
    const fs::path a = dir / "a.log";
    const fs::path cwd = dir / "cwd";
    fs::create_directories(cwd);

    // Force the "threadsafe" (re-exec) death-test style for this test only. The default POSIX
    // "fast" style forks the current process, which -- by the time this test runs -- has already
    // completed the logger's pthread_once init and registered its atexit handler in earlier
    // fixture tests above. A forked child inherits that already-done once-state, so its own
    // std::atexit(logFromLateExitHandler) call below would be the *last* registration in the
    // child's inherited list, making it run BEFORE the logger's exit handler (LIFO) instead of
    // after -- the opposite of what this test means to exercise, and not a configuration that
    // would catch a regression in the ordering this test is for. "threadsafe" re-execs a brand
    // new process instead, so the logger genuinely has not been initialized yet when the lambda
    // below runs, matching the comment just below.
    const std::string savedDeathTestStyle = ::testing::GTEST_FLAG(death_test_style);
    ::testing::GTEST_FLAG(death_test_style) = "threadsafe";
    struct RestoreDeathTestStyle {
        const std::string& saved;
        ~RestoreDeathTestStyle() { ::testing::GTEST_FLAG(death_test_style) = saved; }
    } restoreDeathTestStyle{savedDeathTestStyle};

    // atexit handlers run in reverse order of registration, so registering before the logger initializes
    // (on its first use in the child) makes this handler run after the logger's exit handler.
    EXPECT_EXIT(
        {
            fs::current_path(cwd);
            std::atexit(logFromLateExitHandler);
            if (IS_LOG_SET_OUTPUT_PATH(a.string().c_str(), 0) != 0)
                std::exit(2);
            log_info(IS_LOG_FACILITY_NONE, "logged-before-exit");
            std::exit(0);
        },
        ::testing::ExitedWithCode(0), "");

    EXPECT_EQ(1u, countContaining(a, "logged-before-exit"));
    EXPECT_EQ(1u, countContaining(a, "logged-during-exit"));
    EXPECT_FALSE(fs::exists(cwd / IS_LOG_DEFAULT_OUTPUT_PATH)) << "logger reopened the default file during exit";
}
#endif
