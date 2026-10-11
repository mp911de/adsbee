#include "../application/comms/at/at_console_input.hh"
#include "gtest/gtest.h"
#include "cpp_at.hh"
#include <string>
#include <vector>

namespace {
using Input = ATConsoleInput<32>;
std::vector<std::string> Feed(Input& input, std::string_view bytes) {
    std::vector<std::string> lines;
    for (char c : bytes) if (input.Push(c, 0)) lines.emplace_back(input.line());
    return lines;
}
}

TEST(ATConsoleInput, AcceptsLFAndCRLF) {
    Input input;
    auto lines = Feed(input, "AT+UPTIME?\nAT+ESP32_ENABLE=0\r\n");
    ASSERT_EQ(lines.size(), 2u);
    EXPECT_EQ(lines[0], "AT+UPTIME?\n");
    EXPECT_EQ(lines[1], "AT+ESP32_ENABLE=0\r\n");
}

TEST(ATConsoleInput, ErrorsCannotTurnASuffixIntoACommand) {
    const std::string command = "AT+ESP32_ENABLE=0\r\n";
    for (size_t pos = 0; pos < command.size(); ++pos) {
        Input input;
        for (size_t i = 0; i < command.size(); ++i)
            EXPECT_FALSE(input.Push(command[i], 0, i == pos));
        // An errored newline is untrusted. A clean delimiter restores alignment.
        EXPECT_TRUE(Feed(input, "\n").size() <= 1u);
        EXPECT_EQ(Feed(input, "AT+UPTIME?\n").size(), 1u);
    }
}

TEST(ATConsoleInput, OverflowDiscardsCommandLookingSuffix) {
    Input input;
    EXPECT_TRUE(Feed(input, std::string(40, 'x') + "AT+ESP32_ENABLE=0\n").empty());
    EXPECT_EQ(Feed(input, "AT+UPTIME?\n").size(), 1u);
}

TEST(ATConsoleInput, OverflowOnDelimiterRecoversAtNextLine) {
    Input input;
    EXPECT_TRUE(Feed(input, std::string(32, 'x') + "\n").empty());
    EXPECT_EQ(Feed(input, "AT+UPTIME?\n").size(), 1u);
}

TEST(ATConsoleInput, BinaryLineIsRejected) {
    Input input;
    EXPECT_TRUE(Feed(input, std::string("AT+REBOOT\0\n", 11)).empty());
    EXPECT_EQ(Feed(input, "AT+UPTIME?\n").size(), 1u);
}

TEST(ATConsoleInput, SourcesDoNotCombineCommands) {
    Input usb, network, uart;
    EXPECT_TRUE(Feed(usb, "AT+ESP32_").empty());
    EXPECT_TRUE(Feed(network, "AT+UP").empty());
    EXPECT_EQ(Feed(uart, "AT+UPTIME?\n").size(), 1u);
    EXPECT_EQ(Feed(usb, "ENABLE=0\n")[0], "AT+ESP32_ENABLE=0\n");
    EXPECT_EQ(Feed(network, "TIME?\n")[0], "AT+UPTIME?\n");
}

TEST(ATConsoleInput, FailedUploadQuarantineBelongsToItsSource) {
    Input usb, uart;
    uart.guard.BeginBinaryPayload(10);
    uart.guard.EndBinaryPayload(0, 0);
    EXPECT_EQ(Feed(usb, "AT+UPTIME?\n").size(), 1u);
    EXPECT_TRUE(Feed(uart, "AT+REBOOT\n").empty());
    EXPECT_EQ(uart.guard.DiscardRemainingBytes(), 0u);
    EXPECT_EQ(Feed(uart, "AT+UPTIME?\n").size(), 1u);
}

TEST(ATConsoleInput, CorruptedStateChangeNeverReachesParser) {
    bool enabled = true;
    int calls = 0;
    const CppAT::ATCommandDef_t commands[] = {{
        .command = "ENABLE", .min_args = 1, .max_args = 1,
        .callback = [&](const auto&, char, const std::string_view args[], uint16_t) {
            ++calls;
            return CppAT::ArgToNum(args[0], enabled);
        }
    }};
    CppAT parser(commands, 1, true);
    Input input;
    const std::string command = "AT+ENABLE=0\n";
    for (size_t i = 0; i < command.size(); ++i) {
        if (input.Push(command[i], 0, i == 10)) parser.ParseMessage(input.line());
    }
    EXPECT_TRUE(enabled);
    EXPECT_EQ(calls, 0);
    for (const auto& line : Feed(input, command)) parser.ParseMessage(line);
    EXPECT_FALSE(enabled);
    EXPECT_EQ(calls, 1);
}
