#include "flarm_snapshot.hh"
#include "gtest/gtest.h"
#include <atomic>
#include <thread>

TEST(FlarmSnapshotConcurrency, PublishesCoherentSnapshotsWhileConsumerProcessesPreviousCopy) {
    FlarmSnapshotReceiver receiver;
    std::atomic<bool> done{false}, failed{false};
    std::thread producer([&] {
        for (uint32_t generation = 1; generation <= 10000; ++generation) {
            FlarmSnapshot snapshot;
            snapshot.generation = generation;
            snapshot.count = FlarmSnapshot::kCapacity;
            for (uint32_t i = 0; i < snapshot.count; ++i) {
                auto& target = snapshot.targets[i];
                target.id = i;
                target.version = 3;
                target.fix_secs = generation;
            }
            const auto* bytes = reinterpret_cast<const uint8_t*>(&snapshot);
            if (!receiver.Write(bytes, 2040, 0, generation) ||
                !receiver.Write(bytes + 2040, sizeof(snapshot) - 2040, 2040, generation)) failed = true;
            // An ACK retry must not refresh the original reception timestamp.
            if (!receiver.Write(bytes + 2040, sizeof(snapshot) - 2040, 2040, generation + 1)) failed = true;
        }
        done = true;
    });
    FlarmSnapshot copy;
    uint32_t received_ms = 0;
    do {
        if (!receiver.CopySnapshot(copy, received_ms)) continue;
        EXPECT_TRUE(copy.IsValid());
        EXPECT_EQ(received_ms, copy.generation);
        std::this_thread::yield();
        for (size_t i = 0; i < copy.count; ++i) EXPECT_EQ(copy.targets[i].fix_secs, copy.generation);
    } while (!done);
    producer.join();
    receiver.CopySnapshot(copy, received_ms);
    EXPECT_FALSE(failed);
    EXPECT_EQ(copy.generation, 10000u);
    EXPECT_EQ(received_ms, 10000u);
}

TEST(FlarmSnapshotConcurrency, PartialOrInvalidTransferPreservesPublishedCopy) {
    FlarmSnapshotReceiver receiver;
    FlarmSnapshot snapshot, copy;
    snapshot.generation = 1;
    auto* bytes = reinterpret_cast<uint8_t*>(&snapshot);
    uint32_t received_ms = 0;
    ASSERT_TRUE(receiver.Write(bytes, 2040, 0, 100));
    EXPECT_FALSE(receiver.CopySnapshot(copy, received_ms));
    ASSERT_TRUE(receiver.Write(bytes + 2040, sizeof(snapshot) - 2040, 2040, 101));
    ASSERT_TRUE(receiver.CopySnapshot(copy, received_ms));
    EXPECT_EQ(copy.generation, 1u);
    EXPECT_EQ(received_ms, 100u);
    snapshot.generation = 2;
    snapshot.count = 65;
    ASSERT_TRUE(receiver.Write(bytes, 2040, 0, 200));
    EXPECT_FALSE(receiver.Write(bytes + 2040, sizeof(snapshot) - 2040, 2040, 201));
    EXPECT_FALSE(receiver.CopySnapshot(copy, received_ms));
    EXPECT_EQ(copy.generation, 1u);
    EXPECT_EQ(received_ms, 100u);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
