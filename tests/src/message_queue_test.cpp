#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <depthai/pipeline/MessageQueue.hpp>
#include <depthai/pipeline/datatype/ADatatype.hpp>
#include <thread>

using namespace dai;

TEST_CASE("MessageQueue - Basic operations", "[MessageQueue]") {
    MessageQueue queue(10);

    // Test send and get
    auto msg1 = std::make_shared<ADatatype>();
    queue.send(msg1);
    auto receivedMsg1 = queue.get();
    REQUIRE(receivedMsg1 == msg1);

    // Test tryGet
    auto msg2 = std::make_shared<ADatatype>();
    queue.send(msg2);
    auto receivedMsg2 = queue.tryGet();
    REQUIRE(receivedMsg2 == msg2);

    // Test empty tryGet
    auto nullMsg = queue.tryGet();
    REQUIRE(nullMsg == nullptr);
}

TEST_CASE("MessageQueue - Blocking behavior", "[MessageQueue]") {
    MessageQueue queue(1, true);

    auto msg1 = std::make_shared<ADatatype>();
    auto msg2 = std::make_shared<ADatatype>();

    // Test blocking send
    queue.send(msg1);
    std::thread sendThread([&]() { queue.send(msg2); });
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    REQUIRE(queue.get() == msg1);
    sendThread.join();
    REQUIRE(queue.get() == msg2);
}

TEST_CASE("MessageQueue - Non-blocking behavior", "[MessageQueue]") {
    MessageQueue queue(1, false);

    auto msg1 = std::make_shared<ADatatype>();
    auto msg2 = std::make_shared<ADatatype>();

    // Test non-blocking send
    queue.send(msg1);
    queue.send(msg2);
    REQUIRE(queue.get() == msg2);
}

TEST_CASE("MessageQueue - Multiple producers and consumers", "[MessageQueue]") {
    MessageQueue queue(100, true);
    constexpr int NUM_MESSAGES = 1000;
    constexpr int NUM_PRODUCERS = 10;
    constexpr int NUM_CONSUMERS = 10;
    constexpr int NUM_MESSAGES_PER_PRODUCER = NUM_MESSAGES / NUM_PRODUCERS;
    constexpr int NUM_MESSAGES_PER_CONSUMER = NUM_MESSAGES / NUM_CONSUMERS;

    std::vector<std::thread> producers;
    std::vector<std::thread> consumers;
    std::atomic<int> sentCount{0};
    std::atomic<int> receivedCount{0};

    // Producers
    producers.reserve(NUM_PRODUCERS);
    for(int i = 0; i < NUM_PRODUCERS; ++i) {
        producers.emplace_back([&]() {
            for(int j = 0; j < NUM_MESSAGES_PER_PRODUCER; ++j) {
                auto msg = std::make_shared<ADatatype>();
                queue.send(msg);
                sentCount++;
            }
        });
    }

    // Consumers
    consumers.reserve(NUM_CONSUMERS);
    for(int i = 0; i < NUM_CONSUMERS; ++i) {
        consumers.emplace_back([&]() {
            for(int j = 0; j < NUM_MESSAGES_PER_CONSUMER; ++j) {
                auto msg = queue.get();
                receivedCount++;
            }
        });
    }

    // Wait for producers and consumers to finish
    for(auto& producer : producers) {
        producer.join();
    }
    for(auto& consumer : consumers) {
        consumer.join();
    }

    REQUIRE(sentCount == NUM_MESSAGES);
    REQUIRE(receivedCount == NUM_MESSAGES);
}

TEST_CASE("MessageQueue - Timeout", "[MessageQueue]") {
    MessageQueue queue(10);

    // Test get with timeout
    bool timedOut = false;
    auto msg = queue.get<ADatatype>(std::chrono::milliseconds(100), timedOut);
    REQUIRE(msg == nullptr);
    REQUIRE(timedOut);

    // Test send with timeout
    auto largeMsg = std::make_shared<ADatatype>();
    bool sendTimedOut = !queue.send(largeMsg, std::chrono::milliseconds(100));
    REQUIRE_FALSE(sendTimedOut);
}

TEST_CASE("MessageQueue - Callbacks", "[MessageQueue]") {
    MessageQueue queue(10);
    std::atomic<int> callbackCount{0};

    // Test addCallback and removeCallback
    auto callbackId = queue.addCallback([&]() { callbackCount++; });
    auto msg = std::make_shared<ADatatype>();
    queue.send(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    REQUIRE(callbackCount == 1);

    queue.removeCallback(callbackId);
    queue.send(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    REQUIRE(callbackCount == 1);
}

TEST_CASE("MessageQueue - tryGetAll", "[MessageQueue]") {
    MessageQueue queue(10);

    auto msg1 = std::make_shared<ADatatype>();
    auto msg2 = std::make_shared<ADatatype>();
    auto msg3 = std::make_shared<ADatatype>();

    queue.send(msg1);
    queue.send(msg2);
    queue.send(msg3);

    auto messages = queue.tryGetAll();
    REQUIRE(messages.size() == 3);
    REQUIRE(messages[0] == msg1);
    REQUIRE(messages[1] == msg2);
    REQUIRE(messages[2] == msg3);
}

TEST_CASE("MessageQueue - getAll", "[MessageQueue]") {
    MessageQueue queue(10);

    auto msg1 = std::make_shared<ADatatype>();
    auto msg2 = std::make_shared<ADatatype>();
    auto msg3 = std::make_shared<ADatatype>();

    std::thread producer([&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        queue.send(msg1);
        queue.send(msg2);
        queue.send(msg3);
    });

    auto messages = queue.getAll();
    REQUIRE(messages.size() == 3);
    REQUIRE(messages[0] == msg1);
    REQUIRE(messages[1] == msg2);
    REQUIRE(messages[2] == msg3);

    producer.join();
}

TEST_CASE("MessageQueue - Close", "[MessageQueue]") {
    MessageQueue queue(10);

    // Test close
    queue.close();
    REQUIRE(queue.isClosed());
    REQUIRE_THROWS_AS(queue.get(), MessageQueue::QueueException);
}

TEST_CASE("MessageQueue - Name and properties", "[MessageQueue]") {
    std::string queueName = "TestQueue";
    MessageQueue queue(queueName, 10, false);

    // Test getName
    REQUIRE(queue.getName() == queueName);

    // Test getMaxSize
    REQUIRE(queue.getMaxSize() == 10);

    // Test getBlocking and setBlocking
    REQUIRE_FALSE(queue.getBlocking());
    queue.setBlocking(true);
    REQUIRE(queue.getBlocking());

    // Test setMaxSize
    queue.setMaxSize(20);
    REQUIRE(queue.getMaxSize() == 20);
}