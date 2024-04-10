#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <depthai/pipeline/MessageQueue.hpp>
#include <depthai/pipeline/datatype/ADatatype.hpp>
#include <thread>
#include <atomic>

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
    constexpr int NUM_MESSAGES = 100000;
    constexpr int NUM_PRODUCERS = 100;
    constexpr int NUM_CONSUMERS = 100;
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

    producer.join();
    auto messages = queue.getAll();
    REQUIRE(messages.size() == 3);
    REQUIRE(messages[0] == msg1);
    REQUIRE(messages[1] == msg2);
    REQUIRE(messages[2] == msg3);

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

TEST_CASE("MessageQueue - Changing maxSize at runtime", "[MessageQueue]") {
    MessageQueue queue(10);

    // Fill up the queue
    for (int i = 0; i < 10; ++i) {
        auto msg = std::make_shared<ADatatype>();
        queue.send(msg);
    }

    // Increase maxSize and check if we can send more messages
    queue.setMaxSize(15);
    for (int i = 0; i < 5; ++i) {
        auto msg = std::make_shared<ADatatype>();
        queue.send(msg);
    }
    REQUIRE(queue.getMaxSize() == 15);
    REQUIRE(queue.getSize() == 15);
    REQUIRE(queue.isFull());

    // Decrease maxSize and check if the queue is truncated
    queue.setMaxSize(5);
    REQUIRE(queue.getMaxSize() == 5);
    std::vector<std::shared_ptr<ADatatype>> messages;
    for(int i = 0; i < 5; ++i) {
        messages.push_back(queue.get<ADatatype>());
    }
    REQUIRE(queue.getSize() == 10);
    REQUIRE(queue.isFull());

    // Get all messages
    auto allMessages = queue.getAll();
    REQUIRE(allMessages.size() == 10);
    REQUIRE(queue.getSize() == 0);
    REQUIRE_FALSE(queue.isFull());

    // Check that 5 messages can be sent and received
    for (int i = 0; i < 3; ++i) {
        auto msg = std::make_shared<ADatatype>();
        queue.send(msg);
    }
    REQUIRE(queue.getSize() == 3);
    REQUIRE_FALSE(queue.isFull());
    for (int i = 0; i < 3; ++i) {
        auto msg = queue.get<ADatatype>();
    }
    REQUIRE(queue.getSize() == 0);
}

TEST_CASE("MessageQueue - Adding and removing callbacks at runtime", "[MessageQueue]") {
    MessageQueue queue(10);
    std::atomic<int> callbackCount1{0};
    std::atomic<int> callbackCount2{0};

    // Add first callback
    auto callbackId1 = queue.addCallback([&]() { callbackCount1++; });
    auto msg = std::make_shared<ADatatype>();
    queue.send(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    REQUIRE(callbackCount1 == 1);

    // Add second callback
    auto callbackId2 = queue.addCallback([&]() { callbackCount2++; });
    queue.send(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    REQUIRE(callbackCount1 == 2);
    REQUIRE(callbackCount2 == 1);

    // Remove first callback
    queue.removeCallback(callbackId1);
    queue.send(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    REQUIRE(callbackCount1 == 2);
    REQUIRE(callbackCount2 == 2);

    // Remove second callback
    queue.removeCallback(callbackId2);
    queue.send(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    REQUIRE(callbackCount1 == 2);
    REQUIRE(callbackCount2 == 2);
}

TEST_CASE("MessageQueue - Multi-threaded callbacks", "[MessageQueue]") {
    MessageQueue queue(10);
    std::atomic<int> callbackCount{0};
    std::atomic<int> numberOfCallbacks{0};
    constexpr int NUM_CALLBACKS = 10;

    // Add multiple callbacks from different threads
    std::vector<std::thread> callbackThreads;
    for(int i = 0; i < NUM_CALLBACKS; ++i) {
        callbackThreads.emplace_back([&]() {
            queue.addCallback([&]() { callbackCount++; });
        });
    }
    for (auto& thread : callbackThreads) {
        thread.join();
    }
    // Send messages and check if all callbacks are invoked
    auto msg = std::make_shared<ADatatype>();
    queue.send(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    REQUIRE(callbackCount == NUM_CALLBACKS);
}

TEST_CASE("MessageQueue - Callbacks with blocking queue", "[MessageQueue]") {
    constexpr int QUEUE_SIZE = 5;
    MessageQueue queue(QUEUE_SIZE, true);  // Create a blocking queue
    std::atomic<int> callbackCount{0};

    // Fill up the queue to its maximum capacity
    for (int i = 0; i < QUEUE_SIZE; ++i) {
        auto msg = std::make_shared<ADatatype>();
        queue.send(msg);
    }

    REQUIRE(queue.isFull());

    // Add callbacks
    constexpr int NUM_CALLBACKS = 3;
    for (int i = 0; i < NUM_CALLBACKS; ++i) {
        queue.addCallback([&]() { callbackCount++; });
    }

    // Send a message
    auto success = queue.trySend(std::make_shared<ADatatype>());
    // Check if all callbacks were called
    REQUIRE(callbackCount == NUM_CALLBACKS);
    REQUIRE_FALSE(success);

    // Consume messages from the queue
    std::thread consumeThread([&]() {
        for (int i = 0; i < QUEUE_SIZE; ++i) {
            auto msg = queue.get();
        }
    });

    consumeThread.join();
}

TEST_CASE("MessageQueue - Callbacks on an empty unblocking queue, then send a message", "[MessageQueue]") {
    MessageQueue queue(0, false);  // Create a non-blocking queue
    std::atomic<int> callbackCount{0};

    // Add a callback
    queue.addCallback([&]() { callbackCount++; });

    // Send a message
    auto msg = std::make_shared<ADatatype>();
    queue.send(msg);

    REQUIRE(queue.getSize() == 0);
    // Check if the callback was called
    REQUIRE(callbackCount == 1);
}

TEST_CASE("Sending to a closed queue", "[MessageQueue]") {
    MessageQueue queue(10);
    std::atomic<int> callbackCount{0};

    // Add a callback
    queue.addCallback([&]() { callbackCount++; });

    // Close the queue
    queue.close();

    // Send a message
    auto msg = std::make_shared<ADatatype>();
    REQUIRE_THROWS_AS(queue.send(msg), MessageQueue::QueueException);
}

