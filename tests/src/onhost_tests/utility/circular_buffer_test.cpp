#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_vector.hpp>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include "depthai/utility/CircularBuffer.hpp"

using namespace dai::utility;

template <typename T>
static std::vector<T> collect_forward(CircularBuffer<T>& cb) {
    std::vector<T> out;
    for(auto& x : cb) out.push_back(x);
    return out;
}

template <typename T>
static std::vector<T> collect_reverse(CircularBuffer<T>& cb) {
    std::vector<T> out;
    for(auto it = cb.rbegin(); it != cb.rend(); ++it) out.push_back(*it);
    return out;
}

TEST_CASE("CircularBuffer: empty buffer behavior") {
    CircularBuffer<int> cb(3);

    REQUIRE(cb.size() == 0);

    REQUIRE_THROWS_AS(cb.first(), std::runtime_error);
    REQUIRE_THROWS_AS(cb.last(), std::runtime_error);

    REQUIRE(cb.getBuffer().empty());
    REQUIRE(collect_forward(cb).empty());
    REQUIRE(collect_reverse(cb).empty());
}

TEST_CASE("CircularBuffer: fill without wrap (size < maxSize)") {
    CircularBuffer<int> cb(5);

    cb.add(10);
    cb.add(20);
    cb.add(30);

    REQUIRE(cb.size() == 3);
    REQUIRE(cb.first() == 10);
    REQUIRE(cb.last() == 30);

    REQUIRE_THAT(cb.getBuffer(), Catch::Matchers::Equals(std::vector<int>{10, 20, 30}));
    REQUIRE_THAT(collect_forward(cb), Catch::Matchers::Equals(std::vector<int>{10, 20, 30}));
    REQUIRE_THAT(collect_reverse(cb), Catch::Matchers::Equals(std::vector<int>{30, 20, 10}));
}

TEST_CASE("CircularBuffer: fill exactly to capacity (still no overwrite yet)") {
    CircularBuffer<int> cb(3);

    cb.add(1);
    cb.add(2);
    cb.add(3);

    REQUIRE(cb.size() == 3);
    REQUIRE(cb.first() == 1);
    REQUIRE(cb.last() == 3);

    REQUIRE_THAT(cb.getBuffer(), Catch::Matchers::Equals(std::vector<int>{1, 2, 3}));
    REQUIRE_THAT(collect_forward(cb), Catch::Matchers::Equals(std::vector<int>{1, 2, 3}));
    REQUIRE_THAT(collect_reverse(cb), Catch::Matchers::Equals(std::vector<int>{3, 2, 1}));
}

TEST_CASE("CircularBuffer: single overwrite rotates order correctly") {
    CircularBuffer<int> cb(3);

    cb.add(1);
    cb.add(2);
    cb.add(3);
    cb.add(4);  // overwrites oldest (1)

    REQUIRE(cb.size() == 3);
    REQUIRE(cb.first() == 2);
    REQUIRE(cb.last() == 4);

    REQUIRE_THAT(cb.getBuffer(), Catch::Matchers::Equals(std::vector<int>{2, 3, 4}));
    REQUIRE_THAT(collect_forward(cb), Catch::Matchers::Equals(std::vector<int>{2, 3, 4}));
    REQUIRE_THAT(collect_reverse(cb), Catch::Matchers::Equals(std::vector<int>{4, 3, 2}));
}

TEST_CASE("CircularBuffer: multiple overwrites and wrap-around") {
    CircularBuffer<int> cb(4);

    // Add 1..10
    for(int i = 1; i <= 10; ++i) cb.add(i);

    // Should contain last 4 elements: 7,8,9,10
    REQUIRE(cb.size() == 4);
    REQUIRE(cb.first() == 7);
    REQUIRE(cb.last() == 10);

    REQUIRE_THAT(cb.getBuffer(), Catch::Matchers::Equals(std::vector<int>{7, 8, 9, 10}));
    REQUIRE_THAT(collect_forward(cb), Catch::Matchers::Equals(std::vector<int>{7, 8, 9, 10}));
    REQUIRE_THAT(collect_reverse(cb), Catch::Matchers::Equals(std::vector<int>{10, 9, 8, 7}));
}

TEST_CASE("CircularBuffer: add() returns reference to inserted/overwritten slot") {
    SECTION("When not full, add() returns reference to back()") {
        CircularBuffer<std::string> cb(3);
        auto& r1 = cb.add(std::string("a"));
        auto& r2 = cb.add(std::string("b"));

        REQUIRE(r1 == "a");
        REQUIRE(r2 == "b");

        // Mutate through returned reference and confirm buffer reflects it
        r2 = "B";
        REQUIRE(cb.last() == "B");
        REQUIRE_THAT(cb.getBuffer(), Catch::Matchers::Equals(std::vector<std::string>{"a", "B"}));
    }

    SECTION("When full, add() returns reference to the newest element (just written)") {
        CircularBuffer<int> cb(2);
        cb.add(10);
        cb.add(20);

        // Now overwriting begins
        auto& r = cb.add(30);  // overwrites 10
        REQUIRE(r == 30);
        r = 300;

        REQUIRE(cb.first() == 20);
        REQUIRE(cb.last() == 300);
        REQUIRE_THAT(cb.getBuffer(), Catch::Matchers::Equals(std::vector<int>{20, 300}));
    }
}

TEST_CASE("CircularBuffer: iterator traverses logical order and can mutate elements") {
    CircularBuffer<int> cb(3);
    cb.add(1);
    cb.add(2);
    cb.add(3);
    cb.add(4);  // buffer is logically [2,3,4]

    REQUIRE_THAT(collect_forward(cb), Catch::Matchers::Equals(std::vector<int>{2, 3, 4}));

    // Mutate via iterator
    for(auto& x : cb) x *= 10;

    REQUIRE_THAT(cb.getBuffer(), Catch::Matchers::Equals(std::vector<int>{20, 30, 40}));
    REQUIRE(cb.first() == 20);
    REQUIRE(cb.last() == 40);
}

TEST_CASE("CircularBuffer: reverse_iterator traverses from newest to oldest and can mutate") {
    CircularBuffer<int> cb(3);
    cb.add(1);
    cb.add(2);
    cb.add(3);
    cb.add(4);  // logical [2,3,4]

    REQUIRE_THAT(collect_reverse(cb), Catch::Matchers::Equals(std::vector<int>{4, 3, 2}));

    // Mutate via reverse iteration (e.g., add 1 to each)
    for(auto it = cb.rbegin(); it != cb.rend(); ++it) {
        *it += 1;
    }

    // Original logical [2,3,4] -> [3,4,5]
    REQUIRE_THAT(cb.getBuffer(), Catch::Matchers::Equals(std::vector<int>{3, 4, 5}));
    REQUIRE(cb.first() == 3);
    REQUIRE(cb.last() == 5);
}

TEST_CASE("CircularBuffer: getBuffer() matches iteration for a variety of sequences") {
    CircularBuffer<int> cb(5);

    // Add a bunch, checking at each step
    for(int i = 0; i < 25; ++i) {
        cb.add(i);

        auto gb = cb.getBuffer();
        auto it = collect_forward(cb);

        REQUIRE(gb == it);

        // Reverse should be forward reversed
        auto rev = collect_reverse(cb);
        std::vector<int> gb_rev = gb;
        std::reverse(gb_rev.begin(), gb_rev.end());
        REQUIRE(rev == gb_rev);

        // first/last consistent with getBuffer
        if(!gb.empty()) {
            REQUIRE(cb.first() == gb.front());
            REQUIRE(cb.last() == gb.back());
        }
    }
}

TEST_CASE("CircularBuffer: maxSize=1 edge case") {
    CircularBuffer<int> cb(1);

    cb.add(5);
    REQUIRE(cb.size() == 1);
    REQUIRE(cb.first() == 5);
    REQUIRE(cb.last() == 5);
    REQUIRE_THAT(cb.getBuffer(), Catch::Matchers::Equals(std::vector<int>{5}));

    cb.add(6);
    REQUIRE(cb.size() == 1);
    REQUIRE(cb.first() == 6);
    REQUIRE(cb.last() == 6);
    REQUIRE_THAT(cb.getBuffer(), Catch::Matchers::Equals(std::vector<int>{6}));

    REQUIRE_THAT(collect_forward(cb), Catch::Matchers::Equals(std::vector<int>{6}));
    REQUIRE_THAT(collect_reverse(cb), Catch::Matchers::Equals(std::vector<int>{6}));
}

TEST_CASE("CircularBuffer: sanity check with larger dataset") {
    constexpr size_t N = 50;
    constexpr size_t CAP = 7;

    CircularBuffer<int> cb(CAP);
    for(size_t i = 0; i < N; ++i) cb.add(static_cast<int>(i));

    REQUIRE(cb.size() == CAP);

    // Expected last CAP items: N-CAP .. N-1
    std::vector<int> expected(CAP);
    std::iota(expected.begin(), expected.end(), static_cast<int>(N - CAP));

    REQUIRE_THAT(cb.getBuffer(), Catch::Matchers::Equals(expected));
    REQUIRE_THAT(collect_forward(cb), Catch::Matchers::Equals(expected));

    auto rev = collect_reverse(cb);
    std::reverse(expected.begin(), expected.end());
    REQUIRE_THAT(rev, Catch::Matchers::Equals(expected));
}
