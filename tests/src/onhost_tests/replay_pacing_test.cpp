#include <catch2/catch_all.hpp>
#include <chrono>
#include <limits>
#include <vector>

#include "utility/ReplayPacing.hpp"

using namespace std::chrono_literals;

namespace {

using Pacer = dai::replay::PlaybackPacer;

struct SimulatedClock {
    Pacer::Time current{};
    bool running = true;
    Pacer::Duration oversleep{};
    bool stopAfterSleep = false;
    std::vector<Pacer::Duration> sleeps;

    std::optional<Pacer::Time> send(Pacer& pacer, std::optional<Pacer::Duration> recorded = std::nullopt) {
        return pacer.waitToSend(
            recorded,
            [this] { return running; },
            [this] { return current; },
            [this](Pacer::Duration delay) {
                sleeps.push_back(delay);
                current += delay + oversleep;
                oversleep = Pacer::Duration::zero();
                if(stopAfterSleep) running = false;
            });
    }
};

long long millis(Pacer::Time time) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count();
}

}  // namespace

TEST_CASE("A blocked send does not move the grid", "[pacing]") {
    Pacer pacer(10);
    SimulatedClock clock;
    std::vector<long long> starts;
    for(int frame = 0; frame < 10; ++frame) {
        const auto time = clock.send(pacer);
        REQUIRE(time);
        starts.push_back(millis(*time));
        // The send of frame 5 waits for the consumer to take frame 4 at 650 ms.
        // That acceptance does not move the origin. Frame 6 was due at 600 ms.
        if(frame == 5) clock.current = Pacer::Time(650ms);
        pacer.sent(clock.current);
    }
    REQUIRE(starts == std::vector<long long>{0, 100, 200, 300, 400, 500, 650, 700, 800, 900});
}

TEST_CASE("A late decode sends the backlog immediately and then returns to the grid", "[pacing]") {
    Pacer pacer(10);
    SimulatedClock clock;
    REQUIRE(millis(*clock.send(pacer)) == 0);
    pacer.sent(clock.current);
    // Decoding the next frame passes the 100 ms and 200 ms due times.
    clock.current = Pacer::Time(250ms);
    REQUIRE(millis(*clock.send(pacer)) == 250);
    pacer.sent(clock.current);
    REQUIRE(millis(*clock.send(pacer)) == 250);
    pacer.sent(clock.current);
    REQUIRE(millis(*clock.send(pacer)) == 300);
    pacer.sent(clock.current);
    REQUIRE(millis(*clock.send(pacer)) == 400);
    pacer.sent(clock.current);
    REQUIRE(millis(*clock.send(pacer)) == 500);
}

TEST_CASE("A recorded overrun sends immediately and the next gap stays on the original sum", "[pacing]") {
    Pacer pacer;
    SimulatedClock clock;
    REQUIRE(millis(*clock.send(pacer)) == 0);
    pacer.sent(clock.current);
    clock.current = Pacer::Time(50ms);
    REQUIRE(millis(*clock.send(pacer, 40ms)) == 50);
    pacer.sent(clock.current);
    REQUIRE(millis(*clock.send(pacer, 100ms)) == 140);
}

TEST_CASE("A late wake does not stretch the following interval", "[pacing]") {
    Pacer pacer(10);
    SimulatedClock clock;
    clock.send(pacer);
    pacer.sent(clock.current);
    clock.oversleep = 100ms;
    REQUIRE(millis(*clock.send(pacer)) == 150);
    pacer.sent(clock.current);
    REQUIRE(millis(*clock.send(pacer)) == 200);
}

TEST_CASE("A frame one nanosecond late is sent immediately and the next stays on the grid", "[pacing]") {
    Pacer pacer(10);
    SimulatedClock clock;
    REQUIRE(clock.send(pacer) == Pacer::Time{});
    pacer.sent(clock.current);
    clock.current = Pacer::Time(100ms + 1ns);
    REQUIRE(clock.send(pacer) == Pacer::Time(100ms + 1ns));
    REQUIRE(clock.sleeps.empty());
    pacer.sent(clock.current);
    REQUIRE(clock.send(pacer) == Pacer::Time(200ms));
}

TEST_CASE("A twenty percent late sleep keeps one frame in each interval", "[pacing]") {
    Pacer pacer(250);
    const auto interval = dai::replay::playbackPeriod(250);
    auto current = Pacer::Time{};
    const auto send = [&] {
        return pacer.waitToSend(std::nullopt, [] { return true; }, [&] { return current; }, [&](Pacer::Duration delay) { current += delay + delay / 5; });
    };
    REQUIRE(send() == Pacer::Time{});
    pacer.sent(current);
    for(int frame = 1; frame < 30; ++frame) {
        const auto time = send();
        REQUIRE(time);
        const auto elapsed = *time - Pacer::Time{};
        REQUIRE(elapsed >= interval * frame);
        REQUIRE(elapsed < interval * (frame + 1));
        pacer.sent(current);
    }
}

TEST_CASE("PC-2 preserves long gaps, equal timestamp fallback, and pass intervals", "[pacing]") {
    Pacer pacer;
    SimulatedClock clock;
    clock.send(pacer);
    pacer.sent(clock.current);
    REQUIRE(millis(*clock.send(pacer, 80ms)) == 80);
    pacer.sent(clock.current);
    const auto equalTimestamp = clock.send(pacer, 0ms);
    REQUIRE(equalTimestamp);
    REQUIRE(millis(*equalTimestamp) == 113);
    pacer.sent(clock.current);
    const auto afterGap = clock.send(pacer, 2s);
    REQUIRE(afterGap);
    REQUIRE(*afterGap - *equalTimestamp == 2s);
    pacer.sent(clock.current);
    REQUIRE(*clock.send(pacer, 150ms) - *afterGap == 150ms);
}

TEST_CASE("A positive fps including 1000 takes priority over metadata", "[pacing]") {
    const float fps = GENERATE(20.f, 1000.f);
    Pacer pacer(fps);
    SimulatedClock clock;
    clock.send(pacer);
    pacer.sent(clock.current);
    REQUIRE(*clock.send(pacer, 10s) == Pacer::Time(fps == 1000.f ? 1ms : 50ms));
}

TEST_CASE("PC-3 uses 30 fps when neither rate nor metadata is set", "[pacing]") {
    Pacer pacer(0);
    SimulatedClock clock;
    clock.send(pacer);
    pacer.sent(clock.current);
    const auto first = clock.send(pacer);
    REQUIRE(first);
    REQUIRE(first->time_since_epoch() == std::chrono::duration_cast<Pacer::Duration>(1s) / 30);
    pacer.sent(clock.current);
    REQUIRE(*clock.send(pacer) - *first == std::chrono::duration_cast<Pacer::Duration>(1s) / 30);
}

TEST_CASE("Long pacing waits stop within one bounded sleep", "[pacing]") {
    Pacer pacer;
    SimulatedClock clock;
    clock.send(pacer);
    pacer.sent(clock.current);
    clock.stopAfterSleep = true;
    REQUIRE_FALSE(clock.send(pacer, 60s));
    REQUIRE(clock.sleeps.size() == 1);
    REQUIRE(clock.sleeps.front() <= 50ms);
}

TEST_CASE("Representable pacing handles extreme positive fps without overflow", "[pacing]") {
    REQUIRE(dai::replay::playbackPeriod(std::numeric_limits<double>::min()) == Pacer::Duration::max());
    REQUIRE(dai::replay::playbackPeriod(std::numeric_limits<double>::max()) == Pacer::Duration(1));
    Pacer pacer(std::numeric_limits<float>::min());
    SimulatedClock clock;
    clock.current = Pacer::Time(1s);
    clock.send(pacer);
    pacer.sent(clock.current);
    clock.stopAfterSleep = true;
    REQUIRE_FALSE(clock.send(pacer));
    REQUIRE(clock.sleeps.front() <= 50ms);
}
