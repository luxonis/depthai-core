#include <catch2/catch_all.hpp>

// Include depthai library
#include <depthai/depthai.hpp>

TEST_CASE("Pipeline link and remove") {
    dai::Pipeline p;
    auto x_in = p.create<dai::node::XLinkIn>();
    auto x_out = p.create<dai::node::XLinkOut>();
    x_in->out.link(x_out->input);
    p.remove(x_in);
}

TEST_CASE("Pipeline node creation, link, unlink and removal") {
    dai::Pipeline p;
    auto cam = p.create<dai::node::ColorCamera>();
    auto xlink = p.create<dai::node::XLinkOut>();

    REQUIRE(p.getConnections().size() == 0);
    REQUIRE(p.getAllNodes().size() == 2);

    cam->preview.link(xlink->input);

    REQUIRE(p.getConnections().size() == 1);

    p.remove(xlink);

    REQUIRE(p.getConnections().size() == 0);
    REQUIRE(p.getAllNodes().size() == 1);

    p.remove(cam);

    REQUIRE(p.getConnections().size() == 0);
    REQUIRE(p.getAllNodes().size() == 0);
}

TEST_CASE("Cross pipeline link with InputMap and OutputMap") {
    dai::Pipeline p1;
    dai::Pipeline p2;
    auto s1 = p1.create<dai::node::Script>();
    auto s2 = p2.create<dai::node::Script>();

    // First check if canConnect returns false
    REQUIRE_FALSE(s1->outputs["test"].canConnect(s2->inputs["input1"]));
    // Then check that actually linking throws
    REQUIRE_THROWS(s1->outputs["test"].link(s2->inputs["input1"]));
}

TEST_CASE("Cross pipeline link with Input and Output") {
    dai::Pipeline p1;
    dai::Pipeline p2;
    auto xout = p1.create<dai::node::XLinkOut>();
    auto xin = p2.create<dai::node::XLinkIn>();

    // First check if canConnect returns false
    REQUIRE_FALSE(xin->out.canConnect(xout->input));
    // Then check that actually linking throws
    REQUIRE_THROWS(xin->out.link(xout->input));
}
