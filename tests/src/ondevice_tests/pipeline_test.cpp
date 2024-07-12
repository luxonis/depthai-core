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
