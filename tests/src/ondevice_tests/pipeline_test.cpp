#include <catch2/catch_all.hpp>

// Include depthai library
#include <depthai/depthai.hpp>

// internal nodes
#include <depthai/pipeline/node/internal/XLinkIn.hpp>
#include <depthai/pipeline/node/internal/XLinkOut.hpp>

TEST_CASE("Pipeline link and remove") {
    dai::Pipeline p;
    auto x_in = p.create<dai::node::internal::XLinkIn>();
    auto x_out = p.create<dai::node::internal::XLinkOut>();
    x_in->out.link(x_out->input);
    p.remove(x_in);
}

TEST_CASE("Pipeline node creation, link, unlink and removal") {
    dai::Pipeline p;
    auto cam = p.create<dai::node::ImageManip>();
    auto xlink = p.create<dai::node::internal::XLinkOut>();

    REQUIRE(p.getConnections().size() == 0);
    REQUIRE(p.getAllNodes().size() == 2);

    cam->out.link(xlink->input);

    REQUIRE(p.getConnections().size() == 1);

    p.remove(xlink);

    REQUIRE(p.getConnections().size() == 0);
    REQUIRE(p.getAllNodes().size() == 1);

    p.remove(cam);

    REQUIRE(p.getConnections().size() == 0);
    REQUIRE(p.getAllNodes().size() == 0);
}
