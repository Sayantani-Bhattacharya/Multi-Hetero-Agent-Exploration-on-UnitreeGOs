#include <sstream>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "unitree_kinematics/svg.hpp"

using Catch::Matchers::WithinAbs;

TEST_CASE( "Getting Width works for svg", "[getWidth()]") 
{
    SVG svg(816, 1056);

    REQUIRE_THAT( svg.getWidth(), WithinAbs(816,1.0e-6));
}