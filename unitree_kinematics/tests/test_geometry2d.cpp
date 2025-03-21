#include <sstream>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "unitree_kinematics/geometry2d.hpp"

using unitree_kinematics::normalize_angle;
using unitree_kinematics::normalizeVector;
using unitree_kinematics::Point2D;
using unitree_kinematics::Vector2D;
using unitree_kinematics::PI;
using Catch::Matchers::WithinAbs;

TEST_CASE( "Angles are normalized", "[normalize_angle]" ) 
{
    // Typecast case
    REQUIRE_THAT( normalize_angle(1), WithinAbs(1.0,1.0e-6));
    // Overflow case
    REQUIRE_THAT(normalize_angle(PI+1), WithinAbs(1.0-PI,1.0e-6));
    // Large Overflow case
    REQUIRE_THAT(normalize_angle(400*PI+1), WithinAbs(1.0,1.0e-6));
    // Upper limit case (included)
    REQUIRE_THAT(normalize_angle(PI), WithinAbs(PI,1.0e-6));
    // Lower limit case (not included)
    REQUIRE_THAT(normalize_angle(-PI), WithinAbs(PI,1.0e-6));
    // Zero case
    REQUIRE_THAT(normalize_angle(0), WithinAbs(0,1.0e-6));
    // Simple case with pi
    REQUIRE_THAT(normalize_angle(-PI/4.0), WithinAbs(-PI/4.0,1.0e-6));
    // Overflow case with pi
    REQUIRE_THAT(normalize_angle(3.0*PI/2.0), WithinAbs(-PI/2.0,1.0e-6));
    // Underflow case with pi
    REQUIRE_THAT(normalize_angle(-5.0*PI/2.0), WithinAbs(-PI/2.0,1.0e-6));
}

TEST_CASE( "Relative vector construction through head and tail points works", "[operator-]") 
{
    // Check x.
    REQUIRE_THAT(  (Point2D{-2.6, 0.0} - Point2D{4.0, 0.0}).x, WithinAbs(-6.6,1.0e-6));
    // Check y.
    REQUIRE_THAT(  (Point2D{0.0, 3.0} - Point2D{0.0, 5.3}).y, WithinAbs(-2.3,1.0e-6));
}

TEST_CASE( "Point displacement through relative vector works", "[operator+]") 
{
    // Check x.
    REQUIRE_THAT(  (Point2D{-2.6, 0.0} + Vector2D{1.0, -1.0}).x, WithinAbs(-1.6,1.0e-6));
    // Check y.
    REQUIRE_THAT(  (Point2D{0.0, 3.0} + Vector2D{1.0, -1.0}).y, WithinAbs(2.0,1.0e-6));
}

TEST_CASE( "Stream insertion works for 2d point", "[operator<<]") 
{
    Point2D point{1.2, 3.4};
    std::string str = "[1.2 3.4]";

    std::stringstream sstr;
    sstr << point;
    REQUIRE(sstr.str() == str);
}

TEST_CASE( "Stream extraction works for 2d point", "[operator>>]") // Adapted from Abhishek Sankar
{
    Point2D point_1{}, point_2{};
    std::stringstream sstr_1, sstr_2;

    sstr_1 << "0.7 4.3";
    sstr_1 >> point_1;

    sstr_2 << "[9.5 1.32]";
    sstr_2 >> point_2;

    REQUIRE( point_1.x == 0.7 );
    REQUIRE( point_1.y == 4.3 );
    REQUIRE( point_2.x == 9.5 );
    REQUIRE( point_2.y == 1.32 );
}

TEST_CASE( "Stream insertion operator << works for 2d vector", "[operator<<]") 
{
    Vector2D vec{1.2, 3.4};
    std::string str = "[1.2 3.4]";

    std::stringstream sstr;
    sstr << vec;
    REQUIRE(sstr.str() == str);
}

TEST_CASE( "Stream extraction works for 2d vector", "[operator>>]") // Adapted from Abhishek Sankar
{
    Vector2D vec_1{}, vec_2{};
    std::stringstream sstr_1, sstr_2;

    sstr_1 << "0.7 4.3";
    sstr_1 >> vec_1;

    sstr_2 << "[9.5 1.32]";
    sstr_2 >> vec_2;

    REQUIRE( vec_1.x == 0.7 );
    REQUIRE( vec_1.y == 4.3 );
    REQUIRE( vec_2.x == 9.5 );
    REQUIRE( vec_2.y == 1.32 );
}

TEST_CASE( "Vector normalization works", "[normalizeVector]") 
{
    Vector2D v{-69, 4.20};

    Vector2D v_hat = normalizeVector(v);

    // Check x.
    REQUIRE_THAT(v_hat.x, WithinAbs(-0.998153,1.0e-6));
    // Check y.
    REQUIRE_THAT(v_hat.y, WithinAbs(0.0607571,1.0e-6));
}

TEST_CASE( "Vector subtraction works", "[operator-]") 
{
    Vector2D va{-69, 4.20}, vb{-4.20, 6.9}, vab{0,0};

    vab = va - vb;

    // Check x.
    REQUIRE_THAT(vab.x, WithinAbs(-64.8,1.0e-6));
    // Check y.
    REQUIRE_THAT(vab.y, WithinAbs(-2.7,1.0e-6));
}

TEST_CASE( "Vector addition works", "[operator+]") 
{
    Vector2D va{-69, 4.20}, vb{-4.20, 6.9}, vab{0,0};

    vab = va + vb;

    // Check x.
    REQUIRE_THAT(vab.x, WithinAbs(-73.2,1.0e-6));
    // Check y.
    REQUIRE_THAT(vab.y, WithinAbs(11.1,1.0e-6));
}

TEST_CASE( "Vector scaling works", "[operator*]") 
{
    double scale = -69.0;
    Vector2D v{-69, 4.20}, vs1{0,0}, vs2{0,0};

    vs1 = scale * v;
    vs2 = v * scale;

    // Check x.
    REQUIRE_THAT(vs1.x, WithinAbs(4761,1.0e-6));
    // Check y.
    REQUIRE_THAT(vs1.y, WithinAbs(-289.8,1.0e-6));

    // Check x.
    REQUIRE_THAT(vs2.x, WithinAbs(4761,1.0e-6));
    // Check y.
    REQUIRE_THAT(vs2.y, WithinAbs(-289.8,1.0e-6));
}

TEST_CASE( "Composed vector subtraction works", "[operator-=]") 
{
    Vector2D va{-69, 4.20}, vb{-4.20, 6.9};

    va -= vb;

    // Check x.
    REQUIRE_THAT(va.x, WithinAbs(-64.8,1.0e-6));
    // Check y.
    REQUIRE_THAT(va.y, WithinAbs(-2.7,1.0e-6));
}

TEST_CASE( "Composed vector addition works", "[operator+=]") 
{
    Vector2D va{-69, 4.20}, vb{-4.20, 6.9};

    va += vb;

    // Check x.
    REQUIRE_THAT(va.x, WithinAbs(-73.2,1.0e-6));
    // Check y.
    REQUIRE_THAT(va.y, WithinAbs(11.1,1.0e-6));
}

TEST_CASE( "Composed vector scaling works", "[operator*=]") 
{
    double scale = -69.0;
    Vector2D v{-69, 4.20};

    v *= scale;

    // Check x.
    REQUIRE_THAT(v.x, WithinAbs(4761,1.0e-6));
    // Check y.
    REQUIRE_THAT(v.y, WithinAbs(-289.8,1.0e-6));
}

TEST_CASE( "Dot product works", "[dot()]") 
{
    Vector2D va{-69, 4.20}, vb{-4.20, 6.9};

    double prod = dot(va, vb);

    // Check scalar.
    REQUIRE_THAT(prod, WithinAbs(318.78,1.0e-6));
}

TEST_CASE( "Magnitude works", "[magnitude()]") 
{
    Vector2D v{-69, 4.20};

    double mag = magnitude(v);

    // Check scalar.
    REQUIRE_THAT(mag, WithinAbs(69.1277079,1.0e-6));
}

TEST_CASE( "Angle works", "[magnitude()]") 
{
    Vector2D v1{-69, -4.20}, v2{69, 420}, v3{-0.0,0}, v4{-1,-0.0}, v5{-1, -1.0e-5};
    Vector2D v01{0.0,0.0}, v02{0.0,-0.0}, v03{-0.0,-0.0}, v04{-0.0,0.0};

    // Check angles in one direction
    REQUIRE_THAT(angle(v1,v2), WithinAbs(1.794422067130102,1.0e-6));
    REQUIRE_THAT(angle(v1,v3), WithinAbs(-3.080798097715004,1.0e-6));
    REQUIRE_THAT(angle(v1,v4), WithinAbs(0.060794555874789,1.0e-6));
    REQUIRE_THAT(angle(v1,v5), WithinAbs(0.060784555874789,1.0e-6));
    REQUIRE_THAT(angle(v2,v3), WithinAbs(1.407965142334479,1.0e-6));
    REQUIRE_THAT(angle(v2,v4), WithinAbs(-1.733627511255314,1.0e-6));
    REQUIRE_THAT(angle(v2,v5), WithinAbs(-1.733637511255314,1.0e-6));
    REQUIRE_THAT(angle(v3,v4), WithinAbs(PI,1.0e-6));
    REQUIRE_THAT(angle(v3,v5), WithinAbs(3.141582653589793,1.0e-6));
    REQUIRE_THAT(angle(v4,v5), WithinAbs(-9.999999999621423e-06,1.0e-6));

    // Check angles in the opposite direction
    REQUIRE_THAT(angle(v5,v4), WithinAbs(9.999999999621423e-06,1.0e-6));
    REQUIRE_THAT(angle(v5,v3), WithinAbs(-3.141582653589793,1.0e-6));
    REQUIRE_THAT(angle(v5,v2), WithinAbs(1.733637511255314,1.0e-6));
    REQUIRE_THAT(angle(v5,v1), WithinAbs(-0.060784555874789,1.0e-6));
    REQUIRE_THAT(angle(v4,v3), WithinAbs(3.141592653589793,1.0e-6));
    REQUIRE_THAT(angle(v4,v2), WithinAbs(1.733627511255314,1.0e-6));
    REQUIRE_THAT(angle(v4,v1), WithinAbs(-0.060794555874789,1.0e-6));
    REQUIRE_THAT(angle(v3,v2), WithinAbs(-1.407965142334479,1.0e-6));
    REQUIRE_THAT(angle(v3,v1), WithinAbs(3.080798097715004,1.0e-6));
    REQUIRE_THAT(angle(v2,v1), WithinAbs(-1.794422067130102,1.0e-6));

    // Check zero cases 
    REQUIRE_THAT(angle(v01,v01), WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT(angle(v02,v01), WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT(angle(v03,v01), WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT(angle(v04,v01), WithinAbs(0.0,1.0e-6));
}


