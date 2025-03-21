#include <sstream>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "unitree_kinematics/geometry2d.hpp"
#include "unitree_kinematics/se2d.hpp"

using unitree_kinematics::normalize_angle;
using unitree_kinematics::deg2rad;
using unitree_kinematics::rad2deg;
using unitree_kinematics::Point2D;
using unitree_kinematics::Vector2D;
using unitree_kinematics::Twist2D;
using unitree_kinematics::Transform2D;
using unitree_kinematics::PI;
using Catch::Matchers::WithinAbs;

TEST_CASE( "Stream insertion operator << works for 2d twist", "[operator<<]") 
{
    Twist2D tw{0.69, 6.9, 69.6};
    std::string str = "[0.69 6.9 69.6]"; //typecast to int for integers ??

    std::stringstream sstr;
    sstr << tw;
    REQUIRE(sstr.str() == str);
}

TEST_CASE( "Stream extraction works for 2d twist", "[operator>>]") // Adapted from Abhishek Sankar
{
    Twist2D tw_1{}, tw_2{};
    std::stringstream sstr_1, sstr_2;

    sstr_1 << "0.69 4.20 69.0";
    sstr_1 >> tw_1;

    sstr_2 << "[6969.0 420.0 69.0]";
    sstr_2 >> tw_2;

    REQUIRE_THAT( tw_1.omega, WithinAbs(0.69,1.0e-6));
    REQUIRE_THAT( tw_1.x, WithinAbs(4.20,1.0e-6));
    REQUIRE_THAT( tw_1.y, WithinAbs(69.0,1.0e-6));
    REQUIRE_THAT( tw_2.omega, WithinAbs(6969.0,1.0e-6));
    REQUIRE_THAT( tw_2.x, WithinAbs(420.0,1.0e-6));
    REQUIRE_THAT( tw_2.y, WithinAbs(69.0,1.0e-6));
}

TEST_CASE( "Initialization works for SE(2) identity transform", "[Transform2D()]") 
{
    Transform2D tf;

    REQUIRE_THAT( tf.rotation(), WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( tf.translation().x, WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( tf.translation().y, WithinAbs(0.0,1.0e-6));    
}

TEST_CASE( "Initialization works for SE(2) pure translation transform", "[Transform2D(Vector2D)]") 
{
    Vector2D displacement{69.0, 69.0};
    Transform2D tf{displacement};

    REQUIRE_THAT( tf.rotation(), WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( tf.translation().x, WithinAbs(69.0,1.0e-6));
    REQUIRE_THAT( tf.translation().y, WithinAbs(69.0,1.0e-6));    
}

TEST_CASE( "Initialization works for SE(2) pure rotation transform", "[Transform2D(double)]") 
{
    double angle{-6.9*PI};
    Transform2D tf{angle};

    REQUIRE_THAT( tf.rotation(), WithinAbs(-0.9*PI,1.0e-6));
    REQUIRE_THAT( tf.translation().x, WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( tf.translation().y, WithinAbs(0.0,1.0e-6));    
}

TEST_CASE( "Initialization works for general SE(2) transform", "[Transform2D(Vector2D, double)]") 
{
    Vector2D displacement{4.20, 6.9};
    double angle{-6.9*PI};
    Transform2D tf{displacement, angle};

    REQUIRE_THAT( tf.rotation(), WithinAbs(-0.9*PI,1.0e-6));
    REQUIRE_THAT( tf.translation().x, WithinAbs(4.20,1.0e-6));
    REQUIRE_THAT( tf.translation().y, WithinAbs(6.9,1.0e-6));    
}

TEST_CASE( "SE(2) transformation of a 2D point works", "[Transform2D() Point2D]") 
{
    Vector2D displacement{4.20, 6.9};
    double angle{6.9};

    Transform2D tf{displacement, angle};

    Point2D p{6.9, 4.20};

    Point2D newp = tf(p);

    REQUIRE_THAT( newp.x, WithinAbs(7.399056180434521,1.0e-6));
    REQUIRE_THAT( newp.y, WithinAbs(14.317279794805081,1.0e-6));    
}

TEST_CASE( "SE(2) transformation of a 2D vector works", "[Transform2D() Vector2D]") 
{
    Vector2D displacement{4.20, 6.9};
    double angle{6.9};

    Transform2D tf{displacement, angle};

    Vector2D v{6.9, 4.20};

    Vector2D newv = tf(v);

    REQUIRE_THAT( newv.x, WithinAbs(3.199056180434521,1.0e-6));
    REQUIRE_THAT( newv.y, WithinAbs(7.417279794805081,1.0e-6));    
}

TEST_CASE( "SE(2) transformation of a 2D twist works", "[Transform2D() Twist2D]") 
{
    Vector2D displacement{4.20, 6.9};
    double angle{6.9};

    Transform2D tf{displacement, angle};

    Twist2D v{6.9, 6.9, 4.20};

    Twist2D newv = tf(v);

    REQUIRE_THAT( newv.omega, WithinAbs(6.9,1.0e-6));
    REQUIRE_THAT( newv.x, WithinAbs(50.809056180434524,1.0e-6));
    REQUIRE_THAT( newv.y, WithinAbs(-21.562720205194921,1.0e-6));    
}

TEST_CASE( "Inverse of SE(2) transformation works", "[inv()]") 
{
    Vector2D displacement{4.20, 6.9};
    double angle{-6.9*PI};
    Transform2D tf{displacement, angle};
    
    Transform2D tf_inv = tf.inv();

    REQUIRE_THAT( tf_inv.rotation(), WithinAbs(0.9*PI,1.0e-6));
    REQUIRE_THAT( tf_inv.translation().x, WithinAbs(6.126654629626783,1.0e-6));
    REQUIRE_THAT( tf_inv.translation().y, WithinAbs(5.264418586061781,1.0e-6));  
}

TEST_CASE( "SE(2) composition operator works", "[oprator *=]") 
{
    Vector2D displacement_1{4.20, 6.9};
    double angle_1{-6.9*PI};
    Transform2D tf_1{displacement_1, angle_1};

    Vector2D displacement_2{6.9, 4.20};
    double angle_2{4.2*PI};
    Transform2D tf_2{displacement_2, angle_2};
    
    tf_1 *= tf_2;

    REQUIRE_THAT( tf_1.rotation(), WithinAbs(-0.7*PI,1.0e-6));
    REQUIRE_THAT( tf_1.translation().x, WithinAbs(-1.064418586061780,1.0e-6));
    REQUIRE_THAT( tf_1.translation().y, WithinAbs(0.773345370373218,1.0e-6));  
}

TEST_CASE( "Stream insertion operator << works for SE(2) transform", "[operator<<]") 
{
    Vector2D displacement{6.9, 69.6};
    double angle{69};
    Transform2D tf{displacement, angle};

    std::stringstream lsstr, rsstr;

    lsstr << tf;
    rsstr << "deg: " << rad2deg(normalize_angle(angle)) << " x: 6.9 y: 69.6";

    REQUIRE(lsstr.str() == rsstr.str());
}

TEST_CASE( "Stream extraction works SE(2) transform", "[operator>>]") // Adapted from Abhishek Sankar
{
    Transform2D tf_1{}, tf_2{};
    std::stringstream sstr_a, sstr_b;

    sstr_a << "0.69 4.20 69.0";
    sstr_a >> tf_1;

    sstr_b << "deg: 6969.0 x: 420.0 y: 69.0";
    sstr_b >> tf_2;

    REQUIRE_THAT( tf_1.rotation(), WithinAbs(unitree_kinematics::normalize_angle(unitree_kinematics::deg2rad(0.69)), 1.0e-6 ));
    REQUIRE_THAT( tf_1.translation().x, WithinAbs(4.20, 1.0e-6));
    REQUIRE_THAT( tf_1.translation().y, WithinAbs(69.0, 1.0e-6));
    REQUIRE_THAT( tf_2.rotation(), WithinAbs(unitree_kinematics::normalize_angle(unitree_kinematics::deg2rad(6969.0)), 1.0e-6));
    REQUIRE_THAT( tf_2.translation().x, WithinAbs(420.0, 1.0e-6) );
    REQUIRE_THAT( tf_2.translation().y, WithinAbs(69.0, 1.0e-6) );
}

TEST_CASE( "SE(2) multiplication operator works", "[operator *]") 
{
    Vector2D displacement_1{4.20, 6.9};
    double angle_1{-6.9*PI};
    Transform2D tf_1{displacement_1, angle_1};

    Vector2D displacement_2{6.9, 4.20};
    double angle_2{4.2*PI};
    Transform2D tf_2{displacement_2, angle_2};
    
    Transform2D tf_3{(tf_1 * tf_2).translation(), (tf_1 * tf_2).rotation()};

    REQUIRE_THAT( tf_3.rotation(), WithinAbs(-0.7*PI,1.0e-6));
    REQUIRE_THAT( tf_3.translation().x, WithinAbs(-1.064418586061780,1.0e-6));
    REQUIRE_THAT( tf_3.translation().y, WithinAbs(0.773345370373218,1.0e-6));  
}

TEST_CASE( "Twist integration works", "[integrate_twist()]") 
{
    // Check for pure rotation
    
    Twist2D v1{3.1, 0, 0};
    Transform2D tf1 = integrate_twist(v1);
    
    REQUIRE_THAT( tf1.rotation(), WithinAbs(3.1,1.0e-6));
    REQUIRE_THAT( tf1.translation().x, WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( tf1.translation().y, WithinAbs(0.0,1.0e-6));  

    // Check for pure translation

    Twist2D v2{0, -6.9, -42};
    Transform2D tf2 = integrate_twist(v2);

    REQUIRE_THAT( tf2.rotation(), WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( tf2.translation().x, WithinAbs(-6.9,1.0e-6));
    REQUIRE_THAT( tf2.translation().y, WithinAbs(-42,1.0e-6));  

    // Check for general twist

    Twist2D v3{3.1, 6.9, 42};
    Transform2D tf3 = integrate_twist(v3); 

    REQUIRE_THAT( tf3.rotation(), WithinAbs(3.1,1.0e-6));
    REQUIRE_THAT( tf3.translation().x, WithinAbs(-26.992506368,1.0e-6));
    REQUIRE_THAT( tf3.translation().y, WithinAbs(5.0130388255,1.0e-6)); 

    // Check for large radius of curvature

    Twist2D v4{0.26, -6.9, -4.2};
    Transform2D tf4 = integrate_twist(v4); 

    REQUIRE_THAT( tf4.rotation(), WithinAbs(0.0,1.0e-6));
    REQUIRE_THAT( tf4.translation().x, WithinAbs(-6.9,1.0e-6));
    REQUIRE_THAT( tf4.translation().y, WithinAbs(-4.2,1.0e-6));  

    // Check for improper twist

    Twist2D v5{6.9, -6.9, -4.2};

    REQUIRE_THROWS(integrate_twist(v5));
}
