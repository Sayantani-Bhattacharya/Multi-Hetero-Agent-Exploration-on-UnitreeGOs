#include <cstdio>
#include <cmath>
#include <iostream>
#include "unitree_kinematics/geometry2d.hpp"
#include "unitree_kinematics/se2d.hpp"

namespace unitree_kinematics
{
    // Print 2D Twist
    std::ostream & operator<<(std::ostream & os, const Twist2D & tw)
    {
        return os << "[" << tw.omega << " " << tw.x << " " << tw.y << "]";
    }

    // Read 2D Twist
    std::istream & operator>>(std::istream & is, Twist2D & tw)
    {
        const auto c = is.peek();        // examine the next character without extracting it

        if (c == '[') {
            is.get();         // remove the '[' character from the stream
            is >> tw.omega;
            is >> tw.x;
            is >> tw.y;
            is.get();         // remove the ']' character from the stream
        } else {
            is >> tw.omega >> tw.x >> tw.y;
        }

        is.ignore(100, '\n');
        return is;
    }

    // CONSTRUCTORS.

    // Create an identity transformation.
    Transform2D::Transform2D() : 
    translationVector{0.0, 0.0}, rotationAngle{0.0} 
    {}

    // Create a pure translation transform.
    Transform2D::Transform2D(Vector2D displacement) :
    translationVector{displacement}
    {}

    // Create a pure rotation transform.
    Transform2D::Transform2D(double angle) :
    rotationAngle{normalize_angle(angle)}
    {}

    // Create a transform with translation and rotation.
    // First rotate, then translate (in intermediate frame); which is equivalent to first translate, then rotate (in global frame).
    Transform2D::Transform2D(Vector2D displacement, double angle) :
    translationVector{displacement}, rotationAngle{normalize_angle(angle)}
    {}

    // TRANSFORM DIFFERENT ENTITIES THROUGH operator().

    // Transform a point.
    Point2D Transform2D::operator()(Point2D p) const
    {
        Point2D newp{};

        // Rotate.
        newp.x = p.x * cos(rotationAngle) - p.y * sin(rotationAngle);
        newp.y = p.x * sin(rotationAngle) + p.y * cos(rotationAngle);

        // Translate in global frame.
        newp = newp + translationVector;
        
        return newp;
    }

    // Transform a vector.
    Vector2D Transform2D::operator()(Vector2D v) const
    {
        Vector2D newv{};

        // Rotate.
        newv.x = v.x * cos(rotationAngle) - v.y * sin(rotationAngle);
        newv.y = v.x * sin(rotationAngle) + v.y * cos(rotationAngle);

        return newv;
    }

    // Transform a twist.
    Twist2D Transform2D::operator()(Twist2D v) const
    {
        Twist2D newv{};

        // Multiply with Adjoint.
        newv.omega = v.omega;
        newv.x = v.x * cos(rotationAngle) - v.y * sin(rotationAngle) + translationVector.y * newv.omega;
        newv.y = v.x * sin(rotationAngle) + v.y * cos(rotationAngle) - translationVector.x * newv.omega;

        return newv;
    }

    // RETURN INVERSE.
    Transform2D Transform2D::inv() const
    {
        Vector2D newTranslationVector{};
        double newRotationAngle{};

        // R^T
        newRotationAngle = -rotationAngle;

        // -R^T p
        newTranslationVector.x = -(translationVector.x * cos(newRotationAngle) - translationVector.y * sin(newRotationAngle));
        newTranslationVector.y = -(translationVector.x * sin(newRotationAngle) + translationVector.y * cos(newRotationAngle));

        // Create inverted Transform
        Transform2D inv{newTranslationVector, newRotationAngle};

        return inv;
    }

    // COMPOSE TRANSFORMS.
    Transform2D & Transform2D::operator*=(const Transform2D & rhs)
    {
        double newRotationAngle{};

        // R_a * R_rhs
        newRotationAngle = normalize_angle(rotationAngle + rhs.rotation());

        // Convert translation vector to point (p_rhs)
        Point2D temp{rhs.translation().x, rhs.translation().y};

        // R_a * p_rhs + p_a, which is equivalent to T_a * p_rhs
        Vector2D newTranslationVector{(*this)(temp).x, (*this)(temp).y};

        // Rewrite this transform as the composed transform.
        translationVector = newTranslationVector;
        rotationAngle = newRotationAngle;

        return *this;
    }

    // GETTERS.

    // Get translation vector.
    Vector2D Transform2D::translation() const
    {
        return translationVector;
    }

    // Get rotation angle.
    double Transform2D::rotation() const
    {
        return rotationAngle;
    }

    // Print SE(2) Transform.
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
    {
        return os << "deg: " << rad2deg(tf.rotation()) << " x: " << tf.translation().x << " y: " << tf.translation().y;
    }

    // Read SE(2) Transform.
    std::istream & operator>>(std::istream & is, Transform2D & tf) // Adapted from Abhishek Sankar
    {
        std::string trash_1, trash_2, trash_3;
        double rotationAngle {};
        Vector2D translationVector {};
        char next = is.peek(); //look at next character

        if(next == 'd')
        {
            is >> trash_1; //remove deg:
            is >> rotationAngle; //assign rad value to rotation
            is >> trash_2; //remove x:
            is >> translationVector.x; //assign x value to translation.x
            is >> trash_3; //remove y:
            is >> translationVector.y; //assign y value to translation.y
        }
        else
        {
            is >> rotationAngle >> translationVector.x >> translationVector.y;
        }
        tf = Transform2D(translationVector, deg2rad(rotationAngle));
        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
    {
        lhs *= rhs;

        return lhs;
    }

    Transform2D integrate_twist(const Twist2D & twist)
    {
        // Original Body frame is {b}
        
        // Check if angular increment is reasonable 
        if(normalize_angle(twist.omega) != twist.omega)
        {
            throw std::runtime_error("Angular increment is too large. Check verity of twist's angular velcoity, or decrease timestep.");
            return Transform2D{};
        }
        
        // Maximum radius of curvature according to practical standards.
        double R_max = 30.0, R = 0.0;

        // Check for pure translation
        bool pureTranslationFlag = false;

        // Case when there is no omega
        if(almost_equal(twist.omega, 0.0))
        {
            pureTranslationFlag = true;
        }
        else
        {
            R = magnitude(Vector2D{twist.x, twist.y}) / twist.omega;

            // Case when omega radius of curvature is too large to be considered substantial 
            if(R > R_max)
            {
                pureTranslationFlag = true;
                R = 0.0;
            }
        }

        // Screw axis frame {s}
        double rotation_s = atan2(twist.y, twist.x);
        Vector2D translation_s{-R * sin(rotation_s), R * cos(rotation_s)};
        Transform2D T_bs{translation_s, rotation_s};

        // Screw rotation transform {s} -> {S}
        Transform2D T_sS{twist.omega};

        // New body frame {B}
        if(pureTranslationFlag)
        {
            // Transform2D T_bB{};
            Transform2D T_bB{Vector2D{twist.x, twist.y}};
            // return T_bB;    

            Transform2D T_bA{Vector2D{twist.x, twist.y}};
            return T_bA;     
        }
        else
        {
            Transform2D T_bB = T_bs * T_sS * T_bs.inv();
            return T_bB;        
        }

        // Unanticipated error
        throw std::runtime_error("TWIST INTEGRATION FAILS.");
        return Transform2D{};
    }

}