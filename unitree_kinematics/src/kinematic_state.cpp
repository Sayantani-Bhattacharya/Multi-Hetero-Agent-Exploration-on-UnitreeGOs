#include <cstdio>
#include <cmath>
#include <iostream>
#include "unitree_kinematics/geometry2d.hpp"
#include "unitree_kinematics/se2d.hpp"
#include "unitree_kinematics/kinematic_state.hpp"

namespace unitree_kinematics
{
    // CONSTRUCTORS.

    // Create an identity transformation.
    // DiffDrive::DiffDrive() : 
    // translationVector{0.0, 0.0}, rotationAngle{0.0} 
    // {}

    // // Create a pure translation transform.
    // DiffDrive::DiffDrive(Vector2D displacement) :
    // translationVector{displacement}
    // {}

    // // Create a pure rotation transform.
    // DiffDrive::DiffDrive(double angle) :
    // rotationAngle{normalize_angle(angle)}
    // {}

    // // Create a transform with translation and rotation.
    // // First rotate, then translate (in intermediate frame); which is equivalent to first translate, then rotate (in global frame).
    // DiffDrive::DiffDrive(Vector2D displacement, double angle) :
    // translationVector{displacement}, rotationAngle{normalize_angle(angle)}
    // {}

}