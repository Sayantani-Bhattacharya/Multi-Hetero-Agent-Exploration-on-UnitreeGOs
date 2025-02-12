#ifndef TURTLELIB_DIFFDRIVE_INCLUDE_GUARD_HPP
#define TURTLELIB_DIFFDRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.

#include<iosfwd> // contains forward definitions for iostream objects

#include"unitree_kinematics/geometry2d.hpp"
#include"unitree_kinematics/se2d.hpp"
#include"unitree_kinematics/kinematic_state.hpp"

namespace turtlelib
{

    /// \brief represent a mobile robot's pose
    struct pose
    {
        /// \brief angle with the world frame
        double theta = 0.0;

        /// \brief x position in the world frame
        double x = 0.0;

        /// \brief y position in world frame
        double y = 0.0;
    };

    /// \brief models the kinematics of a differential drive robot with a given wheel track and wheel radius
    class DiffDrive
    {

    private:

    public:

        /// \brief angle of rotation of the right wheel, in radians
        double phi_l;
        /// \brief angle of rotation of the right wheel, in radians
        double phi_r;
        /// \brief angle of rotation of the right wheel, in radians
        pose q;

        /// \brief Initialize the kinematics
        DiffDrive();

        // /// \brief create a transformation that is a pure translation
        // /// \param trans - the vector by which to translate
        // explicit DiffDrive(Vector2D trans);

        // /// \brief create a pure rotation
        // /// \param radians - angle of the rotation, in radians
        // explicit DiffDrive(double radians);

        // /// \brief Create a transformation with a translational and rotational
        // /// component
        // /// \param trans - the translation
        // /// \param radians - the rotation, in radians
        // DiffDrive(Vector2D trans, double radians);

    };
}

#endif
