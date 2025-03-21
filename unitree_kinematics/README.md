# Unitree Kinematics Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- geometry2d - Handles 2D geometry primitives
- se2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. If you needed to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
   
   1. Member function:
    ```
        #include <iostream>
        #include <cmath>

        struct Vector2D {
            double x = 0.0;
            double y = 0.0;

            // Member function to normalize the vector
            void normalize() {
                double mag = std::sqrt(x*x + y*y);

                // Check for division by zero to avoid NaN
                if (mag != 0.0) {
                    x /= mag;
                    y /= mag;
                }
            }
        };

        int main() {
            // Example vector
            Vector2D myVector {3.0, 4.0};

            // Normalize the vector
            myVector.normalize();

            // Display the normalized vector
            std::cout << "Normalized Vector (Design 1): (" << myVector.x << ", " << myVector.y << ")\n";

            return 0;
        }

    ```
        - Pros: 
            Encapsulation: This aligns with C++ Core Guidelines [C.9] which recommends using member functions for operations on class instances to achieve encapsulation.
        - Cons:
            Modifies Object State: C++ Core Guidelines [C.4, P.1] suggests minimizing the use of functions that modify object state, especially if it is not immediately obvious.

   2. Free Function:

    ```
        #include <iostream>
        #include <cmath>

        struct Vector2D {
            double x = 0.0;
            double y = 0.0;
        };

        // Free function to normalize a vector
        Vector2D normalizeVector(const Vector2D& v) {
            double mag = std::sqrt(v.x*v.x + v.y*v.y);

            // Check for division by zero to avoid NaN
            if (mag != 0.0) {
                return {v.x / mag, v.y / mag};
            }

            return v; // Return original vector if magnitude is zero
        }

        int main() {
            // Example vector
            Vector2D myVector {3.0, 4.0};

            // Normalize the vector
            Vector2D normalizedVector = normalizeVector(myVector);

            // Display the normalized vector
            std::cout << "Normalized Vector (Design 2): (" << normalizedVector.x << ", " << normalizedVector.y << ")\n";

            return 0;
        }

    ```
        - Pros:
            Separation of Concerns: This is in line with C++ Core Guidelines, which advocates for separating concerns to improve maintainability.
        - Cons:
            Deviation from Encapsulation: C++ Core Guidelines [C.9] suggests avoiding free functions if they do not require direct access to private class members, as this may break encapsulation.

    3. Operator Overloading
    ```
        #include <iostream>
        #include <cmath>

        struct Vector2D {
            double x = 0.0;
            double y = 0.0;

            // Operator overloading to normalize the vector
            Vector2D operator/(double scalar) const {
                // Check for division by zero to avoid NaN
                if (scalar != 0.0) {
                    return {x / scalar, y / scalar};
                }

                return *this; // Return original vector if scalar is zero
            }

            // Member function to calculate the magnitude of the vector
            double magnitude() const {
                return std::sqrt(x*x + y*y);
            }

            // Member function to normalize the vector
            Vector2D normalize() const {
                double mag = magnitude();

                // Use the overloaded division operator
                return *this / mag;
            }
        };

        int main() {
            // Example vector
            Vector2D myVector {3.0, 4.0};

            // Normalize the vector
            Vector2D normalizedVector = myVector.normalize();

            // Display the normalized vector
            std::cout << "Normalized Vector (Design 3): (" << normalizedVector.x << ", " << normalizedVector.y << ")\n";

            return 0;
        }

    ```
        - Pros: Ensures consistency with standard library types, which aligns with C++ Core Guidelines [F.47].

        - Cons: The choice of operator/= for normalization may not be immediately intuitive. C++ Core Guidelines [F.15] advises against overloading operators in ways that might surprise users.

   - Which of the methods would you implement and why?

   I would implement method 2, since I value maintainability and Method 1 and 3 can surprise the user by making the code less readable.

2. What is the difference between a class and a struct in C++?

    Members of a class have a default access level of private.
    You need to use the public, protected, and private keywords to specify the access level explicitly.

    Members of a struct have a default access level of public.
    You can still use the public, protected, and private keywords to change the access level explicitly if needed.

3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?

    C.8 says use class rather than struct if any members are public.
    C.2 says Use class if the class has an invariant; use struct if the data members can vary independently. Here, translation and rotation are invariants since they're usually applied together (and not independently) for any transform.

4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?

    C.46: By default, declare single-argument constructors explicit.


5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

   Transform2D::operator*=() includes an assignment and changes the members of its owner, therefore it cannot be immutable, as they are supposed to be by default.
