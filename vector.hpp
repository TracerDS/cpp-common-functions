#pragma once

#include <cmath>
#include <algorithm>

namespace Common {
    struct IVector {
        using value_type = float;
    };

    /**
     * @class Vector2
     * @brief A class representing a 2-dimensional vector.
     * 
     * This class provides various operations and utilities for 2D vectors, including
     * basic arithmetic operations, normalization, dot product, and more.
     * 
     * @tparam value_type The type of the vector components.
     */     
    class Vector2 {
    public:
        using value_type = IVector::value_type;
    protected:
        /**
         * @brief A member variable of type value_type representing the x-coordinate.
         */
        value_type m_x;
        /**
         * @brief A member variable of type value_type representing the y-coordinate.
         */
        value_type m_y;
    public:
        /**
         * @brief A constant static member representing a zero vector.
         * 
         * This member is a Vector2 object with all components set to zero.
         * It can be used as a default or initial value for vector operations.
         */
        const static Vector2 zero;
        
        
        /**
         * @brief A constant static member representing an one vector.
         *
         * This instance represents a vector with all components set to one.
         * It can be used as a default or reference vector in various mathematical
         * operations or transformations where a unit vector is required.
         */
        const static Vector2 one;

        /**
         * @brief Default constructor. Initializes the vector to (0, 0, 0).
         */
        constexpr Vector2() noexcept : Vector2(0, 0) {}

        /**
         * @brief Constructs a Vector2 object with the given x, y, and z coordinates.
         * 
         * @param x The x-coordinate of the vector.
         * @param y The y-coordinate of the vector.
         */
        constexpr Vector2(value_type x, value_type y) noexcept : m_x{x}, m_y{y} {}

        /**
         * @brief Constructs a Vector2 object from a Vector3 object.
         * 
         * This constructor initializes a Vector2 object using the x and y coordinates
         * from the given Vector3 object.
         * 
         * @param other The Vector3 object from which to initialize the Vector2 object.
         */
        constexpr Vector2(Vector3 other) noexcept : Vector2(other.GetX(), other.GetY()) {}

        /**
         * @brief Retrieves the X coordinate value.
         * 
         * @return The X coordinate value of type value_type.
         */
        constexpr value_type GetX() const noexcept { return m_x; }

        /**
         * @brief Retrieves the Y coordinate value.
         * 
         * @return The Y coordinate value of type value_type.
         */
        constexpr value_type GetY() const noexcept { return m_y; }

        /**
         * @brief Sets the x-coordinate of the vector.
         * 
         * This function sets the x-coordinate of the vector to the specified value.
         * 
         * @param x The new value for the x-coordinate.
         */
        constexpr void SetX(value_type x) noexcept { m_x = x; }
        
        /**
         * @brief Sets the y-coordinate of the vector.
         * 
         * This function sets the y-coordinate of the vector to the specified value.
         * 
         * @param y The new value for the y-coordinate.
         */
        constexpr void SetY(value_type y) noexcept { m_y = y; }

        /**
         * @brief Calculates the squared length of the vector.
         * 
         * This function returns the squared length of the vector, which is the sum of the squares of its components.
         * It is marked as constexpr, meaning it can be evaluated at compile time, and noexcept, indicating that it does not throw exceptions.
         * 
         * @return The squared length of the vector.
         */
        constexpr value_type Length() const noexcept { return m_x * m_x + m_y * m_y; }
        
        /**
         * @brief Computes the square root of the length of the vector.
         * 
         * This function calculates the square root of the length (magnitude) of the vector.
         * It is a shorthand for computing std::sqrt(Length()).
         * 
         * @return The square root of the length of the vector.
         */
        value_type LengthSqrt() const noexcept { return std::sqrt(Length()); }

        /**
         * @brief Calculates the Euclidean distance between two 2D vectors.
         * 
         * This function computes the distance between two points in 2D space
         * represented by the vectors `a` and `b`. The distance is calculated
         * using the square root of the sum of the squared differences of their
         * respective components.
         * 
         * @param a The first vector (point) in 2D space.
         * @param b The second vector (point) in 2D space.
         * @return value_type The Euclidean distance between vectors `a` and `b`.
         */
        static value_type Distance(const Vector2& a, const Vector2& b) noexcept { return (b - a).LengthSqrt(); }
		
        /**
         * @brief Calculates the distance between this vector and another vector.
         * 
         * @param other The other vector to calculate the distance to.
         * @return value_type The distance between the two vectors.
         */
        value_type Distance(const Vector2& other) const noexcept { return Vector2::Distance(*this, other); }

        /**
         * @brief Normalizes the vector to have a length of 1.
         *
         * This function computes the normalized vector by dividing each component
         * of the vector by its length. If the length of the vector is zero, it
         * returns a zero vector to avoid division by zero.
         *
         * @return A normalized vector with a length of 1, or a zero vector if the
         *         original vector has a length of zero.
         */
        Vector2 Normalize() const noexcept {
            value_type length = LengthSqrt();
            if (length == 0)
                return Vector2::zero;
            return { m_x / length, m_y / length };
        }
        
        /**
         * @brief Normalizes the vector to have a length of 1.
         *
         * This function modifies the vector in place to make its length equal to 1.
         * If the vector's length is zero, the vector components are set to zero.
         *
         * @return A reference to the normalized vector.
         */
        Vector2& Normalize() noexcept {
            value_type length = LengthSqrt();
            if (length != 0) {
                m_x = 0;
                m_y = 0;
            } else {
                m_x /= length;
                m_y /= length;
            }

            return *this;
        }        

        /**
         * @brief Computes the dot product of two 2D vectors.
         * 
         * @param a The first vector.
         * @param b The second vector.
         * @return value_type The dot product of vectors a and b.
         */
        constexpr static value_type Dot(const Vector2& a, const Vector2& b) noexcept {
            return a.m_x * b.m_x + a.m_y * b.m_y;
        }

        /**
         * @brief Computes the dot product of this vector and another vector.
         * 
         * @param other The other vector to compute the dot product with.
         * @return value_type The dot product of the two vectors.
         */
        constexpr value_type Dot(const Vector2& other) const noexcept {
            return Vector2::Dot(*this, other);
        }

        /**
         * @brief Projects a vector onto another vector.
         *
         * This function projects the given vector onto the specified vector (projectOn).
         * If the magnitude of the projectOn vector is too small (close to zero), 
         * the function returns a zero vector to avoid division by zero.
         *
         * @param vector The vector to be projected.
         * @param projectOn The vector onto which the projection is performed.
         * @return The projected vector.
         */
        constexpr static Vector2 Project(const Vector2& vector, const Vector2& projectOn) noexcept {
            auto projectProduct = projectOn.Dot(projectOn);
            if (projectProduct < std::numeric_limits<value_type>::epsilon())
                return Vector2::zero;

            auto product = vector.Dot(projectOn);
            return {
                projectOn.m_x * product / projectProduct,
                projectOn.m_y * product / projectProduct
            };
        }

        /**
         * @brief Projects this vector onto another vector.
         * 
         * This function projects the current vector onto the given vector `projectOn`.
         * 
         * @param projectOn The vector onto which the current vector will be projected.
         * @return Vector2 The result of the projection.
         */
        constexpr Vector2 Project(const Vector2& projectOn) noexcept {
            return Vector2::Project(*this, projectOn);
        }

        /**
         * @brief Reflects vector a around vector b.
         *
         * This function calculates the reflection of vector a around vector b using the formula:
         * Reflect(a, b) = a - 2 * (a.Dot(b)) * b
         *
         * @param a The vector to be reflected.
         * @param b The vector around which a is reflected.
         * @return The reflected vector.
         */
        static Vector2 Reflect(const Vector2& a, const Vector2& b) noexcept {
            return a - b * 2.0f * a.Dot(b);
        }

        /**
         * @brief Reflects this vector around the given vector.
         * 
         * This function calculates the reflection of this vector around the specified vector.
         * 
         * @param other The vector to reflect around.
         * @return Vector2 The reflected vector.
         */
        Vector2 Reflect(const Vector2& other) const noexcept {
			return Vector2::Reflect(*this, other);
        }

        /**
         * @brief Calculates the angle between two 2D vectors.
         * 
         * This function computes the angle between two vectors `a` and `b` using the dot product and magnitudes of the vectors.
         * 
         * @param a The first vector.
         * @param b The second vector.
         * @return The angle in radians between the two vectors. If either vector has zero length, the function returns 0.0f.
         */
        constexpr static value_type Angle(const Vector2& a, const Vector2& b) noexcept {
			auto magnitude = a.Length() * b.Length();
			if (magnitude == 0)
                return 0.0f;

            auto product = a.Dot(b);
			return std::acos(product / magnitude);
        }

        /**
         * @brief Calculates the angle between this vector and another vector.
         * 
         * @param other The other vector to calculate the angle with.
         * @return value_type The angle between the two vectors.
         */
        constexpr value_type Angle(const Vector2& other) noexcept {
			return Vector2::Angle(*this, other);
        }

        /**
         * @brief Linearly interpolates between two vectors a and b by a factor t.
         *
         * This function performs a linear interpolation between the vectors a and b,
         * clamping the interpolation factor t between 0.0 and 1.0.
         *
         * @param a The starting vector.
         * @param b The ending vector.
         * @param t The interpolation factor, clamped between 0.0 and 1.0.
         * @return The interpolated vector.
         */
        constexpr static Vector2 Lerp (
            const Vector2& a,
            const Vector2& b,
            value_type t
        ) noexcept {
            return LerpUnclamped(a, b, std::clamp(t, 0.0f, 1.0f));
        }

        /**
         * @brief Linearly interpolates between this vector and another vector.
         *
         * This function performs a linear interpolation between the current vector
         * and the provided vector `other` based on the interpolation factor `t`.
         *
         * @param other The target vector to interpolate towards.
         * @param t The interpolation factor, typically in the range [0, 1].
         *          - When t = 0, the result is the current vector.
         *          - When t = 1, the result is the target vector `other`.
         *          - Values between 0 and 1 will interpolate linearly between the two vectors.
         * @return A new Vector2 that is the result of the linear interpolation.
         */
        constexpr Vector2 Lerp(const Vector2& other, value_type t) noexcept {
            return Vector2::Lerp(*this, other, t);
        }

        /**
         * @brief Performs Spherical Linear Interpolation (Slerp) between two vectors.
         *
         * Slerp is a method of interpolating between two vectors along a great circle path on a sphere.
         * This function interpolates between vectors `a` and `b` by a factor of `t`.
         *
         * @param a The starting vector.
         * @param b The ending vector.
         * @param t The interpolation factor, typically in the range [0, 1].
         * @return The interpolated vector.
         */
        static Vector2 Slerp(const Vector2& a, const Vector2& b, float t) noexcept {
            auto dotProduct = std::clamp(a.Dot(b), -1.0f, 1.0f);
            float theta = std::acos(dotProduct) * std::clamp(t, 0.0f, 1.0f);

            Vector2 relativeVec = (b - a * dotProduct).Normalize();
            return a * std::cos(theta) + relativeVec * std::sin(theta);
        }

        /**
         * @brief Performs spherical linear interpolation (Slerp) between this vector and another vector.
         * 
         * Slerp is a method of interpolating between two vectors along a great circle on a sphere.
         * It is commonly used in computer graphics for smooth transitions between orientations.
         * 
         * @param other The target vector to interpolate towards.
         * @param t The interpolation factor, typically in the range [0, 1], where 0 returns this vector
         * and 1 returns the other vector.
         * @return Vector2 The interpolated vector.
         */
		Vector2 Slerp(const Vector2& other, float t) const noexcept {
			return Vector2::Slerp(*this, other, t);
		}

        /**
         * @brief Linearly interpolates between two vectors without clamping the interpolant.
         *
         * This function performs a linear interpolation between two vectors `a` and `b` 
         * based on the interpolant `t`. The result is not clamped, meaning `t` can be 
         * outside the range [0, 1].
         *
         * @param a The start vector.
         * @param b The end vector.
         * @param t The interpolant value.
         * @return A vector that is the linear interpolation of `a` and `b` based on `t`.
         */
        constexpr static Vector2 LerpUnclamped (
            const Vector2& a,
            const Vector2& b,
            value_type t
        ) noexcept {
            return {
                a.m_x + t * (b.m_x - a.m_x),
                a.m_y + t * (b.m_y - a.m_y)
            };
        }

        /**
         * @brief Linearly interpolates between this vector and another vector without clamping.
         * 
         * This function performs a linear interpolation between the current vector and the 
         * specified vector 'other' based on the interpolation factor 't'. The interpolation 
         * is not clamped, meaning that 't' can be outside the range [0, 1].
         * 
         * @param other The target vector to interpolate towards.
         * @param t The interpolation factor. A value of 0 returns the current vector, 
         *          a value of 1 returns the 'other' vector, and values outside the range 
         *          [0, 1] will extrapolate accordingly.
         * @return Vector2 The interpolated vector.
         */
        constexpr Vector2 LerpUnclamped(const Vector2& other, value_type t) noexcept {
            return Vector2::LerpUnclamped(*this, other, t);
        }

        /**
         * @brief Adds two Vector2 objects component-wise.
         * 
         * This operator performs a component-wise addition of two Vector2 objects.
         * 
         * @param other The other Vector2 object to add.
         * @return A new Vector2 object that is the result of the addition.
         */
        constexpr Vector2 operator+(const Vector2& other) const noexcept {
            return { m_x + other.m_x, m_y + other.m_y };
        }

        /**
         * @brief Subtracts another Vector2 from this Vector2.
         * 
         * This operator performs component-wise subtraction of the given Vector2
         * from this Vector2 and returns the resulting Vector2.
         * 
         * @param other The Vector2 to subtract from this Vector2.
         * @return A new Vector2 resulting from the component-wise subtraction.
         */
        constexpr Vector2 operator-(const Vector2& other) const noexcept {
            return { m_x - other.m_x, m_y - other.m_y };
        }
        
        /**
         * @brief Multiplies two Vector2 objects component-wise.
         * 
         * This operator performs a component-wise multiplication of the current 
         * Vector2 object with another Vector2 object passed as a parameter.
         * 
         * @param other The Vector2 object to multiply with.
         * @return A new Vector2 object resulting from the component-wise multiplication.
         */
        constexpr Vector2 operator*(const Vector2& other) const noexcept {
            return { m_x * other.m_x, m_y * other.m_y };
        }

        /**
         * @brief Divides the current Vector2 by another Vector2 element-wise.
         * 
         * This operator performs element-wise division of the current vector by the given vector.
         * Each component (x, y, z) of the current vector is divided by the corresponding component
         * of the other vector.
         * 
         * @param other The Vector2 to divide by.
         * @return A new Vector2 resulting from the element-wise division.
         * @note This operation assumes that the components of the other vector are non-zero.
         */
        constexpr Vector2 operator/(const Vector2& other) const noexcept {
            return { m_x / other.m_x, m_y / other.m_y };
        }
        
        /**
         * @brief Adds a scalar value to each component of the vector.
         * 
         * This operator overload allows you to add a scalar value to the x, y, and z components
         * of the vector, resulting in a new vector where each component is increased by the scalar.
         * 
         * @param scalar The scalar value to be added to each component of the vector.
         * @return A new Vector2 object with each component increased by the scalar value.
         */
        constexpr Vector2 operator+(value_type scalar) const noexcept {
            return { m_x + scalar, m_y + scalar };
        }

        /**
         * @brief Subtracts a scalar value from each component of the vector.
         * 
         * This operator returns a new Vector2 instance where each component
         * (x, y, z) of the original vector is reduced by the given scalar value.
         * 
         * @param scalar The scalar value to subtract from each component of the vector.
         * @return A new Vector2 instance with each component reduced by the scalar value.
         */
        constexpr Vector2 operator-(value_type scalar) const noexcept {
            return { m_x - scalar, m_y - scalar };
        }

        /**
         * @brief Multiplies the vector by a scalar value.
         * 
         * This operator overload allows for the multiplication of a Vector2 object
         * by a scalar value, returning a new Vector2 object with each component
         * scaled by the given scalar.
         * 
         * @param scalar The scalar value to multiply with the vector components.
         * @return A new Vector2 object with each component scaled by the scalar value.
         */
        constexpr Vector2 operator*(value_type scalar) const noexcept {
            return { m_x * scalar, m_y * scalar };
        }

        /**
         * @brief Divides the vector by a scalar value.
         * 
         * This operator performs element-wise division of the vector components
         * by the given scalar value.
         * 
         * @param scalar The scalar value to divide the vector by.
         * @return A new Vector2 object with each component divided by the scalar.
         * @note This function is marked as noexcept, indicating that it does not throw exceptions.
         */
        constexpr Vector2 operator/(value_type scalar) const noexcept {
            return { m_x / scalar, m_y / scalar };
        }
        
        /**
         * @brief Adds the components of another Vector2 to this vector.
         * 
         * This operator performs component-wise addition of the given vector to this vector.
         * 
         * @param other The vector to be added.
         * @return A reference to this vector after addition.
         */
        constexpr Vector2& operator+=(const Vector2& other) noexcept {
            this->m_x += other.m_x;
            this->m_y += other.m_y;
            return *this;
        }

        /**
         * @brief Subtracts the components of another Vector2 from this vector.
         *
         * This operator modifies the current vector by subtracting the corresponding
         * components of the given vector from it.
         *
         * @param other The vector to subtract from this vector.
         * @return A reference to the modified vector.
         */
        constexpr Vector2& operator-=(const Vector2& other) noexcept {
            this->m_x -= other.m_x;
            this->m_y -= other.m_y;
            return *this;
        }

        /**
         * @brief Multiplies each component of this vector by the corresponding component of another vector.
         * 
         * This operator performs element-wise multiplication of the vector components.
         * 
         * @param other The vector to multiply with.
         * @return A reference to the resulting vector after multiplication.
         */
        constexpr Vector2& operator*=(const Vector2& other) noexcept {
            this->m_x *= other.m_x;
            this->m_y *= other.m_y;
            return *this;
        }

        /**
         * @brief Divides the components of this vector by the corresponding components of another vector.
         *
         * This operator performs element-wise division of the vector components.
         *
         * @param other The vector to divide by.
         * @return A reference to this vector after the division.
         */
        constexpr Vector2& operator/=(const Vector2& other) noexcept {
            this->m_x /= other.m_x;
            this->m_y /= other.m_y;
            return *this;
        }

        /**
         * @brief Compares this Vector2 object with another for equality.
         * 
         * @param other The other Vector2 object to compare with.
         * @return true if the x, y, and z components of both vectors are equal.
         * @return false otherwise.
         */
        constexpr bool operator==(const Vector2& other) const noexcept {
            return m_x == other.m_x && m_y == other.m_y;
        }
        
        /**
         * @brief Inequality operator for comparing two Vector2 objects.
         *
         * This operator checks if two Vector2 objects are not equal by
         * negating the result of the equality operator.
         *
         * @param other The other Vector2 object to compare with.
         * @return true if the two Vector2 objects are not equal, false otherwise.
         */
        constexpr bool operator!=(const Vector2& other) const noexcept {
            return !(*this == other);
        }

        /**
         * @brief Compares the length squared of the vector to a scalar value.
         * 
         * This operator checks if the length squared of the vector is equal to the given scalar value.
         * 
         * @param scalar The scalar value to compare with the length squared of the vector.
         * @return true if the length squared of the vector is equal to the scalar value, false otherwise.
         */
        bool operator==(value_type scalar) const noexcept { return this->LengthSqrt() == scalar; }

        /**
         * @brief Inequality operator to compare the vector with a scalar value.
         * 
         * This operator checks if the vector is not equal to the given scalar value.
         * It returns true if the vector is not equal to the scalar, and false otherwise.
         * 
         * @param scalar The scalar value to compare with the vector.
         * @return true if the vector is not equal to the scalar, false otherwise.
         */
        bool operator!=(value_type scalar) const noexcept { return !(*this == scalar); }

        /**
         * @brief Compares the length of this vector with another vector.
         * 
         * This operator compares the lengths of two vectors and returns true if the length
         * of this vector is less than the length of the other vector.
         * 
         * @param other The vector to compare with.
         * @return true if the length of this vector is less than the length of the other vector, false otherwise.
         */
        constexpr bool operator<(const Vector2& other) const noexcept {
            return Length() < other.Length();
        }
        
        /**
         * @brief Checks if this vector is less than or equal to another vector.
         * @param other The vector to compare.
         * @return True if this vector is less than or equal to the other vector, false otherwise.
         */
        constexpr bool operator<=(const Vector2& other) const noexcept {
            return Length() <= other.Length();
        }
        /**
         * @brief Checks if this vector is greater than another vector.
         * @param other The vector to compare.
         * @return True if this vector is greater than the other vector, false otherwise.
         */
        constexpr bool operator>(const Vector2& other) const noexcept {
            return Length() > other.Length();
        }
        
        /**
         * @brief Checks if this vector is greater than or equal to another vector.
         * @param other The vector to compare.
         * @return True if this vector is greater than or equal to the other vector, false otherwise.
         */
        constexpr bool operator>=(const Vector2& other) const noexcept {
            return Length() >= other.Length();
        }

        /**
         * @brief Compares this vector with another vector.
         * @param other The vector to compare.
         * @return -1 if this vector is less than the other vector, 1 if greater, 0 if equal.
         */
        constexpr std::strong_ordering operator<=>(const Vector3& other) const noexcept {
            if (operator<(other)) return std::strong_ordering::less;
            if (operator>(other)) return std::strong_ordering::greater;
            return std::strong_ordering::equal;
        }
    };

    constexpr Vector2 Vector2::zero{0, 0};
    constexpr Vector2 Vector2::one {1, 1};

    /**
     * @class Vector3
     * @brief A class representing a 3-dimensional vector.
     * 
     * This class provides various operations and utilities for 3D vectors, including
     * basic arithmetic operations, normalization, dot product, cross product, and more.
     * 
     * @tparam value_type The type of the vector components.
     */     
    class Vector3 {
    public:
        using value_type = IVector::value_type;
    protected:
        /**
         * @brief A member variable of type value_type representing the x-coordinate.
         */
        value_type m_x;
        /**
         * @brief A member variable of type value_type representing the y-coordinate.
         */
        value_type m_y;
        /**
         * @brief A member variable of type value_type representing the z-coordinate.
         */
        value_type m_z;
    public:
        /**
         * @brief A constant static member representing a zero vector.
         * 
         * This member is a Vector3 object with all components set to zero.
         * It can be used as a default or initial value for vector operations.
         */
        const static Vector3 zero;
        
        
        /**
         * @brief A constant static member representing an one vector.
         *
         * This instance represents a vector with all components set to one.
         * It can be used as a default or reference vector in various mathematical
         * operations or transformations where a unit vector is required.
         */
        const static Vector3 one;

        /**
         * @brief Default constructor. Initializes the vector to (0, 0, 0).
         */
        constexpr Vector3() noexcept : Vector3(0, 0, 0) {}

        /**
         * @brief Constructs a Vector3 object with the given x, y, and z coordinates.
         * 
         * @param x The x-coordinate of the vector.
         * @param y The y-coordinate of the vector.
         * @param z The z-coordinate of the vector.
         */
        constexpr Vector3(value_type x, value_type y, value_type z) noexcept : m_x{x}, m_y{y}, m_z{z} {}

        /**
         * @brief Constructs a Vector3 object from a Vector2 object.
         * 
         * This constructor initializes a Vector3 object using the x and y coordinates
         * from the given Vector2 object and sets the z coordinate to 0.
         * 
         * @param other The Vector2 object from which to initialize the Vector3 object.
         */
        constexpr Vector3(Vector2 other) noexcept : Vector3(other.GetX(), other.GetY(), 0) {}

        /**
         * @brief Retrieves the X coordinate value.
         * 
         * @return The X coordinate value of type value_type.
         */
        constexpr value_type GetX() const noexcept { return m_x; }

        /**
         * @brief Retrieves the Y coordinate value.
         * 
         * @return The Y coordinate value of type value_type.
         */
        constexpr value_type GetY() const noexcept { return m_y; }

        /**
         * @brief Retrieves the Z coordinate value.
         * 
         * @return The Z coordinate value of type value_type.
         */
        constexpr value_type GetZ() const noexcept { return m_z; }

        /**
         * @brief Sets the x-coordinate of the vector.
         * 
         * This function sets the x-coordinate of the vector to the specified value.
         * 
         * @param x The new value for the x-coordinate.
         */
        constexpr void SetX(value_type x) noexcept { m_x = x; }
        
        /**
         * @brief Sets the y-coordinate of the vector.
         * 
         * This function sets the y-coordinate of the vector to the specified value.
         * 
         * @param y The new value for the y-coordinate.
         */
        constexpr void SetY(value_type y) noexcept { m_y = y; }
        
        /**
         * @brief Sets the z-coordinate of the vector.
         * 
         * This function sets the z-coordinate of the vector to the specified value.
         * 
         * @param z The new value for the z-coordinate.
         */
        constexpr void SetZ(value_type z) noexcept { m_y = z; }

        /**
         * @brief Calculates the squared length of the vector.
         * 
         * This function returns the squared length of the vector, which is the sum of the squares of its components.
         * It is marked as constexpr, meaning it can be evaluated at compile time, and noexcept, indicating that it does not throw exceptions.
         * 
         * @return The squared length of the vector.
         */
        constexpr value_type Length() const noexcept { return m_x * m_x + m_y * m_y; }
        
        /**
         * @brief Computes the square root of the length of the vector.
         * 
         * This function calculates the square root of the length (magnitude) of the vector.
         * It is a shorthand for computing std::sqrt(Length()).
         * 
         * @return The square root of the length of the vector.
         */
        value_type LengthSqrt() const noexcept { return std::sqrt(Length()); }

        /**
         * @brief Calculates the Euclidean distance between two 3D vectors.
         * 
         * This function computes the distance between two points in 3D space
         * represented by the vectors `a` and `b`. The distance is calculated
         * using the square root of the sum of the squared differences of their
         * respective components.
         * 
         * @param a The first vector (point) in 3D space.
         * @param b The second vector (point) in 3D space.
         * @return value_type The Euclidean distance between vectors `a` and `b`.
         */
        static value_type Distance(const Vector3& a, const Vector3& b) noexcept { return (b - a).LengthSqrt(); }
		
        /**
         * @brief Calculates the distance between this vector and another vector.
         * 
         * @param other The other vector to calculate the distance to.
         * @return value_type The distance between the two vectors.
         */
        value_type Distance(const Vector3& other) const noexcept { return Vector3::Distance(*this, other); }

        /**
         * @brief Normalizes the vector to have a length of 1.
         *
         * This function computes the normalized vector by dividing each component
         * of the vector by its length. If the length of the vector is zero, it
         * returns a zero vector to avoid division by zero.
         *
         * @return A normalized vector with a length of 1, or a zero vector if the
         *         original vector has a length of zero.
         */
        Vector3 Normalize() const noexcept {
            value_type length = LengthSqrt();
            if (length == 0)
                return Vector3::zero;
            return { m_x / length, m_y / length, m_z / length };
        }
        
        /**
         * @brief Normalizes the vector to have a length of 1.
         *
         * This function modifies the vector in place to make its length equal to 1.
         * If the vector's length is zero, the vector components are set to zero.
         *
         * @return A reference to the normalized vector.
         */
        Vector3& Normalize() noexcept {
            value_type length = LengthSqrt();
            if (length != 0) {
                m_x = 0;
                m_y = 0;
                m_z = 0;
            } else {
                m_x /= length;
                m_y /= length;
                m_z /= length;
            }

            return *this;
        }        

        /**
         * @brief Computes the dot product of two 3D vectors.
         * 
         * @param a The first vector.
         * @param b The second vector.
         * @return value_type The dot product of vectors a and b.
         */
        constexpr static value_type Dot(const Vector3& a, const Vector3& b) noexcept {
            return a.m_x * b.m_x + a.m_y * b.m_y + a.m_z * b.m_z;
        }

        /**
         * @brief Computes the dot product of this vector and another vector.
         * 
         * @param other The other vector to compute the dot product with.
         * @return value_type The dot product of the two vectors.
         */
        constexpr value_type Dot(const Vector3& other) const noexcept {
            return Vector3::Dot(*this, other);
        }

        /**
         * @brief Projects a vector onto another vector.
         *
         * This function projects the given vector onto the specified vector (projectOn).
         * If the magnitude of the projectOn vector is too small (close to zero), 
         * the function returns a zero vector to avoid division by zero.
         *
         * @param vector The vector to be projected.
         * @param projectOn The vector onto which the projection is performed.
         * @return The projected vector.
         */
        constexpr static Vector3 Project(const Vector3& vector, const Vector3& projectOn) noexcept {
            auto projectProduct = projectOn.Dot(projectOn);
            if (projectProduct < std::numeric_limits<value_type>::epsilon())
                return Vector3::zero;

            auto product = vector.Dot(projectOn);
            return {
                projectOn.m_x * product / projectProduct,
                projectOn.m_y * product / projectProduct,
                projectOn.m_z * product / projectProduct
            };
        }

        /**
         * @brief Projects this vector onto another vector.
         * 
         * This function projects the current vector onto the given vector `projectOn`.
         * 
         * @param projectOn The vector onto which the current vector will be projected.
         * @return Vector3 The result of the projection.
         */
        constexpr Vector3 Project(const Vector3& projectOn) noexcept {
            return Vector3::Project(*this, projectOn);
        }

        /**
         * @brief Reflects vector a around vector b.
         *
         * This function calculates the reflection of vector a around vector b using the formula:
         * Reflect(a, b) = a - 2 * (a.Dot(b)) * b
         *
         * @param a The vector to be reflected.
         * @param b The vector around which a is reflected.
         * @return The reflected vector.
         */
        static Vector3 Reflect(const Vector3& a, const Vector3& b) noexcept {
            return a - b * 2.0f * a.Dot(b);
        }

        /**
         * @brief Reflects this vector around the given vector.
         * 
         * This function calculates the reflection of this vector around the specified vector.
         * 
         * @param other The vector to reflect around.
         * @return Vector3 The reflected vector.
         */
        Vector3 Reflect(const Vector3& other) const noexcept {
			return Vector3::Reflect(*this, other);
        }

        /**
         * @brief Calculates the angle between two 3D vectors.
         * 
         * This function computes the angle between two vectors `a` and `b` using the dot product and magnitudes of the vectors.
         * 
         * @param a The first vector.
         * @param b The second vector.
         * @return The angle in radians between the two vectors. If either vector has zero length, the function returns 0.0f.
         */
        constexpr static value_type Angle(const Vector3& a, const Vector3& b) noexcept {
			auto magnitude = a.Length() * b.Length();
			if (magnitude == 0)
                return 0.0f;

            auto product = a.Dot(b);
			return std::acos(product / magnitude);
        }

        /**
         * @brief Calculates the angle between this vector and another vector.
         * 
         * @param other The other vector to calculate the angle with.
         * @return value_type The angle between the two vectors.
         */
        constexpr value_type Angle(const Vector3& other) noexcept {
			return Vector3::Angle(*this, other);
        }

        /**
         * @brief Computes the cross product of two 3D vectors.
         * 
         * This function calculates the cross product of two 3D vectors `a` and `b`.
         * The cross product is a vector that is perpendicular to both `a` and `b`,
         * and it has a magnitude equal to the area of the parallelogram that the vectors span.
         * 
         * @param a The first 3D vector.
         * @param b The second 3D vector.
         * @return Vector3 The resulting 3D vector from the cross product of `a` and `b`.
         */
        constexpr static Vector3 Cross(const Vector3& a, const Vector3& b) noexcept {
            return {
                a.m_y * b.m_z - a.m_z * b.m_y,
                a.m_z * b.m_x - a.m_x * b.m_z,
                a.m_x * b.m_y - a.m_y * b.m_x
            };
        }

        /**
         * @brief Computes the cross product of this vector with another vector.
         * 
         * @param other The vector to compute the cross product with.
         * @return A new Vector3 representing the cross product of this vector and the other vector.
         */
        constexpr Vector3 Cross(const Vector3& other) noexcept {
            return Vector3::Cross(*this, other);
        }

        /**
         * @brief Linearly interpolates between two vectors a and b by a factor t.
         *
         * This function performs a linear interpolation between the vectors a and b,
         * clamping the interpolation factor t between 0.0 and 1.0.
         *
         * @param a The starting vector.
         * @param b The ending vector.
         * @param t The interpolation factor, clamped between 0.0 and 1.0.
         * @return The interpolated vector.
         */
        constexpr static Vector3 Lerp (
            const Vector3& a,
            const Vector3& b,
            value_type t
        ) noexcept {
            return LerpUnclamped(a, b, std::clamp(t, 0.0f, 1.0f));
        }

        /**
         * @brief Linearly interpolates between this vector and another vector.
         *
         * This function performs a linear interpolation between the current vector
         * and the provided vector `other` based on the interpolation factor `t`.
         *
         * @param other The target vector to interpolate towards.
         * @param t The interpolation factor, typically in the range [0, 1].
         *          - When t = 0, the result is the current vector.
         *          - When t = 1, the result is the target vector `other`.
         *          - Values between 0 and 1 will interpolate linearly between the two vectors.
         * @return A new Vector3 that is the result of the linear interpolation.
         */
        constexpr Vector3 Lerp(const Vector3& other, value_type t) noexcept {
            return Vector3::Lerp(*this, other, t);
        }

        /**
         * @brief Performs Spherical Linear Interpolation (Slerp) between two vectors.
         *
         * Slerp is a method of interpolating between two vectors along a great circle path on a sphere.
         * This function interpolates between vectors `a` and `b` by a factor of `t`.
         *
         * @param a The starting vector.
         * @param b The ending vector.
         * @param t The interpolation factor, typically in the range [0, 1].
         * @return The interpolated vector.
         */
        static Vector3 Slerp(const Vector3& a, const Vector3& b, float t) noexcept {
            auto dotProduct = std::clamp(a.Dot(b), -1.0f, 1.0f);
            float theta = std::acos(dotProduct) * std::clamp(t, 0.0f, 1.0f);

            Vector3 relativeVec = (b - a * dotProduct).Normalize();
            return a * std::cos(theta) + relativeVec * std::sin(theta);
        }

        /**
         * @brief Performs spherical linear interpolation (Slerp) between this vector and another vector.
         * 
         * Slerp is a method of interpolating between two vectors along a great circle on a sphere.
         * It is commonly used in computer graphics for smooth transitions between orientations.
         * 
         * @param other The target vector to interpolate towards.
         * @param t The interpolation factor, typically in the range [0, 1], where 0 returns this vector
         * and 1 returns the other vector.
         * @return Vector3 The interpolated vector.
         */
		Vector3 Slerp(const Vector3& other, float t) const noexcept {
			return Vector3::Slerp(*this, other, t);
		}

        /**
         * @brief Linearly interpolates between two vectors without clamping the interpolant.
         *
         * This function performs a linear interpolation between two vectors `a` and `b` 
         * based on the interpolant `t`. The result is not clamped, meaning `t` can be 
         * outside the range [0, 1].
         *
         * @param a The start vector.
         * @param b The end vector.
         * @param t The interpolant value.
         * @return A vector that is the linear interpolation of `a` and `b` based on `t`.
         */
        constexpr static Vector3 LerpUnclamped (
            const Vector3& a,
            const Vector3& b,
            value_type t
        ) noexcept {
            return {
                a.m_x + t * (b.m_x - a.m_x),
                a.m_y + t * (b.m_y - a.m_y),
                a.m_z + t * (b.m_z - a.m_z)
            };
        }

        /**
         * @brief Linearly interpolates between this vector and another vector without clamping.
         * 
         * This function performs a linear interpolation between the current vector and the 
         * specified vector 'other' based on the interpolation factor 't'. The interpolation 
         * is not clamped, meaning that 't' can be outside the range [0, 1].
         * 
         * @param other The target vector to interpolate towards.
         * @param t The interpolation factor. A value of 0 returns the current vector, 
         *          a value of 1 returns the 'other' vector, and values outside the range 
         *          [0, 1] will extrapolate accordingly.
         * @return Vector3 The interpolated vector.
         */
        constexpr Vector3 LerpUnclamped(const Vector3& other, value_type t) noexcept {
            return Vector3::LerpUnclamped(*this, other, t);
        }

        /**
         * @brief Adds two Vector3 objects component-wise.
         * 
         * This operator performs a component-wise addition of two Vector3 objects.
         * 
         * @param other The other Vector3 object to add.
         * @return A new Vector3 object that is the result of the addition.
         */
        constexpr Vector3 operator+(const Vector3& other) const noexcept {
            return { m_x + other.m_x, m_y + other.m_y, m_z + other.m_z };
        }

        /**
         * @brief Subtracts another Vector3 from this Vector3.
         * 
         * This operator performs component-wise subtraction of the given Vector3
         * from this Vector3 and returns the resulting Vector3.
         * 
         * @param other The Vector3 to subtract from this Vector3.
         * @return A new Vector3 resulting from the component-wise subtraction.
         */
        constexpr Vector3 operator-(const Vector3& other) const noexcept {
            return { m_x - other.m_x, m_y - other.m_y, m_z - other.m_z };
        }
        
        /**
         * @brief Multiplies two Vector3 objects component-wise.
         * 
         * This operator performs a component-wise multiplication of the current 
         * Vector3 object with another Vector3 object passed as a parameter.
         * 
         * @param other The Vector3 object to multiply with.
         * @return A new Vector3 object resulting from the component-wise multiplication.
         */
        constexpr Vector3 operator*(const Vector3& other) const noexcept {
            return { m_x * other.m_x, m_y * other.m_y, m_z * other.m_z };
        }

        /**
         * @brief Divides the current Vector3 by another Vector3 element-wise.
         * 
         * This operator performs element-wise division of the current vector by the given vector.
         * Each component (x, y, z) of the current vector is divided by the corresponding component
         * of the other vector.
         * 
         * @param other The Vector3 to divide by.
         * @return A new Vector3 resulting from the element-wise division.
         * @note This operation assumes that the components of the other vector are non-zero.
         */
        constexpr Vector3 operator/(const Vector3& other) const noexcept {
            return { m_x / other.m_x, m_y / other.m_y, m_z / other.m_z };
        }
        
        /**
         * @brief Adds a scalar value to each component of the vector.
         * 
         * This operator overload allows you to add a scalar value to the x, y, and z components
         * of the vector, resulting in a new vector where each component is increased by the scalar.
         * 
         * @param scalar The scalar value to be added to each component of the vector.
         * @return A new Vector3 object with each component increased by the scalar value.
         */
        constexpr Vector3 operator+(value_type scalar) const noexcept {
            return { m_x + scalar, m_y + scalar, m_z + scalar };
        }

        /**
         * @brief Subtracts a scalar value from each component of the vector.
         * 
         * This operator returns a new Vector3 instance where each component
         * (x, y, z) of the original vector is reduced by the given scalar value.
         * 
         * @param scalar The scalar value to subtract from each component of the vector.
         * @return A new Vector3 instance with each component reduced by the scalar value.
         */
        constexpr Vector3 operator-(value_type scalar) const noexcept {
            return { m_x - scalar, m_y - scalar, m_z - scalar };
        }

        /**
         * @brief Multiplies the vector by a scalar value.
         * 
         * This operator overload allows for the multiplication of a Vector3 object
         * by a scalar value, returning a new Vector3 object with each component
         * scaled by the given scalar.
         * 
         * @param scalar The scalar value to multiply with the vector components.
         * @return A new Vector3 object with each component scaled by the scalar value.
         */
        constexpr Vector3 operator*(value_type scalar) const noexcept {
            return { m_x * scalar, m_y * scalar, m_z * scalar };
        }

        /**
         * @brief Divides the vector by a scalar value.
         * 
         * This operator performs element-wise division of the vector components
         * by the given scalar value.
         * 
         * @param scalar The scalar value to divide the vector by.
         * @return A new Vector3 object with each component divided by the scalar.
         * @note This function is marked as noexcept, indicating that it does not throw exceptions.
         */
        constexpr Vector3 operator/(value_type scalar) const noexcept {
            return { m_x / scalar, m_y / scalar, m_z / scalar };
        }
        
        /**
         * @brief Adds the components of another Vector3 to this vector.
         * 
         * This operator performs component-wise addition of the given vector to this vector.
         * 
         * @param other The vector to be added.
         * @return A reference to this vector after addition.
         */
        constexpr Vector3& operator+=(const Vector3& other) noexcept {
            this->m_x += other.m_x;
            this->m_y += other.m_y;
            this->m_z += other.m_z;
            return *this;
        }

        /**
         * @brief Subtracts the components of another Vector3 from this vector.
         *
         * This operator modifies the current vector by subtracting the corresponding
         * components of the given vector from it.
         *
         * @param other The vector to subtract from this vector.
         * @return A reference to the modified vector.
         */
        constexpr Vector3& operator-=(const Vector3& other) noexcept {
            this->m_x -= other.m_x;
            this->m_y -= other.m_y;
            this->m_z -= other.m_z;
            return *this;
        }

        /**
         * @brief Multiplies each component of this vector by the corresponding component of another vector.
         * 
         * This operator performs element-wise multiplication of the vector components.
         * 
         * @param other The vector to multiply with.
         * @return A reference to the resulting vector after multiplication.
         */
        constexpr Vector3& operator*=(const Vector3& other) noexcept {
            this->m_x *= other.m_x;
            this->m_y *= other.m_y;
            this->m_z *= other.m_z;
            return *this;
        }

        /**
         * @brief Divides the components of this vector by the corresponding components of another vector.
         *
         * This operator performs element-wise division of the vector components.
         *
         * @param other The vector to divide by.
         * @return A reference to this vector after the division.
         */
        constexpr Vector3& operator/=(const Vector3& other) noexcept {
            this->m_x /= other.m_x;
            this->m_y /= other.m_y;
            this->m_z /= other.m_z;
            return *this;
        }

        /**
         * @brief Compares this Vector3 object with another for equality.
         * 
         * @param other The other Vector3 object to compare with.
         * @return true if the x, y, and z components of both vectors are equal.
         * @return false otherwise.
         */
        constexpr bool operator==(const Vector3& other) const noexcept {
            return m_x == other.m_x && m_y == other.m_y && m_z == other.m_z;
        }
        
        /**
         * @brief Inequality operator for comparing two Vector3 objects.
         *
         * This operator checks if two Vector3 objects are not equal by
         * negating the result of the equality operator.
         *
         * @param other The other Vector3 object to compare with.
         * @return true if the two Vector3 objects are not equal, false otherwise.
         */
        constexpr bool operator!=(const Vector3& other) const noexcept {
            return !(*this == other);
        }

        /**
         * @brief Compares the length squared of the vector to a scalar value.
         * 
         * This operator checks if the length squared of the vector is equal to the given scalar value.
         * 
         * @param scalar The scalar value to compare with the length squared of the vector.
         * @return true if the length squared of the vector is equal to the scalar value, false otherwise.
         */
        bool operator==(value_type scalar) const noexcept { return this->LengthSqrt() == scalar; }

        /**
         * @brief Inequality operator to compare the vector with a scalar value.
         * 
         * This operator checks if the vector is not equal to the given scalar value.
         * It returns true if the vector is not equal to the scalar, and false otherwise.
         * 
         * @param scalar The scalar value to compare with the vector.
         * @return true if the vector is not equal to the scalar, false otherwise.
         */
        bool operator!=(value_type scalar) const noexcept { return !(*this == scalar); }

        /**
         * @brief Compares the length of this vector with another vector.
         * 
         * This operator compares the lengths of two vectors and returns true if the length
         * of this vector is less than the length of the other vector.
         * 
         * @param other The vector to compare with.
         * @return true if the length of this vector is less than the length of the other vector, false otherwise.
         */
        constexpr bool operator<(const Vector3& other) const noexcept {
            return Length() < other.Length();
        }
        
        /**
         * @brief Checks if this vector is less than or equal to another vector.
         * @param other The vector to compare.
         * @return True if this vector is less than or equal to the other vector, false otherwise.
         */
        constexpr bool operator<=(const Vector3& other) const noexcept {
            return Length() <= other.Length();
        }
        /**
         * @brief Checks if this vector is greater than another vector.
         * @param other The vector to compare.
         * @return True if this vector is greater than the other vector, false otherwise.
         */
        constexpr bool operator>(const Vector3& other) const noexcept {
            return Length() > other.Length();
        }
        
        /**
         * @brief Checks if this vector is greater than or equal to another vector.
         * @param other The vector to compare.
         * @return True if this vector is greater than or equal to the other vector, false otherwise.
         */
        constexpr bool operator>=(const Vector3& other) const noexcept {
            return Length() >= other.Length();
        }

        /**
         * @brief Compares this vector with another vector.
         * @param other The vector to compare.
         * @return -1 if this vector is less than the other vector, 1 if greater, 0 if equal.
         */
        constexpr std::strong_ordering operator<=>(const Vector3& other) const noexcept {
            if (operator<(other)) return std::strong_ordering::less;
            if (operator>(other)) return std::strong_ordering::greater;
            return std::strong_ordering::equal;
        }
    };

    constexpr Vector3 Vector3::zero{0, 0, 0};
    constexpr Vector3 Vector3::one {1, 1, 1};
}