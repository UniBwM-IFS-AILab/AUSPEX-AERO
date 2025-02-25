#ifndef VECTOR3D_H
#define VECTOR3D_H

#include <cmath>
#include <iostream>

class Vector3D {
private:
    double x, y, z;

public:
    // Tolerance for floating point comparisons
    constexpr static double EPSILON = 1e-6;

    /// Constructors
    constexpr Vector3D() : x(0.0), y(0.0), z(0.0) {}
    constexpr Vector3D(double x, double y, double z) : x(x), y(y), z(z) {}
    constexpr Vector3D(double x, double y) : x(x), y(y), z(0.0) {}
    constexpr Vector3D(const Vector3D&) = default;

    /// Destructor
    ~Vector3D() = default;

    /// Assignment Operator
    Vector3D& operator=(const Vector3D&) = default;

    /// Operator Overloads
    Vector3D& operator+=(const Vector3D& v) { x += v.x; y += v.y; z += v.z; return *this; }
    Vector3D& operator-=(const Vector3D& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    Vector3D& operator*=(double scalar) { x *= scalar; y *= scalar; z *= scalar; return *this; }
    Vector3D& operator/=(double scalar) { x /= scalar; y /= scalar; z /= scalar; return *this; }

    [[nodiscard]] constexpr Vector3D operator+(const Vector3D& v) const { return {x + v.x, y + v.y, z + v.z}; }
    [[nodiscard]] constexpr Vector3D operator-(const Vector3D& v) const { return {x - v.x, y - v.y, z - v.z}; }
    [[nodiscard]] constexpr Vector3D operator*(double scalar) const { return {x * scalar, y * scalar, z * scalar}; }
    [[nodiscard]] constexpr Vector3D operator/(double scalar) const { return {x / scalar, y / scalar, z / scalar}; }

    [[nodiscard]] constexpr bool operator==(const Vector3D& v) const {
        return std::fabs(x - v.x) < EPSILON && std::fabs(y - v.y) < EPSILON && std::fabs(z - v.z) < EPSILON;
    }

    /// Accessors (Getters)
    [[nodiscard]] constexpr double getX() const { return x; }
    [[nodiscard]] constexpr double getY() const { return y; }
    [[nodiscard]] constexpr double getZ() const { return z; }

    /// Mutable Access (Reference Getters)
    double& refX() { return x; }
    double& refY() { return y; }
    double& refZ() { return z; }

    /// Mutators (Setters)
    void setX(double newX) { x = newX; }
    void setY(double newY) { y = newY; }
    void setZ(double newZ) { z = newZ; }
    void set(double newX, double newY, double newZ) { x = newX; y = newY; z = newZ; }

    /// Vector Operations
    [[nodiscard]] constexpr double dotProduct(const Vector3D& v) const {
        return x * v.x + y * v.y + z * v.z;
    }

    [[nodiscard]] constexpr Vector3D crossProduct(const Vector3D& v) const {
        return { y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x };
    }

    [[nodiscard]] double magnitude() const {
        return std::hypot(x, y, z);  // More numerically stable than sqrt(x² + y² + z²)
    }

    /// Normalize Vector (Modify in-place)
    Vector3D& normalize() {
        double mag = magnitude();
        if (mag > EPSILON) {
            *this /= mag;
        } else {
            *this = {0, 0, 0};  // Set to zero vector if too small
        }
        return *this;
    }

    /// Normalized Copy
    [[nodiscard]] Vector3D normalized() const {
        Vector3D copy = *this;
        return copy.normalize();
    }

    /// Display the Vector
    friend std::ostream& operator<<(std::ostream& os, const Vector3D& v) {
        return os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    }
};

#endif  // VECTOR3D_H
