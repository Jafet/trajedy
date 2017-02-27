/*
 * Basic 2D vector.
 */
#pragma once
#include <ostream>
#include <utility>

template <class N>
struct v2
{
    N x, y;

    v2() = default;
    v2(const v2&) = default;
    v2(v2&&) = default;
    v2& operator=(const v2&) = default;
    v2(N a): v2(a, a) { }
    v2(N x, N y): x(x), y(y) { }

    template <class N2>
    explicit v2(const v2 <N2>& v): x(v.x), y(v.y) { }

    v2 operator+(const v2& v) const {
        v2 t {*this};
        t += v;
        return t;
    }
    v2& operator+=(const v2& v) {
        x += v.x, y += v.y;
        return *this;
    }
    v2 operator-(const v2& v) const {
        v2 t {*this};
        t -= v;
        return t;
    }
    v2& operator-=(const v2& v) {
        x -= v.x, y -= v.y;
        return *this;
    }
    v2 operator*(const v2& v) const {
        v2 t {*this};
        t *= v;
        return t;
    }
    v2& operator*=(const v2& v) {
        x *= v.x, y *= v.y;
        return *this;
    }
    v2 operator-() const {
        v2 t(*this);
        t = -t;
        return t;
    }
    v2& operator-() {
        x = -x, y = -y;
        return *this;
    }
    bool operator==(const v2& v) const {
        return x == v.x && y == v.y;
    }
    bool operator!=(const v2& v) const {
        return !(*this == v);
    }

    N norm2() const {
        return x*x + y*y;
    }

    void swap(v2& v) {
        std::swap(x, v.x);
        std::swap(y, v.y);
    }

    template <class N2>
    explicit operator v2 <N2> () const {
        return v2 <N2> (x, y);
    }
};

namespace std {
    template <class N>
    void swap(v2 <N>& u, v2 <N>& v) {
        u.swap(v);
    }
}

template <class N>
std::ostream& operator<<(std::ostream& out, const v2 <N>& v) {
    return out << '(' << v.x << ", " << v.y << ')';
}
