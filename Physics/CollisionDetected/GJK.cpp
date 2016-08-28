#include "GJK.h"
#include "../Settings.h"

namespace PE {

const float GJK::EPS = 5e-4f;

const Vector3* GJK::getSimplex() const
{
    return m_simplex;
}

int GJK::getNSimplex() const
{
    return m_nsimplex;
}

bool GJK::compute(Vector3& resultDir, Shape* shapeA, Shape* shapeB)
{
    int i;
    m_nsimplex = 1;
    m_supportPoint = shapeB->bounds().getCenter() - shapeA->bounds().getCenter();
    if (isNull(m_supportPoint))
        m_supportPoint.set(1.0f, 0.0f, 0.0f);
    support(m_simplex[0], shapeA, shapeB, m_supportPoint);
    for(;;) {
        m_sM = support(m_simplex[m_nsimplex], shapeA, shapeB, - m_supportPoint);
        if ((m_supportPoint.lengthSquared() + m_sM) < EPS) {
            resultDir = m_supportPoint;
            break;
        }
        ++m_nsimplex;
        switch (m_nsimplex) {
            case 4:
                if (nearestPoint(m_supportPoint, m_simplex[0], m_simplex[1], m_simplex[2], m_simplex[3], m_simplex_temp, m_nsimplex)) {
                    //std::swap(m_simplex, m_simplex_temp);
                    //for(i = 0; i < m_nsimplex; ++i)
                    //    m_simplex[i] = m_simplex_temp[i];
                    return true;
                } else {
                    //std::swap(m_simplex, m_simplex_temp);
                    for(i = 0; i < m_nsimplex; ++i)
                        m_simplex[i] = m_simplex_temp[i];
                }
                break;
            case 3:
                if (nearestPoint(m_supportPoint, m_simplex[0], m_simplex[1], m_simplex[2], m_simplex_temp, m_nsimplex)) {
                    //m_simplex[3] = 0.0f;
                    //std::swap(m_simplex, m_simplex_temp);
                    m_nsimplex = 3;
                    //for(i = 0; i < m_nsimplex; ++i)
                    //    m_simplex[i] = m_simplex_temp[i];
                    return true;
                } else {
                    if (m_nsimplex != 3)
                        for(i = 0; i < m_nsimplex; ++i)
                            m_simplex[i] = m_simplex_temp[i];
                }
                break;
            case 2:
                nearestPoint(m_supportPoint, m_simplex[0], m_simplex[1], m_simplex, m_nsimplex);
                break;
            default:
                resultDir = m_simplex[0];
                return false;
        }
    }
    return false;
}

float GJK::sign(float a)
{
    if (a < PE_EPSf)
        return 0.0f;
    if (a > 0.0f)
        return 1.0f;
    return -1.0f;
}

bool GJK::isNull(const Vector2& v)
{
    if (std::fabs(v.x) > PE_EPSf)
        return false;
    if (std::fabs(v.y) > PE_EPSf)
        return false;
    return true;
}

bool GJK::isNull(const Vector3& v)
{
    if (std::fabs(v.x) > PE_EPSf)
        return false;
    if (std::fabs(v.y) > PE_EPSf)
        return false;
    if (std::fabs(v.z) > PE_EPSf)
        return false;
    return true;
}

bool GJK::nearestPoint(Vector3& result, const Vector3& point1, const Vector3& point2,
                       Vector3* simplex, int& n)
{
    Vector3 vec21 = point2 - point1;
    float lS = vec21.lengthSquared();
    if (lS < PE_EPSf * PE_EPSf) {
        result = point1;
        simplex[0] = point1;
        n = 1;
        if (isNull(result))
            return true;
        return false;
    }
    float dpi = - (dot(point1, vec21) / lS);
    if (dpi < 0.0f) {
        result = point1;
        simplex[0] = point1;
        n = 1;
    } else if (dpi > 1.0f) {
        result = point2;
        simplex[0] = point2;
        n = 1;
    } else {
        result = (vec21 * dpi) + point1;
        simplex[0] = point1;
        simplex[1] = point2;
        n = 2;
        if (isNull(result))
            return true;
    }
    return false;
}

bool GJK::nearestPoint(Vector3& result, const Vector3& point1, const Vector3& point2, const Vector3& point3,
                       Vector3* simplex, int& n)
{
    Vector3 vec12 = point2 - point1;
    Vector3 vec23 = point3 - point2;
    Vector3 vec31 = point1 - point3;
    Vector3 dir = cross(vec31, vec12);
    float lengthSquared = dir.lengthSquared();
    if (lengthSquared < PE_EPSf * PE_EPSf) {
        result = point1;
        simplex[0] = point1;
        n = 1;
        if (isNull(result))
            return true;
        return false;
    }
    _nearestPoint_triangle(result, point1, point2, point3, vec12, vec23, vec31, dir, simplex, n);
    if (isNull(result))
        return true;
    return false;
}

bool GJK::nearestPoint(Vector3& result, const Vector3& point1, const Vector3& point2,
                       const Vector3& point3, const Vector3& point4,
                       Vector3* simplex, int& n)
{
    Vector3 vec12 = point2 - point1,
        vec13 = point3 - point1,
        vec14 = point4 - point1, vec23, vec34, vec24;
    Vector3 dir1 = cross(vec12, vec13), // 1, 2, 3
        dir2, dir3, dir4;
    float lengthSquared = dir1.lengthSquared(), set;
    if (lengthSquared < PE_EPSf * PE_EPSf) {
        if (isNull(point2 - point1)) {
            vec14 = - vec14;
            vec34 = point4 - point3;
            _nearestPoint_triangle(result, point1, point3, point4, vec13, vec34, vec14, dir1, simplex, n);
            if (isNull(result))
                return true;
            if (n == 3) {
                simplex[0] = point1;
                simplex[1] = point3;
                simplex[2] = point4;
            }
            return false;
        } else {
            vec14 = - vec14;
            vec24 = point4 - point2;
            _nearestPoint_triangle(result, point1, point2, point4, vec12, vec24, vec14, dir1, simplex, n);
            if (isNull(result))
                return true;
            if (n == 3) {
                simplex[0] = point1;
                simplex[1] = point2;
                simplex[2] = point4;
            }
            return false;
        }
    }
    set = dot(vec14, dir1);
    if (std::fabs(set) < PE_EPSf * PE_EPSf) {
        vec13 = point1 - point3;
        _nearestPoint_triangle(result, point1, point2, point3, vec12, vec23, vec13, dir1, simplex, n);
        if (isNull(result))
            return true;
        return false;
    }
    Vector3 p2, p3;
    if (set > 0.0f)  {
        p2 = point3;
        p3 = point2;
        std::swap(vec12, vec13);
        dir1 = - dir1;
    } else {
        p2 = point2;
        p3 = point3;
    }
    vec23 = p3 - p2;
    vec34 = point4 - p3;
    vec24 = point4 - p2;

    dir2 = cross(vec14, vec12); // 1, 4, 2
    dir3 = cross(vec13, vec14); // 1, 3, 4
    dir4 = cross(vec24, vec23); // 2, 4, 3
    float zn1 = - dot(point1, dir1), zn2 = - dot(point1, dir2),
            zn3 = - dot(point1, dir3), zn4 = - dot(p2, dir4);
    if (zn1 >= 0.0f) {
        if (zn2 >= 0.0f) {
            if (zn3 >= 0.0f) {
                if (zn4 >= 0.0f) {
                    //imposible
                    result = point1;
                    simplex[0] = point1;
                    n = 1;
                } else {
                    vec13 = - vec13;
                    vec14 = - vec14;
                    _nearestPoint_3triangles(result, p2, p3, point1, point4, vec23, vec13, vec12, vec34,
                                             vec14, vec24, dir1, dir2, dir3, simplex, n);
                }
            } else {
                if (zn4 >= 0.0f) {
                    vec13 = - vec13;
                    vec24 = - vec24;
                    _nearestPoint_3triangles(result, p3, point1, p2, point4, vec13, vec12, vec23, vec14,
                                             vec24, vec34, dir1, dir4, dir2, simplex, n);
                } else {
                    vec13 = - vec13;
                    vec14 = - vec14;
                    _nearestPoint_2triangles(result, p2, p3, point1, point4, vec23, vec13, vec12, vec34,
                                             vec14, dir1, dir2, simplex, n);
                }
            }
        } else {
            if (zn3 >= 0.0f) {
                if (zn4 >= 0.0f) {
                    vec13 = - vec13;
                    vec34 = - vec34;
                    _nearestPoint_3triangles(result, point1, p2, p3, point4, vec12, vec23, vec13, vec24,
                                             vec34, vec14, dir1, dir3, dir4, simplex, n);
                } else {
                    vec14 = - vec14;
                    _nearestPoint_2triangles(result, point4, point1, p3, p2, vec14, vec13, vec34, vec12,
                                             vec23, dir3, dir1, simplex, n);
                }
            } else {
                if (zn4 >= 0.0f) {
                    vec13 = - vec13;
                    vec34 = - vec34;
                    _nearestPoint_2triangles(result, point1, p2, p3, point4, vec12, vec23, vec13, vec24, vec34, dir1, dir4, simplex, n);
                } else {
                    vec13 = - vec13;
                    _nearestPoint_triangle(result, point1, p2, p3, vec12, vec23, vec13, dir1, simplex, n);
                    if (n == 3) {
                        simplex[0] = point1;
                        simplex[1] = p2;
                        simplex[2] = p3;
                    }
                }
            }
        }
    } else {
        if (zn2 >= 0.0f) {
            if (zn3 >= 0.0f) {
                if (zn4 >= 0.0f) {
                    vec12 = - vec12;
                    vec24 = - vec24;
                    _nearestPoint_3triangles(result, p2, point1, point4, p3, vec12, vec14, vec24, vec13,
                                             vec34, vec23, dir2, dir3, dir4, simplex, n);
                } else {
                    vec12 = - vec12;
                    vec24 = - vec24;
                    _nearestPoint_2triangles(result, p2, point1, point4, p3, vec12, vec14, vec24, vec13, vec34, dir2, dir3, simplex, n);
                }
            } else {
                if (zn4 >= 0.0f) {
                    vec24 = - vec24;
                    vec12 = - vec12;
                    vec34 = - vec34;
                    vec23 = - vec23;
                    _nearestPoint_2triangles(result, point1, point4, p2, p3, vec14, vec24, vec12, vec34, vec23, dir2, dir4, simplex, n);
                } else {
                    vec24 = - vec24;
                    vec12 = - vec12;
                    _nearestPoint_triangle(result, point1, point4, p2, vec14, vec24, vec12, dir2, simplex, n);
                    if (n == 3) {
                        simplex[0] = point1;
                        simplex[1] = point4;
                        simplex[2] = p2;
                    }
                }
            }
        } else {
            if (zn3 >= 0.0f) {
                if (zn4 >= 0.0f) {
                    vec14 = - vec14;
                    vec23 = - vec23;
                    _nearestPoint_2triangles(result, point1, p3, point4, p2, vec13, vec34, vec14, vec23, vec24, dir3, dir4, simplex, n);
                } else {
                    vec14 = - vec14;
                    _nearestPoint_triangle(result, point1, p3, point4, vec13, vec34, vec14, dir3, simplex, n);
                    if (n == 3) {
                        simplex[0] = point1;
                        simplex[1] = p3;
                        simplex[2] = point4;
                    }
                }
            } else {
                if (zn4 >= 0.0f) {
                    vec34 = - vec34;
                    vec23 = - vec23;
                    _nearestPoint_triangle(result, p2, point4, p3, vec24, vec34, vec23, dir4, simplex, n);
                    if (n == 3) {
                        simplex[0] = p2;
                        simplex[1] = point4;
                        simplex[2] = p3;
                    }
                } else {
                    return true;
                }
            }
        }
    }
    return false;
}

float GJK::support(Vector3& supportVertex, const Shape* shapeA, const Shape* shapeB, const Vector3& dir)
{
    Vector3 vA, vB;
    float sM = shapeA->support(vA, dir) + shapeB->support(vB, (-dir));
    supportVertex = vA - vB;
    return sM;
}

void GJK::_nearestPoint_edge(Vector3& result, const Vector3& point1, const Vector3& point2,
                             Vector3& vec12, Vector3* simplex, int& n)
{
    float lS = vec12.lengthSquared();
    float t = - dot(point1, vec12) / lS;
    if (t < 0.0f) {
        result = point1;
        simplex[0] = point1;
        n = 1;
    } else if (t > 1.0f) {
        result = point2;
        simplex[0] = point2;
        n = 1;
    } else {
        result = point1 + (vec12 * t);
        simplex[0] = point1;
        simplex[1] = point2;
        n = 2;
    }
}

void GJK::_nearestPoint_2edges(Vector3& result, const Vector3& point1,
                               const Vector3& point2, const Vector3& point3,
                               Vector3& vec12, Vector3& vec23,
                               Vector3* simplex, int& n)
{
    float lS = vec12.lengthSquared(), t;
    t = - dot(point1, vec12) / lS;
    if (t < 0.0f) {
        result = point1;
        simplex[0] = point1;
        n = 1;
    } else if (t > 1.0f) {
        _nearestPoint_edge(result, point2, point3, vec23, simplex, n);
    } else {
        result = point1 + (vec12 * t);
        simplex[0] = point1;
        simplex[1] = point2;
        n = 2;
    }
}

void GJK::_nearestPoint_triangle(Vector3& result,
                                 const Vector3& point1, const Vector3& point2, const Vector3& point3,
                                 Vector3& vec12, Vector3& vec23, Vector3& vec31, Vector3& dir,
                                 Vector3* simplex, int& n)
{
    float zn1 = dot(point1, cross(dir, vec12)),
          zn2 = dot(point2, cross(dir, vec23)),
          zn3 = dot(point3, cross(dir, vec31));
    if (zn1 <= 0.0f) {
        if (zn2 <= 0.0f) {
            if (zn3 <= 0.0f) {
                zn1 = dot(point1, dir) / dir.lengthSquared();
                result = dir * zn1;
                simplex[0] = point1;
                simplex[1] = point2;
                simplex[2] = point3;
                n = 3;
            } else {
                _nearestPoint_edge(result, point3, point1, vec31, simplex, n);
            }
        } else {
            if (zn3 <= 0.0f) {
                _nearestPoint_edge(result, point2, point3, vec23, simplex, n);
            } else {
                _nearestPoint_2edges(result, point2, point3, point1, vec23, vec31, simplex, n);
            }
        }
    } else {
        if (zn2 <= 0.0f) {
            if (zn3 <= 0.0f) {
                _nearestPoint_edge(result, point1, point2, vec12, simplex, n);
            } else {
                _nearestPoint_2edges(result, point3, point1, point2, vec31, vec12, simplex, n);
            }
        } else {
            if (zn3 <= 0.0f) {
                _nearestPoint_2edges(result, point1, point2, point3, vec12, vec23, simplex, n);
            } else { // imposible
                result = point1;
                simplex[0] = point1;
                n = 1;
            }
        }
    }
}

void GJK::_nearestPoint_2triangles(Vector3& result,
                                   const Vector3& point1, const Vector3& point2, const Vector3& point3, const Vector3& point4,
                                   Vector3& vec12, Vector3& vec23, Vector3& vec31,
                                   Vector3& vec24, Vector3& vec43, Vector3& dir1, Vector3& dir2,
                                   Vector3* simplex, int& n)
{
    float zn1, zn2 = dot(point2, cross(dir1, vec23)), zn3;
    if (zn2 <= 0.0f) {
        zn1 = dot(point1, cross(dir1, vec12));
        zn3 = dot(point3, cross(dir1, vec31));
        if (zn1 <= 0.0f) {
            if (zn3 <= 0.0f) {
                zn1 = dot(point1, dir1) / dir1.lengthSquared();
                result = dir1 * zn1;
                simplex[0] = point1;
                simplex[1] = point2;
                simplex[2] = point3;
                n = 3;
            } else {
                _nearestPoint_2edges(result, point4, point3, point1, vec43, vec31, simplex, n);
            }
        } else {
            if (zn3 <= 0.0f) {
                _nearestPoint_2edges(result, point1, point2, point4, vec12, vec24, simplex, n);
            } else {
                _nearestPoint_2edges(result, point3, point1, point2, vec31, vec12, simplex, n);
            }
        }
    } else {
        // 2, 4, 3
        zn2 = - dot(point3, cross(dir2, vec23));
        float zn1b = dot(point2, cross(dir2, vec24)),
              zn3b = dot(point4, cross(dir2, vec43));
        if (zn2 <= 0.0f) {
            if (zn1b <= 0.0f) {
                if (zn3b <= 0.0f) {
                    zn1 = dot(point2, dir2) / dir2.lengthSquared();
                    result = dir2 * zn1;
                    simplex[0] = point2;
                    simplex[1] = point4;
                    simplex[2] = point3;
                    n = 3;
                } else {
                    _nearestPoint_2edges(result, point4, point3, point1, vec43, vec31, simplex, n);
                }
            } else {
                if (zn3b <= 0.0f) {
                    _nearestPoint_2edges(result, point1, point2, point4, vec12, vec24, simplex, n);
                } else {
                    _nearestPoint_2edges(result, point2, point4, point3, vec24, vec43, simplex, n);
                }
            }
        } else {
            float t = - dot(point2, vec23) / vec23.lengthSquared();
            if ((t >= 0.0f) && (t <= 1.0f)) {
                result = point2 + (vec23 * t);
                simplex[0] = point2;
                simplex[1] = point3;
                n = 2;
            } else {
                if (zn1b <= 0.0f) {
                    if (zn3b <= 0.0f) {
                        if (t < 0.0f) {
                            result = point2;
                            simplex[0] = point2;
                        } else {
                            result = point3;
                            simplex[0] = point3;
                        }
                        n = 1;
                    } else {
                        _nearestPoint_2edges(result, point4, point3, point1, vec43, vec31, simplex, n);
                    }
                } else {
                    if (zn3b <= 0.0f) {
                        _nearestPoint_2edges(result, point1, point2, point4, vec12, vec24, simplex, n);
                    } else { // imposible
                        result = point1;
                        simplex[0] = point1;
                        n = 1;
                    }
                }
            }
        }
    }
}

void GJK::_nearestPoint_2triangles(Vector3& result,
                                   const Vector3& point1, const Vector3& point2, const Vector3& point3, const Vector3& point4,
                                   Vector3& vec12, Vector3& vec23, Vector3& vec24,
                                   Vector3& dir1, Vector3& dir2,
                                   Vector3* simplex, int& n)
{
    float zn1, zn2 = dot(point2, cross(dir1, vec23));
    if (zn2 <= 0.0f) {
        zn1 = dot(point1, cross(dir1, vec12));
        if (zn1 <= 0.0f) {
            zn1 = dot(point1, dir1) / dir1.lengthSquared();
            result = dir1 * zn1;
            simplex[0] = point1;
            simplex[1] = point2;
            simplex[2] = point3;
            n = 3;
        } else {
            _nearestPoint_2edges(result, point1, point2, point4, vec12, vec24, simplex, n);
        }
    } else {
        // 2, 4, 3
        zn2 = - dot(point3, cross(dir2, vec23));
        float zn1b = dot(point2, cross(dir2, vec24));
        if (zn2 <= 0.0f) {
            if (zn1b <= 0.0f) {
                zn1 = dot(point2, dir2) / dir2.lengthSquared();
                result = dir2 * zn1;
                simplex[0] = point2;
                simplex[1] = point4;
                simplex[2] = point3;
                n = 3;
            } else {
                _nearestPoint_2edges(result, point1, point2, point4, vec12, vec24, simplex, n);
            }
        } else {
            float t = - dot(point2, vec23) / vec23.lengthSquared();
            if ((t >= 0.0f) && (t <= 1.0f)) {
                result = point2 + (vec23 * t);
                simplex[0] = point2;
                simplex[1] = point3;
                n = 2;
            } else {
                if (zn1b <= 0.0f) {
                    if (t < 0.0f) {
                        result = point2;
                        simplex[0] = point2;
                    } else {
                        result = point3;
                        simplex[0] = point3;
                    }
                    n = 1;
                } else {
                    _nearestPoint_2edges(result, point1, point2, point4, vec12, vec24, simplex, n);
                }
            }
        }
    }
}

void GJK::_nearestPoint_3triangles(Vector3& result,
                                   const Vector3& point1, const Vector3& point2, const Vector3& point3, const Vector3& point4,
                                   Vector3& vec12, Vector3& vec23, Vector3& vec31, Vector3& vec24, Vector3& vec43, Vector3& vec14,
                                   Vector3& dir1, Vector3& dir2, Vector3& dir3,
                                   Vector3* simplex, int& n)
{
    float zn2, zn3 = dot(point3, cross(dir1, vec31));
    if (zn3 <= 0.0f)
    {
        //_nearestPoint_2triangles(result, point1, point2, point3, point4, vec12, vec23, vec24, dir1, dir2, simplex, n);
        _nearestPoint_2triangles(result, point1, point2, point3, point4, vec12, vec24, vec31, vec24, vec43, dir1, dir2, simplex, n);
    } else {
        zn2 = dot(point2, cross(dir1, vec23));
        if (zn2 <= 0.0f) {
            vec43 = - vec43;
            vec14 = - vec14;
            //_nearestPoint_2triangles(result, point2, point3, point1, point4, vec23, vec31, vec43, dir1, dir3, simplex, n);
            _nearestPoint_2triangles(result, point2, point3, point1, point4, vec23, vec31, vec12, vec43, vec14, dir1, dir3, simplex, n);
        } else {
            vec14 = - vec14;
            vec23 = - vec23;
            vec31 = - vec31;
            //_nearestPoint_2triangles(result, point2, point4, point3, point1, vec24, vec43, vec14, dir2, dir3, simplex, n);
            _nearestPoint_2triangles(result, point2, point4, point3, point1, vec24, vec43, vec23, vec14, vec31, dir2, dir3, simplex, n);
        }
    }
}

}
