#ifndef QUADSPHERE_MATH_H
#define QUADSPHERE_MATH_H

#include <cstdint>
#include <cmath>
#include <algorithm>
#include <glm/glm/glm.hpp>

// ============================================================================
// ForceEngine Quadsphere Mathematics
// Spec: SPHERE.md
// Represents Earth as a Rotated Gnomonic Quadsphere of radius R = 6,371,000.0m
// ============================================================================

namespace Quadsphere {

constexpr double EARTH_RADIUS = 6371000.0; // Mean Earth radius in meters

struct FaceEdgeTransition {
    uint8_t targetFace;
    uint8_t axisSwap; // 0: keep (u,v), 1: swap (v,u)
    int8_t  signU;    // +1 or -1 applied to new u
    int8_t  signV;    // +1 or -1 applied to new v
};

// 24-entry cross-face transition table (SPHERE.md Section 2.2)
// Edge indices: 0 = -u (Left), 1 = +u (Right), 2 = -v (Bottom), 3 = +v (Top)
constexpr FaceEdgeTransition kFaceNeighbors[6][4] = {
    // -u (Left)       +u (Right)      -v (Bottom)     +v (Top)
    { {3, 0, +1, +1}, {1, 0, +1, +1}, {5, 1, +1, -1}, {4, 1, -1, +1} }, // Face 0 (+X)
    { {0, 0, +1, +1}, {2, 0, +1, +1}, {5, 0, +1, +1}, {4, 0, +1, +1} }, // Face 1 (+Y)
    { {1, 0, +1, +1}, {3, 0, +1, +1}, {5, 1, -1, +1}, {4, 1, +1, -1} }, // Face 2 (-X)
    { {2, 0, +1, +1}, {0, 0, +1, +1}, {5, 0, -1, -1}, {4, 0, -1, -1} }, // Face 3 (-Y)
    { {3, 0, -1, -1}, {1, 0, +1, +1}, {0, 1, +1, -1}, {2, 1, -1, +1} }, // Face 4 (+Z)
    { {3, 0, -1, -1}, {1, 0, +1, +1}, {2, 1, +1, -1}, {0, 1, -1, +1} }  // Face 5 (-Z)
};

// Ocean-Corner Cube Rotation Matrix R_cube = Rz(107 deg) * Ry(25 deg)
// SPHERE.md Section 1.2
inline glm::dmat3 getCubeRotationMatrix() {
    constexpr double radZ = 107.0 * (M_PI / 180.0);
    constexpr double radY = 25.0  * (M_PI / 180.0);
    const double cz = std::cos(radZ), sz = std::sin(radZ);
    const double cy = std::cos(radY), sy = std::sin(radY);

    // Column-major glm::dmat3 constructor:
    // Row 0: cz*cy,  -sz,  cz*sy
    // Row 1: sz*cy,   cz,  sz*sy
    // Row 2: -sy,      0,  cy
    return glm::dmat3(
        cz * cy,  sz * cy, -sy,     // Column 0
        -sz,      cz,       0.0,    // Column 1
        cz * sy,  sz * sy,  cy      // Column 2
    );
}

// 1.1 Cube Face Vector c(F, u, v)
inline glm::dvec3 getCubeFaceVector(uint8_t face, double u, double v) {
    switch (face) {
        case 0: return glm::dvec3( 1.0,    u,    v); // Face 0 (+X)
        case 1: return glm::dvec3(  -u,  1.0,    v); // Face 1 (+Y)
        case 2: return glm::dvec3(-1.0,   -u,    v); // Face 2 (-X)
        case 3: return glm::dvec3(   u, -1.0,    v); // Face 3 (-Y)
        case 4: return glm::dvec3(  -v,    u,  1.0); // Face 4 (+Z)
        case 5: return glm::dvec3(   v,    u, -1.0); // Face 5 (-Z)
        default: return glm::dvec3(1.0, 0.0, 0.0);
    }
}

// Forward: Quadsphere (F, u, v, h) -> ECEF (SPHERE.md Section 1.3)
inline glm::dvec3 quadsphereToECEF(uint8_t face, double u, double v, double h = 0.0, double earthRadius = EARTH_RADIUS) {
    glm::dvec3 c = getCubeFaceVector(face, u, v);
    glm::dvec3 d_cube = glm::normalize(c);
    static const glm::dmat3 R_cube = getCubeRotationMatrix();
    glm::dvec3 n_up = R_cube * d_cube;
    return (earthRadius + h) * n_up;
}

// Outward surface normal on unit sphere for (F, u, v)
inline glm::dvec3 quadsphereToNormal(uint8_t face, double u, double v) {
    glm::dvec3 c = getCubeFaceVector(face, u, v);
    glm::dvec3 d_cube = glm::normalize(c);
    static const glm::dmat3 R_cube = getCubeRotationMatrix();
    return R_cube * d_cube;
}

// Inverse: ECEF -> Quadsphere (F, u, v, h) (SPHERE.md Section 1.3)
inline void ecefToQuadsphere(const glm::dvec3& posECEF, uint8_t& outFace, double& outU, double& outV, double& outH, double earthRadius = EARTH_RADIUS) {
    double r = glm::length(posECEF);
    outH = r - earthRadius;
    if (r <= 1e-9) {
        outFace = 0; outU = 0.0; outV = 0.0;
        return;
    }
    glm::dvec3 n_up = posECEF / r;
    static const glm::dmat3 R_cube = getCubeRotationMatrix();
    static const glm::dmat3 R_cube_T = glm::transpose(R_cube);
    glm::dvec3 d_cube = R_cube_T * n_up;

    double ax = std::abs(d_cube.x);
    double ay = std::abs(d_cube.y);
    double az = std::abs(d_cube.z);

    if (ax >= ay && ax >= az) {
        if (d_cube.x > 0.0) {
            outFace = 0;
            outU = d_cube.y / d_cube.x;
            outV = d_cube.z / d_cube.x;
        } else {
            outFace = 2;
            outU = d_cube.y / d_cube.x;
            outV = -d_cube.z / d_cube.x;
        }
    } else if (ay >= ax && ay >= az) {
        if (d_cube.y > 0.0) {
            outFace = 1;
            outU = -d_cube.x / d_cube.y;
            outV = d_cube.z / d_cube.y;
        } else {
            outFace = 3;
            outU = -d_cube.x / d_cube.y;
            outV = -d_cube.z / d_cube.y;
        }
    } else {
        if (d_cube.z > 0.0) {
            outFace = 4;
            outU = d_cube.y / d_cube.z;
            outV = -d_cube.x / d_cube.z;
        } else {
            outFace = 5;
            outU = -d_cube.y / d_cube.z;
            outV = -d_cube.x / d_cube.z;
        }
    }
}

// Geodetic (Lat, Lon, H) -> ECEF (SPHERE.md Section 1.3)
// Inputs: latDeg (-90..+90), lonDeg (-180..+180)
inline glm::dvec3 geodeticToECEF(double latDeg, double lonDeg, double h = 0.0, double earthRadius = EARTH_RADIUS) {
    double phi = latDeg * (M_PI / 180.0);
    double lambda = lonDeg * (M_PI / 180.0);
    double cosPhi = std::cos(phi);
    glm::dvec3 n_up(
        cosPhi * std::cos(lambda),
        cosPhi * std::sin(lambda),
        std::sin(phi)
    );
    return (earthRadius + h) * n_up;
}

// ECEF -> Geodetic (Lat, Lon, H)
inline void ecefToGeodetic(const glm::dvec3& posECEF, double& outLatDeg, double& outLonDeg, double& outH, double earthRadius = EARTH_RADIUS) {
    double r = glm::length(posECEF);
    outH = r - earthRadius;
    if (r <= 1e-9) {
        outLatDeg = 0.0; outLonDeg = 0.0;
        return;
    }
    glm::dvec3 n = posECEF / r;
    double phi = std::asin(std::clamp(n.z, -1.0, 1.0));
    double lambda = std::atan2(n.y, n.x);
    outLatDeg = phi * (180.0 / M_PI);
    outLonDeg = lambda * (180.0 / M_PI);
}

// Quadsphere (F, u, v, h) -> Geodetic (Lat, Lon)
inline void quadsphereToGeodetic(uint8_t face, double u, double v, double& outLatDeg, double& outLonDeg) {
    glm::dvec3 ecef = quadsphereToECEF(face, u, v, 0.0);
    double dummyH;
    ecefToGeodetic(ecef, outLatDeg, outLonDeg, dummyH);
}

// Geodetic (Lat, Lon) -> Quadsphere (F, u, v)
inline void geodeticToQuadsphere(double latDeg, double lonDeg, uint8_t& outFace, double& outU, double& outV) {
    glm::dvec3 ecef = geodeticToECEF(latDeg, lonDeg, 0.0);
    double dummyH;
    ecefToQuadsphere(ecef, outFace, outU, outV, dummyH);
}

// Local East-North-Up (ENU) Tangent Frame Basis at (Lat, Lon)
// Output: R_ecef_to_enu transforms ECEF vectors to local camera ENU (X: East, Y: Up, Z: -North)
inline glm::dmat3 getECEFtoENU(double latDeg, double lonDeg) {
    double phi = latDeg * (M_PI / 180.0);
    double lambda = lonDeg * (M_PI / 180.0);
    double sinPhi = std::sin(phi), cosPhi = std::cos(phi);
    double sinLam = std::sin(lambda), cosLam = std::cos(lambda);

    glm::dvec3 E(-sinLam, cosLam, 0.0);
    glm::dvec3 N(-sinPhi * cosLam, -sinPhi * sinLam, cosPhi);
    glm::dvec3 U(cosPhi * cosLam, cosPhi * sinLam, sinPhi);

    // Row 0: E, Row 1: U, Row 2: -N
    // In glm column-major format:
    return glm::dmat3(
        glm::dvec3(E.x, U.x, -N.x), // Column 0
        glm::dvec3(E.y, U.y, -N.y), // Column 1
        glm::dvec3(E.z, U.z, -N.z)  // Column 2
    );
}

// Inverse rotation: local camera ENU -> ECEF
inline glm::dmat3 getENUtoECEF(double latDeg, double lonDeg) {
    return glm::transpose(getECEFtoENU(latDeg, lonDeg));
}

// 2D Morton Z-Order Packing (up to 26 bits each)
inline uint64_t morton2D(uint32_t x, uint32_t y) {
    auto part1by1 = [](uint32_t n) -> uint64_t {
        uint64_t val = static_cast<uint64_t>(n) & 0x03FFFFFFULL;
        val = (val | (val << 16)) & 0x0000FFFF0000FFFFULL;
        val = (val | (val << 8))  & 0x00FF00FF00FF00FFULL;
        val = (val | (val << 4))  & 0x0F0F0F0F0F0F0F0FULL;
        val = (val | (val << 2))  & 0x3333333333333333ULL;
        val = (val | (val << 1))  & 0x5555555555555555ULL;
        return val;
    };
    return (part1by1(y) << 1) | part1by1(x);
}

inline void morton2D_decode(uint64_t m, uint32_t& outX, uint32_t& outY) {
    auto compact1by1 = [](uint64_t n) -> uint32_t {
        n &= 0x5555555555555555ULL;
        n = (n | (n >> 1))  & 0x3333333333333333ULL;
        n = (n | (n >> 2))  & 0x0F0F0F0F0F0F0F0FULL;
        n = (n | (n >> 4))  & 0x00FF00FF00FF00FFULL;
        n = (n | (n >> 8))  & 0x0000FFFF0000FFFFULL;
        n = (n | (n >> 16)) & 0x0000000003FFFFFFULL;
        return static_cast<uint32_t>(n);
    };
    outX = compact1by1(m);
    outY = compact1by1(m >> 1);
}

// 64-bit Tile Key (SPHERE.md Section 4.2)
// Bits [60..58]: face (0..5), [57..52]: lod (0..63), [51..0]: morton2D(tx, ty)
inline uint64_t packTileKey(uint8_t face, uint16_t lod, uint32_t tx, uint32_t ty) {
    uint64_t f = static_cast<uint64_t>(face & 0x7);
    uint64_t l = static_cast<uint64_t>(lod & 0x3F);
    uint64_t m = morton2D(tx, ty) & 0x000FFFFFFFFFFFFFULL;
    return (f << 58) | (l << 52) | m;
}

inline void unpackTileKey(uint64_t key, uint8_t& outFace, uint16_t& outLod, uint32_t& outTx, uint32_t& outTy) {
    outFace = static_cast<uint8_t>((key >> 58) & 0x7);
    outLod = static_cast<uint16_t>((key >> 52) & 0x3F);
    uint64_t m = key & 0x000FFFFFFFFFFFFFULL;
    morton2D_decode(m, outTx, outTy);
}

// Get (u, v) bounding box of tile (F, L, tx, ty)
inline void getTileUVBounds(uint16_t lod, uint32_t tx, uint32_t ty,
                            double& minU, double& maxU, double& minV, double& maxV) {
    double tilesPerSide = static_cast<double>(1ULL << lod);
    double span = 2.0 / tilesPerSide;
    minU = -1.0 + tx * span;
    maxU = minU + span;
    minV = -1.0 + ty * span;
    maxV = minV + span;
}

} // namespace Quadsphere

#endif // QUADSPHERE_MATH_H
