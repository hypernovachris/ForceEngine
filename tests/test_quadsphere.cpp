#include <iostream>
#include <cassert>
#include <random>
#include <vector>
#include <iomanip>
#include "Terrain/QuadsphereMath.h"
#include "Terrain/QuadsphereLoader.h"

int main() {
    std::cout << "=== Testing Quadsphere Mathematical Foundations (SPHERE.md) ===" << std::endl;

    // 1. Verify Ocean-Corner Cube Rotation Matrix
    glm::dmat3 R = Quadsphere::getCubeRotationMatrix();
    glm::dmat3 I = glm::transpose(R) * R;
    double orthoErr = 0.0;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            double expected = (r == c) ? 1.0 : 0.0;
            orthoErr = std::max(orthoErr, std::abs(I[c][r] - expected));
        }
    }
    std::cout << "R_cube orthonormality error: " << orthoErr << std::endl;
    assert(orthoErr < 1e-12 && "R_cube must be strictly orthonormal!");
    std::cout << "[PASS] R_cube orthonormality verified." << std::endl;

    // 2. Verify 2D Morton encoding roundtrip
    for (uint32_t x : {0u, 1u, 127u, 1024u, 32767u, 1048575u}) {
        for (uint32_t y : {0u, 1u, 128u, 2048u, 65535u, 2097151u}) {
            uint64_t m = Quadsphere::morton2D(x, y);
            uint32_t rx = 0, ry = 0;
            Quadsphere::morton2D_decode(m, rx, ry);
            assert(rx == x && ry == y && "Morton 2D encode/decode mismatch!");
        }
    }
    std::cout << "[PASS] Morton 2D bit manipulation verified." << std::endl;

    // 3. Verify Tile Key packing/unpacking
    for (uint8_t f = 0; f < 6; ++f) {
        for (uint16_t l = 0; l <= 20; ++l) {
            uint32_t tx = (1u << (l > 15 ? 15 : l)) - 1;
            uint32_t ty = tx / 2;
            uint64_t key = Quadsphere::packTileKey(f, l, tx, ty);
            uint8_t rf; uint16_t rl; uint32_t rx, ry;
            Quadsphere::unpackTileKey(key, rf, rl, rx, ry);
            assert(rf == f && rl == l && rx == tx && ry == ty && "TileKey unpack mismatch!");
        }
    }
    std::cout << "[PASS] 64-bit Tile Key packing verified." << std::endl;

    // 4. Test 10,000 Random Sphere Points Roundtrip
    std::mt19937_64 rng(1337);
    std::uniform_real_distribution<double> distAngle(0.0, 2.0 * M_PI);
    std::uniform_real_distribution<double> distZ(-1.0, 1.0);
    std::uniform_real_distribution<double> distH(-1000.0, 9000.0);

    double maxRandomErr = 0.0;
    const int numRandom = 10000;

    for (int i = 0; i < numRandom; ++i) {
        double theta = distAngle(rng);
        double z = distZ(rng);
        double rPlane = std::sqrt(std::max(0.0, 1.0 - z * z));
        double x = rPlane * std::cos(theta);
        double y = rPlane * std::sin(theta);
        double h = distH(rng);

        glm::dvec3 n(x, y, z);
        glm::dvec3 originalP = (Quadsphere::EARTH_RADIUS + h) * n;

        uint8_t face; double u, v, h_out;
        Quadsphere::ecefToQuadsphere(originalP, face, u, v, h_out);

        glm::dvec3 reconstructedP = Quadsphere::quadsphereToECEF(face, u, v, h_out);

        double err = glm::length(originalP - reconstructedP);
        maxRandomErr = std::max(maxRandomErr, err);
        assert(err < 1e-4 && "ECEF <-> Quadsphere random roundtrip exceeded 1e-4 meters!");
    }
    std::cout << "Max roundtrip error across " << numRandom << " random points: "
              << maxRandomErr << " meters" << std::endl;
    std::cout << "[PASS] Random sphere point roundtrip verified (< 1e-4m)." << std::endl;

    // 5. Test All 12 Cube Edges Roundtrip
    // Edges are at u = +/-1 or v = +/-1 on all 6 faces
    double maxEdgeErr = 0.0;
    for (uint8_t f = 0; f < 6; ++f) {
        for (int edge = 0; edge < 4; ++edge) {
            for (double t = -1.0; t <= 1.0; t += 0.05) {
                double u = 0.0, v = 0.0;
                if (edge == 0) { u = -1.0; v = t; }
                else if (edge == 1) { u = 1.0; v = t; }
                else if (edge == 2) { u = t; v = -1.0; }
                else { u = t; v = 1.0; }

                glm::dvec3 P1 = Quadsphere::quadsphereToECEF(f, u, v, 250.0);

                uint8_t f2; double u2, v2, h2;
                Quadsphere::ecefToQuadsphere(P1, f2, u2, v2, h2);

                glm::dvec3 P2 = Quadsphere::quadsphereToECEF(f2, u2, v2, h2);
                double err = glm::length(P1 - P2);
                maxEdgeErr = std::max(maxEdgeErr, err);
                assert(err < 1e-4 && "Cube edge roundtrip error exceeded 1e-4 meters!");
            }
        }
    }
    std::cout << "Max roundtrip error across all 12 cube edges: " << maxEdgeErr << " meters" << std::endl;
    std::cout << "[PASS] All 12 cube edges roundtrip verified (< 1e-4m)." << std::endl;

    // 6. Test Geodetic (Lat, Lon) conversion for Somerset / NJ coords
    double testLat = 40.592676, testLon = -74.515211;
    uint8_t njFace; double njU, njV;
    Quadsphere::geodeticToQuadsphere(testLat, testLon, njFace, njU, njV);
    double revLat, revLon;
    Quadsphere::quadsphereToGeodetic(njFace, njU, njV, revLat, revLon);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Somerset GPS (" << testLat << ", " << testLon << ") -> Face "
              << (int)njFace << " (u=" << njU << ", v=" << njV << ") -> ("
              << revLat << ", " << revLon << ")" << std::endl;
    assert(std::abs(testLat - revLat) < 1e-6 && std::abs(testLon - revLon) < 1e-6);
    std::cout << "[PASS] Geodetic GPS roundtrip verified." << std::endl;

    // 7. Test 5mm Quantization + Paeth + ZigZag + Byte-Shuffle + Zstd Roundtrip
    std::vector<float> originalElevations(129 * 129);
    for (int y = 0; y < 129; ++y) {
        for (int x = 0; x < 129; ++x) {
            // Synthetic smooth hills with subtle noise
            originalElevations[y * 129 + x] = 45.0f + 12.0f * std::sin(x * 0.05f) + 8.0f * std::cos(y * 0.04f);
        }
    }

    std::vector<uint8_t> compressedBlob;
    int32_t quantizedMinZ = 0;
    bool compOk = QuadsphereLoader::compressTilePayload(originalElevations.data(), compressedBlob, quantizedMinZ);
    assert(compOk && "compressTilePayload failed!");
    std::cout << "Original tile float size: " << (129 * 129 * 4) << " bytes"
              << ", Compressed Zstd size: " << compressedBlob.size() << " bytes (ratio: "
              << (float)(129 * 129 * 4) / compressedBlob.size() << "x)" << std::endl;

    std::vector<uint16_t> decompressedHeights;
    bool decompOk = QuadsphereLoader::decompressTilePayload(compressedBlob.data(), compressedBlob.size(), quantizedMinZ, decompressedHeights);
    assert(decompOk && "decompressTilePayload failed!");
    assert(decompressedHeights.size() == 129 * 129 && "Decompressed size mismatch!");

    float maxElevationError = 0.0f;
    for (size_t i = 0; i < originalElevations.size(); ++i) {
        float reconstructed = (float)quantizedMinZ * 0.005f + (float)decompressedHeights[i] * 0.005f;
        float diff = std::abs(reconstructed - originalElevations[i]);
        maxElevationError = std::max(maxElevationError, diff);
        // Error must be within 5mm quantization resolution (0.0025m max rounding delta)
        assert(diff <= 0.0026f && "Reconstruction error exceeded 5mm quantization step!");
    }
    std::cout << "Max reconstruction quantization delta: " << maxElevationError << " meters" << std::endl;
    std::cout << "[PASS] 5mm Quantization + Paeth + ZigZag + Byte-Shuffle + Zstd roundtrip verified." << std::endl;

    std::cout << "\nALL QUADSPHERE MATH TESTS PASSED SUCCESSFULLY!" << std::endl;
    return 0;
}
