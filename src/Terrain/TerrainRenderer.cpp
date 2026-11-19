#include "Terrain/TerrainRenderer.h"
#include "Mesh.h"
#include "RHI/RHI_Device.h"
#include <algorithm>
#include <cmath>
#include <iostream>

TerrainRenderer::TerrainRenderer() { m_lodCounts.resize(24, 0); }

TerrainRenderer::~TerrainRenderer() {
  m_gpuTileCache.clear();
  m_visibleTiles.clear();
  m_renderList.clear();
  m_loadQueue.clear();
}

bool TerrainRenderer::init(SDL_GPUTextureFormat colorFormat,
                           SDL_GPUTextureFormat depthFormat) {
  m_shader = std::make_shared<Shader>("assets/shaders/terrain.hlsl",
                                      "assets/shaders/terrain.hlsl");
  if (!m_shader || !m_shader->vertexShader || !m_shader->fragmentShader) {
    std::cerr << "TerrainRenderer: Failed to load terrain shader from "
                 "assets/shaders/terrain.hlsl"
              << std::endl;
    return false;
  }

  SDL_GPUVertexAttribute terrainAttributes[4] = {};
  terrainAttributes[0].location = 0;
  terrainAttributes[0].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3;
  terrainAttributes[0].offset = offsetof(Vertex, position);

  terrainAttributes[1].location = 1;
  terrainAttributes[1].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3;
  terrainAttributes[1].offset = offsetof(Vertex, normal);

  terrainAttributes[2].location = 2;
  terrainAttributes[2].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2;
  terrainAttributes[2].offset = offsetof(Vertex, texCoords);

  terrainAttributes[3].location = 3;
  terrainAttributes[3].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3;
  terrainAttributes[3].offset = offsetof(Vertex, tangent);

  SDL_GPUVertexBufferDescription terrainBinding[1] = {};
  terrainBinding[0].slot = 0;
  terrainBinding[0].pitch = sizeof(Vertex);
  terrainBinding[0].input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX;

  m_pipeline = std::make_shared<RHI_Pipeline>();
  if (!m_pipeline->create(m_shader->vertexShader.get(),
                          m_shader->fragmentShader.get(), terrainAttributes, 4,
                          terrainBinding, 1, colorFormat, depthFormat,
                          true /* depthTest */)) {
    std::cerr << "TerrainRenderer: Failed to create terrain graphics pipeline"
              << std::endl;
    m_pipeline.reset();
    return false;
  }

  buildSharedIndexBuffer();

  // 2. Initialize Quadsphere static grid pipeline & geometry
  SDL_GPUVertexAttribute quadAttributes[3] = {};
  quadAttributes[0].location = 0;
  quadAttributes[0].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2;
  quadAttributes[0].offset = offsetof(QuadsphereGridVertex, uv);

  quadAttributes[1].location = 1;
  quadAttributes[1].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2;
  quadAttributes[1].offset = offsetof(QuadsphereGridVertex, grid);

  quadAttributes[2].location = 2;
  quadAttributes[2].format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT;
  quadAttributes[2].offset = offsetof(QuadsphereGridVertex, isSkirt);

  SDL_GPUVertexBufferDescription quadBinding[1] = {};
  quadBinding[0].slot = 0;
  quadBinding[0].pitch = sizeof(QuadsphereGridVertex);
  quadBinding[0].input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX;

  m_quadShader = std::make_shared<Shader>("assets/shaders/terrain.hlsl",
                                          "assets/shaders/terrain.hlsl");
  if (m_quadShader && m_quadShader->vertexShader &&
      m_quadShader->fragmentShader) {
    m_quadPipeline = std::make_shared<RHI_Pipeline>();
    m_quadPipeline->create(m_quadShader->vertexShader.get(),
                           m_quadShader->fragmentShader.get(), quadAttributes,
                           3, quadBinding, 1, colorFormat, depthFormat,
                           true /* depthTest */);
  }

  buildStaticQuadsphereMesh();

  SDL_GPUSamplerCreateInfo samplerInfo = {};
  samplerInfo.min_filter = SDL_GPU_FILTER_LINEAR;
  samplerInfo.mag_filter = SDL_GPU_FILTER_LINEAR;
  samplerInfo.mipmap_mode = SDL_GPU_SAMPLERMIPMAPMODE_NEAREST;
  samplerInfo.address_mode_u = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
  samplerInfo.address_mode_v = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
  samplerInfo.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
  m_quadSampler =
      SDL_CreateGPUSampler(RHI_Device::getInstance().getDevice(), &samplerInfo);

  return true;
}

void TerrainRenderer::buildSharedIndexBuffer() {
  // 128x128 quads = 98,304 indices + 4 outer skirts (3,072 indices) = 101,376
  // indices
  const uint32_t quadsAcross = 128;
  const uint32_t quadsDown = 128;
  const uint32_t dim = quadsAcross + 1; // 129

  std::vector<uint32_t> indices;
  indices.reserve(quadsAcross * quadsDown * 6 + 4 * quadsAcross * 6);

  // Surface grid (CCW viewed from top +Y)
  for (uint32_t j = 0; j < quadsDown; ++j) {
    for (uint32_t i = 0; i < quadsAcross; ++i) {
      uint32_t v0 = j * dim + i;
      uint32_t v1 = j * dim + (i + 1);
      uint32_t v2 = (j + 1) * dim + i;
      uint32_t v3 = (j + 1) * dim + (i + 1);

      indices.push_back(v0);
      indices.push_back(v1);
      indices.push_back(v2);

      indices.push_back(v1);
      indices.push_back(v3);
      indices.push_back(v2);
    }
  }

  // Outer Skirts: North, South, West, East
  const uint32_t topGridCount = dim * dim; // 16,641
  const uint32_t nBase = topGridCount;
  const uint32_t sBase = nBase + dim;
  const uint32_t wBase = sBase + dim;
  const uint32_t eBase = wBase + dim;

  // 1. North Skirt (j = dim - 1)
  for (uint32_t i = 0; i < quadsAcross; ++i) {
    uint32_t top0 = (dim - 1) * dim + i;
    uint32_t top1 = (dim - 1) * dim + (i + 1);
    uint32_t skirt0 = nBase + i;
    uint32_t skirt1 = nBase + (i + 1);

    indices.push_back(top0);
    indices.push_back(top1);
    indices.push_back(skirt0);

    indices.push_back(top1);
    indices.push_back(skirt1);
    indices.push_back(skirt0);
  }

  // 2. South Skirt (j = 0)
  for (uint32_t i = 0; i < quadsAcross; ++i) {
    uint32_t top0 = i;
    uint32_t top1 = i + 1;
    uint32_t skirt0 = sBase + i;
    uint32_t skirt1 = sBase + (i + 1);

    indices.push_back(top0);
    indices.push_back(skirt0);
    indices.push_back(top1);

    indices.push_back(top1);
    indices.push_back(skirt0);
    indices.push_back(skirt1);
  }

  // 3. West Skirt (i = 0)
  for (uint32_t j = 0; j < quadsDown; ++j) {
    uint32_t top0 = j * dim + 0;
    uint32_t top1 = (j + 1) * dim + 0;
    uint32_t skirt0 = wBase + j;
    uint32_t skirt1 = wBase + (j + 1);

    indices.push_back(top0);
    indices.push_back(skirt0);
    indices.push_back(top1);

    indices.push_back(top1);
    indices.push_back(skirt0);
    indices.push_back(skirt1);
  }

  // 4. East Skirt (i = dim - 1)
  for (uint32_t j = 0; j < quadsDown; ++j) {
    uint32_t top0 = j * dim + (dim - 1);
    uint32_t top1 = (j + 1) * dim + (dim - 1);
    uint32_t skirt0 = eBase + j;
    uint32_t skirt1 = eBase + (j + 1);

    indices.push_back(top0);
    indices.push_back(top1);
    indices.push_back(skirt0);

    indices.push_back(top1);
    indices.push_back(skirt1);
    indices.push_back(skirt0);
  }

  m_sharedIndexCount = static_cast<uint32_t>(indices.size());
  Uint32 totalBytes = m_sharedIndexCount * sizeof(uint32_t);

  m_sharedIndexBuffer.create(SDL_GPU_BUFFERUSAGE_INDEX, totalBytes);
  m_sharedIndexBuffer.uploadData(indices.data(), totalBytes);
}

void TerrainRenderer::buildStaticQuadsphereMesh() {
  const int dim = 131; // 131 x 131 vertices
  std::vector<QuadsphereGridVertex> vertices(dim * dim);

  for (int gy = 0; gy < dim; ++gy) {
    for (int gx = 0; gx < dim; ++gx) {
      int vIdx = gy * dim + gx;
      float isSkirt =
          (gx == 0 || gx == dim - 1 || gy == 0 || gy == dim - 1) ? 1.0f : 0.0f;
      int i = std::clamp(gx - 1, 0, 128);
      int j = std::clamp(gy - 1, 0, 128);

      QuadsphereGridVertex &v = vertices[vIdx];
      v.uv[0] = static_cast<float>(i) / 128.0f;
      v.uv[1] = static_cast<float>(j) / 128.0f;
      v.grid[0] = static_cast<float>(i);
      v.grid[1] = static_cast<float>(j);
      v.isSkirt = isSkirt;
    }
  }

  Uint32 vBytes =
      static_cast<Uint32>(vertices.size() * sizeof(QuadsphereGridVertex));
  m_quadStaticVertexBuffer.create(SDL_GPU_BUFFERUSAGE_VERTEX, vBytes);
  m_quadStaticVertexBuffer.uploadData(vertices.data(), vBytes);

  // 130 x 130 quads = 101,400 indices
  const uint32_t quadsAcross = 130;
  const uint32_t quadsDown = 130;
  std::vector<uint32_t> indices;
  indices.reserve(quadsAcross * quadsDown * 6);

  for (uint32_t gy = 0; gy < quadsDown; ++gy) {
    for (uint32_t gx = 0; gx < quadsAcross; ++gx) {
      uint32_t v0 = gy * dim + gx;
      uint32_t v1 = gy * dim + (gx + 1);
      uint32_t v2 = (gy + 1) * dim + gx;
      uint32_t v3 = (gy + 1) * dim + (gx + 1);

      indices.push_back(v0);
      indices.push_back(v1);
      indices.push_back(v2);

      indices.push_back(v1);
      indices.push_back(v3);
      indices.push_back(v2);
    }
  }

  m_quadStaticIndexCount = static_cast<uint32_t>(indices.size());
  Uint32 iBytes = m_quadStaticIndexCount * sizeof(uint32_t);
  m_quadStaticIndexBuffer.create(SDL_GPU_BUFFERUSAGE_INDEX, iBytes);
  m_quadStaticIndexBuffer.uploadData(indices.data(), iBytes);
}

bool TerrainRenderer::loadPack(const std::string &packPath) {
  if (packPath.find(".fquadspherepack") != std::string::npos) {
    if (!QuadsphereLoader::openPack(packPath, m_quadPack)) {
      std::cerr << "TerrainRenderer: Failed to open quadsphere pack "
                << packPath << std::endl;
      return false;
    }
    m_packPath = packPath;
    m_packLoaded = true;
    m_isQuadsphere = true;

    m_quadGpuCache.clear();
    m_visibleTiles.clear();
    m_quadRenderList.clear();
    m_loadQueue.clear();

    std::cout << "TerrainRenderer: Loaded Quadsphere pack " << packPath << " ("
              << m_quadPack.header.tileCount << " tiles, LOD "
              << m_quadPack.header.minLOD << ".." << m_quadPack.header.maxLOD
              << ")" << std::endl;
    return true;
  }

  m_isQuadsphere = false;
  if (!TerrainLoader::openPack(packPath, m_pack)) {
    std::cerr << "TerrainRenderer: Failed to open pack " << packPath
              << std::endl;
    return false;
  }

  m_packPath = packPath;
  m_packLoaded = true;

  // Reset caches
  m_gpuTileCache.clear();
  m_visibleTiles.clear();
  m_renderList.clear();
  m_loadQueue.clear();

  std::cout << "TerrainRenderer: Loaded container " << packPath << " ("
            << m_pack.header.tileCount << " tiles, LOD " << m_pack.header.minLOD
            << ".." << m_pack.header.maxLOD << ", elev ["
            << m_pack.header.globalMinElevation << "m, "
            << m_pack.header.globalMaxElevation << "m])" << std::endl;
  return true;
}

bool TerrainRenderer::hasElevation(float posX, float posZ,
                                   float &outElevation) const {
  if (!m_packLoaded)
    return false;

  if (m_isQuadsphere) {
    outElevation = 30.0f;
    return true;
  }

  float relE = posX - m_pack.startEast;
  float relN = -posZ - m_pack.startNorth;

  float maxE = m_pack.tilesX_LOD0 * m_pack.header.baseTileSize;
  float maxN = m_pack.tilesY_LOD0 * m_pack.header.baseTileSize;

  if (relE < 0.0f || relN < 0.0f || relE > maxE || relN > maxN) {
    return false;
  }

  for (int lod = 0; lod <= (int)m_pack.header.maxLOD; ++lod) {
    float tileSz = m_pack.header.baseTileSize * static_cast<float>(1 << lod);
    if (relE < 0.0f || relN < 0.0f)
      continue;

    uint32_t tx = static_cast<uint32_t>(relE / tileSz);
    uint32_t ty = static_cast<uint32_t>(relN / tileSz);

    const auto *entry = TerrainLoader::findTileIndex(m_pack, lod, tx, ty);
    if (!entry)
      continue;

    DecodedTerrainTile tile;
    if (TerrainLoader::loadTile(m_pack, lod, tx, ty, tile)) {
      stitchTileMegachunkSeams(lod, tx, ty, tile);
      float fracX = (relE - tx * tileSz) / tileSz;
      float fracZ = (relN - ty * tileSz) / tileSz;
      fracX = std::clamp(fracX, 0.0f, 1.0f);
      fracZ = std::clamp(fracZ, 0.0f, 1.0f);

      float fx = fracX * 128.0f;
      float fz = fracZ * 128.0f;
      int ix = std::clamp((int)fx, 0, 127);
      int iz = std::clamp((int)fz, 0, 127);
      float rx = fx - ix;
      float rz = fz - iz;

      float h00 = tile.elevations[iz * 129 + ix];
      float h10 = tile.elevations[iz * 129 + (ix + 1)];
      float h01 = tile.elevations[(iz + 1) * 129 + ix];
      float h11 = tile.elevations[(iz + 1) * 129 + (ix + 1)];

      float h0 = h00 * (1.0f - rx) + h10 * rx;
      float h1 = h01 * (1.0f - rx) + h11 * rx;
      outElevation = h0 * (1.0f - rz) + h1 * rz;
      return true;
    }
  }

  return false;
}

float TerrainRenderer::sampleElevation(float posX, float posZ) const {
  float el = 0.0f;
  if (hasElevation(posX, posZ, el)) {
    return el;
  }
  return 0.0f;
}

bool TerrainRenderer::hasElevationGeodetic(double latDeg, double lonDeg,
                                           float &outElevation) const {
  if (m_isQuadsphere) {
    return QuadsphereLoader::hasElevation(m_quadPack, latDeg, lonDeg,
                                          outElevation);
  }
  if (!m_packLoaded)
    return false;
  glm::dvec3 origin(m_pack.header.ecefAnchor[0], m_pack.header.ecefAnchor[1],
                    m_pack.header.ecefAnchor[2]);
  glm::dvec3 ecef = TerrainLoader::geodeticToECEF(latDeg, lonDeg, 0.0);
  glm::vec3 enu =
      TerrainLoader::ecefToENU(ecef, origin, m_pack.header.enuRotation);
  return hasElevation(enu.x, -enu.y, outElevation);
}

float TerrainRenderer::sampleElevationGeodetic(double latDeg,
                                               double lonDeg) const {
  float el = 0.0f;
  if (hasElevationGeodetic(latDeg, lonDeg, el)) {
    return el;
  }
  return 30.0f;
}

void TerrainRenderer::prewarmDestinationTiles(float posX, float posZ) {
  if (!m_packLoaded)
    return;
  float relE = posX - m_pack.startEast;
  float relN = -posZ - m_pack.startNorth;

  for (int lod = m_pack.header.maxLOD; lod >= 0; --lod) {
    float tileSz = m_pack.header.baseTileSize * static_cast<float>(1 << lod);
    int centerTx = static_cast<int>(relE / tileSz);
    int centerTy = static_cast<int>(relN / tileSz);

    int radius = (lod <= 2) ? 1 : 0;
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        int tx = centerTx + dx;
        int ty = centerTy + dy;
        if (tx < 0 || ty < 0)
          continue;

        const auto *entry = TerrainLoader::findTileIndex(m_pack, lod, tx, ty);
        if (entry) {
          if (m_gpuTileCache.find(entry->tileKey) == m_gpuTileCache.end()) {
            uploadTileMesh(lod, tx, ty, entry->tileKey);
          }
        }
      }
    }
  }
}

void TerrainRenderer::requestTileLoad(uint16_t lod, uint32_t tx, uint32_t ty,
                                      uint64_t key) {
  for (const auto &req : m_loadQueue) {
    if (req.key == key)
      return;
  }
  m_loadQueue.push_back({lod, tx, ty, key});
}

void TerrainRenderer::stitchTileMegachunkSeams(uint16_t lod, uint32_t tx,
                                               uint32_t ty,
                                               DecodedTerrainTile &tile) const {
  const uint32_t dim = 129;
  const int nBlend = 16;
  uint32_t mcTileSpan = 128 >> lod;
  if (mcTileSpan == 0)
    return;

  bool hasW = (tx > 0 && (tx % mcTileSpan == 0));
  bool hasE = ((tx + 1) % mcTileSpan == 0);
  bool hasS = (ty > 0 && (ty % mcTileSpan == 0));
  bool hasN = ((ty + 1) % mcTileSpan == 0);

  if (!hasW && !hasE && !hasS && !hasN)
    return;

  auto cosineWeight = [](int dist, int n) -> float {
    if (dist > n)
      return 0.0f;
    float t = static_cast<float>(dist) / static_cast<float>(n);
    return 0.5f * (1.0f + std::cos(3.14159265358979323846f * t));
  };

  DecodedTerrainTile tileW, tileE, tileS, tileN, tileDiag;
  bool okW = hasW && TerrainLoader::loadTile(m_pack, lod, tx - 1, ty, tileW);
  bool okE = hasE && TerrainLoader::loadTile(m_pack, lod, tx + 1, ty, tileE);
  bool okS = hasS && TerrainLoader::loadTile(m_pack, lod, tx, ty - 1, tileS);
  bool okN = hasN && TerrainLoader::loadTile(m_pack, lod, tx, ty + 1, tileN);

  // Diagonal neighbor for 2D corner harmonization
  int diagDx = 0, diagDy = 0;
  if (hasW)
    diagDx = -1;
  if (hasE)
    diagDx = 1;
  if (hasS)
    diagDy = -1;
  if (hasN)
    diagDy = 1;

  bool okDiag = false;
  if (diagDx != 0 && diagDy != 0) {
    okDiag = TerrainLoader::loadTile(m_pack, lod, tx + diagDx, ty + diagDy,
                                     tileDiag);
  }

  // 1. East/West correction deltaX(y)
  std::vector<float> deltaX(dim, 0.0f);
  int xCol = -1;
  if (okW) {
    xCol = 0;
    for (uint32_t y = 0; y < dim; ++y) {
      float hSelf = tile.getElevation(0, y);
      float hNb = tileW.getElevation(dim - 1, y);
      deltaX[y] = 0.5f * (hNb - hSelf);
    }
  } else if (okE) {
    xCol = dim - 1;
    for (uint32_t y = 0; y < dim; ++y) {
      float hSelf = tile.getElevation(dim - 1, y);
      float hNb = tileE.getElevation(0, y);
      deltaX[y] = 0.5f * (hNb - hSelf);
    }
  }

  // 2. North/South correction deltaY(x)
  std::vector<float> deltaY(dim, 0.0f);
  int yRow = -1;
  if (okS) {
    yRow = 0;
    for (uint32_t x = 0; x < dim; ++x) {
      float hSelf = tile.getElevation(x, 0);
      float hNb = tileS.getElevation(x, dim - 1);
      deltaY[x] = 0.5f * (hNb - hSelf);
    }
  } else if (okN) {
    yRow = dim - 1;
    for (uint32_t x = 0; x < dim; ++x) {
      float hSelf = tile.getElevation(x, dim - 1);
      float hNb = tileN.getElevation(x, 0);
      deltaY[x] = 0.5f * (hNb - hSelf);
    }
  }

  // 3. Corner harmonization: compute 4-way average at junction corner
  float deltaCorner = 0.0f;
  bool hasCorner = (xCol != -1 && yRow != -1);
  if (hasCorner) {
    float h0 = tile.getElevation(xCol, yRow);
    float hX =
        (okW) ? tileW.getElevation(dim - 1, yRow) : tileE.getElevation(0, yRow);
    float hY =
        (okS) ? tileS.getElevation(xCol, dim - 1) : tileN.getElevation(xCol, 0);
    float hD = 0.0f;
    if (okDiag) {
      int dX = (diagDx < 0) ? (dim - 1) : 0;
      int dY = (diagDy < 0) ? (dim - 1) : 0;
      hD = tileDiag.getElevation(dX, dY);
    } else {
      hD = hX + hY - h0;
    }

    float hTargetCorner = 0.25f * (h0 + hX + hY + hD);
    deltaCorner = hTargetCorner - h0;

    float dX_corner = deltaX[yRow];
    float dY_corner = deltaY[xCol];
    float cornerDiffX = deltaCorner - dX_corner;
    float cornerDiffY = deltaCorner - dY_corner;

    for (int k = 0; k <= nBlend; ++k) {
      float w = cosineWeight(k, nBlend);
      int yIdx = (yRow == 0) ? k : (dim - 1 - k);
      deltaX[yIdx] += cornerDiffX * w;

      int xIdx = (xCol == 0) ? k : (dim - 1 - k);
      deltaY[xIdx] += cornerDiffY * w;
    }
  }

  // 4. Coons-patch separable 2D blending
  for (uint32_t y = 0; y < dim; ++y) {
    int distY = (yRow == -1) ? 999 : std::abs(static_cast<int>(y) - yRow);
    float wY = (yRow != -1) ? cosineWeight(distY, nBlend) : 0.0f;

    for (uint32_t x = 0; x < dim; ++x) {
      int distX = (xCol == -1) ? 999 : std::abs(static_cast<int>(x) - xCol);
      float wX = (xCol != -1) ? cosineWeight(distX, nBlend) : 0.0f;

      if (wX == 0.0f && wY == 0.0f)
        continue;

      float corr = 0.0f;
      if (hasCorner) {
        corr = deltaX[y] * wX + deltaY[x] * wY - deltaCorner * (wX * wY);
      } else if (wX > 0.0f) {
        corr = deltaX[y] * wX;
      } else if (wY > 0.0f) {
        corr = deltaY[x] * wY;
      }

      tile.elevations[y * dim + x] += corr;
    }
  }
}

bool TerrainRenderer::uploadTileMesh(uint16_t lod, uint32_t tx, uint32_t ty,
                                     uint64_t key) {
  const auto *entry = TerrainLoader::findTileIndex(m_pack, lod, tx, ty);
  if (!entry) {
    return false;
  }

  DecodedTerrainTile tile;
  if (!TerrainLoader::loadTile(m_pack, lod, tx, ty, tile)) {
    return false;
  }
  stitchTileMegachunkSeams(lod, tx, ty, tile);

  const uint32_t quadsAcross = 128;
  const uint32_t dim = 129;
  float tileSize = m_pack.header.baseTileSize * static_cast<float>(1 << lod);
  float step = tileSize / static_cast<float>(quadsAcross);

  float originE = m_pack.startEast + tx * tileSize;
  float originN = m_pack.startNorth + ty * tileSize;

  const size_t totalVertices = (dim * dim) + (4 * dim);
  std::vector<Vertex> vertices(totalVertices);

  for (uint32_t j = 0; j < dim; ++j) {
    for (uint32_t i = 0; i < dim; ++i) {
      uint32_t vIdx = j * dim + i;
      float worldE = originE + i * step;
      float worldN = originN + j * step;
      float height = tile.elevations[vIdx];

      Vertex &v = vertices[vIdx];
      v.position[0] = worldE;
      v.position[1] = height;
      v.position[2] = -worldN;

      float hL = (i > 0) ? tile.elevations[j * dim + (i - 1)] : height;
      float hR = (i < dim - 1) ? tile.elevations[j * dim + (i + 1)] : height;
      float hD = (j > 0) ? tile.elevations[(j - 1) * dim + i] : height;
      float hU = (j < dim - 1) ? tile.elevations[(j + 1) * dim + i] : height;

      glm::vec3 n = glm::normalize(glm::vec3(-(hR - hL) / (2.0f * step), 1.0f,
                                             -(hU - hD) / (2.0f * step)));
      v.normal[0] = n.x;
      v.normal[1] = n.y;
      v.normal[2] = n.z;

      v.texCoords[0] = static_cast<float>(i) / static_cast<float>(quadsAcross);
      v.texCoords[1] = static_cast<float>(j) / static_cast<float>(quadsAcross);

      v.tangent[0] = static_cast<float>(lod);
      v.tangent[1] = 0.0f;
      v.tangent[2] = 0.0f;
    }
  }

  // Geometry Skirts
  float skirtDepth = std::max(25.0f, tileSize * 0.25f);
  const uint32_t topGridCount = dim * dim;
  const uint32_t nBase = topGridCount;
  const uint32_t sBase = nBase + dim;
  const uint32_t wBase = sBase + dim;
  const uint32_t eBase = wBase + dim;

  // North skirt (j = 128)
  for (uint32_t i = 0; i < dim; ++i) {
    const Vertex &topV = vertices[(dim - 1) * dim + i];
    Vertex &skirtV = vertices[nBase + i];
    skirtV = topV;
    skirtV.position[1] = topV.position[1] - skirtDepth;
  }
  // South skirt (j = 0)
  for (uint32_t i = 0; i < dim; ++i) {
    const Vertex &topV = vertices[i];
    Vertex &skirtV = vertices[sBase + i];
    skirtV = topV;
    skirtV.position[1] = topV.position[1] - skirtDepth;
  }
  // West skirt (i = 0)
  for (uint32_t j = 0; j < dim; ++j) {
    const Vertex &topV = vertices[j * dim + 0];
    Vertex &skirtV = vertices[wBase + j];
    skirtV = topV;
    skirtV.position[1] = topV.position[1] - skirtDepth;
  }
  // East skirt (i = 128)
  for (uint32_t j = 0; j < dim; ++j) {
    const Vertex &topV = vertices[j * dim + (dim - 1)];
    Vertex &skirtV = vertices[eBase + j];
    skirtV = topV;
    skirtV.position[1] = topV.position[1] - skirtDepth;
  }

  auto vb = std::make_unique<RHI_Buffer>();
  Uint32 vSize = static_cast<Uint32>(vertices.size() * sizeof(Vertex));
  if (!vb->create(SDL_GPU_BUFFERUSAGE_VERTEX, vSize))
    return false;
  if (!vb->uploadData(vertices.data(), vSize))
    return false;

  CachedTerrainMesh mesh;
  mesh.vertexBuffer = std::move(vb);
  mesh.tileKey = key;
  mesh.lod = lod;
  mesh.tileX = tx;
  mesh.tileY = ty;
  mesh.lastUsedFrame = m_frameCounter;

  m_gpuTileCache[key] = std::move(mesh);
  return true;
}

bool TerrainRenderer::uploadQuadsphereTile(uint8_t face, uint16_t lod,
                                           uint32_t tx, uint32_t ty,
                                           uint64_t key) {
  DecodedQuadsphereTile decoded;
  if (!QuadsphereLoader::loadTile(m_quadPack, face, lod, tx, ty, decoded)) {
    return false;
  }

  SDL_GPUDevice *device = RHI_Device::getInstance().getDevice();

  SDL_GPUTextureCreateInfo texInfo = {};
  texInfo.type = SDL_GPU_TEXTURETYPE_2D;
  texInfo.format = SDL_GPU_TEXTUREFORMAT_R16_UNORM;
  texInfo.width = 129;
  texInfo.height = 129;
  texInfo.layer_count_or_depth = 1;
  texInfo.num_levels = 1;
  texInfo.sample_count = SDL_GPU_SAMPLECOUNT_1;
  texInfo.usage = SDL_GPU_TEXTUREUSAGE_SAMPLER;

  SDL_GPUTexture *gpuTex = SDL_CreateGPUTexture(device, &texInfo);
  if (!gpuTex) {
    return false;
  }

  Uint32 dataSize =
      static_cast<Uint32>(decoded.heights.size() * sizeof(uint16_t));
  SDL_GPUTransferBufferCreateInfo tbufInfo = {};
  tbufInfo.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
  tbufInfo.size = dataSize;
  SDL_GPUTransferBuffer *tbuf = SDL_CreateGPUTransferBuffer(device, &tbufInfo);

  if (tbuf) {
    void *map = SDL_MapGPUTransferBuffer(device, tbuf, false);
    memcpy(map, decoded.heights.data(), dataSize);
    SDL_UnmapGPUTransferBuffer(device, tbuf);

    SDL_GPUCommandBuffer *cmdBuf = SDL_AcquireGPUCommandBuffer(device);
    SDL_GPUCopyPass *copyPass = SDL_BeginGPUCopyPass(cmdBuf);

    SDL_GPUTextureTransferInfo srcInfo = {};
    srcInfo.transfer_buffer = tbuf;
    srcInfo.offset = 0;

    SDL_GPUTextureRegion dstRegion = {};
    dstRegion.texture = gpuTex;
    dstRegion.w = 129;
    dstRegion.h = 129;
    dstRegion.d = 1;

    SDL_UploadToGPUTexture(copyPass, &srcInfo, &dstRegion, false);
    SDL_EndGPUCopyPass(copyPass);

    SDL_SubmitGPUCommandBuffer(cmdBuf);
    SDL_ReleaseGPUTransferBuffer(device, tbuf);
  }

  CachedQuadsphereTile cached;
  cached.texture = gpuTex;
  cached.tileKey = key;
  cached.face = face;
  cached.lod = lod;
  cached.tx = tx;
  cached.ty = ty;
  cached.tileMinZ = decoded.tileMinZ;
  cached.lastUsedFrame = m_frameCounter;

  m_quadGpuCache[key] = cached;
  return true;
}

void TerrainRenderer::evictOldMeshes() {
  if (m_gpuTileCache.size() <= m_maxCacheTiles)
    return;

  std::vector<uint64_t> candidates;
  for (const auto &pair : m_gpuTileCache) {
    if (pair.second.lastUsedFrame == m_frameCounter)
      continue;

    bool isProtected = false;
    for (const auto &vt : m_visibleTiles) {
      if (vt.tileKey == pair.first) {
        isProtected = true;
        break;
      }
    }
    if (!isProtected) {
      for (const auto &req : m_loadQueue) {
        if (req.key == pair.first) {
          isProtected = true;
          break;
        }
      }
    }
    if (!isProtected) {
      candidates.push_back(pair.first);
    }
  }

  std::sort(candidates.begin(), candidates.end(), [&](uint64_t a, uint64_t b) {
    return m_gpuTileCache[a].lastUsedFrame < m_gpuTileCache[b].lastUsedFrame;
  });

  size_t toRemove = m_gpuTileCache.size() - m_maxCacheTiles;
  for (size_t i = 0; i < std::min(toRemove, candidates.size()); ++i) {
    m_gpuTileCache.erase(candidates[i]);
  }
}

void TerrainRenderer::selectTileRecursive(
    uint16_t lod, uint32_t tx, uint32_t ty, const glm::vec3 &camPos,
    const std::array<FrustumPlane, 6> &planes) {
  const auto *entry = TerrainLoader::findTileIndex(m_pack, lod, tx, ty);
  if (!entry)
    return;

  float tileSize = m_pack.header.baseTileSize * static_cast<float>(1 << lod);
  float minE = m_pack.startEast + tx * tileSize;
  float maxE = minE + tileSize;
  float minN = m_pack.startNorth + ty * tileSize;
  float maxN = minN + tileSize;

  glm::vec3 boxMin(minE, entry->minElevation - 2.0f, -maxN);
  glm::vec3 boxMax(maxE, entry->maxElevation + 2.0f, -minN);

  glm::vec3 closest;
  closest.x = std::clamp(camPos.x, boxMin.x, boxMax.x);
  closest.y = std::clamp(camPos.y, boxMin.y, boxMax.y);
  closest.z = std::clamp(camPos.z, boxMin.z, boxMax.z);
  float dist = glm::distance(camPos, closest);

  if (dist > 20000.0f)
    return;

  if (m_enableFrustumCulling && dist > (tileSize * 1.5f)) {
    float pad = tileSize * 0.15f;
    glm::vec3 paddedMin = boxMin - glm::vec3(pad, pad * 0.5f, pad);
    glm::vec3 paddedMax = boxMax + glm::vec3(pad, pad * 0.5f, pad);
    if (!TerrainLoader::isBoxInFrustum(planes, paddedMin, paddedMax)) {
      return;
    }
  }

  float splitDist = tileSize * m_lodSplitMultiplier;
  bool shouldSplit = (lod > 0) && (dist < splitDist);

  if (shouldSplit) {
    uint16_t childLod = lod - 1;
    uint32_t cX0 = 2 * tx;
    uint32_t cY0 = 2 * ty;

    const auto *e00 = TerrainLoader::findTileIndex(m_pack, childLod, cX0, cY0);
    const auto *e10 =
        TerrainLoader::findTileIndex(m_pack, childLod, cX0 + 1, cY0);
    const auto *e01 =
        TerrainLoader::findTileIndex(m_pack, childLod, cX0, cY0 + 1);
    const auto *e11 =
        TerrainLoader::findTileIndex(m_pack, childLod, cX0 + 1, cY0 + 1);

    // Quadtree rule: Only subdivide if ALL 4 children exist in the container.
    // If any child is missing (e.g. pruned by adaptive pruning or beyond
    // domain), this node is a leaf and renders itself continuously with zero
    // holes.
    if (e00 && e10 && e01 && e11) {
      bool ready00 =
          (m_gpuTileCache.find(e00->tileKey) != m_gpuTileCache.end());
      bool ready10 =
          (m_gpuTileCache.find(e10->tileKey) != m_gpuTileCache.end());
      bool ready01 =
          (m_gpuTileCache.find(e01->tileKey) != m_gpuTileCache.end());
      bool ready11 =
          (m_gpuTileCache.find(e11->tileKey) != m_gpuTileCache.end());

      if (ready00 && ready10 && ready01 && ready11) {
        selectTileRecursive(childLod, cX0, cY0, camPos, planes);
        selectTileRecursive(childLod, cX0 + 1, cY0, camPos, planes);
        selectTileRecursive(childLod, cX0, cY0 + 1, camPos, planes);
        selectTileRecursive(childLod, cX0 + 1, cY0 + 1, camPos, planes);
        return;
      } else {
        if (!ready00)
          requestTileLoad(childLod, cX0, cY0, e00->tileKey);
        if (!ready10)
          requestTileLoad(childLod, cX0 + 1, cY0, e10->tileKey);
        if (!ready01)
          requestTileLoad(childLod, cX0, cY0 + 1, e01->tileKey);
        if (!ready11)
          requestTileLoad(childLod, cX0 + 1, cY0 + 1, e11->tileKey);
      }
    }
  }

  VisibleTerrainTile vt;
  vt.lod = lod;
  vt.tileX = tx;
  vt.tileY = ty;
  vt.tileKey = entry->tileKey;
  vt.distanceToCam = dist;
  vt.boundsMin = boxMin;
  vt.boundsMax = boxMax;

  m_visibleTiles.push_back(vt);
  if (lod < 8)
    m_lodCounts[lod]++;
}

void TerrainRenderer::selectQuadsphereTileRecursive(
    uint8_t face, uint16_t lod, uint32_t tx, uint32_t ty,
    const glm::dvec3 &camPosECEF, const std::array<FrustumPlane, 6> &planes) {
  const auto *entry =
      QuadsphereLoader::findTileIndex(m_quadPack, face, lod, tx, ty);
  if (!entry)
    return;

  double minU, maxU, minV, maxV;
  Quadsphere::getTileUVBounds(lod, tx, ty, minU, maxU, minV, maxV);

  double u_c = 0.5 * (minU + maxU);
  double v_c = 0.5 * (minV + maxV);
  float tileMinZ = entry->quantizedMinZ * 0.005f;
  glm::dvec3 centerECEF =
      Quadsphere::quadsphereToECEF(face, u_c, v_c, tileMinZ);

  double dist = glm::length(centerECEF - camPosECEF);
  if (dist > 35000.0)
    return;

  double span = maxU - minU;
  double tileMetricWidth = span * Quadsphere::EARTH_RADIUS;

  float splitDist = static_cast<float>(tileMetricWidth * m_lodSplitMultiplier);
  bool shouldSplit = (lod < m_quadPack.header.maxLOD) && (dist < splitDist);

  if (shouldSplit) {
    uint16_t childLod = lod + 1;
    uint32_t cX0 = 2 * tx;
    uint32_t cY0 = 2 * ty;

    const QuadsphereTileIndexEntry *children[4] = {
        QuadsphereLoader::findTileIndex(m_quadPack, face, childLod, cX0, cY0),
        QuadsphereLoader::findTileIndex(m_quadPack, face, childLod, cX0 + 1, cY0),
        QuadsphereLoader::findTileIndex(m_quadPack, face, childLod, cX0, cY0 + 1),
        QuadsphereLoader::findTileIndex(m_quadPack, face, childLod, cX0 + 1, cY0 + 1)
    };
    uint32_t childX[4] = {cX0, cX0 + 1, cX0, cX0 + 1};
    uint32_t childY[4] = {cY0, cY0, cY0 + 1, cY0 + 1};

    bool anyChildExists = false;
    bool allExistingReady = true;

    for (int i = 0; i < 4; ++i) {
      if (children[i]) {
        anyChildExists = true;
        if (m_quadGpuCache.find(children[i]->key) == m_quadGpuCache.end()) {
          allExistingReady = false;
          requestTileLoad(childLod, childX[i], childY[i], children[i]->key);
        }
      }
    }

    if (anyChildExists) {
      if (allExistingReady) {
        for (int i = 0; i < 4; ++i) {
          if (children[i]) {
            selectQuadsphereTileRecursive(face, childLod, childX[i], childY[i],
                                          camPosECEF, planes);
          }
        }
        return;
      }
      // If some existing children are still loading to GPU cache,
      // we fall through and render this parent tile until they are ready.
    }
  }

  VisibleTerrainTile vt;
  vt.lod = lod;
  vt.tileX = tx;
  vt.tileY = ty;
  vt.tileKey = entry->key;
  vt.distanceToCam = static_cast<float>(dist);

  m_visibleTiles.push_back(vt);
  if (lod >= m_quadPack.header.minLOD) {
    size_t lodIdx = lod - m_quadPack.header.minLOD;
    if (lodIdx < m_lodCounts.size())
      m_lodCounts[lodIdx]++;
  }
}

void TerrainRenderer::update(const glm::vec3 &camPos,
                             const glm::vec3 &camForward, const glm::mat4 &view,
                             const glm::mat4 &proj, float aspect, float dt,
                             const glm::dvec3 *optCamECEF) {
  if (!m_packLoaded)
    return;

  if (m_isQuadsphere) {
    glm::dvec3 camPosECEF;
    if (optCamECEF) {
      camPosECEF = *optCamECEF;
    } else {
      camPosECEF = Quadsphere::geodeticToECEF(40.592676, -74.515211, 50.0);
    }

    m_frameCounter++;
    m_visibleTiles.clear();
    m_loadQueue.clear();
    for (size_t i = 0; i < m_lodCounts.size(); ++i)
      m_lodCounts[i] = 0;

    glm::mat4 vp = proj * view;
    auto planes = TerrainLoader::extractFrustumPlanes(vp, camPos, camForward,
                                                      0.2f, 35000.0f);

    uint16_t rootLOD = m_quadPack.header.minLOD;
    for (const auto &entry : m_quadPack.indexTable) {
      uint8_t f;
      uint16_t l;
      uint32_t tx, ty;
      Quadsphere::unpackTileKey(entry.key, f, l, tx, ty);
      if (l == rootLOD) {
        selectQuadsphereTileRecursive(f, l, tx, ty, camPosECEF, planes);
      }
    }

    m_quadRenderList.clear();
    int newUploads = 0;
    const int maxUploadsPerFrame = 64;

    for (const auto &vt : m_visibleTiles) {
      auto it = m_quadGpuCache.find(vt.tileKey);
      if (it != m_quadGpuCache.end()) {
        it->second.lastUsedFrame = m_frameCounter;
        m_quadRenderList.push_back(&(it->second));
      } else if (newUploads < maxUploadsPerFrame) {
        uint8_t f;
        uint16_t l;
        uint32_t tx, ty;
        Quadsphere::unpackTileKey(vt.tileKey, f, l, tx, ty);
        if (uploadQuadsphereTile(f, l, tx, ty, vt.tileKey)) {
          newUploads++;
          auto newIt = m_quadGpuCache.find(vt.tileKey);
          if (newIt != m_quadGpuCache.end()) {
            newIt->second.lastUsedFrame = m_frameCounter;
            m_quadRenderList.push_back(&(newIt->second));
          }
        }
      }
    }

    for (const auto &req : m_loadQueue) {
      if (newUploads >= maxUploadsPerFrame)
        break;
      if (m_quadGpuCache.find(req.key) == m_quadGpuCache.end()) {
        uint8_t f;
        uint16_t l;
        uint32_t tx, ty;
        Quadsphere::unpackTileKey(req.key, f, l, tx, ty);
        if (uploadQuadsphereTile(f, l, tx, ty, req.key)) {
          newUploads++;
        }
      }
    }
    return;
  }

  m_frameCounter++;
  m_visibleTiles.clear();
  m_loadQueue.clear();
  for (int i = 0; i < 8; ++i)
    m_lodCounts[i] = 0;

  glm::mat4 vp = proj * view;
  auto planes = TerrainLoader::extractFrustumPlanes(vp, camPos, camForward,
                                                    0.2f, 20000.0f);

  uint16_t rootLOD = m_pack.header.maxLOD;
  float rootTileSz =
      m_pack.header.baseTileSize * static_cast<float>(1 << rootLOD);
  uint32_t rootTilesX = std::max(
      1u, static_cast<uint32_t>(std::ceil(
              (m_pack.tilesX_LOD0 * m_pack.header.baseTileSize) / rootTileSz)));
  uint32_t rootTilesY = std::max(
      1u, static_cast<uint32_t>(std::ceil(
              (m_pack.tilesY_LOD0 * m_pack.header.baseTileSize) / rootTileSz)));

  for (uint32_t rY = 0; rY < rootTilesY; ++rY) {
    for (uint32_t rX = 0; rX < rootTilesX; ++rX) {
      selectTileRecursive(rootLOD, rX, rY, camPos, planes);
    }
  }

  m_renderList.clear();
  int newUploads = 0;
  const int maxUploadsPerFrame = 64;

  for (const auto &vt : m_visibleTiles) {
    auto it = m_gpuTileCache.find(vt.tileKey);
    if (it != m_gpuTileCache.end()) {
      it->second.lastUsedFrame = m_frameCounter;
      m_renderList.push_back(&(it->second));
    } else if (newUploads < maxUploadsPerFrame) {
      if (uploadTileMesh(vt.lod, vt.tileX, vt.tileY, vt.tileKey)) {
        newUploads++;
        auto newIt = m_gpuTileCache.find(vt.tileKey);
        if (newIt != m_gpuTileCache.end()) {
          newIt->second.lastUsedFrame = m_frameCounter;
          m_renderList.push_back(&(newIt->second));
        }
      }
    }
  }

  for (const auto &req : m_loadQueue) {
    if (newUploads >= maxUploadsPerFrame)
      break;
    if (m_gpuTileCache.find(req.key) == m_gpuTileCache.end()) {
      if (uploadTileMesh(req.lod, req.tx, req.ty, req.key)) {
        newUploads++;
      }
    }
  }

  evictOldMeshes();
}

void TerrainRenderer::render(SDL_GPURenderPass *pass,
                             SDL_GPUCommandBuffer *cmdBuf,
                             const glm::mat4 &view, const glm::mat4 &proj,
                             const glm::vec3 &camPos,
                             const glm::dvec3 *optCamECEF) {
  if (m_isQuadsphere) {
    if (!m_quadPipeline || !pass || !cmdBuf || m_quadRenderList.empty())
      return;

    SDL_BindGPUGraphicsPipeline(pass, m_quadPipeline->getPipeline());

    SDL_GPUBufferBinding vb = {m_quadStaticVertexBuffer.getBuffer(), 0};
    SDL_BindGPUVertexBuffers(pass, 0, &vb, 1);

    SDL_GPUBufferBinding ib = {m_quadStaticIndexBuffer.getBuffer(), 0};
    SDL_BindGPUIndexBuffer(pass, &ib, SDL_GPU_INDEXELEMENTSIZE_32BIT);

    glm::dvec3 camPosECEF;
    if (optCamECEF) {
      camPosECEF = *optCamECEF;
    } else {
      camPosECEF = Quadsphere::geodeticToECEF(40.592676, -74.515211, 50.0);
    }

    double camLat, camLon, camAlt;
    Quadsphere::ecefToGeodetic(camPosECEF, camLat, camLon, camAlt);
    glm::mat4 R_ecef_to_enu =
        glm::mat4(Quadsphere::getECEFtoENU(camLat, camLon));

    glm::mat4 viewRot = view * R_ecef_to_enu;
    viewRot[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);

    glm::vec3 sunECEF = glm::normalize(
        glm::vec3(glm::dmat3(Quadsphere::getENUtoECEF(camLat, camLon)) *
                  glm::dvec3(0.42, 0.82, -0.38)));

    for (const auto *tile : m_quadRenderList) {
      if (!tile || !tile->texture)
        continue;

      QuadsphereTerrainUniforms uniforms = {};
      QuadsphereLoader::evaluateControlPatch(tile->face, tile->lod, tile->tx,
                                             tile->ty, camPosECEF, uniforms);
      uniforms.projection = proj;
      uniforms.viewRot = viewRot;

      double minU, maxU, minV, maxV;
      Quadsphere::getTileUVBounds(tile->lod, tile->tx, tile->ty, minU, maxU,
                                  minV, maxV);
      float spanMeters =
          static_cast<float>((maxU - minU) * Quadsphere::EARTH_RADIUS);
      float skirtDepth = std::clamp(spanMeters * 0.04f, 15.0f, 40.0f);

      uniforms.tileParams =
          glm::vec4(tile->tileMinZ, skirtDepth, static_cast<float>(tile->lod),
                    m_lodColorTint ? 1.0f : 0.0f);
      uniforms.sunDir = glm::vec4(sunECEF, 0.0f);

      SDL_PushGPUVertexUniformData(cmdBuf, 0, &uniforms, sizeof(uniforms));
      SDL_PushGPUFragmentUniformData(cmdBuf, 0, &uniforms, sizeof(uniforms));

      SDL_GPUTextureSamplerBinding binding = {tile->texture, m_quadSampler};
      SDL_BindGPUVertexSamplers(pass, 0, &binding, 1);

      SDL_DrawGPUIndexedPrimitives(pass, m_quadStaticIndexCount, 1, 0, 0, 0);
    }
    return;
  }

  if (!m_pipeline || !pass || !cmdBuf || m_renderList.empty())
    return;

  SDL_BindGPUGraphicsPipeline(pass, m_pipeline->getPipeline());

  TerrainUniforms tu = {};
  tu.model = glm::mat4(1.0f);
  tu.view = view;
  tu.projection = proj;
  tu.viewPos = glm::vec4(camPos, 1.0f);
  tu.terrainParams =
      glm::vec4(m_lodColorTint ? 1.0f : 0.0f, m_pack.header.baseTileSize,
                m_earthCurvature ? 1.0f : 0.0f, 0.0f);

  SDL_PushGPUVertexUniformData(cmdBuf, 0, &tu, sizeof(tu));
  SDL_PushGPUFragmentUniformData(cmdBuf, 0, &tu, sizeof(tu));

  SDL_GPUBufferBinding ib = {m_sharedIndexBuffer.getBuffer(), 0};
  SDL_BindGPUIndexBuffer(pass, &ib, SDL_GPU_INDEXELEMENTSIZE_32BIT);

  for (const auto *mesh : m_renderList) {
    if (!mesh || !mesh->vertexBuffer)
      continue;
    SDL_GPUBufferBinding vb = {mesh->vertexBuffer->getBuffer(), 0};
    SDL_BindGPUVertexBuffers(pass, 0, &vb, 1);
    SDL_DrawGPUIndexedPrimitives(pass, m_sharedIndexCount, 1, 0, 0, 0);
  }
}
