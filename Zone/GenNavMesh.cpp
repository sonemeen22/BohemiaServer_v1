#include "GenNavMesh.h"
#include <algorithm>
#include <iostream>

bool NavMeshManager::LoadUnityGeometry(const char* path, InputMesh& out)
{
    std::ifstream in(path, std::ios::binary);
    if (!in)
    {
        std::cerr << "Failed to open geometry file: " << path << std::endl;
        return false;
    }

    int meshCount = 0;
    in.read((char*)&meshCount, sizeof(int));

    for (int m = 0; m < meshCount; ++m)
    {
        int vcount = 0;
        in.read((char*)&vcount, sizeof(int));

        int baseVertex = (int)(out.verts.size() / 3);

        for (int i = 0; i < vcount; ++i)
        {
            float x, y, z;
            in.read((char*)&x, sizeof(float));
            in.read((char*)&y, sizeof(float));
            in.read((char*)&z, sizeof(float));

            out.verts.push_back(x);
            out.verts.push_back(y);
            out.verts.push_back(z);
        }

        int icount = 0;
        in.read((char*)&icount, sizeof(int));

        for (int i = 0; i < icount; ++i)
        {
            int idx;
            in.read((char*)&idx, sizeof(int));
            out.tris.push_back(baseVertex + idx);
        }
    }

    //std::cout << "Loaded vertices: " << out.verts.size() / 3 << std::endl;
    //std::cout << "Loaded triangles: " << out.tris.size() / 3 << std::endl;
    return true;
}

int NavMeshManager::GenNavMesh()
{
    const char* inputPath = "./Data/navmesh_geometry.bin";
    const char* outputPath = "./Data/navmesh.bin";

    InputMesh input;
    if (!LoadUnityGeometry(inputPath, input))
        return -1;

    std::cout << "Loaded vertices: " << input.verts.size() / 3
        << ", triangles: " << input.tris.size() / 3 << std::endl;

    // ---------- 自动缩放几何高度，如果地形高度太低 ----------
    float ymin = input.verts[1]; // y坐标最小值
    float ymax = input.verts[1]; // y坐标最大值
    for (size_t i = 1; i < input.verts.size(); i += 3)
    {
        ymin = std::min(ymin, input.verts[i]);
        ymax = std::max(ymax, input.verts[i]);
    }

    float heightRange = ymax - ymin;
    if (heightRange < 1.0f)
    {
        float scale = 1.0f / std::max(heightRange, 0.01f);
        std::cout << "Scaling geometry y by factor: " << scale << std::endl;
        for (size_t i = 1; i < input.verts.size(); i += 3)
            input.verts[i] = (input.verts[i] - ymin) * scale;
    }

    rcConfig cfg{};
    cfg.cs = 0.2f;
    cfg.ch = 0.2f;

    cfg.walkableSlopeAngle = 45.0f;
    cfg.walkableHeight = 2;   // 使用最小 span 保证通过
    cfg.walkableClimb = 1;    // 最小攀爬
    cfg.walkableRadius = 1;

    cfg.maxEdgeLen = (int)(12.0f / cfg.cs);
    cfg.maxSimplificationError = 1.3f;

    cfg.minRegionArea = 1;   // 尽量保留小区域
    cfg.mergeRegionArea = 5;

    cfg.maxVertsPerPoly = 6;
    cfg.detailSampleDist = 1.0f;
    cfg.detailSampleMaxError = 0.5f;

    rcCalcBounds(input.verts.data(), (int)(input.verts.size() / 3), cfg.bmin, cfg.bmax);
    rcCalcGridSize(cfg.bmin, cfg.bmax, cfg.cs, &cfg.width, &cfg.height);

    rcContext ctx;
    rcHeightfield* solid = rcAllocHeightfield();
    if (!rcCreateHeightfield(&ctx, *solid, cfg.width, cfg.height, cfg.bmin, cfg.bmax, cfg.cs, cfg.ch))
    {
        std::cerr << "Failed to create heightfield" << std::endl;
        return -2;
    }

    std::vector<unsigned char> areas(input.tris.size() / 3, RC_WALKABLE_AREA);
    rcRasterizeTriangles(&ctx, input.verts.data(), (int)(input.verts.size() / 3),
        input.tris.data(), areas.data(), (int)areas.size(), *solid, cfg.walkableClimb);

    std::cout << "Heightfield created: width=" << cfg.width << ", height=" << cfg.height
        << ", spans=" << solid->spans << std::endl;

    rcFilterLowHangingWalkableObstacles(&ctx, cfg.walkableClimb, *solid);
    rcFilterLedgeSpans(&ctx, cfg.walkableHeight, cfg.walkableClimb, *solid);
    rcFilterWalkableLowHeightSpans(&ctx, cfg.walkableHeight, *solid);

    rcCompactHeightfield* chf = rcAllocCompactHeightfield();
    rcBuildCompactHeightfield(&ctx, cfg.walkableHeight, cfg.walkableClimb, *solid, *chf);
    std::cout << "CompactHeightfield spans: " << chf->spanCount << std::endl;

    rcErodeWalkableArea(&ctx, cfg.walkableRadius, *chf);
    rcBuildDistanceField(&ctx, *chf);
    rcBuildRegions(&ctx, *chf, 0, cfg.minRegionArea, cfg.mergeRegionArea);

    rcContourSet* cset = rcAllocContourSet();
    rcBuildContours(&ctx, *chf, cfg.maxSimplificationError, cfg.maxEdgeLen, *cset);

    rcPolyMesh* pmesh = rcAllocPolyMesh();
    rcBuildPolyMesh(&ctx, *cset, cfg.maxVertsPerPoly, *pmesh);
    std::cout << "PolyMesh nverts: " << pmesh->nverts << ", npolys: " << pmesh->npolys << std::endl;

    if (pmesh->npolys == 0)
    {
        std::cerr << "Error: no polygons generated! Check input geometry and cfg parameters." << std::endl;
        return -3;
    }

    rcPolyMeshDetail* dmesh = rcAllocPolyMeshDetail();
    rcBuildPolyMeshDetail(&ctx, *pmesh, *chf, cfg.detailSampleDist, cfg.detailSampleMaxError, *dmesh);

    dtNavMeshCreateParams params{};
    params.verts = pmesh->verts;
    params.vertCount = pmesh->nverts;
    params.polys = pmesh->polys;
    params.polyAreas = pmesh->areas;
    params.polyFlags = pmesh->flags;
    params.polyCount = pmesh->npolys;
    params.nvp = pmesh->nvp;

    params.detailMeshes = dmesh->meshes;
    params.detailVerts = dmesh->verts;
    params.detailVertsCount = dmesh->nverts;
    params.detailTris = dmesh->tris;
    params.detailTriCount = dmesh->ntris;

    params.walkableHeight = 1.0f;
    params.walkableRadius = 0.5f;
    params.walkableClimb = 0.3f;

    rcVcopy(params.bmin, pmesh->bmin);
    rcVcopy(params.bmax, pmesh->bmax);
    params.cs = cfg.cs;
    params.ch = cfg.ch;

    unsigned char* navData = nullptr;
    int navDataSize = 0;
    if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
    {
        std::cerr << "Failed to create navmesh data" << std::endl;
        return -4;
    }

    std::ofstream out(outputPath, std::ios::binary);
    out.write((char*)navData, navDataSize);
    out.close();

    std::cout << "NavMesh built successfully: " << outputPath
        << ", size=" << navDataSize << " bytes" << std::endl;

    dtFree(navData);
    rcFreePolyMeshDetail(dmesh);
    rcFreePolyMesh(pmesh);
    rcFreeContourSet(cset);
    rcFreeCompactHeightfield(chf);
    rcFreeHeightField(solid);

    return 0;
}


