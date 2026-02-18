#include "stl_reader.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <map>
#include <ostream>
#include <set>
#include <utility>

namespace {
std::pair<size_t, size_t> edgeKey(size_t a, size_t b) {
    return {std::min(a, b), std::max(a, b)};
}
} // namespace

bool StlReader::read(const std::string& path) 
{
    triangles_.clear();
    header_.clear();
    std::ifstream f(path, std::ios::binary);
    if (!f) 
        return false;
    char buf[6] = {};
    f.read(buf, 5);
    f.seekg(0);
    bool ascii = (std::strncmp(buf, "solid", 5) == 0);
    if (ascii) 
    {
        std::string line;
        if (!std::getline(f, line))
            return false;
        header_ = line;
        while (std::getline(f, line)) 
        {
            size_t fn = line.find("facet normal");
            if (fn != std::string::npos) 
            {
                Triangle t;
                float x, y, z;
                if (sscanf(line.c_str() + fn, "facet normal %f %f %f", &x, &y, &z) != 3) continue;
                t.normal.x = x; t.normal.y = y; t.normal.z = z;
                std::getline(f, line); // outer loop
                for (Vec3* v : {&t.v0, &t.v1, &t.v2}) 
                {
                    if (!std::getline(f, line))
                        return false;
                    size_t vx = line.find("vertex");
                    if (vx == std::string::npos || sscanf(line.c_str() + vx, "vertex %f %f %f", &v->x, &v->y, &v->z) != 3) return false;
                }
                std::getline(f, line); // endloop
                std::getline(f, line); // endfacet
                triangles_.push_back(t);
            }
        }
        return !triangles_.empty();
    }

    char h[80];
    f.read(h, 80);
    header_.assign(h, 80);
    uint32_t n;
    f.read(reinterpret_cast<char*>(&n), 4);
    if (!f || n > 100000000)
        return false;
    triangles_.resize(n);
    for (uint32_t i = 0; i < n; ++i)
    {
        f.read(reinterpret_cast<char*>(&triangles_[i].normal.x), 12);
        f.read(reinterpret_cast<char*>(&triangles_[i].v0.x), 12);
        f.read(reinterpret_cast<char*>(&triangles_[i].v1.x), 12);
        f.read(reinterpret_cast<char*>(&triangles_[i].v2.x), 12);
        f.ignore(2);
    }
    return !!f;
}

void StlReader::checkRightHandWinding(std::ostream& out) const 
{
    if (originalFacetNormals_.size() != indexedTriangles_.size())
        return;
    
    size_t ok = 0, wrong = 0;
    const float tol = 1e-5f;
    for (size_t i = 0; i < indexedTriangles_.size(); ++i) 
    {
        Triangle t = getTriangle(i);
        float ex = t.v1.x - t.v0.x, ey = t.v1.y - t.v0.y, ez = t.v1.z - t.v0.z;
        float fx = t.v2.x - t.v0.x, fy = t.v2.y - t.v0.y, fz = t.v2.z - t.v0.z;
        float nx = ey * fz - ez * fy, ny = ez * fx - ex * fz, nz = ex * fy - ey * fx;
        float len = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (len <= 0.f)
            continue;
        nx /= len; ny /= len; nz /= len;
        const Vec3& orig = originalFacetNormals_[i];
        float dot = nx * orig.x + ny * orig.y + nz * orig.z;
        if (dot > tol) 
            ++ok;
        else if (dot < -tol) 
        { 
            ++wrong; out << "  triangle " << i << " opposite winding (dot=" << dot << ")\n";
        }
    }
    out << "Right-hand rule: " << ok << " OK, " << wrong << " opposite winding\n";
}

void StlReader::removeDuplicateVertices() 
{
    originalFacetNormals_.resize(triangles_.size());
    for (size_t i = 0; i < triangles_.size(); ++i)
    {
        originalFacetNormals_[i] = triangles_[i].normal;
    }
    vertices_.clear();
    indexedTriangles_.clear();
    std::map<Vec3, size_t> idx;
    auto id = [&](const Vec3& v)
    {
        auto it = idx.find(v);
        if (it != idx.end())
            return it->second;
        size_t i = vertices_.size();
        idx[v] = i;
        vertices_.push_back(v);
        return i;
    };
    for (const Triangle& t : triangles_)
    {
        indexedTriangles_.push_back({ id(t.v0), id(t.v1), id(t.v2) });
    }
    triangles_.clear();
}

StlReader::Triangle StlReader::getTriangle(size_t i) const 
{
    Triangle t;
    const IndexedTri& id = indexedTriangles_.at(i);
    t.v0 = vertices_[id.v0];
    t.v1 = vertices_[id.v1];
    t.v2 = vertices_[id.v2];
    float ex = t.v1.x - t.v0.x, ey = t.v1.y - t.v0.y, ez = t.v1.z - t.v0.z;
    float fx = t.v2.x - t.v0.x, fy = t.v2.y - t.v0.y, fz = t.v2.z - t.v0.z;
    t.normal.x = ey * fz - ez * fy;
    t.normal.y = ez * fx - ex * fz;
    t.normal.z = ex * fy - ey * fx;
    float len = std::sqrt(t.normal.x * t.normal.x + t.normal.y * t.normal.y + t.normal.z * t.normal.z);
    if (len > 0.f)
    { 
        t.normal.x /= len; 
        t.normal.y /= len; 
        t.normal.z /= len;
    }
    return t;
}

double StlReader::volume() const 
{
    double sum = 0;
    for (const IndexedTri& id : indexedTriangles_) 
    {
        const Vec3& a = vertices_[id.v0], b = vertices_[id.v1], c = vertices_[id.v2];
        sum += (a.x * (b.y * c.z - b.z * c.y) + a.y * (b.z * c.x - b.x * c.z) + a.z * (b.x * c.y - b.y * c.x)) / 6.0;
    }
    return sum < 0 ? -sum : sum;
}

bool StlReader::volumeFromFile(const std::string& path, double& outVolume) 
{
    StlReader r;
    if (!r.read(path)) return false;
    r.removeDuplicateVertices();
    outVolume = r.volume();
    return true;
}

void StlReader::writeOneFacet(std::ostream& f, const Triangle& t) 
{
    f << "  facet normal " << t.normal.x << " " << t.normal.y << " " << t.normal.z << "\n";
    f << "    outer loop\n";
    f << "      vertex " << t.v0.x << " " << t.v0.y << " " << t.v0.z << "\n";
    f << "      vertex " << t.v1.x << " " << t.v1.y << " " << t.v1.z << "\n";
    f << "      vertex " << t.v2.x << " " << t.v2.y << " " << t.v2.z << "\n";
    f << "    endloop\n  endfacet\n";
}

bool StlReader::writeAsciiStl(const std::string& path) const 
{
    std::ofstream f(path);
    if (!f)
        return false;
    std::string name = header_;
    while (!name.empty() && (name.back() == '\0' || name.back() == ' '))
        name.pop_back();
    if (name.empty())
        name = "triangles";
    f << "solid " << name << "\n";
    for (size_t i = 0; i < indexedTriangles_.size(); ++i)
        writeOneFacet(f, getTriangle(i));
    f << "endsolid " << name << "\n";
    return !!f;
}

bool StlReader::writeAsciiStl(const std::string& path, const std::vector<size_t>& onlyIndices) const 
{
    std::ofstream f(path);
    if (!f)
        return false;
    f << "solid even_hits\n";
    for (size_t k : onlyIndices)
        if (k < indexedTriangles_.size()) writeOneFacet(f, getTriangle(k));
    f << "endsolid even_hits\n";
    return !!f;
}

void StlReader::addCaps(const std::vector<size_t>& triangleIndices, std::vector<Triangle>& outTriangles) const 
{
    outTriangles.clear();
    for (size_t i : triangleIndices)
        if (i < indexedTriangles_.size())
            outTriangles.push_back(getTriangle(i));

    if (vertices_.empty()) return;

    struct DirectedEdge { size_t from, to, triIdx; };
    std::map<std::pair<size_t, size_t>, std::vector<DirectedEdge>> edgeToDirected;
    for (size_t ti : triangleIndices) 
    {
        if (ti >= indexedTriangles_.size()) continue;
        const IndexedTri& id = indexedTriangles_[ti];
        size_t v0 = id.v0, v1 = id.v1, v2 = id.v2;
        edgeToDirected[edgeKey(v0, v1)].push_back({ v0, v1, ti });
        edgeToDirected[edgeKey(v1, v2)].push_back({ v1, v2, ti });
        edgeToDirected[edgeKey(v2, v0)].push_back({ v2, v0, ti });
    }
    std::vector<DirectedEdge> boundaryEdges;
    for (const auto& kv : edgeToDirected) 
    {
        if (kv.second.size() != 1) continue;
        boundaryEdges.push_back({ kv.second[0].from, kv.second[0].to, kv.second[0].triIdx });
    }
    if (boundaryEdges.empty()) return;

    std::set<std::pair<size_t, size_t>> used;
    std::map<size_t, std::vector<std::pair<size_t, size_t>>> nextMap;
    for (const auto& e : boundaryEdges) 
        nextMap[e.from].push_back({ e.to, e.triIdx });

    auto capOneLoop = [&](const std::vector<size_t>& loop, size_t triIdxForNormal) 
    {
        if (loop.size() < 3) return;
        float cx = 0, cy = 0, cz = 0;
        for (size_t vi : loop) 
        {
            cx += vertices_[vi].x; cy += vertices_[vi].y; cz += vertices_[vi].z;
        }
        float n = (float)loop.size();
        cx /= n; cy /= n; cz /= n;
        Vec3 C = { cx, cy, cz };
        Triangle adjTri = getTriangle(triIdxForNormal);
        for (size_t i = 0; i < loop.size(); ++i) 
        {
            size_t a = loop[i], b = loop[(i + 1) % loop.size()];
            const Vec3& va = vertices_[a], vb = vertices_[b];
            float nx = (va.y - C.y) * (vb.z - C.z) - (va.z - C.z) * (vb.y - C.y);
            float ny = (va.z - C.z) * (vb.x - C.x) - (va.x - C.x) * (vb.z - C.z);
            float nz = (va.x - C.x) * (vb.y - C.y) - (va.y - C.y) * (vb.x - C.x);
            float len = std::sqrt(nx * nx + ny * ny + nz * nz);
            if (len <= 1e-10f) continue;
            nx /= len; ny /= len; nz /= len;
            float dot = nx * adjTri.normal.x + ny * adjTri.normal.y + nz * adjTri.normal.z;
            Triangle cap;
            cap.v0 = C;
            cap.v1 = dot >= 0.f ? va : vb;
            cap.v2 = dot >= 0.f ? vb : va;
            cap.normal.x = nx; cap.normal.y = ny; cap.normal.z = nz;
            if (dot < 0.f) { cap.normal.x = -cap.normal.x; cap.normal.y = -cap.normal.y; cap.normal.z = -cap.normal.z; }
            outTriangles.push_back(cap);
        }
    };

    for (;;) 
    {
        size_t from = 0, to = 0, triIdx = 0;
        bool found = false;
        for (const auto& e : boundaryEdges) 
        {
            if (used.count({ e.from, e.to })) continue;
            from = e.from; to = e.to; triIdx = e.triIdx; found = true;
            break;
        }
        if (!found) break;

        std::vector<size_t> loop;
        std::vector<size_t> triOnLoop;
        size_t start = from;
        loop.push_back(from);
        triOnLoop.push_back(triIdx);
        used.insert({ from, to });
        loop.push_back(to);
        triOnLoop.push_back(triIdx);
        from = to;
        to = loop.back();
        while (to != start) 
        {
            size_t nextV = 0, nextTri = 0;
            for (const auto& p : nextMap[to]) 
            {
                if (used.count({ to, p.first })) continue;
                nextV = p.first; nextTri = p.second; break;
            }
            if (nextV == 0) break;

            auto it = std::find(loop.begin(), loop.end(), nextV);
            if (it == loop.end()) 
            {
                used.insert({ to, nextV });
                loop.push_back(nextV);
                triOnLoop.push_back(nextTri);
                from = to;
                to = nextV;
                continue;
            }
            if (nextV == start) 
            {
                used.insert({ to, nextV });
                break;
            }
            size_t idx = static_cast<size_t>(it - loop.begin());
            std::vector<size_t> subLoop(loop.begin() + idx, loop.end());
            used.insert({ to, nextV });
            capOneLoop(subLoop, triOnLoop[idx]);
            loop.resize(idx + 1);
            triOnLoop.resize(idx + 1);
            to = loop.back();
        }
        capOneLoop(loop, triOnLoop[0]);
    }
}

void StlReader::computeFluidMesh(std::vector<Triangle>& outFluid, std::ostream& cleanMeshOut,
    float originOffset, float tMin, float tEps) const
{
    outFluid.clear();
    std::vector<size_t> evenHitTriangles;
    for (size_t i = 0; i < indexedTriangles_.size(); ++i) {
        Triangle tri = getTriangle(i);
        float cx = (tri.v0.x + tri.v1.x + tri.v2.x) / 3.f;
        float cy = (tri.v0.y + tri.v1.y + tri.v2.y) / 3.f;
        float cz = (tri.v0.z + tri.v1.z + tri.v2.z) / 3.f;
        Vec3 rayOrig = { cx + originOffset * tri.normal.x, cy + originOffset * tri.normal.y, cz + originOffset * tri.normal.z };
        Vec3 rayDir = { tri.normal.x, tri.normal.y, tri.normal.z };
        std::vector<std::pair<float, size_t>> hits;
        for (size_t k = 0; k < indexedTriangles_.size(); ++k) {
            if (k == i) continue;
            float t;
            if (rayIntersect(k, rayOrig, rayDir, t) && t > tMin)
                hits.push_back({ t, k });
        }
        std::sort(hits.begin(), hits.end());
        int distinctHits = 0;
        float lastT = -1e30f;
        for (const auto& h : hits) {
            if (h.first - lastT > tEps) { ++distinctHits; lastT = h.first; }
        }
        if (distinctHits > 0 && (distinctHits & 1) == 0)
            evenHitTriangles.push_back(i);
    }
    addCaps(evenHitTriangles, outFluid);
    for (size_t i = evenHitTriangles.size(); i < outFluid.size(); ++i) {
        std::swap(outFluid[i].v1, outFluid[i].v2);
        outFluid[i].normal.x = -outFluid[i].normal.x;
        outFluid[i].normal.y = -outFluid[i].normal.y;
        outFluid[i].normal.z = -outFluid[i].normal.z;
    }
    cleanMesh(outFluid, cleanMeshOut);
}

void StlReader::cleanMesh(std::vector<Triangle>& triangles, std::ostream& out) 
{
    const size_t initialTris = triangles.size();
    if (initialTris == 0) { out << "No triangles.\n"; return; }

    std::vector<Vec3> verts;
    std::map<Vec3, size_t> vToIdx;
    auto getIdx = [&](const Vec3& v) {
        auto it = vToIdx.find(v);
        if (it != vToIdx.end()) return it->second;
        size_t i = verts.size();
        vToIdx[v] = i;
        verts.push_back(v);
        return i;
    };

    std::vector<std::array<size_t, 3>> indexed;
    size_t degenerate = 0;
    for (const Triangle& t : triangles) 
    {
        size_t i = getIdx(t.v0), j = getIdx(t.v1), k = getIdx(t.v2);
        if (i == j || j == k || k == i) { ++degenerate; continue; }
        indexed.push_back({{ i, j, k }});
    }

    const size_t totalVertexRefs = initialTris * 3;

    std::set<std::array<size_t, 3>> seen;
    std::vector<std::array<size_t, 3>> uniqueTris;
    size_t dupTris = 0;
    for (const auto& tri : indexed) 
    {
        std::array<size_t, 3> key = {{ tri[0], tri[1], tri[2] }};
        std::sort(key.begin(), key.end());
        if (!seen.insert(key).second) { ++dupTris; continue; }
        uniqueTris.push_back(tri);
    }

    size_t dupEdges = 0;
    std::map<std::pair<size_t, size_t>, int> edgeCount;
    for (const auto& tri : uniqueTris) 
    {
        edgeCount[edgeKey(tri[0], tri[1])]++;
        edgeCount[edgeKey(tri[1], tri[2])]++;
        edgeCount[edgeKey(tri[2], tri[0])]++;
    }
    for (const auto& kv : edgeCount)
        if (kv.second > 2) ++dupEdges;

    triangles.clear();
    for (const auto& tri : uniqueTris) 
    {
        const Vec3& a = verts[tri[0]], b = verts[tri[1]], c = verts[tri[2]];
        float ex = b.x - a.x, ey = b.y - a.y, ez = b.z - a.z;
        float fx = c.x - a.x, fy = c.y - a.y, fz = c.z - a.z;
        float nx = ey * fz - ez * fy, ny = ez * fx - ex * fz, nz = ex * fy - ey * fx;
        float len = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (len <= 1e-10f) continue;
        nx /= len; ny /= len; nz /= len;
        Triangle t;
        t.v0 = a; t.v1 = b; t.v2 = c;
        t.normal.x = nx; t.normal.y = ny; t.normal.z = nz;
        triangles.push_back(t);
    }

    out << "Clean triangles report:\n";
    out << "  Duplicate triangles removed: " << dupTris << "\n";
    out << "  Vertices: " << totalVertexRefs << " refs -> " << verts.size() << " unique (merged " << (totalVertexRefs - verts.size()) << " duplicate positions)\n";
    out << "  Degenerate triangles removed: " << degenerate << "\n";
    if (dupEdges > 0) out << "  Non-manifold edges (shared by >2 triangles): " << dupEdges << "\n";
    out << "  Triangles before: " << initialTris << "  after: " << triangles.size() << "\n";
}

bool StlReader::writeAsciiStlFromTriangles(const std::string& path, const std::vector<Triangle>& triangles) 
{
    std::ofstream f(path);
    if (!f) return false;
    f << "solid fluid\n";
    for (const Triangle& t : triangles) writeOneFacet(f, t);
    f << "endsolid fluid\n";
    return !!f;
}

bool StlReader::checkWatertight(std::ostream& out) const 
{
    if (indexedTriangles_.empty()) { out << "Watertight: no triangles\n"; return false; }

    const float areaEps = 1e-10f;
    int duplicateTriangles = 0;
    std::set<std::array<size_t, 3>> seen;
    for (const IndexedTri& id : indexedTriangles_) 
    {
        std::array<size_t, 3> key = {{ id.v0, id.v1, id.v2 }};
        std::sort(key.begin(), key.end());
        if (!seen.insert(key).second) ++duplicateTriangles;
    }
    if (duplicateTriangles > 0)
        out << "Duplicate triangles: " << duplicateTriangles << "\n";

    std::map<std::pair<size_t, size_t>, int> edgeCount;
    for (const IndexedTri& id : indexedTriangles_) 
    {
        edgeCount[edgeKey(id.v0, id.v1)]++;
        edgeCount[edgeKey(id.v1, id.v2)]++;
        edgeCount[edgeKey(id.v2, id.v0)]++;
    }
    int boundaryEdges = 0, nonManifoldEdges = 0;
    for (const auto& kv : edgeCount) 
    {
        if (kv.second == 1) ++boundaryEdges;
        else if (kv.second > 2) ++nonManifoldEdges;
    }
    out << "Edges: " << edgeCount.size() << " unique; " << boundaryEdges << " boundary (count=1), " << nonManifoldEdges << " non-manifold (count>2)\n";

    int degenerate = 0;
    for (const IndexedTri& id : indexedTriangles_) 
    {
        const Vec3& a = vertices_[id.v0], b = vertices_[id.v1], c = vertices_[id.v2];
        float ex = b.x - a.x, ey = b.y - a.y, ez = b.z - a.z;
        float fx = c.x - a.x, fy = c.y - a.y, fz = c.z - a.z;
        float nx = ey * fz - ez * fy, ny = ez * fx - ex * fz, nz = ex * fy - ey * fx;
        float area2 = nx * nx + ny * ny + nz * nz;
        if (area2 <= areaEps * areaEps) ++degenerate;
    }
    if (degenerate > 0) out << "Degenerate triangles (zero area): " << degenerate << "\n";

    out << "Vertices: " << vertices_.size() << " unique (from " << indexedTriangles_.size() << " triangles)\n";

    bool watertight = (duplicateTriangles == 0 && boundaryEdges == 0 && nonManifoldEdges == 0 && degenerate == 0);
    out << "Watertight: " << (watertight ? "yes" : "no") << "\n";
    return watertight;
}

bool StlReader::rayIntersect(size_t triIndex, const Vec3& ro, const Vec3& rd, float& t_out) const 
{
    Triangle t = getTriangle(triIndex);
    const Vec3& v0 = t.v0, v1 = t.v1, v2 = t.v2;
    const float eps = 1e-6f;
    float e1x = v1.x - v0.x, e1y = v1.y - v0.y, e1z = v1.z - v0.z;
    float e2x = v2.x - v0.x, e2y = v2.y - v0.y, e2z = v2.z - v0.z;
    float hx = rd.y * e2z - rd.z * e2y, hy = rd.z * e2x - rd.x * e2z, hz = rd.x * e2y - rd.y * e2x;
    float a = e1x * hx + e1y * hy + e1z * hz;
    if (a > -eps && a < eps) 
        return false;
    float f = 1.f / a;
    float sx = ro.x - v0.x, sy = ro.y - v0.y, sz = ro.z - v0.z;
    float u = f * (sx * hx + sy * hy + sz * hz);
    if (u < 0.f || u > 1.f) 
        return false;
    float qx = sy * e1z - sz * e1y, qy = sz * e1x - sx * e1z, qz = sx * e1y - sy * e1x;
    float v = f * (rd.x * qx + rd.y * qy + rd.z * qz);
    if (v < 0.f || u + v > 1.f)
        return false;
    float tt = f * (e2x * qx + e2y * qy + e2z * qz);
    if (tt <= eps)
        return false;
    t_out = tt;
    return true;
}
