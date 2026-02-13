#include "stl_reader.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

static const char* simple_stl() {
    static const char* paths[] = { "simple.stl", "tests/simple.stl" };
    std::ifstream f(paths[0]);
    if (f) return paths[0];
    return paths[1];
}

static void test_volume_from_file() {
    const char* path = simple_stl();
    double vol = 0.;
    bool ok = StlReader::volumeFromFile(path, vol);
    assert(ok && "volumeFromFile(simple.stl) should succeed");
    // Tetrahedron (1,0,0),(0,1,0),(0,0,1) from origin -> volume 1/6
    assert(std::fabs(vol - 1.0 / 6.0) < 1e-6 && "volume should be 1/6");
}

static void test_read_triangle_count() {
    StlReader r;
    bool ok = r.read(simple_stl());
    assert(ok && "read(simple.stl) should succeed");
    r.removeDuplicateVertices();
    assert(r.triangleCount() == 1 && "one triangle in simple.stl");
}

static void test_clean_mesh_removes_duplicates() {
    StlReader::Triangle t{};
    t.normal.x = 0; t.normal.y = 0; t.normal.z = 1;
    t.v0.x = 0; t.v0.y = 0; t.v0.z = 0;
    t.v1.x = 1; t.v1.y = 0; t.v1.z = 0;
    t.v2.x = 0; t.v2.y = 1; t.v2.z = 0;
    std::vector<StlReader::Triangle> tris = { t, t };
    std::ostringstream out;
    StlReader::cleanMesh(tris, out);
    assert(tris.size() == 1 && "cleanMesh should remove duplicate triangle");
}

static void test_clean_mesh_removes_degenerate() {
    StlReader::Triangle t{};
    t.normal.x = 0; t.normal.y = 0; t.normal.z = 1;
    t.v0.x = 0; t.v0.y = 0; t.v0.z = 0;
    t.v1.x = 0; t.v1.y = 0; t.v1.z = 0;  // degenerate: v0 == v1
    t.v2.x = 1; t.v2.y = 0; t.v2.z = 0;
    std::vector<StlReader::Triangle> tris = { t };
    std::ostringstream out;
    StlReader::cleanMesh(tris, out);
    assert(tris.empty() && "cleanMesh should remove degenerate triangle");
}

static void test_write_read_roundtrip() {
    StlReader::Triangle t{};
    t.normal.x = 1; t.normal.y = 1; t.normal.z = 1;
    t.v0.x = 1; t.v0.y = 0; t.v0.z = 0;
    t.v1.x = 0; t.v1.y = 1; t.v1.z = 0;
    t.v2.x = 0; t.v2.y = 0; t.v2.z = 1;
    std::vector<StlReader::Triangle> tris = { t };
    const char* path = "tests_roundtrip.stl";
    bool written = StlReader::writeAsciiStlFromTriangles(path, tris);
    assert(written && "writeAsciiStlFromTriangles should succeed");
    StlReader r;
    assert(r.read(path) && "read roundtrip file");
    r.removeDuplicateVertices();
    assert(r.triangleCount() == 1 && "roundtrip should have one triangle");
    double vol = r.volume();
    assert(vol > 0.1 && vol < 0.2 && "roundtrip volume should be ~1/6");
    std::remove(path);
}

static void test_vec3_ordering() {
    StlReader::Vec3 a{ 0, 0, 0 };
    StlReader::Vec3 b{ 1, 0, 0 };
    StlReader::Vec3 c{ 0, 1, 0 };
    assert(a < b);
    assert(a < c);
    assert(!(b < c));
    assert(!(a < a));
}

static void test_ray_intersect() {
    StlReader r;
    assert(r.read(simple_stl()));
    r.removeDuplicateVertices();
    StlReader::Vec3 ro{ 0.25f, 0.25f, -1.f };
    StlReader::Vec3 rd{ 0.f, 0.f, 1.f };
    float t = 0.f;
    bool hit = r.rayIntersect(0, ro, rd, t);
    assert(hit && t > 0.f && "ray from below through centroid should hit");
    rd.z = -1.f;
    hit = r.rayIntersect(0, ro, rd, t);
    assert(!hit && "ray pointing away should not hit");
}

// --- addCaps: single triangle has 3 boundary edges -> one loop -> 3 cap triangles -> 4 total
static void test_add_caps() {
    StlReader r;
    assert(r.read(simple_stl()));
    r.removeDuplicateVertices();
    std::vector<size_t> subset = { 0 };
    std::vector<StlReader::Triangle> fluid;
    r.addCaps(subset, fluid);
    assert(fluid.size() == 4 && "one original triangle + 3 caps");
}

// --- Full pipeline: when data file exists, run computeFluidMesh and check fluid volume
static void test_full_pipeline() {
    const char* path = "../data/V4D_Cold-Plate_1.stl";
    std::ifstream f(path);
    if (!f) return;  // skip if data file not present (e.g. in CI)
    StlReader r;
    assert(r.read(path));
    r.removeDuplicateVertices();
    std::vector<StlReader::Triangle> fluid;
    std::ostringstream discard;
    r.computeFluidMesh(fluid, discard);
    assert(!fluid.empty() && "data mesh should yield fluid triangles");
    const char* outPath = "test_pipeline_out.stl";
    assert(StlReader::writeAsciiStlFromTriangles(outPath, fluid) && "write pipeline output");
    double vol = 0.;
    assert(StlReader::volumeFromFile(outPath, vol) && "read pipeline output");
    assert(vol > 0.01 && "pipeline fluid volume positive");
    std::remove(outPath);
}

// --- I/O: read missing file
static void test_read_missing_file() {
    StlReader r;
    assert(!r.read("nonexistent_does_not_exist.stl") && "read missing file should fail");
}

// --- I/O: volumeFromFile on missing file
static void test_volume_from_file_missing() {
    double vol = -999.;
    bool ok = StlReader::volumeFromFile("nonexistent_does_not_exist.stl", vol);
    assert(!ok && "volumeFromFile(missing) should fail");
    assert(vol == -999. && "vol should be unchanged on failure");
}

// --- I/O: writeAsciiStl then read back (full-mesh write path)
static void test_write_ascii_stl_roundtrip() {
    StlReader r;
    assert(r.read(simple_stl()));
    r.removeDuplicateVertices();
    const char* outPath = "test_solid_out.stl";
    assert(r.writeAsciiStl(outPath) && "writeAsciiStl should succeed");
    double vol = 0.;
    assert(StlReader::volumeFromFile(outPath, vol) && "read back written solid");
    assert(std::fabs(vol - 1.0 / 6.0) < 1e-5 && "solid roundtrip volume 1/6");
    std::remove(outPath);
}

// --- I/O: write to invalid path (empty or unwritable) should fail
static void test_write_invalid_path() {
    StlReader::Triangle t{};
    t.normal.x = t.normal.y = t.normal.z = 1;
    t.v0.x = 1; t.v0.y = 0; t.v0.z = 0;
    t.v1.x = 0; t.v1.y = 1; t.v1.z = 0;
    t.v2.x = 0; t.v2.y = 0; t.v2.z = 1;
    std::vector<StlReader::Triangle> tris = { t };
    assert(!StlReader::writeAsciiStlFromTriangles("", tris) && "write to empty path should fail");
}

// --- Geometry quality report: checkWatertight + checkRightHandWinding produce expected report content
static void test_geometry_quality_report_content() {
    StlReader r;
    assert(r.read(simple_stl()));
    r.removeDuplicateVertices();
    std::ostringstream out;
    r.checkWatertight(out);
    r.checkRightHandWinding(out);
    std::string s = out.str();
    assert(s.find("Watertight") != std::string::npos && "report should contain Watertight");
    assert(s.find("Edges:") != std::string::npos && "report should contain Edges");
    assert(s.find("Vertices:") != std::string::npos && "report should contain Vertices");
    assert(s.find("Right-hand rule:") != std::string::npos && "report should contain Right-hand rule");
}

int main() {
    test_volume_from_file();
    test_read_triangle_count();
    test_clean_mesh_removes_duplicates();
    test_clean_mesh_removes_degenerate();
    test_write_read_roundtrip();
    test_vec3_ordering();
    test_ray_intersect();
    test_add_caps();
    test_full_pipeline();
    test_read_missing_file();
    test_volume_from_file_missing();
    test_write_ascii_stl_roundtrip();
    test_write_invalid_path();
    test_geometry_quality_report_content();
    std::cout << "All tests passed.\n";
    return 0;
}
