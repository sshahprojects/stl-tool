#ifndef STL_READER_H
#define STL_READER_H

#include <iosfwd>
#include <string>
#include <vector>

class StlReader {
public:
    struct Vec3 {
        float x, y, z;
        bool operator<(const Vec3& o) const {
            if (x != o.x) return x < o.x;
            if (y != o.y) return y < o.y;
            return z < o.z;
        }
    };
    struct Triangle {
        Vec3 normal;
        Vec3 v0, v1, v2;
    };
    struct IndexedTri { size_t v0, v1, v2; };

    bool read(const std::string& path);
    void removeDuplicateVertices();

    const std::string& header() const { return header_; }
    size_t triangleCount() const { return indexedTriangles_.size(); }
    const std::vector<Vec3>& vertices() const { return vertices_; }
    const std::vector<IndexedTri>& indexedTriangles() const { return indexedTriangles_; }
    Triangle getTriangle(size_t i) const;
    /** Volume (signed tetrahedron sum from origin). Call after removeDuplicateVertices(). */
    double volume() const;

    /** Load STL from path, deduplicate, return volume. Returns false if read fails. */
    static bool volumeFromFile(const std::string& path, double& outVolume);

    /** Check if vertex order (v0,v1,v2) matches right-hand rule vs original facet normals. Call after removeDuplicateVertices(). */
    void checkRightHandWinding(std::ostream& out) const;

    /** Write set of triangles as ASCII STL. Call after removeDuplicateVertices(). Returns false on write error. */
    bool writeAsciiStl(const std::string& path) const;
    /** Write only triangles whose indices are in onlyIndices (sorted, unique). */
    bool writeAsciiStl(const std::string& path, const std::vector<size_t>& onlyIndices) const;

    /** Append cap triangles to close boundary loops of the given triangle subset. Fills outTriangles with original subset + caps. */
    void addCaps(const std::vector<size_t>& triangleIndices, std::vector<Triangle>& outTriangles) const;

    /** Compute fluid set of triangles: even-hit interior selection, addCaps, flip cap normals, cleanMesh. Call after removeDuplicateVertices(). Fills outFluid. */
    void computeFluidMesh(std::vector<Triangle>& outFluid, std::ostream& cleanMeshOut,
        float originOffset = 1e-4f, float tMin = 1e-2f, float tEps = 1e-4f) const;

    /** Write a list of triangles to an ASCII STL file (e.g. output from addCaps). */
    static bool writeAsciiStlFromTriangles(const std::string& path, const std::vector<Triangle>& triangles);

    /** Write one facet to stream. */
    static void writeOneFacet(std::ostream& f, const Triangle& t);

    /** Remove duplicate triangles, merge duplicate vertices, remove degenerate triangles. Modifies triangles in place; reports counts to out. */
    static void cleanMesh(std::vector<Triangle>& triangles, std::ostream& out);

    /** Möller–Trumbore: ray origin ro, unit direction rd, triangle index. Returns true and t_out (t > 0) if hit. */
    bool rayIntersect(size_t triIndex, const Vec3& ro, const Vec3& rd, float& t_out) const;

    /** Check watertightness after removeDuplicateVertices(): no duplicate triangles, every edge shared by exactly 2 triangles, degenerate check. Writes report to out. Returns true if watertight. */
    bool checkWatertight(std::ostream& out) const;

private:
    std::string header_;
    std::vector<Triangle> triangles_;
    std::vector<Vec3> vertices_;
    std::vector<IndexedTri> indexedTriangles_;
    std::vector<Vec3> originalFacetNormals_;
};

#endif
