# Algorithm and Implementation Report: STL Fluid Volume Tool

## 1. Algorithm Selection: Point-in-Polyhedron

**Chosen approach: even-hit ray casting (ray parity test).**

- **What it is:** For each triangle, we cast a ray from just outside the facet (centroid + ε along the outward normal) in the direction of the normal. We count how many other triangles the ray intersects (at distinct distances). If the count is **even**, the triangle is classified as **interior** (facing into the cavity); if odd, it is exterior and discarded.
- **Why this over alternatives:**
  - **Winding number / ray casting from inside:** Would require a known point inside the cavity; we don’t assume one. Even-hit works from the surface only.
  - **Voxel / grid methods:** More memory and setup; we need a watertight surface, not a volume grid. Ray casting gives a triangle subset directly.
  - **BSP or spatial partitioning:** Would improve performance for large N but adds complexity; for the target scale (hundreds to moderate thousands of triangles), brute-force O(N²) ray–triangle tests are acceptable.
- **Ray–triangle test:** Möller–Trumbore for robustness and simplicity (no extra libraries). Hits are filtered by a minimum distance \( t_{\min} \) to avoid self-intersection and grazing hits.

---

## 2. Implementation Details

**Key design decisions:**

- **Single class, no external geometry libs:** All geometry (Vec3, triangles, ray intersect, volume, watertightness) is implemented in `StlReader` using only C++17 and the STL. No Eigen, CGAL, or similar — keeps the project self-contained and easy to build.
- **Indexed triangles after load:** STL is read into a vertex table and an indexed triangle list (`IndexedTri`: v0, v1, v2). Duplicate vertices are merged (`removeDuplicateVertices()`). This gives a single source of truth for positions and correct normals from geometry.
- **Pipeline in one entry point:** The full fluid-extraction pipeline (even-hit → addCaps → cap flip → cleanMesh) lives in `StlReader::computeFluidMesh()`. The driver (`main`) only does CLI, I/O, and reporting. Pipeline logic is not duplicated.
- **Capping with boundary loops:** Boundary edges (edge count = 1) are found from an undirected edge map. Loops are traced via a “next” map (vertex → next vertex along boundary). Each loop is closed with a fan of cap triangles from the loop centroid. Non–simple loops (repeated vertex) are split into sub-loops and capped separately.
- **Clean step before write:** The fluid set of triangles is still unindexed. `cleanMesh` merges duplicate vertices, drops degenerate triangles, removes duplicate triangles (canonical triple), and recomputes normals so the output is suitable for volume and watertightness checks.

**Data structures:**

- **Vec3:** `float x, y, z` with `operator<` for use as map key (vertex dedup, edge canonicalization).
- **Triangle:** `normal`, `v0`, `v1`, `v2` (Vec3) for unindexed output (e.g. after addCaps).
- **IndexedTri:** `size_t v0, v1, v2` for indexed representation.
- **Edge map:** `(min(a,b), max(a,b))` → count (or list of incident triangles) for boundary detection.
- **Next map:** For each vertex, list of (next vertex, triangle index) for boundary edges to trace loops.

---

## 3. Complexity Analysis

- **Time**
  - **Even-hit:** For each of N triangles we test the ray against the other N−1 triangles → **O(N²)** ray–triangle tests. Möller–Trumbore per test is O(1). Dominant cost for large N.
  - **Capping:** Edge map O(N), loop tracing O(boundary edges), cap creation O(cap triangles). **O(N)** in practice.
  - **cleanMesh:** Vertex merge O(N log N) with a map; duplicate triangle removal O(N log N) with set/map of canonical triples. **O(N log N)**.
  - **Volume / watertightness:** O(N) over triangles/edges.
- **Space**
  - **StlReader:** O(N) vertices, O(N) indexed triangles, O(N) facet normals. Temporary copies for fluid subset and caps: O(N). **O(N)** overall.

---

## 4. Edge Cases

- **Degenerate triangles:** Triangles with two or more identical vertices (zero area). Dropped in `cleanMesh`; not used in volume. Watertightness check reports them.
- **Duplicate vertices:** Merged in `removeDuplicateVertices()` and again in `cleanMesh` for the fluid set of triangles so that edge counts and watertightness are correct.
- **Self-intersection / grazing hits:** Ray origin is offset by ε along the normal; only hits with \( t > t_{\min} \) are counted so the starting facet and near-grazing hits are ignored.
- **Hit merging:** Hits closer than \( t_{\varepsilon} \) can be merged into one “distinct” hit to avoid double-counting at shared edges/vertices (documented in design; tuning available via parameters).
- **Non–simple boundary loops:** If a vertex appears twice while tracing a loop, the path is split into a closed sub-loop (which is capped) and a remaining path; tracing continues so all boundary edges are capped.
- **Missing file / write failure:** Read and write return false; caller prints a message and exits with non-zero status. No crash on I/O error.
- **Empty or single-triangle input:** Pipeline runs; fluid set may be empty or minimal. Volume and reports still produced where applicable.

---

## 5. Testing Strategy

- **Unit tests (14 total):**
  - **Volume:** `volumeFromFile` on a small STL; expected volume 1/6 for a single tetrahedron.
  - **Read and dedup:** Read `simple.stl`, `removeDuplicateVertices()`, check triangle count.
  - **cleanMesh:** Duplicate triangles removed; degenerate triangles (repeated vertices) removed.
  - **Write/read roundtrip:** Write one triangle, read back, check triangle count and volume.
  - **Vec3:** `operator<` ordering for vertex maps.
  - **Ray–triangle:** `rayIntersect` — ray through centroid hits; ray pointing away misses.
  - **addCaps:** One triangle (3 boundary edges) → one loop → 3 caps → 4 triangles total.
  - **Full pipeline:** When the cold-plate STL exists: read → removeDuplicateVertices → computeFluidMesh → write → volumeFromFile; assert fluid volume > 0.
  - **I/O:** Missing file (read, volumeFromFile), invalid path (write), writeAsciiStl roundtrip.
  - **Geometry quality report:** `checkWatertight` and `checkRightHandWinding` output; assert report contains "Watertight", "Edges", "Vertices", "Right-hand rule".
- **Correctness:** Volume consistency (pipeline output vs `volumeFromFile` on written STL); geometry report confirms watertight and orientation for both solid and fluid outputs. Validation mode runs the same checks on any STL without running the pipeline.

---

## 6. General

### Trade-offs

- **Accuracy vs performance:** We use exact ray–triangle tests and floating-point vertex positions. No iterative tolerance for “inside” — even-hit is discrete. Trade-off: correct classification for clean geometry; very thin or badly scaled meshes may need tuned ε, \( t_{\min} \), \( t_{\varepsilon} \).
- **Performance vs development time:** No spatial acceleration (e.g. BVH). O(N²) is acceptable for the target size; implementing and testing a BVH was deferred to keep the first version simple and reliable.
- **Scope:** Only closed, manifold-like input is designed for. Non-manifold or open input may produce unexpected boundaries; we report non-manifold edges but do not attempt automatic repair.

### Future Improvements

- **Spatial acceleration:** BVH (or similar) for ray–triangle tests to scale to very large triangle counts.
- **Binary STL output option:** For large outputs, binary STL would reduce file size and write time.
- **Optional mesh healing:** Detect and fix small gaps or non-manifold edges before or after capping.
- **More tests:** Binary STL read path; malformed STL handling; explicit checkWatertight return-value assertion; integration test for `--validate` CLI.
- **Parameter tuning:** Expose or document ε, \( t_{\min} \), \( t_{\varepsilon} \) for different units/scales.

### Tool Selection

- **Language and standard:** C++17. No external geometry or linear-algebra libraries (no Eigen, CGAL, etc.) so the tool builds with only a compiler and standard library. Eases portability and avoids dependency drift.
- **I/O:** Custom ASCII and binary STL read; ASCII-only write. No mesh library (e.g. OpenMesh, libigl) to keep the codebase minimal and the pipeline explicit.
- **Build:** Plain `build.sh` scripts (e.g. `clang++ -std=c++17`); no CMake or package manager required for basic use.
- **Testing:** Single executable (`test_runner`) with asserts; no test framework dependency. Fast feedback for development and CI.
