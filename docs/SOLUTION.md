# Technical Write-Up: STL Fluid Volume Extraction

## 1. Problem and goal

Given a closed set of triangles in STL format (e.g. a cold plate or mechanical part), we want to:

1. **Identify the fluid cavity** — the interior volume where coolant or fluid would sit.
2. **Produce a watertight surface of that cavity** — a closed surface suitable for volume computation and downstream meshing simulation or analysis.
3. **Retain the full solid set of triangles** for reference.

The tool reads one input STL and writes two outputs: the full set of triangles as `solid_volume.stl` and the closed fluid cavity as `fluid_volume.stl`, both in the project’s `output/` directory.

---

## 2. Solution overview

The implementation is modular: the fluid-extraction pipeline is implemented in **`StlReader::computeFluidMesh()`** (even-hit, capping, cap flip, clean). The application entry point (`main`) only parses the command line, calls either the full pipeline or validation mode, and prints the report.

The pipeline has four main stages:

1. **Load and index** — Read the STL (ASCII or binary), merge duplicate vertices into a single vertex table, and build an indexed triangle list. This gives a clean base set of triangles and correct facet normals from geometry.
2. **Even-hit interior selection** — For each triangle, cast a ray from its centroid along its outward normal. Count how many other triangles the ray hits (at distinct distances). Triangles with an *even* number of hits are classified as *interior* (facing into the cavity) and kept; the rest are discarded. The result is a subset of triangles that bounds the fluid region but has open boundaries (holes).
3. **Capping** — Find boundary edges of this subset (edges belonging to exactly one triangle). Trace boundary loops and close each loop with a fan of triangles from the loop’s centroid. Cap normals are oriented outward so the combined set of triangles is consistently oriented and watertight.
4. **Clean and write** — Remove duplicate triangles, merge duplicate vertex positions in the fluid set of triangles, drop degenerate triangles, then write the result as ASCII STL and run a watertightness check.

Volume is computed via the signed-tetrahedron formula (sum of (1/6) · (origin, v0, v1, v2)); the final fluid volume is reported and can be checked against the written `fluid_volume.stl`.

**Algorithm choice:** Even-hit ray casting was chosen over winding-number or voxel methods because it needs no known point inside the cavity and yields a triangle subset directly. Möller–Trumbore is used for ray–triangle tests (no extra libraries). BVH or similar was deferred for simplicity; brute-force \( O(N^2) \) is acceptable for the target scale.

---

## 3. Algorithm details

### 3.1 Even-hit criterion

For each triangle \( T_i \):

- Compute centroid \( C \) and outward unit normal \( n \).
- Ray origin: \( O = C + \varepsilon n \) (small \( \varepsilon \), e.g. \( 10^{-4} \)), so the ray starts just outside the facet.
- Ray direction: \( n \).
- For every other triangle \( T_k \), compute ray–triangle intersection (Möller–Trumbore). Collect hit distances \( t > t_{\min} \) (e.g. \( t_{\min} = 10^{-2} \) to avoid self-intersection and grazing hits).
- If the number of distinct hits is *even*, \( T_i \) is interior (ray enters and exits the solid an equal number of times before escaping), so it is kept; otherwise it is discarded.

This gives the set of triangles that form the boundary of the fluid cavity, with holes where the cavity is open.

### 3.2 Boundary loops and capping

- Build an undirected edge map for the selected triangles: each edge \( (a,b) \) is stored as the canonical pair \( (\min(a,b), \max(a,b)) \). Count how many triangles use each edge.
- **Boundary edges** are those with count 1. Each has a direction (from, to) given by the single incident triangle.
- Build a “next” map: for each vertex \( v \), list \( (w, \text{triIdx}) \) for each boundary edge \( v \to w \).
- **Trace loops:** Start from an unused boundary edge, follow the “next” map until the start vertex is reached. If a vertex appears twice in the current path (repeated vertex in the loop), split into a closed sub-loop and a remaining path; cap the sub-loop and continue. This handles non–simple boundary loops.
- **Cap geometry:** For each loop, compute centroid \( C \) of the loop vertices. For each consecutive edge \( (a,b) \) along the loop, form a cap triangle \( (C, a, b) \). Normal is from the cross product of \( (a - C) \times (b - C) \); orientation is aligned with an adjacent triangle so the cap is outward. Vertices are ordered (and normals flipped if needed) so the cap is consistently oriented.
- The original subset plus all cap triangles form a closed body. Cap triangles are then flipped (swap v1/v2, negate normal) in the driver so the final set of triangles are consistently oriented.

### 3.3 Cleaning the set of triangles

The fluid set of triangles is still unindexed (each triangle has three vertex positions). **cleanMesh**:

- Build a unique vertex table: merge vertices that have the same coordinates (e.g. via a map), and replace triangle vertex references by indices into this table.
- Drop degenerate triangles (two or more vertex indices equal).
- Remove duplicate triangles (same triple of vertex indices, up to permutation; canonicalize by sorting the triple).
- Recompute facet normals from the merged geometry.
- Optionally report non-manifold edges (shared by more than two triangles).

The result is a clean, indexed set of triangles suitable for volume and watertightness checks.

### 3.4 Volume and watertightness

- **Volume:** Signed tetrahedron formula from the origin: for each triangle \( (a,b,c) \), add \( (1/6) \bigl( a \cdot (b \times c) \bigr) \). Take the absolute value for a positive volume. Used for both the full set of triangles and the fluid set of triangles.
- **Watertightness:** After deduplicating vertices and building an indexed set of triangles from the written STL, check: no duplicate triangles, every edge shared by exactly two triangles, no degenerate triangles. If all hold, the set of triangles is closed and watertight.

---

## 4. Implementation notes

- **Modularity:** The full fluid pipeline (even-hit, addCaps, cap flip, cleanMesh) is in **`StlReader::computeFluidMesh(outFluid, cleanMeshOut, originOffset, tMin, tEps)`**. The driver (`main`) limits itself to CLI handling and calling `runPipeline()` or `runValidateMode()`; it does not duplicate pipeline logic.
- **No external geometry libs:** C++17 and standard library only (no Eigen, CGAL, or mesh libraries); keeps the tool self-contained and easy to build.
- **Data structures:** Vec3 (with `operator<` for map keys), Triangle (normal + v0,v1,v2), IndexedTri (v0,v1,v2 indices); edge map (canonical edge → count); next map (vertex → next vertex along boundary) for loop tracing.
- **STL I/O:** Both ASCII and binary STL are supported. Output is ASCII only (`solid_volume.stl`, `fluid_volume.stl`). Custom reader/writer; no mesh library.
- **Coordinate system:** All logic is in the same coordinate system as the input STL; no global transform is applied.
- **Parameters:** Ray origin offset \( \varepsilon \), minimum hit distance \( t_{\min} \), and hit-merging distance \( t_{\varepsilon} \) are defaulted in `computeFluidMesh`; they can be tuned if needed for different inputs or scales.
- **Complexity:** Time — even-hit \( O(N^2) \), capping \( O(N) \), cleanMesh \( O(N \log N) \), volume/watertightness \( O(N) \). Space — \( O(N) \) for vertices, triangles, and temporaries.
- **Edge cases:** Degenerates and duplicate vertices are dropped/merged in cleanMesh and removeDuplicateVertices. Self-intersection and grazing hits avoided with \( t > t_{\min} \); hit merging via \( t_{\varepsilon} \). Non–simple boundary loops are split into sub-loops and capped. Missing file or write failure returns false; caller exits with message. Empty or minimal input runs without crashing.

---

## 5. Output and validation

- **output/solid_volume.stl** — Full input set of triangles, ASCII, after vertex deduplication. Use for reference and full solid volume.
- **output/fluid_volume.stl** — Fluid cavity set of triangles, ASCII, after cleanMesh. Intended to be watertight.

The program prints: solid and fluid volumes, output paths, and a **geometry quality report** for both output STLs. The report includes:
- **Watertightness** — No duplicate triangles, every edge shared by exactly two triangles, no degenerate triangles.
- **Edge and vertex counts** — Unique edges; boundary (count=1) and non-manifold (count>2) edges.
- **Right-hand rule** — Consistency of facet orientation (computed normal vs stored normal).
- **Volume** — Volume from the signed-tetrahedron formula for each output.

**Validation mode** (`stl_tool --validate <path.stl>`) runs the same quality checks on an arbitrary STL file without running the fluid/solid pipeline, for use as a standalone validation tool.

---

## 6. Testing, trade-offs, and tool choice

- **Testing:** 14 unit tests cover volume, read/dedup, cleanMesh (duplicates and degenerates), write/read roundtrip, Vec3 ordering, ray–triangle, addCaps, full pipeline (when data exists), I/O failures, and geometry report content. Correctness is validated by volume consistency (pipeline output vs volumeFromFile on written STL) and watertight/orientation reports.
- **Trade-offs:** Exact ray–triangle tests and discrete even-hit; no iterative tolerance. No BVH — \( O(N^2) \) accepted for target scale to keep the first version simple. Designed for closed, manifold-like input; non-manifold edges are reported but not auto-repaired.
- **Future improvements:** Spatial acceleration (e.g. BVH) for large \( N \); binary STL output; optional healing; more tests (binary read, malformed STL, `--validate` CLI); expose or document \( \varepsilon \), \( t_{\min} \), \( t_{\varepsilon} \) for different scales.
- **Tool selection:** C++17, no Eigen/CGAL. Custom ASCII/binary STL I/O. Build via `build.sh` (e.g. `clang++ -std=c++17`); tests are a single executable with asserts, no test framework.
