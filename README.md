# STL fluid volume tool

Reads an STL mesh, extracts a fluid volume (even-hit interior), caps the boundary, and writes both the full solid and the fluid volume as ASCII STL.

## Setup

- **Requirements:** C++17 compiler (e.g. `clang++`).
- **Build:** From the `src/` directory run:
  ```bash
  cd src
  ./build.sh
  ```
  This produces `stl_tool` in `src/`.

**Tests:** From the `tests/` directory run `./build.sh` then `./test_runner`.

## Usage

**Normal mode** — Run from `src/` with the input STL path to run the full pipeline and write outputs:

```bash
./stl_tool <input.stl>
```

Example with data in `data/`:

```bash
./stl_tool ../data/V4D_Cold-Plate_1.stl
```

**Output** (written to `output/` at the project root):

- `output/solid_volume.stl` — full mesh (ASCII STL).
- `output/fluid_volume.stl` — fluid volume (watertight).

The tool prints solid and fluid volumes, output paths, and a **geometry quality report** for both STLs (watertight check, edge/vertex counts, right-hand rule, volume).

**Validation mode** — Check geometry quality of any STL without running the pipeline:

```bash
./stl_tool --validate <path.stl>
```

Prints a geometry quality report (watertight, edges, vertices, orientation, volume) for that file.

## Design

- **Modular pipeline:** The fluid-extraction algorithm lives in `StlReader::computeFluidMesh()` (even-hit selection, capping, cap orientation, mesh cleaning). The driver in `main.cpp` only parses arguments, calls `runPipeline()` or `runValidateMode()`, and prints results. Pipeline logic is not duplicated.
- **Even-hit criterion:** For each triangle, a ray is cast from its centroid (slightly offset along the facet normal). Triangles with an *even* number of distinct ray hits are treated as interior and kept; the rest are discarded to form the fluid cavity.
- **Caps:** Boundary edges of the even-hit subset are found (edges shared by only one triangle). Boundary loops are traversed and closed with cap triangles (fan from loop centroid), with normals oriented consistently (caps flipped so the closed mesh is outwardly oriented).
- **Clean mesh:** Before writing the fluid STL, the mesh is cleaned: duplicate triangles removed, duplicate vertex positions merged, degenerate triangles dropped. This reduces vertex count and ensures a single vertex table for the watertight check.
- **Output location:** Output is always written to `output/` relative to the project root (i.e. `../output/` when running from `src/`), so results stay out of the source tree.
