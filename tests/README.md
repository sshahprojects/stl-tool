# Unit tests

## Running tests

From the `tests/` directory:

```bash
./build.sh
./test_runner
```

Expect output: `All tests passed.`

## Test count and speed

There are **14 tests**. They run in a few milliseconds except the full-pipeline test, which runs only when `../data/V4D_Cold-Plate_1.stl` exists and may take a second or two.

## What’s covered

- **Volume** — `volumeFromFile` on a small STL; expected volume 1/6 for a single tetrahedron.
- **Read and dedup** — Read `simple.stl`, call `removeDuplicateVertices()`, check triangle count.
- **cleanMesh** — Duplicate triangles are removed; degenerate triangles (repeated vertices) are removed.
- **Write/read roundtrip** — Write one triangle with `writeAsciiStlFromTriangles`, read back, check one triangle and volume in a reasonable range.
- **Vec3** — `operator<` ordering for vertices.
- **Ray–triangle** — `rayIntersect`: ray from below through centroid hits; ray pointing away misses.
- **addCaps** — Single triangle (3 boundary edges) → one loop → 3 caps → 4 triangles total.
- **Full pipeline** — When `../data/V4D_Cold-Plate_1.stl` exists: read → removeDuplicateVertices → **computeFluidMesh** → write → volumeFromFile; assert fluid volume > 0. Skipped if the data file is not present.
- **I/O: read missing file** — `read("nonexistent...")` returns false.
- **I/O: volumeFromFile missing** — `volumeFromFile("nonexistent...", vol)` returns false and leaves `vol` unchanged.
- **I/O: writeAsciiStl roundtrip** — Read `simple.stl`, `writeAsciiStl` to temp file, read back with `volumeFromFile`, assert volume ~1/6.
- **I/O: write invalid path** — `writeAsciiStlFromTriangles("", tris)` returns false.
- **Geometry quality report** — Read STL, run `checkWatertight` and `checkRightHandWinding` to a stream; assert report contains "Watertight", "Edges", "Vertices", "Right-hand rule".

## What’s not covered

- **Binary STL** — Only ASCII STL is exercised.
- **checkWatertight return value** — Report content is asserted; the boolean return is not.
- **Malformed STL** — Corrupt or partial files are not tested.
