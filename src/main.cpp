#include "stl_reader.h"
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <sys/stat.h>

static void printGeometryQualityReport(const std::string& path, const std::string& label, std::ostream& out) {
    StlReader r;
    if (!r.read(path)) {
        out << label << ": failed to read " << path << "\n";
        return;
    }
    r.removeDuplicateVertices();
    out << "--- " << label << " (" << path << ") ---\n";
    r.checkWatertight(out);
    r.checkRightHandWinding(out);
    double vol = 0.;
    if (StlReader::volumeFromFile(path, vol))
        out << "Volume: " << std::fixed << std::setprecision(10) << vol << "\n";
    out << "\n";
}

static int runValidateMode(const std::string& path) {
    StlReader r;
    if (!r.read(path)) {
        std::cerr << "validate: read failed: " << path << "\n";
        return 1;
    }
    r.removeDuplicateVertices();
    std::cout << "Geometry quality report\n";
    std::cout << "--- " << path << " ---\n";
    r.checkWatertight(std::cout);
    r.checkRightHandWinding(std::cout);
    std::cout << "Volume: " << std::fixed << std::setprecision(10) << r.volume() << "\n";
    return 0;
}

static int runPipeline(const std::string& inputPath, const std::string& outDir) {
    if (mkdir(outDir.c_str(), 0755) != 0 && errno != EEXIST) {
        std::cerr << "Cannot create output directory '" << outDir << "': " << std::strerror(errno) << "\n";
        return 1;
    }
    StlReader r;
    if (!r.read(inputPath)) {
        std::cerr << "read failed: " << inputPath << "\n";
        return 1;
    }
    r.removeDuplicateVertices();
    std::ostringstream discard;
    r.checkRightHandWinding(discard);
    if (!r.writeAsciiStl(outDir + "solid_volume.stl")) {
        std::cerr << "write ASCII STL failed\n";
        return 1;
    }
    const double fullVolume = r.volume();

    std::vector<StlReader::Triangle> fluid;
    r.computeFluidMesh(fluid, discard);

    const std::string fluidPath = outDir + "fluid_volume.stl";
    if (!StlReader::writeAsciiStlFromTriangles(fluidPath, fluid)) {
        std::cerr << "write fluid STL failed\n";
        return 1;
    }

    std::cout << "Solid geometry volume: " << std::fixed << std::setprecision(10) << fullVolume << "\n";
    double fluidVolume = 0.;
    if (StlReader::volumeFromFile(fluidPath, fluidVolume))
        std::cout << "Fluid geometry volume: " << std::fixed << std::setprecision(10) << fluidVolume << "\n";
    else
        std::cerr << "Failed to compute volume of fluid STL\n";
    std::cout << "Output: " << outDir << "solid_volume.stl, " << outDir << "fluid_volume.stl\n";

    std::cout << "\nGeometry quality report\n";
    printGeometryQualityReport(outDir + "solid_volume.stl", "Solid", std::cout);
    printGeometryQualityReport(fluidPath, "Fluid", std::cout);
    return 0;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << (argv[0] ? argv[0] : "stl_tool") << " <input.stl>\n";
        std::cerr << "       " << (argv[0] ? argv[0] : "stl_tool") << " --validate <path.stl>\n";
        return 1;
    }
    const std::string arg1 = argv[1];
    if (arg1 == "--validate") {
        if (argc < 3) {
            std::cerr << "Usage: " << (argv[0] ? argv[0] : "stl_tool") << " --validate <path.stl>\n";
            return 1;
        }
        return runValidateMode(argv[2]);
    }
    return runPipeline(arg1, "../output/");
}
