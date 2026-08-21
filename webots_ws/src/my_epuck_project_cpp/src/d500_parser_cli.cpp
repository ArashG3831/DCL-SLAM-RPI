#include "my_epuck_project_cpp/d500_parser.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

using namespace my_epuck_project_cpp;

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: d500_parser_cli RAW_FILE\n";
    return 2;
  }
  std::ifstream input(argv[1], std::ios::binary);
  if (!input) {
    std::cerr << "cannot open " << argv[1] << "\n";
    return 2;
  }
  std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(input)), {});
  D500StreamParser parser;
  // Feed deterministic chunks to exercise stream state rather than relying on
  // a single read matching a serial-driver read boundary.
  std::size_t offset = 0;
  while (offset < bytes.size()) {
    const std::size_t n = std::min<std::size_t>(17, bytes.size() - offset);
    parser.feed(bytes.data() + offset, n, 100.0 + offset * 1e-6);
    offset += n;
  }
  CompletedScan scan;
  std::size_t completed = 0;
  std::size_t valid_bins = 0;
  std::vector<float> ranges;
  while (parser.take_completed_scan(scan)) {
    ++completed;
    valid_bins = scan.valid_bins;
    ranges = scan.ranges;
  }
  std::cout << "{\"packet_count\":" << parser.packet_count()
            << ",\"bad_crc_count\":" << parser.bad_crc_count()
            << ",\"completed_scan_count\":" << parser.completed_scan_count()
            << ",\"completed_returned\":" << completed
            << ",\"valid_bins\":" << valid_bins << ",\"ranges\":[";
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    if (i != 0) std::cout << ',';
    if (std::isfinite(ranges[i])) std::cout << std::setprecision(9) << ranges[i];
    else std::cout << "null";
  }
  std::cout << "]}\n";
  return 0;
}
