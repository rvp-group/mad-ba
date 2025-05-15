#include <srrg_system_utils/timestamp_filter.h>
#include <iostream>
#include <iomanip>
#include "fstream"

using TimestampVector = std::vector<double>;

TimestampVector loadTextFile(const std::string& filepath_);

int main(int argc, char **argv) {

    if (argc != 3) {
      std::cerr << "** usage: rosrun srrg2_hokuyo rosbag_from_images <path to input file> <path to output file>" 
          << std::endl;
      std::cerr << "** file format must be: timestamp tx ty tz qx qy qz qw" << std::endl;
      return 0;
    }

    TimestampFilter stampFilter;
  
    const std::string& input_path = argv[1];
    const std::string& output_path = argv[2];
 
    TimestampVector vtimestamps = loadTextFile(argv[1]);
    std::cerr.precision(15);
    std::ofstream output_file;
    output_file.open(output_path);
    output_file << "### old_timestamp new_timestamp";

    for (size_t index = 0; index < vtimestamps.size(); ++index){
      const double& curr_timestamp = vtimestamps[index];
      stampFilter.setMeasurement(curr_timestamp);
      if(stampFilter.status() == stampFilter.Status::Ready){
        const double& new_timestamp = stampFilter.stamp();
        output_file << std::setprecision(15) << curr_timestamp << " " << new_timestamp << std::endl;
      }
    }

    std::cerr << "file written successfully!" << std::endl;
    output_file.close();
    return 0;
}


TimestampVector loadTextFile(const std::string& filepath_)  {
  // ldg open file in following format # timestamp tx ty tz qx qy qz qw
  std::ifstream text_file(filepath_, std::ios::in);
  if (!text_file.good() || !text_file.is_open()) {
    throw std::runtime_error("unable to text file: " + filepath_);
  }
        
  // ldg skip the first line (TUM format header information)
  std::cerr << "reading text file|header begin" << std::endl;
  std::string buffer;
  std::getline(text_file, buffer);
  std::cerr << buffer << std::endl;
  std::cerr << "reading text file|header end" << std::endl;
  TimestampVector vtimestamps;  
  // ldg read file by tokens
  double timestamp = 0;
  double tx, ty, tz, qx, qy, qz, qw;
  while (text_file >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
    assert(timestamp > 0);
    vtimestamps.emplace_back(timestamp);
  }
  vtimestamps.shrink_to_fit();
  std::cerr << "reading text file|loaded number of timestamps: "
            << vtimestamps.size() << std::endl;
  text_file.close();

  return vtimestamps;
}
