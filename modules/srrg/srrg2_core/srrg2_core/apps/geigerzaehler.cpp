// ds THIS CODE WAS CREATED BASED ON: http://kitti.is.tue.mpg.de/kitti/devkit_odometry.zip
// ds minimally modified to avoid C++11 warnings and provided a brief result dump to stdout

#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <limits>
#include <set>
#include <srrg_system_utils/system_utils.h>
#include <vector>

// ds readability
typedef Eigen::Matrix<float, 4, 4> Matrix4f;
typedef std::vector<Matrix4f, Eigen::aligned_allocator<Matrix4f>> Matrix4fVector;

// static parameter
float lengths_small[] = {5, 10, 25, 50, 75, 100, 150, 200};
float lengths_large[] = {100, 200, 300, 400, 500, 600, 700, 800};
float* lengths        = lengths_large;
int32_t num_lengths   = 8;

constexpr size_t minimum_number_of_samples_per_length = 3;

struct errors {
  int32_t first_frame;
  float r_err;
  float t_err;
  float len;
  float speed;
  errors(int32_t first_frame, float r_err, float t_err, float len, float speed) :
    first_frame(first_frame),
    r_err(r_err),
    t_err(t_err),
    len(len),
    speed(speed) {
  }
};

Matrix4fVector loadPoses(std::string file_name) {
  Matrix4fVector poses;

  // ds going modern
  std::ifstream pose_file(file_name, std::ifstream::in);

  // ds grab a line from the ground truth
  std::string buffer_line;
  while (std::getline(pose_file, buffer_line)) {
    // ds get it to a std::stringstream
    std::istringstream buffer_stream(buffer_line);

    // ds information fields (KITTI format)
    Matrix4f pose(Matrix4f::Identity());
    for (uint8_t u = 0; u < 3; ++u) {
      for (uint8_t v = 0; v < 4; ++v) {
        buffer_stream >> pose(u, v);
      }
    }
    poses.push_back(pose);
  }

  pose_file.close();
  return poses;
}

std::vector<float> trajectoryDistances(Matrix4fVector& poses) {
  std::vector<float> dist;
  dist.push_back(0);
  for (std::size_t i = 1; i < poses.size(); i++) {
    Matrix4f P1 = poses[i - 1];
    Matrix4f P2 = poses[i];
    float dx    = P1(0, 3) - P2(0, 3);
    float dy    = P1(1, 3) - P2(1, 3);
    float dz    = P1(2, 3) - P2(2, 3);
    dist.push_back(dist[i - 1] + std::sqrt(dx * dx + dy * dy + dz * dz));
  }
  return dist;
}

int32_t lastFrameFromSegmentLength(std::vector<float>& dist, int32_t first_frame, float len) {
  for (std::size_t i = first_frame; i < dist.size(); i++)
    if (dist[i] > dist[first_frame] + len)
      return i;
  return -1;
}

inline float rotationError(Matrix4f& pose_error) {
  float a = pose_error(0, 0);
  float b = pose_error(1, 1);
  float c = pose_error(2, 2);
  float d = 0.5 * (a + b + c - 1.0);
  return std::acos(std::max(std::min(d, 1.0f), -1.0f));
}

inline float translationError(Matrix4f& pose_error) {
  float dx = pose_error(0, 3);
  float dy = pose_error(1, 3);
  float dz = pose_error(2, 3);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::vector<errors> calcSequenceErrors(Matrix4fVector& poses_gt, Matrix4fVector& poses_result) {
  // error std::vector
  std::vector<errors> error_statistics;

  // parameters
  int32_t step_size = 10; // every second

  // pre-compute distances (from ground truth as reference)
  std::vector<float> dist = trajectoryDistances(poses_gt);

  // for all start positions do
  std::multiset<size_t> evaluated_lengths;
  for (std::size_t first_frame = 0; first_frame < poses_gt.size(); first_frame += step_size) {
    // for all segment lengths do
    for (int32_t i = 0; i < num_lengths; i++) {
      // current length
      float len = lengths[i];

      // compute last frame
      int32_t last_frame = lastFrameFromSegmentLength(dist, first_frame, len);

      // continue, if sequence not long enough
      if (last_frame == -1) {
        continue;
      } else {
        evaluated_lengths.insert(len);
      }

      // compute rotational and translational errors
      Matrix4f pose_delta_gt     = poses_gt[first_frame].inverse() * poses_gt[last_frame];
      Matrix4f pose_delta_result = poses_result[first_frame].inverse() * poses_result[last_frame];
      Matrix4f pose_error        = pose_delta_result.inverse() * pose_delta_gt;
      float r_err                = rotationError(pose_error);
      float t_err                = translationError(pose_error);

      // compute speed
      float num_frames = (float) (last_frame - first_frame + 1);
      float speed      = len / (0.1 * num_frames);

      // write to file
      error_statistics.push_back(errors(first_frame, r_err / len, t_err / len, len, speed));
    }
  }

  // ds check sample distribution
  size_t number_of_valid_lengths = 0;
  for (size_t i = 0; i < static_cast<size_t>(num_lengths); ++i) {
    if (evaluated_lengths.count(static_cast<size_t>(lengths[i])) > 6) {
      std::cerr << lengths[i] << " : " << evaluated_lengths.count(static_cast<size_t>(lengths[i]))
                << std::endl;
      ++number_of_valid_lengths;
    }
  }

  // ds if we did not manage to sample enough lengths and are not running on small samples already
  if (number_of_valid_lengths < 3 && lengths != lengths_small) {
    std::cerr << "calcSequenceErrors|not enough samples, switching to smaller intervals"
              << std::endl;
    lengths = lengths_small;
    return calcSequenceErrors(poses_gt, poses_result);
  } else {
    return error_statistics;
  }
}

void saveSequenceErrors(std::vector<errors>& err, std::string file_name) {
  // open file
  FILE* fp;
  fp = fopen(file_name.c_str(), "w");

  // write to file
  for (std::vector<errors>::iterator it = err.begin(); it != err.end(); it++)
    fprintf(fp, "%d %f %f %f %f\n", it->first_frame, it->r_err, it->t_err, it->len, it->speed);

  // close file
  fclose(fp);
  std::cerr << "saveSequenceErrors|wrote to: '" << file_name << "'" << std::endl;
}

void savePathPlot(Matrix4fVector& poses_gt, Matrix4fVector& poses_result, std::string file_name) {
  // parameters
  int32_t step_size = 3;

  // open file
  FILE* fp = fopen(file_name.c_str(), "w");

  // save x/z coordinates of all frames to file
  for (std::size_t i = 0; i < poses_gt.size(); i += step_size)
    fprintf(fp,
            "%f %f %f %f\n",
            poses_gt[i](0, 3),
            poses_gt[i](2, 3),
            poses_result[i](0, 3),
            poses_result[i](2, 3));

  // close file
  fclose(fp);
  std::cerr << "savePathPlot|wrote to: '" << file_name << "'" << std::endl;
}

std::vector<int32_t> computeRoi(Matrix4fVector& poses_gt, Matrix4fVector& poses_result) {
  float x_min = std::numeric_limits<int32_t>::max();
  float x_max = std::numeric_limits<int32_t>::min();
  float z_min = std::numeric_limits<int32_t>::max();
  float z_max = std::numeric_limits<int32_t>::min();

  for (Matrix4fVector::iterator it = poses_gt.begin(); it != poses_gt.end(); it++) {
    float x = (*it)(0, 3);
    float z = (*it)(2, 3);
    if (x < x_min)
      x_min = x;
    if (x > x_max)
      x_max = x;
    if (z < z_min)
      z_min = z;
    if (z > z_max)
      z_max = z;
  }

  for (Matrix4fVector::iterator it = poses_result.begin(); it != poses_result.end(); it++) {
    float x = (*it)(0, 3);
    float z = (*it)(2, 3);
    if (x < x_min)
      x_min = x;
    if (x > x_max)
      x_max = x;
    if (z < z_min)
      z_min = z;
    if (z > z_max)
      z_max = z;
  }

  float dx = 1.1 * (x_max - x_min);
  float dz = 1.1 * (z_max - z_min);
  float mx = 0.5 * (x_max + x_min);
  float mz = 0.5 * (z_max + z_min);
  float r  = 0.5 * std::max(dx, dz);

  std::vector<int32_t> roi;
  roi.push_back((int32_t)(mx - r));
  roi.push_back((int32_t)(mx + r));
  roi.push_back((int32_t)(mz - r));
  roi.push_back((int32_t)(mz + r));
  return roi;
}

int32_t plotPathPlot(std::string dir, std::vector<int32_t>& roi, int32_t idx) {
  // gnuplot file name
  char command[10240];
  char file_name[2560];
  sprintf(file_name, "%02d.gp", idx);
  std::string full_name = dir + "/" + file_name;

  // ds system calls
  int32_t result = 0;

  // create png + eps
  for (int32_t i = 0; i < 1 /*PNG only*/; i++) {
    // open file
    FILE* fp = fopen(full_name.c_str(), "w");

    // save gnuplot instructions
    if (i == 0) {
      fprintf(fp, "set term png size 900,900\n");
      fprintf(fp, "set output \"%02d.png\"\n", idx);
    } else {
      fprintf(fp, "set term postscript eps enhanced color\n");
      fprintf(fp, "set output \"%02d.eps\"\n", idx);
    }

    fprintf(fp, "set size ratio -1\n");
    fprintf(fp, "set xrange [%d:%d]\n", roi[0], roi[1]);
    fprintf(fp, "set yrange [%d:%d]\n", roi[2], roi[3]);
    fprintf(fp, "set xlabel \"x [m]\"\n");
    fprintf(fp, "set ylabel \"z [m]\"\n");
    fprintf(
      fp, "plot \"%02d.txt\" using 1:2 lc rgb \"#FF0000\" title 'Ground Truth' w lines,", idx);
    fprintf(fp, "\"%02d.txt\" using 3:4 lc rgb \"#0000FF\" title 'Visual Odometry' w lines,", idx);
    fprintf(fp,
            "\"< head -1 %02d.txt\" using 1:2 lc rgb \"#000000\" pt 4 ps 1 lw 2 title 'Sequence "
            "Start' w points\n",
            idx);

    // close file
    fclose(fp);
    std::cerr << "plotPathPlot|wrote to: '" << full_name << "'" << std::endl;

    // run gnuplot => create png + eps
    sprintf(command, "cd %s; gnuplot %s", dir.c_str(), file_name);
    result = system(command);
    std::cerr << "plotPathPlot|wrote to: '" << file_name << "'" << std::endl;
  }

  // create pdf and crop TODO re-enable
  //  sprintf(command, "cd %s; ps2pdf %02d.eps %02d_large.pdf", dir.c_str(), idx, idx);
  //  result = system(command);
  //  sprintf(command, "cd %s; pdfcrop %02d_large.pdf %02d.pdf", dir.c_str(), idx, idx);
  //  result = system(command);
  //  sprintf(command, "cd %s; rm %02d_large.pdf", dir.c_str(), idx);
  //  result = system(command);
  return result;
}

void saveErrorPlots(std::vector<errors>& seq_err, std::string plot_error_dir, const char* prefix) {
  // file names
  char file_name_tl[1024];
  sprintf(file_name_tl, "%s/%s_tl.txt", plot_error_dir.c_str(), prefix);
  char file_name_rl[1024];
  sprintf(file_name_rl, "%s/%s_rl.txt", plot_error_dir.c_str(), prefix);
  //  char file_name_ts[1024];
  //  sprintf(file_name_ts, "%s/%s_ts.txt", plot_error_dir.c_str(), prefix);
  //  char file_name_rs[1024];
  //  sprintf(file_name_rs, "%s/%s_rs.txt", plot_error_dir.c_str(), prefix);

  // open files
  FILE* fp_tl = fopen(file_name_tl, "w");
  FILE* fp_rl = fopen(file_name_rl, "w");
  //  FILE* fp_ts = fopen(file_name_ts, "w");
  //  FILE* fp_rs = fopen(file_name_rs, "w");

  // for each segment length do
  std::cerr << "---------------------------- ERROR STATISTICS ----------------------------"
            << std::endl;
  float total_error_rotation    = 0;
  float total_error_translation = 0;
  uint32_t number_of_lengths    = 0;
  for (int32_t i = 0; i < num_lengths; i++) {
    float t_err = 0;
    float r_err = 0;
    float num   = 0;

    // for all errors do
    for (std::vector<errors>::iterator it = seq_err.begin(); it != seq_err.end(); it++) {
      if (fabs(it->len - lengths[i]) < 1.0) {
        t_err += it->t_err;
        r_err += it->r_err;
        num++;
      }
    }

    // ds we require at least 2 lengths to be evaluated
    if (num > 1) {
      fprintf(fp_tl, "%f %f\n", lengths[i], t_err / num);
      fprintf(fp_rl, "%f %f\n", lengths[i], r_err / num);
      total_error_rotation += r_err / num * (180 / M_PI) * 100;
      total_error_translation += t_err / num * 100;
      ++number_of_lengths;

      // ds info
      std::printf("length: %f error rotation (deg/100m): %9.6f error translation (%%): %9.6f\n",
                  lengths[i],
                  r_err / num * (180 / M_PI) * 100,
                  t_err / num * 100);
    }
  }
  std::cerr << "---------------------------- ---------------- ----------------------------"
            << std::endl;
  std::printf("average error rotation (deg/100m): %9.6f error translation (%%): %9.6f\n",
              total_error_rotation / number_of_lengths,
              total_error_translation / number_of_lengths);
  std::cerr << "---------------------------- ---------------- ----------------------------"
            << std::endl;

  //  // for each driving speed do (in m/s)
  //  for (float speed = 2; speed < 25; speed += 2) {
  //    float t_err = 0;
  //    float r_err = 0;
  //    float num   = 0;
  //
  //    // for all errors do
  //    for (std::vector<errors>::iterator it = seq_err.begin(); it != seq_err.end(); it++) {
  //      if (fabs(it->speed - speed) < 2.0) {
  //        t_err += it->t_err;
  //        r_err += it->r_err;
  //        num++;
  //      }
  //    }
  //
  //    // ds we require at least 2 lengths to be evaluated
  //    if (num > 1) {
  //      fprintf(fp_ts, "%f %f\n", speed, t_err / num);
  //      fprintf(fp_rs, "%f %f\n", speed, r_err / num);
  //    }
  //  }

  // close files
  fclose(fp_tl);
  fclose(fp_rl);
  //  fclose(fp_ts);
  //  fclose(fp_rs);
}

int32_t plotErrorPlots(std::string dir, const char* prefix) {
  char command[1024];

  // ds system calls
  int32_t result = 0;

  // for all four error plots do
  for (int32_t i = 0; i < 2; i++) {
    // create suffix
    char suffix[16];
    switch (i) {
      case 0:
        sprintf(suffix, "tl");
        break;
      case 1:
        sprintf(suffix, "rl");
        break;
      case 2:
        sprintf(suffix, "ts");
        break;
      case 3:
        sprintf(suffix, "rs");
        break;
    }

    // gnuplot file name
    char file_name[1024];
    char full_name[1024];
    sprintf(file_name, "%s_%s.gp", prefix, suffix);
    sprintf(full_name, "%s/%s", dir.c_str(), file_name);

    // create png + eps
    for (int32_t j = 0; j < 1 /*PNG only*/; j++) {
      // open file
      FILE* fp = fopen(full_name, "w");

      // save gnuplot instructions
      if (j == 0) {
        fprintf(fp, "set term png size 500,250 font \"Helvetica\" 11\n");
        fprintf(fp, "set output \"%s_%s.png\"\n", prefix, suffix);
      } else {
        fprintf(fp, "set term postscript eps enhanced color\n");
        fprintf(fp, "set output \"%s_%s.eps\"\n", prefix, suffix);
      }

      // start plot at 0
      fprintf(fp, "set size ratio 0.5\n");
      fprintf(fp, "set yrange [0:*]\n");

      // x label
      if (i <= 1)
        fprintf(fp, "set xlabel \"Path Length [m]\"\n");
      else
        fprintf(fp, "set xlabel \"Speed [km/h]\"\n");

      // y label
      if (i == 0 || i == 2)
        fprintf(fp, "set ylabel \"Translation Error [%%]\"\n");
      else
        fprintf(fp, "set ylabel \"Rotation Error [deg/m]\"\n");

      // plot error curve
      fprintf(fp, "plot \"%s_%s.txt\" using ", prefix, suffix);
      switch (i) {
        case 0:
          fprintf(fp, "1:($2*100) title 'Translation Error'");
          break;
        case 1:
          fprintf(fp, "1:($2*57.3) title 'Rotation Error'");
          break;
        case 2:
          fprintf(fp, "($1*3.6):($2*100) title 'Translation Error'");
          break;
        case 3:
          fprintf(fp, "($1*3.6):($2*57.3) title 'Rotation Error'");
          break;
      }
      fprintf(fp, " lc rgb \"#0000FF\" pt 4 w linespoints\n");

      // close file
      fclose(fp);

      // run gnuplot => create png + eps
      sprintf(command, "cd %s; gnuplot %s", dir.c_str(), file_name);
      result = system(command);
      std::cerr << "plotErrorPlots|wrote to: '" << dir + "/" + file_name << "'" << std::endl;
    }

    // create pdf and crop TODO re-enable
    //    sprintf(command,
    //            "cd %s; ps2pdf %s_%s.eps %s_%s_large.pdf",
    //            dir.c_str(),
    //            prefix,
    //            suffix,
    //            prefix,
    //            suffix);
    //    result = system(command);
    //    sprintf(command,
    //            "cd %s; pdfcrop %s_%s_large.pdf %s_%s.pdf",
    //            dir.c_str(),
    //            prefix,
    //            suffix,
    //            prefix,
    //            suffix);
    //    result = system(command);
    //    sprintf(command, "cd %s; rm %s_%s_large.pdf", dir.c_str(), prefix, suffix);
    //    result = system(command);
  }

  return result;
}

void saveStats(std::vector<errors> err, std::string dir) {
  float t_err = 0;
  float r_err = 0;

  // for all errors do => compute sum of t_err, r_err
  for (std::vector<errors>::iterator it = err.begin(); it != err.end(); it++) {
    t_err += it->t_err;
    r_err += it->r_err;
  }

  // open file
  FILE* fp = fopen((dir + "/stats.txt").c_str(), "w");

  // save errors
  float num = err.size();
  fprintf(fp, "%f %f\n", t_err / num, r_err / num);

  // close file
  fclose(fp);
  std::cerr << "saveStats|wrote to '" << dir + "/stats.txt"
            << "'" << std::endl;
}

int eval(const std::string& file_trajectory_test_,
         const std::string& file_trajectory_ground_truth_,
         const std::string& file_sequence_) {
  // ground truth and result directories
  const std::string result_dir     = "results";
  const std::string error_dir      = result_dir + "/errors";
  const std::string plot_path_dir  = result_dir + "/plot_path";
  const std::string plot_error_dir = result_dir + "/plot_error";

  // create output directories TODO evil evil
  int32_t result = system(("mkdir -p " + result_dir).c_str());
  result         = system(("mkdir -p " + error_dir).c_str());
  result         = system(("mkdir -p " + plot_path_dir).c_str());
  result         = system(("mkdir -p " + plot_error_dir).c_str());
  if (result < 0) {
    std::cerr << "eval|ERROR: system io error" << std::endl;
    return -1;
  }

  // read ground truth and result poses
  Matrix4fVector poses_gt     = loadPoses(file_trajectory_ground_truth_);
  Matrix4fVector poses_result = loadPoses(file_trajectory_test_);

  // ds parse sequence number
  const std::string sequence_number_literal = file_sequence_.substr(0, 2);
  const uint32_t sequence_number            = std::stoi(sequence_number_literal);

  // plot status
  printf("Processing: %s (sequence: %i), poses: %lu/%lu\n",
         file_sequence_.c_str(),
         sequence_number,
         poses_result.size(),
         poses_gt.size());

  // check for errors
  if (poses_gt.size() == 0 || poses_result.size() != poses_gt.size()) {
    std::cerr << "eval|ERROR: Couldn't read (all) poses of: " << file_sequence_ << std::endl;
    return -1;
  }

  // compute sequence errors
  std::vector<errors> seq_err = calcSequenceErrors(poses_gt, poses_result);
  saveSequenceErrors(seq_err, error_dir + "/" + file_sequence_);

  // for first half => plot trajectory and compute individual stats
  // save + plot bird's eye view trajectories
  savePathPlot(poses_gt, poses_result, plot_path_dir + "/" + file_sequence_);
  std::vector<int32_t> roi = computeRoi(poses_gt, poses_result);
  result                   = plotPathPlot(plot_path_dir, roi, sequence_number);
  if (result != 0) {
    std::cerr << "eval|ERROR: unable to plot path" << std::endl;
    return -1;
  }

  // save + plot individual errors
  saveErrorPlots(seq_err, plot_error_dir, sequence_number_literal.c_str());
  result = plotErrorPlots(plot_error_dir, sequence_number_literal.c_str());
  if (result != 0) {
    std::cerr << "eval|ERROR: unable to plot error statistics" << std::endl;
    return -1;
  }

  return result;
}

const std::string banner =
  "\n\nUsage: ./srrg_kitti_evaluate_odometry_app -gt <string> -odom <string> -seq <string>\n"
  "Example: ./srrg_kitti_evaluate_odometry_app -gt 00.txt -odom tracker_output.txt -seq 00.txt\n"
  "Options:\n"
  "------------------------------------------\n"
  "-gt <string>                path to ground_truth file\n"
  "-odom <string>              path to tracker odometry path\n"
  "-seq <string>               sequence number, e.g. '00.txt' if not set, then seq=gt\n"
  "-h                          this help\n";

// ds parameters
std::string file_tracked_odometry = "";
std::string file_ground_truth     = "";
std::string file_sequence         = "";

// ds parse parameters
int c = 1;

int32_t main(int32_t argc, char** argv) {
  // ds always expects 6 arguments
  if (argc != 7) {
    std::cerr << banner << std::endl;
    return -1;
  }

  // ds parameters
  std::string file_tracked_odometry = "";
  std::string file_ground_truth     = "";
  std::string file_sequence         = "";

  // ds parse parameters
  int c = 1;
  while (c < argc) {
    if (!strcmp(argv[c], "-h")) {
      std::cerr << banner << std::endl;
      return -1;
    } else if (!strcmp(argv[c], "-gt")) {
      c++;
      file_ground_truth = argv[c];
    } else if (!strcmp(argv[c], "-odom")) {
      c++;
      file_tracked_odometry = argv[c];
    } else if (!strcmp(argv[c], "-seq")) {
      c++;
      file_sequence = argv[c];
    }
    c++;
  }

  // ds input validation
  if (file_sequence.empty())
    file_sequence = file_ground_truth;

  if (file_sequence.find(".txt") == std::string::npos) {
    std::cerr << "ERROR: missing file ending (e.g. txt) in provided sequence file: "
              << file_sequence << std::endl;
    return -1;
  }

  // ds configuration
  std::cerr << "file_ground_truth: " << file_ground_truth << std::endl;
  std::cerr << "file_tracked_odometry: " << file_tracked_odometry << std::endl;
  std::cerr << "file_sequence: " << file_sequence << std::endl;

  // run evaluation
  return eval(file_tracked_odometry, file_ground_truth, file_sequence);
}
