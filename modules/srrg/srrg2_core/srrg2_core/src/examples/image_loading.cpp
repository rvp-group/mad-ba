#include <iostream>
#include <srrg_data_structures/matrix.h>
#include <srrg_system_utils/system_utils.h>

#include <srrg_image/image_data.h>

#include <vector>

#define EXAMPLE_LOG std::cerr << "srrg2_core::examples::image_loading| "

using namespace std;
using namespace srrg2_core;

const char * banner[]={
    "srrg_image_loading: load and manipulate openCV/srrg images",
    "                  : full test runs w/ a xtion depth image",
    "usage: srrg_image_loading <image_file>",
    0
};


int main(int argc, char ** argv) {

  if(argc < 2 || !strcmp(argv[1],"-h")) {
    printBanner(banner);
    return 0;
  }
  
  cv::Mat cv_image = cv::imread(argv[1], cv::IMREAD_UNCHANGED);
  cv::imshow("read image", cv_image*5);
  EXAMPLE_LOG << "OpenCV image reading" << std::endl;    
  EXAMPLE_LOG << "Press any Key to continue" << std::endl;
  cv::waitKey(0);

  // go to srrg
  ImageUInt16 img_u16;
  img_u16.fromCv(cv_image);
  cv::Mat cv_u16;
  img_u16.toCv(cv_u16);
  cv::imshow("re-converted", cv_u16*5);
  EXAMPLE_LOG << "SRRG image after conversion" << std::endl;    
  EXAMPLE_LOG << "Press any Key to continue" << std::endl;
  cv::waitKey(0); 

  
  //bdc create a mask with 1 where the current image contains 0
  ImageUInt8 mask = (img_u16 == 0);
  cv::Mat cv_mask;
  mask.toCv(cv_mask);
  cv::imshow("mask", cv_mask*1000);
  EXAMPLE_LOG << "Mask Image of 0s elements" << std::endl;    
  EXAMPLE_LOG << "Press any Key to continue" << std::endl;
  cv::waitKey(0); 
  
  
  //bdc check size and compare it with opencv size
  cv::Size opencv_size = cv_image.size();
  ImageSize srrg_size = img_u16.imageSize();
  if(srrg_size == opencv_size)
    EXAMPLE_LOG << "Image Size check (comparison w/ openCV)" << std::endl; 
  // convert to float and multiply each element by 1e-3
  ImageFloat img_f;
  img_u16.convertTo(img_f, 1e-3);
  // see what you get
  cv::Mat cv_img_f;
  img_f.toCv(cv_img_f);
  cv::imshow("img_f", cv_img_f);
  EXAMPLE_LOG << "Converted Image from mm to meters" << std::endl; 
  EXAMPLE_LOG << "Press any Key to continue" << std::endl;     
  cv::waitKey(0); 
  

  return 0;
}
