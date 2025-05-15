#include <srrg_system_utils/system_utils.h>
#include <srrg_boss/id_context.h>
#include <srrg_boss/serializer.h>
#include <srrg_boss/deserializer.h>
#include <srrg_messages/instances.h>
#include <srrg_messages/message_handlers/message_file_source.h>

#define EXAMPLE_LOG std::cerr << "srrg2_core::examples::message_read_example| "

using namespace srrg2_core;
using namespace std;

const char* banner[]={
    "srrg_message_read_example: testing messages ",
    "usage: srrg_message_read_example <in> <out>",
    0
};

int main(int argc, char** argv) {
  messages_registerTypes();

  if (argc == 3) {
    EXAMPLE_LOG << "Read from file" << argv[1] << endl;
    EXAMPLE_LOG << "Write to file" << argv[2] << endl;
  } else {
    printBanner(banner);
    return 0;
  }

  MessageFileSource src;
  src.open(argv[1]);
  
  vector < BaseSensorMessagePtr > msgs;
 
  EXAMPLE_LOG << "Deserializing" << endl;
  BaseSensorMessagePtr msg;
  while( (msg=src.getMessage()) ){
    EXAMPLE_LOG << "Got Message: " << msg->className() << endl;
    msgs.push_back(msg);

    // Here you can dynamic_cast to a specific type
    // e.g.
    
    /*
      ImageMessage* img=dynamic_cast<ImageMessage*>(msg.get());
      if (img) {
      // and have access to its fields
      EXAMPLE_LOG << "img->image(): " << img->image() << endl;
      }
    */
  }

  
  EXAMPLE_LOG << "Serializing" << endl;
  Serializer ser;
  ser.setFilePath(argv[2]);
  for (size_t i=0; i<msgs.size();++i) {
    ser.writeObject( *(msgs[i].get()) );
  }

  EXAMPLE_LOG << "Destroying objects!" << endl;
}
