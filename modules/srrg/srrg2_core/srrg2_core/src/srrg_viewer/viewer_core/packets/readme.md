## Viewer Packets 
The viewer works with packets. Each time you call a canvas `putX` function, the canvas creates a proper
packet, copies the payload passed and serializes the packet onto the viewer buffer.
Each packet has a **UNIQUE** packet type (a `uint8_t`) that will be used a different levels of the 
viewer to identify and process a packet.

#### How to extend packets
Packets are grouped by their type of payload (since the serialization will be different). Therefore
we have:
  - `packet_base.h`: Contains `PacketBase` which is the base class for packets ( it only has its unique type ). Here there are also `PacketCommands_`  - which can be use for specific payloadless gl commands (e.g. popAttribute) - and  `PacketInfo_` - has a sequence number in it.
  - `packet_eigen.h`: Contains packet with static Eigen payloads (e.g. a Vector3f). Can be used for many gl actions (putColor, putBox...)
  - `packet_array.h`: Packets with array of Eigen objects (e.g. array of Vector3f)
  - `packet_scalar.h`: Contains only a scalar (e.g. a double). Used for gl commands and actions (setPointSize, putSphere)
  - `packet_stl_vector.h`: Generic packet that contains an stl vector. This is an intermediate class. 
  - `packet_point_vector_cloud.h`: Packets containing `VectorClouds` are all here. Derived from `PacketStlVector`.
  - `packet_matchable_vector_cloud.h`: Packets containing matchable stl vector. Derived from `PacketStlVector`.
  - `packet_point_matrix_cloud.h`: Packets containing `MatrixClouds`. Treated differently w.r.t. stl containers.
  - `packet_string.h`: Contains text. Size is fixed to 255 characters.
  - `packet_cv_mat.h`: [DISMISSED] Payload is a cv::Mat

Depending on what you need you should specialize the proper packet.

In order to create your packet you should:
  1. Create your packet (specializing the proper base class depending on the payload)
  2. Assign a **UNIQUE** packet type
  3. Add your packet to the `PacketFactory` with the macro `add_packet_type(PacketType);`
  4. [recommended] test serialization and deserialization in `test_viewer_packets_serialization.h`
	
#### Packets ID Map
Here you can find an overview of the packet types. When you add a new packet, please keep the type consistent with its group (obviously when it's possible)
  - `0x00` and `0xFF`: Special packets, cannot by changed
  - `0x0x`: Base packets, see `packet_base.h`
  - `0x1x`: Scalar payload packets, see `packet_scalar.h`
  - `0x2x`: Eigen payload packets, see `packet_eigen_objects.h`
  - `0x3x` + `0x4x` + `0x5x`: Point vector cloud payload packets, see `packet_point_vector_cloud.h`
  - `0x6x` + `0x7x` + `0x8x`: Point matrix cloud payload packets, see `packet_point_matrix_cloud.h`
  - `0x9x`: Matchable vector payload packets, see `packet_matchable_vector_cloud.h`
  - `0xAx`: String payload packets, see `packet_string.h`
  - `0xCx`: cv::Mat payload packets, see `packet_cv_mat.h` [DISMISSED]
  - `0xEx`: Array payload packets, see `packet_array.h`
	
All those packets id are reported also in `packets.h`.
	
# BE SURE THAT YOUR NEW PACKET ID IS NOT ALREADY TAKEN OTHERWISE YOU BRAKE EVERYTHING. PLEASE KEEP THE FILE TIDY AND ORGANIZED, IT CAN EASILY BECOME UNREADABLE. KEEP IN MIND THAT THIS STUFF IS REALLY FRAGILE, SO BE CAREFUL WHEN YOU MODIFY IT :)