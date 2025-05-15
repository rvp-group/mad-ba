#pragma once
#include "packets/packet_array.h" //ia almost deprecated
#include "packets/packet_base.h"
#include "packets/packet_cv_mat.h" // ds FIXME
#include "packets/packet_eigen_object.h"
#include "packets/packet_matchable_vector_cloud.h"
#include "packets/packet_point_matrix_cloud.h"
#include "packets/packet_point_vector_cloud.h"
#include "packets/packet_scalar.h"
#include "packets/packet_stl_vector.h"
#include "packets/packet_string.h"
#include "packets/packet_gl_list.h"

/* Irvin's advises on packet numbering.
 * Each packet MUST have a unique number to identify packet type,
 * otherwise memory leaks and stuff like this will happen.
 * packet UNIQUE ID numbering:
 *
 * - 0x00/0xFF -> special packets
 * - 0x0*      -> command packets
 * - 0x1*      -> scalar packets
 * - 0x2*      -> eigen packets
 * - 0x3*      -> pcl vector packets
 * - 0x4*      -> pcl vector packets
 * - 0x5*      -> FREE
 * - 0x6*      -> pcl matrix packets
 * - 0x7*      -> pcl matrix packets
 * - 0x8*      -> FREE
 * - 0x9*      -> matchable vector cloud packets
 * - 0xA*      -> text
 * - 0xB*      -> FREE
 * - 0xC*      -> OpenCV Image packets
 * - 0xD*      -> FREE
 * - 0xE*      -> array packets (almost deprecated)
 */

// ds ADD YOUR PACKET TYPE in packet_factory.h
