add_executable(converter_clf_to_srrg converter_clf_to_srrg.cpp)
target_link_libraries(converter_clf_to_srrg
  srrg2_messages_library
)

add_executable(converter_icl_to_srrg converter_icl_to_srrg.cpp)
target_link_libraries(converter_icl_to_srrg
  srrg2_messages_library
)

add_executable(converter_tum_to_srrg converter_tum_to_srrg.cpp)
target_link_libraries(converter_tum_to_srrg
  srrg2_messages_library
)

add_executable(converter_kitti_to_srrg converter_kitti_to_srrg.cpp)
target_link_libraries(converter_kitti_to_srrg
  srrg2_messages_library
  stdc++fs
)

add_executable(converter_euroc_to_srrg converter_euroc_to_srrg.cpp)
target_link_libraries(converter_euroc_to_srrg
  srrg2_messages_library
)

add_executable(converter_malaga_to_srrg converter_malaga_to_srrg.cpp)
target_link_libraries(converter_malaga_to_srrg
  srrg2_messages_library
)

add_executable(extract_gt_from_srrg extract_gt_from_srrg.cpp)
target_link_libraries(extract_gt_from_srrg
  srrg2_messages_library
)

# add_executable(geigerzaehler geigerzaehler.cpp)
# target_link_libraries(geigerzaehler
#   srrg2_system_utils_library
# )

add_executable(image_matcher_frobenius image_matcher_frobenius.cpp)
target_link_libraries(image_matcher_frobenius
  srrg2_messages_library
)

add_executable(image_matcher_features image_matcher_features.cpp)
target_link_libraries(image_matcher_features
  srrg2_messages_library
)
