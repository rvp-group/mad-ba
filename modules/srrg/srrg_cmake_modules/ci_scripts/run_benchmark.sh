#!/bin/bash
#ds this script is intended to be sourced!
#ds it performs dataset download and executes the specified benchmark
#ds it is an disk space friendly alternative to the run_benchmarks.sh version since downloaded datasets are removed after processing

#ds check input parameters
if [ "$#" -ne 5 ]; then
  echo "ERROR: call as $0 BENCHMARK_BINARY_ABSOLUTE_PATH DATASET_NAME SEQUENCE_NAME RESULT_TOOL_ABSOLUTE_PATH RESULT_PATH"
  exit -1
fi

#ds check if required external variables are not set
if [ -z "$SRRG_SCRIPT_PATH" ]; then
  echo "environment variable SRRG_SCRIPT_PATH not set"
  exit -1
fi

#ds set parameters
echo -e "\e[1;96m--------------------------------------------------------------------------------\e[0m"
echo -e "\e[1;96mbash version: '${BASH_VERSION}'\e[0m"
BENCHMARK_BINARY_ABSOLUTE_PATH="$1"
DATASET_NAME="$2"
SEQUENCE_NAME="$3"
RESULT_TOOL_ABSOLUTE_PATH="$4"
RESULT_PATH="$5"
LOCAL_DATASET_PATH="/datasets/" #ds TODO parametrize

#ds known datasets (public) in a map <name, google drive id> ADD YOUR DATASET_SEQUENCE HERE
#ds TODO move this list into a separate file
declare -A DATASET_SEQUENCES_AVAILABLE=(
["kitti_00"]="1zV3l9cy938XnL7BvO_iuvEOBYUC6_7p3"
["kitti_01"]="10k1mEADiqXtTr-YQ2aJpD_6oy69Wn6rz"
["kitti_02"]="1h-tPOM9qMkme0SNmw3gryDXaZCTqzCd4"
["kitti_03"]="1Qm8CPyEvCOYN4iabbeqaXZfXFfS9S3UL"
["kitti_04"]="1XpvOnzO4QVKGRODzp2E4uEiAx_WJXZYX"
["kitti_05"]="16cBc6L2JFNwOYmJeU1_y-SnbDKeCuZKZ"
["kitti_06"]="1uwtM-D4v1mRHJ2rkm2VPi8cCOA4aBdsm"
["kitti_07"]="1Sg7Tf1M94jcpsBhQoghKJYAwvi8gGRF8"
["kitti_08"]="16zmwqT9sSI_w4uFVCy3lD9fFjQ0qmuLu"
["kitti_09"]="1RNCtunrLxEEPHx23s_vosKCb6a1hWJ3N"
["kitti_10"]="1akWTiQWPHFBaVpI1qKPT-bmuY9LSa-Nd"
#["kitti_11"]="1kwe481bbG3sFIbJyvJdDnp_whjsPt44_"
#["kitti_12"]="1PtQcGBGUdgKshbQH7FwM3KyxKMI7t7V9"
#["kitti_13"]="1znn-nyTJVA5PA56tY66MCRmJA_sC3nNa"
#["kitti_14"]="1UvmW1HHIoNClozzT2eEWZgS8k926flI0"
#["kitti_15"]="1E27-YwuBJhCY-707NyJEymKLLzkMflFc"
#["kitti_16"]="1FveSOkpF5XW5XzP0ziz_GUTKmbCPaA01"
#["kitti_17"]="1ft8zJZEOmS8Z5dFO1Iv4xxKMlO2J-2j9"
#["kitti_18"]="10BMr-a177we9EgPxsHAMB7N9jKYfaWe0"
#["kitti_19"]="1AgwWdx1BHnY55z7eLntUCiQchzsLpcHr"
#["kitti_20"]="1SVxUHdX8eRphIGcWPwbiqS6pKN62RCDd"
#["kitti_21"]="15ci2MHHRPbuVwvGhPb83bI2MnFKPBYuZ"
["icl_living_room_0"]="1HBEmz0qBxFUTrk1K4pJIUNPpJYhgMHYD"
["icl_living_room_1"]="1kojNwhrWbdK3nR08M2vIUv66wW-NZHo1"
["icl_living_room_2"]="1lJW4CcPiZmPOwJtMJR2kdfKc2sWtsJ_F"
["icl_living_room_3"]="17CaZbEGhBUMFwxTJSv71yl6VNtMF7i9k"
["icl_office_room_0"]="11_Ms0a9RVgjwEVQ88CzTzP_DOR7ZpbgR"
["icl_office_room_1"]="156ULtiSmisW42kccWE43kjr55qkj7gbt"
["icl_office_room_2"]="1Qnx4En3SnfaUCOfiiyu5DFpBywkoLdp7"
["icl_office_room_3"]="1ENF8aKAi9z92T8Res0vlY7O-icBLUU_F"
["tum_fr1_room"]="1qPavF3iHuoeG7P_cVUUQ_nyXKE0miR-z"
["tum_fr2_desk"]="1oE9VPUYcu5XLzFI15XnuyO-d6Ok_EttC"
["tum_fr2_large_with_loop"]="16T5ObpdfqTjVah7WE1aAtQiRLOwPOyMd"
["tum_fr3_long"]="16zdgsLWRiyLrDXokm9dp6w7I6ZE5Gqqv"
["euroc_mh_01"]="1g6_QudgZetukSTe6MwYxauaei_8n_lza"
["euroc_mh_02"]="1XzSHNZtEnt67D9Twb_kOwuA-cSlgB7LE"
["euroc_mh_03"]="1mG30GL0WvClVWpDxWkOtCwWvFgNcK3Bv"
["euroc_mh_04"]="1ylbX7EdKkOVRl7CURBbgLcyo3ou1rS1E"
["euroc_mh_05"]="10l5x4rXDy-wT6dKArQJj-ZIgtJYPPf4a"
["euroc_v1_01"]="12eV1sR9CxSfsY1_aMoP8Qkh0Yu2LLzOB"
["euroc_v1_02"]="1OX0VAU1GpTqSyEdissCDn-447Q1qZkU4"
["euroc_v1_03"]="10KSoatF7z4tOZfW0zyPawJkMURMV7bKY"
["euroc_v2_01"]="1RBZlWS4enKmH5hmfIGx-8AaWcrvYMXRB"
["euroc_v2_02"]="1ZzuP_BqMGF86YlbKtUtz3M3W7v8WDSHr"
["euroc_v2_03"]="1g8oymW8y0dN8cJlwUj-sBx4uYp9jKVIp"
["malaga_01"]="1sPh8JJ7CR6f5unSd9cXK7vu3P0tfCMIz"
["malaga_02"]="1g57csCzlUxnVbQrtxBSsjmd9_wkPcq17"
["malaga_03"]="1zFUvdkUj63MEZrZIMmGyq9stlEgPjY2Y"
["malaga_04"]="100iMTF-17LtR6meT8mxiTEHUPbsxJDYG"
["malaga_05"]="15NNCvZV16nD-bIAqD7FodT6DxlUm426J"
["malaga_06"]="1QFjWk2bFdeewzKYonMyVDXDJAqDze8jO"
["malaga_07"]="1AMyrlETNiXevqSKjzPAZfgL8aQaCD1L5"
["malaga_08"]="1NZdzB1x2dqSRbFuVv_GvLBRbB-Y58oz1"
["malaga_09"]="1xZYcXujAmUIAeV_8CkkDn5r4gkfUO0PH"
["malaga_10"]="1vX0teUsoy-h2C_DGeGxL_BhaqpKvRf4U"
["malaga_11"]="1lsVLHXSaZBrkNML2fP2k_2Qri-YBhh1x"
["malaga_12"]="12ElGG6h0rr8KOrU23JqPjUUkx-kffOiy"
["malaga_13"]="1AsLIcmV-3cCMRrOu55A8KzDe3aJLcVUR"
["malaga_14"]="1vfp52gOsr-99jf280QxRi1hj-xD81E-3"
["malaga_15"]="1hqDrJrOmpLxcPNfG94n9amI9brvTDvJl"
)

#ds retrieve dataset (if available)
DATASET_SEQUENCE_NAME="${DATASET_NAME}_${SEQUENCE_NAME}"
DATASET_DRIVE_ID=${DATASET_SEQUENCES_AVAILABLE[$DATASET_SEQUENCE_NAME]}
echo "DATASET_DRIVE_ID: '${DATASET_DRIVE_ID}'"

#ds if dataset download ID could not be retrieved, escape with failure
#ds a file that is not available for download is also not available on disk
if [ -z "$DATASET_DRIVE_ID" ]; then
  echo "dataset sequence with name: '${DATASET_SEQUENCE_NAME}' is not registered"
  exit -1
fi

#ds create dataset folder
mkdir -p "$DATASET_SEQUENCE_NAME"

#ds check if the chosen sequence is provided locally
echo -e "\e[1;96mavailable local datasets in '$LOCAL_DATASET_PATH':\e[0m"
ls "$LOCAL_DATASET_PATH"

#ds loop over all dataset names and check if one matches the requested dataset name
LOCAL_DATASET_SEQUENCE_PATH=""
AVAILABLE_DATASETS=($(ls $LOCAL_DATASET_PATH))
for AVAILABLE_DATASET in "${AVAILABLE_DATASETS[@]}"
  do
    if [[ "$AVAILABLE_DATASET" == "$DATASET_NAME" ]]; then
      echo -e "\e[1;96mfound matching dataset: '$AVAILABLE_DATASET' containing sequences:\e[0m"
      ls "${LOCAL_DATASET_PATH}/${AVAILABLE_DATASET}"
      AVAILABLE_SEQUENCES=($(ls ${LOCAL_DATASET_PATH}/${AVAILABLE_DATASET}))
      for AVAILABLE_SEQUENCE in "${AVAILABLE_SEQUENCES[@]}"
        do
          if [[ "$AVAILABLE_SEQUENCE" == "$SEQUENCE_NAME" ]]; then
            echo -e "\e[1;96mfound matching sequence: '$AVAILABLE_SEQUENCE' containing files:\e[0m"
            LOCAL_DATASET_SEQUENCE_PATH="${LOCAL_DATASET_PATH}/${AVAILABLE_DATASET}/${AVAILABLE_SEQUENCE}"
            ls "$LOCAL_DATASET_SEQUENCE_PATH"
          fi
      done
    fi
done

#ds if we could not locate the dataset sequence on disk
if [ -z "$LOCAL_DATASET_SEQUENCE_PATH" ]; then
  echo -e "\e[1;96munable to locate matching dataset sequence - download required\e[0m"

  #ds download dataset into a temporary folder that corresponds to the drive ID
  source ${SRRG_SCRIPT_PATH}/drive_download_and_extract_file.sh "$DATASET_DRIVE_ID" "$DATASET_SEQUENCE_NAME"

  #ds move into the extracted folder (exported variable by download script)
  EXTRACTED_FOLDER=($(ls ${DATASET_SEQUENCE_NAME}))
  echo "EXTRACTED_FOLDER: '${EXTRACTED_FOLDER[0]}'"
  cd "${DATASET_SEQUENCE_NAME}/${EXTRACTED_FOLDER[0]}"
else
  echo -e "\e[1;96mfound dataset sequence '${LOCAL_DATASET_SEQUENCE_PATH}' - no download necessary!\e[0m"

  #ds establish symlinks in target folder
  echo -e "\e[1;96msetting up symlinks:\e[0m"
  cd "${DATASET_SEQUENCE_NAME}"
  ln -s "${LOCAL_DATASET_SEQUENCE_PATH}/binary/"
  ln -s "${LOCAL_DATASET_SEQUENCE_PATH}/messages.json"
  ln -s "${LOCAL_DATASET_SEQUENCE_PATH}/gt.txt"

  #ds specific symlinks TODO blast once unified
  if [[ $DATASET_NAME == "kitti" ]]; then
    ln -s "${LOCAL_DATASET_SEQUENCE_PATH}/times.txt"
    ln -s "${LOCAL_DATASET_SEQUENCE_PATH}/calib.txt"
  fi
  if [[ $DATASET_NAME == "malaga" ]]; then
    ln -s "${LOCAL_DATASET_SEQUENCE_PATH}/times.txt"
  fi
  if [[ $DATASET_NAME == "euroc" ]]; then
    ln -s "${LOCAL_DATASET_SEQUENCE_PATH}/gt.csv"
  fi

  #ds list created links
  ls -al
fi

#ds run benchmark binary (absolute path must be provided)
$($BENCHMARK_BINARY_ABSOLUTE_PATH)

#ds run visualization tools (if applicable) TODO map as well reduce this boilerplate code
if [[ $DATASET_NAME == "kitti" ]]; then
  echo "running KITTI evaluation tools"
  ${RESULT_TOOL_ABSOLUTE_PATH} -gt "gt.txt" -odom "trajectory.txt" -seq "${SEQUENCE_NAME}.txt"
  cp "results/plot_path/${SEQUENCE_NAME}.png" "${RESULT_PATH}/kitti/${SEQUENCE_NAME}/"
  cp "results/plot_error/${SEQUENCE_NAME}_tl.png" "${RESULT_PATH}/kitti/${SEQUENCE_NAME}/"
  cp "results/plot_error/${SEQUENCE_NAME}_rl.png" "${RESULT_PATH}/kitti/${SEQUENCE_NAME}/"
fi
if [[ $DATASET_NAME == "icl" ]]; then
  echo "running ICL evaluation tools"
  ${RESULT_TOOL_ABSOLUTE_PATH} "gt.txt" "trajectory.txt" --plot "trajectory_error.png"
  cp "trajectory_error.png" "${RESULT_PATH}/icl/${SEQUENCE_NAME}/"
fi
if [[ $DATASET_NAME == "tum" ]]; then
  echo "running TUM evaluation tools"
  ${RESULT_TOOL_ABSOLUTE_PATH} "gt.txt" "trajectory.txt" --plot "trajectory_error.png"
  cp "trajectory_error.png" "${RESULT_PATH}/tum/${SEQUENCE_NAME}/"
fi
if [[ $DATASET_NAME == "euroc" ]]; then
  echo "running EuRoC evaluation tools"
  ${RESULT_TOOL_ABSOLUTE_PATH} "gt.txt" "trajectory.txt" --plot "trajectory_error.png"
  cp "trajectory_error.png" "${RESULT_PATH}/euroc/${SEQUENCE_NAME}/"
fi
if [[ $DATASET_NAME == "malaga" ]]; then
  echo "running Malaga evaluation tools"
  ${RESULT_TOOL_ABSOLUTE_PATH} -gt "gt.txt" -odom "trajectory.txt" -seq "${SEQUENCE_NAME}.txt"
  cp "results/plot_path/${SEQUENCE_NAME}.png" "${RESULT_PATH}/malaga/${SEQUENCE_NAME}/"
  cp "results/plot_error/${SEQUENCE_NAME}_tl.png" "${RESULT_PATH}/malaga/${SEQUENCE_NAME}/"
  cp "results/plot_error/${SEQUENCE_NAME}_rl.png" "${RESULT_PATH}/malaga/${SEQUENCE_NAME}/"
fi

#ds cleanup benchmark files
ls -al
cd "../.."
rm -rf "$DATASET_SEQUENCE_NAME"
echo -e "\e[1;96m--------------------------------------------------------------------------------\e[0m"
