#!/bin/bash

OUTPUT_FILE=user_package_list.json
OUTPUT_PATH=/tmp/${OUTPUT_FILE}

if [ $# -ne 1 ];then
  echo "usage: ${0} {USER_PACKAGES_BASE_DIR}"
  echo "USER_PACKAGES_BASE_DIR is usually the following directory:"
  echo "{Catkin workspace directory}/src/platform/user"
  exit 1
fi
BASE_DIR=${1}

echo "Output the list of user programs to ${OUTPUT_PATH}..."
# Create new file
echo "{" > ${OUTPUT_PATH}

echo "$(printf %2s)\"user_program_list\": [" >> ${OUTPUT_PATH}

ROSPACK_PATH_ARRAY=($(rospack list | grep ${BASE_DIR} | cut -d ' ' -f 2))
for ROSPACK_PATH in ${ROSPACK_PATH_ARRAY[@]}; do
  echo "$(printf %4s){" >> ${OUTPUT_PATH}

  PACK=`basename ${ROSPACK_PATH}`
  echo "$(printf %6s)\"user\": \"${PACK}\"," >> ${OUTPUT_PATH}


  echo "$(printf %6s)\"launch\": [" >> ${OUTPUT_PATH}
  LAUNCH_DIR_PATH=${ROSPACK_PATH}/launch
  LAUNCH_PATHS=($(find ${LAUNCH_DIR_PATH} -type f))
  for LAUNCH_FILE_PATH in ${LAUNCH_PATHS[@]}; do
    echo "$(printf %8s)\"$(basename ${LAUNCH_FILE_PATH})\"," >> ${OUTPUT_PATH}
  done
  # Remove the comma from the last line ($ s/ ...)
  sed -i -e "$ s/,\$//" ${OUTPUT_PATH}
  echo "$(printf %6s)]" >> ${OUTPUT_PATH}
 
  echo "$(printf %4s)}," >> ${OUTPUT_PATH}

done

# Remove the comma from the last line ($ s/ ...)
sed -i -e "$ s/,\$//" ${OUTPUT_PATH}
echo "$(printf %2s)]" >> ${OUTPUT_PATH}
echo "}" >> ${OUTPUT_PATH}

echo
echo "Done."

