#!/bin/bash

CONTAINER_PREFIX=ib2
OUTPUT_FILE=container_image_list.json
OUTPUT_PATH=/tmp/${OUTPUT_FILE}

echo "Output the list of container images (container with a ${CONTAINER_PREFIX} prefix) to ${OUTPUT_PATH}..."
# Create new file
echo "{" > ${OUTPUT_PATH}

echo "$(printf %2s)\"container_image_list\": [" >> ${OUTPUT_PATH}

CONTAINER_IMAGE_ARRAY=($(docker images --format "{{.Repository}}:{{.Tag}}" | grep -e "^${CONTAINER_PREFIX}"))
for CONTAINER_IMAGE in ${CONTAINER_IMAGE_ARRAY[@]}; do
  echo "$(printf %4s)\"${CONTAINER_IMAGE}\"," >> ${OUTPUT_PATH}
done
  # Remove the comma from the last line ($ s/ ...)
  sed -i -e "$ s/,\$//" ${OUTPUT_PATH}

echo "$(printf %2s)]" >> ${OUTPUT_PATH}
echo "}" >> ${OUTPUT_PATH}

echo
echo "Done."

