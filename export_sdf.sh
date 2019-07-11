#!/bin/bash
DIR=$(dirname "$(readlink -f "$0")")

rsdf_files=$(find "$DIR" -name "*.rsdf")
while read -r rsdf_file; do
    erb "${rsdf_file}" > "${rsdf_file%.*}".sdf
done <<< "${rsdf_files}"
