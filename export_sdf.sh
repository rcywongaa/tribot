#!/bin/bash

rsdf_files=$(find -name "*.rsdf")
while read -r rsdf_file; do
    erb "${rsdf_file}" > "${rsdf_file%.*}".sdf
done <<< "${rsdf_files}"
