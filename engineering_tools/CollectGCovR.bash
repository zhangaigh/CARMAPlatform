
#!/bin/bash

# This script is meant to generate .gcov files from source code built using code coverage compiler flags
# such that .gcno and .gcda files have been generated

# Helper function to convert the results of find into a bash array
# This runs on the current directory 
#
# $1 An initialized array
# $2 The regex pattern to find this is passed to the -iname command
# $3 Any additional inputs to find
find_as_array() {
	local -n ref_array=$1 # Pass the output array by reference
	# Run the find command and store the results in the array
	while IFS=  read -r -d $'\0'; do
	    ref_array+=("$REPLY")
	done < <(find . -iname "$2" -print0 $3)
}

echo "Running gcovr"
gcovr -k -r . # Run gcovr with -k to ensure generated .gcov files are preserved -r . makes it run in the current directory

# Grab resulting gcov files and move them to the ./coverage_reports directory
gcov_file_array=()
find_as_array gcov_file_array "*.gcov" "-type f"

# Print the array if needed
#echo "GCov Array"
#printf '%s\n' "${gcov_file_array[@]}"

echo "Moving Files"
mkdir coverage_reports/gcov

for gcov_file in "${gcov_file_array[@]}"
do
   base_file_name=${gcov_file/$(dirname ${gcov_file})/''} 
   mv ${gcov_file} ./coverage_reports/gcov/${base_file_name}
done

echo "Files Moved"

exit 0

