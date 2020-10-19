#!/bin/bash
echo one
catkin_test_results > tmp_test_results.txt
echo two
cat tmp_test_results.txt
lines_string=$(wc -l tmp_test_results.txt)
echo three
lines_array=(${lines_string// / })
if [ ${lines_array[0]} -gt 1 ]
then
    echo Too many lines, it means something failed
    exit 1
else 
    echo All good
fi
exit 0
