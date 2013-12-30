#!/bin/bash


function float_cond()
{
    local cond=0
    if [[ $# -gt 0 ]]; then
        cond=$(echo "$*" | bc -q 2>/dev/null)
        if [[ -z "$cond" ]]; then cond=0; fi
        if [[ "$cond" != 0  &&  "$cond" != 1 ]]; then cond=0; fi
    fi
    local stat=$((cond == 0))
    return $stat
}

echo Running Iteration 0
minVal="$(./SPARSE-RRT_test Solution0.txt)"
echo Iteration 0 starts with value $minVal
for i in `seq 1 100`;
do
    echo Running Iteration $i
    filename="Solution$i.txt"
    val="$(./SPARSE-RRT_test $filename)"
    if float_cond "$val < $minVal"; then
	cp $filename Solution.txt
	echo Minimum is $i with value $val
	minVal=$val
    fi
done

unset filename
unset val
