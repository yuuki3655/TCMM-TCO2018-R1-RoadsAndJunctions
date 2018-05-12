#!/bin/bash -e

prev=( $(git show HEAD:score.txt | sed '$d' | awk '{printf "%.13f\n", $NF}') )
curr=( $(sed '$d' score.txt | awk '{printf "%.13f\n", $NF}') )

win=0
lose=0
tie=0
for (( i=0; i<${#prev[@]}; i++ )); do
  if (( $(echo ${prev[i]}' > '${curr[i]} | bc -l) )); then
    win=$((win+1))
  elif (( $(echo ${prev[i]}' < '${curr[i]} | bc -l) )); then
    lose=$((lose+1))
  else
    tie=$((tie+1))
  fi
done

echo "win = "${win}
echo "lose = "${lose}
echo "tie = "${tie}
