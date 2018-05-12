#!/bin/bash -e

readonly TEST_CASES="31131 3472 17912 26518 24316 26498 17006 29122 30835 25649 4720 1863 2364 9723 14518 3185 24729 29824 12846 27592 30758 12742 16828 7455 7162 30252 15281 4446 30850 32533 3341 3850 10782 23632 31349 19353 10872 2121 21234 19832 14249 22257 4719 25304 4362 26479 12273 14652 9690 1155 5012 25028 4500 8126 28745 20023 526 21348 32081 4817 25425 5916 19320 30930 27964 4958 12818 11715 28375 16281 4023 7046 29560 14119 23230 13605 7576 28913 15738 17495 21362 17890 14903 977 6497 29197 4850 4691 13815 31901 28892 2670 31708 12928 1933 19221 21869 8147 12296 15595"

make release

javac RoadsAndJunctionsVisForScoring.java

rm -f score.txt

test_case=1
for i in $(seq 1 16); do
  echo "Run "${test_case}"/116" ;
  java RoadsAndJunctionsVisForScoring -exec "./release" -seed $i >> score.txt
  test_case=$((test_case+1))
done

for i in ${TEST_CASES}; do
  echo "Run "${test_case}"/116" ;
  java RoadsAndJunctionsVisForScoring -exec "./release" -seed $i >> score.txt
  test_case=$((test_case+1))
done

awk '{s+=$5} END {printf "Total = %.13f\n", s}' score.txt | tee -a score.txt
