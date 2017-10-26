#!/bin/bash 
COUNTER=0
touch motorfailure_10epucks.txt
touch temp.txt;
touch temp2.txt;
while [  $COUNTER -lt 100 ]; do
    argos3 -c experiments/epuck_brownian_motorfailure_10epucks > temp.txt
    sed '/t=/!d' temp.txt > temp2.txt
    sed 's/t= //' temp2.txt > temp.txt
    sed -e '1,2d;4,9d' temp.txt >> motorfailure_10epucks.txt
done
rm temp.txt
rm temp2.txt
