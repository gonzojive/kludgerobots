#!/bin/bash

#for i in 1 2 3 4 5 6 7 8 9 10 11 12
for (( i = 1; i <= 134; i++ ))
do
python circlesMain.py "traingen/training-pairs/input$i.dat" "traingen/training-pairs/label$i.dat"
done