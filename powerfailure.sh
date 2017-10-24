#!/bin/bash 
         COUNTER=0
         while [  $COUNTER -lt 100 ]; do
             echo The counter is $COUNTER
             argos3 -c experiments/epuck_brownian_powerfailure.argos
             let COUNTER=COUNTER+1 

         done

