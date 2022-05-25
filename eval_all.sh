#!/bin/bash

ARRAY_LEN_STR=(1 2 4 8 16 32 64)

function usage () {
  echo "ERROR: args invalid"
  echo "USAGE: eval_all.bash PLATFORM"
  echo "  PLATFORM : \"uros-serial\" \"uros-udp\" \"uros-rtps\" \"mros2-asp3\" \"mros2-mbed\""
}

### setup operation ###
if [ $# -ne 1 ];
then
  usage
  exit 1
else
  PLATFORM=$1
fi

### create directory in `results/` ###
mkdir -p results/${PLATFORM}

### run evaluation ###
echo "INFO: all evaluation for \"${PLATFORM}\" start"

echo "INFO: evaluation for \"string\" start"
for LEN_STR in ${ARRAY_LEN_STR[@]} ;
do
  echo "INFO: please RESET the board"
  read -n1 -rsp $'    : press any key to continue or Ctrl+C to exit...\n'
  ./eval.sh ${PLATFORM} string ${LEN_STR}
done
echo "INFO: evaluation for \"string\" end"
echo ""

echo "INFO: evaluation for \"uint16\" start"
./eval.sh ${PLATFORM} uint16
echo "INFO: evaluation for \"uint16\" end"
echo ""

echo "INFO: evaluation for \"twist\" start"
./eval.sh ${PLATFORM} twist
echo "INFO: evaluation for \"twist\" end"
echo ""

echo "INFO: all evaluation for \"${PLATFORM}\" finished"
exit 0