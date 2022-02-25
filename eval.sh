#!/bin/bash

function usage () {
  echo "ERROR: args invalid"
  echo "USAGE: eval.bash PLATFORM APP [LEN_STR]"
  echo "  PLATFORM : \"asp3\", \"mbed\""
  echo "  APP      : \"string\", \"uint16\", \"twist\""
  echo "  [LEN_STR]: the length of \"string\""
}

### setup operation ###
if [ $# -ne 2 -a $# -ne 3 ];
then
  usage
  exit 1
else
  PLATFORM=$1
  APP=$2
  APPNAME=mros2_eval_${APP}
  if [ ${APP} = "string" ];
  then
    if [ $# -eq 3 ];
    then
      LEN_STR=$3
    else
      usage
      exit 1
    fi
  fi
fi

### run evaluation ###
echo "INFO: make sure \"${APP}\" with \"${PLATFORM}\" is ready to pub/sub on the board"
read -n1 -rsp $'    : press any key to continue or Ctrl+C to exit...\n'
echo "INFO: evaluation start"

cd host_ws
source install/local_setup.bash

if [ ${APP} = "string" ];
then
  ros2 run ${APPNAME} sub_node ${PLATFORM} ${LEN_STR} &
  pid_sub=$!
  sleep 1
  ros2 run ${APPNAME} pub_node ${PLATFORM} ${LEN_STR}
  pid_pub=$!
  wait $pid_sub $pid_pub
else
  ros2 run ${APPNAME} sub_node ${PLATFORM} &
  pid_sub=$!
  sleep 1
  ros2 run ${APPNAME} pub_node ${PLATFORM}
  pid_pub=$!
  wait $pid_sub $pid_pub
fi

cd ..
echo "INFO: evaluation of \"${APP}\" with \"${PLATFORM}\" finished"
exit 0