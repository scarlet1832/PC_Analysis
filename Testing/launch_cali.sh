#!/bin/bash

launch_help()
{
    echo "Usage: [ -r | --ros_version ]
       [ -t | --tag ]
       [ -y | --yaml ]
       [ -h | --help  ]"
    exit 2
}
launch_cali()
{
  SHORT=r:,t:,y:,h
  LONG=ros_version:,tag:,yaml:,help
  OPTS=$(getopt -a -n launchCali --options $SHORT --longoptions $LONG -- "$@")
  docker_path="/home/demo/Desktop/ivusw-rosprocessor-0fa22e86a17b/build/docker"
  ppp="internal"
  VALID_ARGUMENTS=$# # Returns the count of arguments that are in short or long options

  if [ "$VALID_ARGUMENTS" -eq 0 ]; then
    launch_help
  fi

  eval set -- "$OPTS"

  while :
  do
    case "$1" in
      -r | --ros_version )
        ros_version="$2"
        shift 2
        ;;
      -t | --tag )
        tag="$2"
        shift 2
        ;;
      -y | --yaml )
        yaml="$2"
        shift 2
        ;;
      -h | --help)
        help
        ;;
      --)
        shift;
        break
        ;;
      *)
        echo "Unexpected option: $1"
        help
        ;;
    esac
  done

  cmd="--no-check-certificate -t 20 -T 10 https://s3-us-west-2.amazonaws.com/iv-bitbucket-pipeline/build/TAG/${tag}/ros-${ros_version}-innovusion-driver-${tag}-${ppp}.deb -P ${docker_path} --user-agent=Mozilla/5.0"
  FILE="$docker_path/ros-${ros_version}-innovusion-driver-${tag}-${ppp}.deb"
  if [[ -f "$FILE" ]]; then
    echo "$FILE exists."
  else
    echo "$FILE not exists, start download."
    wget $cmd
  #  rm $FILE
  fi
  echo "start install ros-${ros_version}-innovusion-driver-${tag}-${ppp}.deb"
  sudo dpkg -i $FILE
  echo "start visualization"
  conda "deactivate"
  if [[ -n $yaml ]]; then
     calibration="calibration:=$yaml"
     echo "use yaml: $yaml"
  fi
  sleep 1
  gnome-terminal --tab -- "bash -c 'roslaunch innovusion_pointcloud innovusion_points.launch publish_cali_data:=true ${calibration}; exec bash'" --title="roslaunch"
  gnome-terminal --tab -- "bash -c 'rviz -d /home/demo/Desktop/Innovusion.rviz; exec bash'" --title="rviz"
  gnome-terminal --tab -- "bash -c 'rosrun rqt_reconfigure rqt_reconfigure; exec bash'" --title="rqt_reconfigure"
  gnome-terminal --tab -- "bash -c 'rosrun rqt_logger_level rqt_logger_level; exec bash'" --title="rqt_logger_level"
}