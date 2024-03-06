#!/bin/bash

#conda deactivate
#d="deactivate"
#conda "$d"
#var=$(conda deactivate)
#echo $var
gnome-terminal --tab -e "bash -c 'conda deactivate; exec bash'" --title="rqt_logger_level"