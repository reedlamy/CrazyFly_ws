#!/bin/bash
# Execute roslaunch file ('launch_mission.launch') for CF mission planning (w/ user inputs)

# Ask for static or mobile charger case (list)
replan=0
num_chargers=0
charge_type=$(zenity --width=150 --height=175 --list --radiolist \
             --title 'Crazyflie Mission Planning' \
             --text 'Select the type of chargers:' \
             --column 'Select' \
             --column 'Charger Type' TRUE "Static" FALSE "Mobile")
if [ -z $charge_type ]
then
    exit 0
elif [ $charge_type == "Mobile" ]
then
    replan=$(zenity --width=120 --height=175 --list --radiolist \
             --title 'Crazyflie Mission Planning' \
             --text 'Replanning Mission?' \
             --column 'Select' \
             --column 'Response' TRUE "Yes" FALSE "No")
    if [ -z $replan ]
    then
        exit 0
    fi
    [[ $replan == "Yes" ]] && replan=1 || replan=0

    # Ask for number of mobile chargers (text entry)
    num_chargers=$(zenity --entry --text "Enter number of mobile chargers:" \
           --title "Crazyflie Mission Planning")

    if [ -z $num_chargers ]
    then
        exit 0
    fi
fi

# Ask for Crazyflie numbers separated by comma (text entry) and make string of names
cf_nums=$(zenity --entry --text "Enter Crazyflie numbers separated by a comma:" \
       --title "Crazyflie Mission Planning" --entry-text "")
if [ -z $cf_nums ]
then
    exit 0
fi
cf_nums=$(echo $cf_nums | tr -d ' ')
IFS=','
read -ra cf_nums <<< "$cf_nums"
num_cfs=${#cf_nums[@]}

IFS=
cf_names="CF${cf_nums[0]}"
for((i=1;i<$num_cfs;i++))
do
    cf_names="${cf_names},CF${cf_nums[$i]}"
done

# Ask for desired number of charging stations (slider) and make string of names
##num_chargers=$(zenity --scale --title "Crazyflie Mission Planning" \
##               --text "Select the desired number of chargers" \
##               --min-value=0 --max-value=10 --step=1 --value=$num_cfs)
##if [ -z $num_chargers ]
##then
##    exit 0
##fi
##
##if [ $num_chargers -eq 0 ]
##then
##    pad_names=""
##else
##    pad_names="Pad1"
##    for((j=2;j<=$num_chargers;j++))
##    do
##        pad_names="${pad_names},Pad${j}"
##    done
##fi

# Ask for workspace dimensions
work_width=$(zenity --entry --text "Enter workspace X length (meters):" \
             --title "Crazyflie Mission Planning" --entry-text "3.5")
if [ -z $work_width ]
then
    exit 0
fi
work_length=$(zenity --entry --text "Enter workspace Y length(meters):" \
              --title "Crazyflie Mission Planning" --entry-text "4")
if [ -z $work_length ]
then
    exit 0
fi

# Ask for mission resolution (entry)
res=$(zenity --entry --text "Enter mission resolution (meters):" \
      --title "Crazyflie Mission Planning" --entry-text "0.2")
if [ -z $res ]
then
    exit 0
fi

# Ask for map name (entry)
map_name=$(zenity --entry --text "Enter .bmp map file name (no extension):" \
           --title "Crazyflie Mission Planning" --entry-text "map_mh370")
if [ -z $map_name ]
then
    exit 0
fi

# Launch Crazyflie Mission with roslaunch and input arguments

echo charge_type_arg:=$charge_type cf_names_arg:=$cf_names pad_names_arg:=$pad_names work_width_arg:=$work_width work_length_arg:=$work_length res_arg:=$res map_name_arg:=$map_name replan_arg:=$replan num_chargers_arg:=$num_chargers
                                       

roslaunch launch_mission.launch charge_type_arg:=$charge_type \
                                cf_names_arg:=$cf_names \
                                pad_names_arg:=$pad_names \
                                work_width_arg:=$work_width \
                                work_length_arg:=$work_length \
                                res_arg:=$res \
                                map_name_arg:=$map_name \
                                replan_arg:=$replan \
                                num_chargers_arg:=$num_chargers
                                            

exit 0


