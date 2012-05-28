###
# FitPC2i sensors aren't working with lm-sensors.
# Denis on the forums wrote a bash script for this (http://www.fit-pc.com/forum/viewtopic.php?f=9&t=401)
# Works OK, needs root.
#

#!/bin/bash

if [[ $EUID -ne 0 ]]; then
  echo "You must be a root user" 2>&1
  exit 1
fi

PCI_DEV_ADDR=00:0.0
REG_MCR=D0
REG_MDR=D4

B0_INIT=FFFFFFFF
B0_EXIT=00000000


function init_thermal {
        setpci -s $PCI_DEV_ADDR $REG_MDR.l=$B0_INIT
        setpci -s $PCI_DEV_ADDR $REG_MCR.l=E004B000
}
function exit_thermal {
        setpci -s $PCI_DEV_ADDR $REG_MDR.l=$B0_EXIT
        setpci -s $PCI_DEV_ADDR $REG_MCR.l=E004B000
}

# sch_read_reg: $1 - port, $2 - register
function sch_read_reg {

        setpci -s $PCI_DEV_ADDR $REG_MCR.l=D0"$1""$2"00
        for i in `seq 6 9`; do
                R1=`lspci -s $PCI_DEV_ADDR -xxx | grep "d0:" | cut -d " " -f $i`
                R2=$R1" "$R2
        done
        echo $R2
}

function convert_to_celsius {
        R1=0x$1
        let R2=$R1

        C1=$((1680*$R2*$R2/1000000))
        C2=$((82652*$R2/100000))
        C3=$(($C1 - $C2 + 127 ))
        echo $C3
}

# main:
init_thermal

unconverted=`sch_read_reg "04" "B1" | cut -d " " -f 4`
sensor0_celsius=`convert_to_celsius $unconverted`

unconverted=`sch_read_reg "04" "B1" | cut -d " " -f 3`
sensor1_celsius=`convert_to_celsius $unconverted`

surface_celsius=$((($sensor0_celsius + sensor1_celsius)/2 - 7 ))

echo "CPU core: $sensor0_celsius-$sensor1_celsius C, CPU surface: $surface_celsius C."

exit_thermal
