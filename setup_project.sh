#!/usr/bin/env bash
#Colors
Purple='\033[0;35m'
Green='\033[0;32m'
BGreen='\033[1;32m'
Cyan='\033[0;36m'
BCyan='\033[1;36m'
Yellow='\033[0;33m'
BYellow='\033[1;33m'
Red='\033[0;31m'
White='\033[0;37m'
BWhite='\033[1;37m'
Color_Off='\033[0m'
# Functions
SetupEnvironmentVariables() {

    FILE_PATH=$HOME/.zshrc
    WS_ROOT=$(dirname "$(readlink -f "$0")")

    if [ ! -f $FILE_PATH ]; then
        FILE_PATH=$HOME/.bashrc
        printf "${Yellow}Why don't you install zsh?\n"
    fi
    if [[ -z "${SPOT_MARLEY_WS_ROOT}" ]]; then
        echo "# Spot Marley project env variables" >>$FILE_PATH
        echo "export SPOT_MARLEY_WS_ROOT=$WS_ROOT" >>$FILE_PATH
    fi
    printf "${BGreen}Workspace root path: ${Color_Off}[${BWhite} $WS_ROOT ${Color_Off}]\n"
}

#Actual script
printf "${Purple}#####################################\n"
printf "${Purple}######${BCyan} WELCOME TO MARLEY WORLD ${Purple}######\n"
printf "${Purple}#####################################\n"
printf "\n"
printf "${Purple}###${BCyan} Environmental Variables Setup ${Purple}###\n"
SetupEnvironmentVariables

printf "\n"
printf "${BCyan}Done\n"
printf "\n"
#refresh terminal
exec $SHELL
