#!/bin/bash

# Script for setting up the development environment.
#source /usr/share/virtualenvwrapper/virtualenvwrapper.sh

NAME=py_trees

##############################################################################
# Colours
##############################################################################

BOLD="\e[1m"
CYAN="\e[36m"
GREEN="\e[32m"
RED="\e[31m"
YELLOW="\e[33m"
RESET="\e[0m"

padded_message ()
{
  line="........................................"
  printf "%s%s${2}\n" ${1} "${line:${#1}}"
}

pretty_header ()
{
  echo -e "${BOLD}${1}${RESET}"
}

pretty_print ()
{
  echo -e "${GREEN}${1}${RESET}"
}

pretty_warning ()
{
  echo -e "${YELLOW}${1}${RESET}"
}

pretty_error ()
{
  echo -e "${RED}${1}${RESET}"
}

##############################################################################
# Methods
##############################################################################

install_package ()
{
  PACKAGE_NAME=$1
  dpkg -s ${PACKAGE_NAME} > /dev/null
  if [ $? -ne 0 ]; then
    sudo apt-get -q -y install ${PACKAGE_NAME} > /dev/null
  else
    pretty_print "  $(padded_message ${PACKAGE_NAME} "found")"
    return 0
  fi
  if [ $? -ne 0 ]; then
    pretty_error "  $(padded_message ${PACKAGE_NAME} "failed")"
    return 1
  fi
  pretty_warning "  $(padded_message ${PACKAGE_NAME} "installed")"
  return 0
}

generate_ui ()
{
  NAME=$1
pyuic5 --from-imports -o ${NAME}_ui.py ${NAME}.ui
  if [ $? -ne 0 ]; then
    pretty_error "  $(padded_message ${NAME} "failed")"
    return 1
  fi
  pretty_print "  $(padded_message ${NAME} "generated")"
  return 0 
}

generate_qrc ()
{
  NAME=$1
pyrcc5 -o ${NAME}_rc.py ${NAME}.qrc
  if [ $? -ne 0 ]; then
    pretty_error "  $(padded_message ${NAME} "failed")"
    return 1
  fi
  pretty_print "  $(padded_message ${NAME} "generated")"
  return 0 
}

##############################################################################

# Ensure reproducible results so we can avoid committing ad-nauseum to github
#  https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=872285
export QT_HASH_SEED=0

echo ""

echo -e "${CYAN}Dependencies${RESET}"
install_package pyqt5-dev-tools || return

echo ""

echo -e "${CYAN}Generating UIs${RESET}"
generate_ui main_window
generate_ui web_view

echo ""

echo -e "${CYAN}Generating QRCs${RESET}"
generate_qrc images
generate_qrc web_app

echo ""
echo "I'm grooty, you should be too."
echo ""

