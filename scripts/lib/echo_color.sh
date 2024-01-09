#!/bin/bash

function echo_red    { echo -e "\033[1;31m$@\033[0m"; }
function echo_green  { echo -e "\033[1;32m$@\033[0m"; }
function echo_yellow { echo -e "\033[1;33m$@\033[0m"; }
function echo_blue   { echo -e "\033[1;34m$@\033[0m"; }
function echo_purple { echo -e "\033[1;35m$@\033[0m"; }
function echo_cyan   { echo -e "\033[1;36m$@\033[0m"; }
function echo_white  { echo -e "\033[1;37m$@\033[0m"; }
function echo_brown  { echo -e "\033[0;33m$@\033[0m"; }

function echo_build  { echo -e "\033[1;34m$@\033[0m"; }   # blue
function echo_tests  { echo -e "\033[1;35m$@\033[0m"; }   # purple

function echo_yellow_title {
    echo_yellow "=========================================="
    echo_yellow " $1"
    echo_yellow "=========================================="
}
function echo_cyan_title {
    echo_cyan "=========================================="
    echo_cyan " $1"
    echo_cyan "=========================================="
}
function echo_cyan_title {
    echo_cyan "=========================================="
    echo_cyan " $1"
    echo_cyan "=========================================="
}
