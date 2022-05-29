#!/bin/sh

rosnode cleanup  

killall -9 roscore
killall -9 rosmaster