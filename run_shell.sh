
cd build

echo " run_shell: compile code............................. "

cmake .. 
echo " please enture make result ***************************start******************************"
make -j8 
echo " please enture make result ***************************end******************************"
sleep 2
echo " run_shell: ready run run_vo "
cd ../bin

./run_vo 
# GLOG_logtostderr=1 ./run_vo
cd ..


