if [ ! -d "build" ]; 
        then 
        mkdir build
        echo " run_shell:  new build folder"
fi
cd build
echo " run_shell: deleting content in build folder............................. "
rm -r *

echo " run_shell: compile code............................. "

cmake .. 
echo " please enture make result ***************************start******************************"
make -j8 
echo " please enture make result ***************************end******************************"
sleep 3
echo " run_shell: ready run run_vo "
sleep 0.5
cd ../bin

./run_vo 
# GLOG_logtostderr=1 ./run_vo
cd ..


