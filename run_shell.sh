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
make -j8

echo " run_shell: ready run run_vo "
cd ../bin
./run_vo

cd ..


