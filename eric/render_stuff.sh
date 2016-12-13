#!/bin/bash -x

cd ~/gvt/gravit/build
make -j8
cd -
rm gvt*

RUN_DIR=/home/03755/ericlee/hman/test_all/eric
sbatch ${RUN_DIR}/run_stuff_async_size_1.slurm
