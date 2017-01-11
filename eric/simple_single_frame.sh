#!/bin/bash -x
#SBATCH -J gvt # job name
#SBATCH -o gvt.o%j        # output and error file name (%j expands to jobID)
#SBATCH -N 1
#SBATCH -n 1             # total number of mpi tasks requested
#SBATCH -p vis # queue (partition) -- normal, development, etc.
#SBATCH -t 02:00:00        # run time (hh:mm:ss) - 1.5 hours
#SBATCH -A GraviT

export MV2_ENABLE_AFFINITY=0
export LD_LIBRARY_PATH=/opt/apps/gcc/4.9.1/lib64:$LD_LIBRARY_PATH
EXE=/home/03755/ericlee/gvt/gravit/build/bin/gvtMpi
#MODEL=/home/03755/ericlee/hman/data/simple/
MODEL=data/simple/

# simple
eye="0 0 350"
look="0 0 0"
lpos="0 0 100"
lcolor="455 455 455"

# dolphins
#eye="0 0 520"
#look="13 0 550"
#lpos="10 50 520"
#lcolor="0 0 450"

#nodes=1
#width=512
#height=512
#depth=8
name=simple

export MY_MPIRUN_OPTIONS="MV2_DEBUG_SHOW_BACKTRACE=1 MV2_SHOW_ENV_INFO=1 MV2_DEBUG_MEM_USAGE_VERBOSE=1 MV2_SHOW_CPU_BINDING=1"

${EXE} -i ${MODEL} -t 0 -W ${width} -H ${height} -m ${name} --eye ${eye} --look ${look} --lpos ${lpos} --lcolor ${lcolor} --ray-depth ${depth}
#vglrun ${EXE} --gl -i ${MODEL} -t 0 -W ${width} -H ${height} -m ${name} --eye ${eye} --look ${look} --lpos ${lpos} --lcolor ${lcolor} --ray-depth ${depth} --warmup 10 --active 30000
#bin/gvtPly -image -file /home/03755/ericlee/hman/data/simple/  -eye 0,0,100 -look 0,0,0
