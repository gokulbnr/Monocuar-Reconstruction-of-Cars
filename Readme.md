Things to do before running the code:

1. Ceres Compilation
 - ./Ceres/CMakeLists.txt - filepath on line number 16 requires editing based on where ceres-bin files were stored in your system whn you installed ceres.
 - Follow the following commands:
   cd Ceres
   cmake .
   make

2. Select the sequence to run on
 - The code to run is Run.m. Before running the script, comment off all the sequences that you dont want the main script Code.m to run on. You may leave multiple sequences uncommented to have it run on multiple sequences in one run.
 - The results will show up as and when they are generated. Moreover, the results also get saved in the ./Results/ directory. You need not run the code again in case you misseed to see any results in between.

Gokul B. Nair
