set(libraries "")

set(exe_name "app-trajecrepbyspltest")

use_package(LISTLibAIV libraries RuntimeFiles) # NLOPT IPOPT ACOLC are listed in LISTLibAIV?

add_executable_from_current_source_tree(NAME ${exe_name} LIBRARIES ${libraries})

copy_runtime_files(RuntimeFiles)