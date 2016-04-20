set(libraries "")

set(exe_name "AIV-simpleapp")

use_package(LISTLibBaseCommon libraries RuntimeFiles)
use_package(LISTLibBaseSys libraries RuntimeFiles)
use_package(LISTLibPhysicsHelper libraries RuntimeFiles)
use_package(LISTLibRenderingGraphicsHelper libraries RuntimeFiles)
use_package(LISTLibAIV libraries RuntimeFiles) # NLOPT IPOPT ACOLC are listed in LISTLibAIV?
use_package(NLOPT libraries RuntimeFiles) # no need for this if we change /install/LISTLibAIVConfig.cmake?
use_package(IPOPT libraries RuntimeFiles) # no need for this if we change /install/LISTLibAIVConfig.cmake?
use_package(ADOLC libraries RuntimeFiles) # no need for this if we change /install/LISTLibAIVConfig.cmake?

add_executable_from_current_source_tree(NAME ${exe_name} LIBRARIES ${libraries})

copy_runtime_files(RuntimeFiles)
