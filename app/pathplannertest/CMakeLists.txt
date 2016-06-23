set(libraries "")

set(lib_name "app-pathplannertest")

use_package(LISTLibPhysicsHelper libraries RuntimeFiles)
use_package(LISTLibRenderingGraphicsHelper libraries RuntimeFiles)
use_package(LISTLibXDEHardwareSpaceMouse libraries RuntimeFiles)

add_executable_from_current_source_tree(NAME ${lib_name} LIBRARIES ${libraries})

copy_runtime_files(RuntimeFiles)
