set(libraries "")

set(exe_name "app-systemidentification")

use_package(NLOPT libraries RuntimeFiles)
use_package(Eigen libraries RuntimeFiles)

add_executable_from_current_source_tree(NAME ${exe_name} LIBRARIES ${libraries})

copy_runtime_files(RuntimeFiles)
