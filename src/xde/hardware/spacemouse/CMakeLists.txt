set(libraries "")

use_package(Boost libraries RuntimeFiles)
use_package(HID libraries RuntimeFiles)
use_package(LISTLibBaseCommon libraries RuntimeFiles)
use_package(LISTLibBaseSys libraries RuntimeFiles)

include_package(Eigen)

set(lib_name "xde-hardware-spacemouse")

add_library_from_tree(NAME ${lib_name})
