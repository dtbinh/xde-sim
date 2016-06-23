set(libraries "")

use_package(Boost libraries RuntimeFiles)
use_package(LISTLibBaseCommon libraries RuntimeFiles)
use_package(LISTLibBaseSys libraries RuntimeFiles)
use_package(LISTLibPhysicsHelper libraries RuntimeFiles)
use_package(LISTLibRenderingGraphicsHelper libraries RuntimeFiles)

include_package(Eigen)
include_package(NLOPT)
include_package(IPOPT)
include_package(ADOLC)

set(lib_name "aiv")

add_library_from_tree(NAME ${lib_name})