#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AC_PRECLAND_ENABLED
#define AC_PRECLAND_ENABLED 0 //1 AKGL
#endif

#ifndef AC_PRECLAND_BACKEND_DEFAULT_ENABLED
#define AC_PRECLAND_BACKEND_DEFAULT_ENABLED AC_PRECLAND_ENABLED
#endif

#ifndef AC_PRECLAND_COMPANION_ENABLED
#define AC_PRECLAND_COMPANION_ENABLED AC_PRECLAND_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AC_PRECLAND_IRLOCK_ENABLED
#define AC_PRECLAND_IRLOCK_ENABLED AC_PRECLAND_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AC_PRECLAND_SITL_ENABLED
#define AC_PRECLAND_SITL_ENABLED (AC_PRECLAND_BACKEND_DEFAULT_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#ifndef AC_PRECLAND_SITL_GAZEBO_ENABLED
#define AC_PRECLAND_SITL_GAZEBO_ENABLED (AC_PRECLAND_BACKEND_DEFAULT_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif
